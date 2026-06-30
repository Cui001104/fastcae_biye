#include "GearInfillSelector.h"

#include "GearAutoOpt/data/GearOptGeometryBridge.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"

#include <QDebug>
#include <QPair>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <limits>

namespace GearAutoOpt {

namespace {

struct ScoredIndividual {
	Individual ind;
	double score = 0.0;
	double minDistNorm = 0.0;
	double localResidualNorm = 0.0;
	double alphaUsed = 1.0;
	double betaUsed = 0.0;
	int neighborCount = 0;
	QString caseHash;
};

double clamp01(double v)
{
	return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
}

QVector<double> inputBoundsLower(const GearOptConfig& cfg)
{
	return {
	    cfg.x1Bound.lower,
	    cfg.x2Bound.lower,
	    cfg.ca1Bound.lower,
	    cfg.lca1Bound.lower,
	    cfg.ca2Bound.lower,
	    cfg.lca2Bound.lower,
	    cfg.hubRatioBound.lower,
	};
}

QVector<double> inputBoundsUpper(const GearOptConfig& cfg)
{
	return {
	    cfg.x1Bound.upper,
	    cfg.x2Bound.upper,
	    cfg.ca1Bound.upper,
	    cfg.lca1Bound.upper,
	    cfg.ca2Bound.upper,
	    cfg.lca2Bound.upper,
	    cfg.hubRatioBound.upper,
	};
}

QVector<double> normalizeByBounds(const QVector<double>& x,
                                  const QVector<double>& lo,
                                  const QVector<double>& hi)
{
	QVector<double> out;
	out.reserve(x.size());
	for (int i = 0; i < x.size(); ++i) {
		const double l = i < lo.size() ? lo[i] : 0.0;
		const double h = i < hi.size() ? hi[i] : 1.0;
		const double span = h - l;
		out.append(span > 1e-12 ? clamp01((x[i] - l) / span) : 0.0);
	}
	return out;
}

double distance(const QVector<double>& a, const QVector<double>& b)
{
	const int n = std::min(a.size(), b.size());
	double sum = 0.0;
	for (int i = 0; i < n; ++i) {
		const double d = a[i] - b[i];
		sum += d * d;
	}
	return std::sqrt(sum);
}

double normalizedSpaceDistance(const QVector<double>& a, const QVector<double>& b)
{
	const int n = std::min(a.size(), b.size());
	if (n <= 0)
		return 1.0;
	return clamp01(distance(a, b) / std::sqrt(static_cast<double>(n)));
}

double minNormalizedDistanceTo(const QVector<double>& xNorm,
                               const QVector<QVector<double>>& others)
{
	double nearest = std::numeric_limits<double>::max();
	for (const QVector<double>& ex : others) {
		nearest = std::min(nearest, normalizedSpaceDistance(xNorm, ex));
	}
	return nearest == std::numeric_limits<double>::max() ? 1.0 : nearest;
}

QString formatDesignVarsLine(const GearDesignPoint& dp)
{
	return QStringLiteral("x1=%1 x2=%2 ca1=%3 lca1=%4 ca2=%5 lca2=%6 hubRatio=%7")
	    .arg(dp.x1, 0, 'g', 8)
	    .arg(dp.x2, 0, 'g', 8)
	    .arg(dp.ca1, 0, 'g', 8)
	    .arg(dp.lca1, 0, 'g', 8)
	    .arg(dp.ca2, 0, 'g', 8)
	    .arg(dp.lca2, 0, 'g', 8)
	    .arg(dp.hubRatio, 0, 'g', 8);
}

InfillScoringConfig distanceOnlyScoring()
{
	InfillScoringConfig c;
	c.alpha = 1.0;
	c.beta = 0.0;
	c.neighborK = 3;
	return c;
}

QVector<double> normalizedResiduals(const QVector<double>& residuals)
{
	QVector<double> out;
	out.reserve(residuals.size());
	double maxResidual = 0.0;
	for (double r : residuals) {
		if (std::isfinite(r) && r > maxResidual)
			maxResidual = r;
	}
	const bool hasScale = maxResidual > 1e-12;
	for (double r : residuals) {
		if (!hasScale || !std::isfinite(r) || r <= 0.0)
			out.append(0.0);
		else
			out.append(clamp01(r / maxResidual));
	}
	return out;
}

GearDesignPoint designPointForCaseHash(const Individual& ind,
                                       const GearOptConfig& cfg,
                                       const GearDesignPoint& basePoint,
                                       double fixedMeshSizeMm,
                                       bool runMeshAuto,
                                       double fixedWidthMm)
{
	GearDesignPoint dp = ind.toDesignPoint();
	if (cfg.useOptimizationBase) {
		dp.module      = basePoint.module;
		dp.z1          = basePoint.z1;
		dp.z2          = basePoint.z2;
		dp.alpha       = basePoint.alpha;
	}
	dp.commonWidth = fixedWidthMm;
	applySurrogateInputVars(dp, surrogateInputVars(dp));
	checkGearOptDesignPointBounds(dp, cfg);
	dp.applyRunSimDefaults(cfg, fixedMeshSizeMm, runMeshAuto);
	return dp;
}

Population selectImpl(const Population& surrogatePareto,
                      const QVector<SurrogateSample>& existingSamples,
                      const QVector<double>& lo,
                      const QVector<double>& hi,
                      const GearOptConfig* cfg,
                      const GearDesignPoint* basePoint,
                      double fixedMeshSizeMm,
                      bool runMeshAuto,
                      const QSet<QString>& existingCaseHashes,
                      const QSet<QString>& existingDesignHashes,
                      const QSet<QString>& failedCaseHashes,
                      const QVector<double>& sampleResiduals,
                      const InfillScoringConfig& scoring,
                      int k)
{
	if (k <= 0 || surrogatePareto.isEmpty())
		return {};

	const auto logMsg = [&](const QString& msg) {
		if (scoring.logFn)
			scoring.logFn(msg);
		else
			qDebug().noquote() << msg;
	};

	QVector<QVector<double>> existingNorm;
	QVector<double> residualNormAll = normalizedResiduals(sampleResiduals);
	QVector<double> existingResidualNorm;
	QSet<QString> knownCaseHashes = existingCaseHashes;
	QSet<QString> knownDesignHashes = existingDesignHashes;
	for (int si = 0; si < existingSamples.size(); ++si) {
		const SurrogateSample& s = existingSamples[si];
		if (!s.x.isEmpty()) {
			existingNorm.append(normalizeByBounds(s.x, lo, hi));
			existingResidualNorm.append(si < residualNormAll.size() ? residualNormAll[si] : 0.0);
		}
		if (!s.caseHash.isEmpty())
			knownCaseHashes.insert(s.caseHash);
		if (!s.designHash.isEmpty())
			knownDesignHashes.insert(s.designHash);
	}

	QVector<ScoredIndividual> scored;
	const double minDesignDist = scoring.minDesignDistNorm > 0.0 ? scoring.minDesignDistNorm : 1e-4;
	for (const Individual& ind : surrogatePareto) {
		GearDesignPoint dp = ind.toDesignPoint();
		if (cfg && basePoint) {
			dp = designPointForCaseHash(ind, *cfg, *basePoint, fixedMeshSizeMm, runMeshAuto,
			                            basePoint->commonWidth);
		}
		QString reliefReason;
		if (!validateReliefDesign(dp, &reliefReason)) {
			logInvalidReliefDesign(dp, reliefReason);
			continue;
		}
		const QVector<double> xNorm = normalizeByBounds(surrogateInputVars(dp), lo, hi);
		const double distExisting = minNormalizedDistanceTo(xNorm, existingNorm);
		if (!existingNorm.isEmpty() && distExisting < minDesignDist) {
			logMsg(QStringLiteral("[Infill] skip near-duplicate design vars minDist=%1 < %2 | %3")
			           .arg(distExisting, 0, 'g', 6)
			           .arg(minDesignDist, 0, 'g', 6)
			           .arg(formatDesignVarsLine(dp)));
			continue;
		}

		if (cfg && basePoint) {
			const QString caseH = GearOptResultDatabase::caseHash(dp);
			if (failedCaseHashes.contains(caseH)) {
				logMsg(QStringLiteral("[Infill] skip failed case_hash=%1").arg(caseH));
				continue;
			}
			if (knownCaseHashes.contains(caseH)) {
				logMsg(QStringLiteral("[Infill] skip existing cache: %1 | %2")
				           .arg(caseH, formatDesignVarsLine(dp)));
				continue;
			}
			const QString designH = GearOptResultDatabase::designHash(dp);
			if (knownDesignHashes.contains(designH)) {
				logMsg(QStringLiteral("[Infill] skip existing design_hash: %1 | %2")
				           .arg(designH, formatDesignVarsLine(dp)));
				continue;
			}
		}

		QVector<QPair<double, int>> neighbors;
		neighbors.reserve(existingNorm.size());
		double nearest = std::numeric_limits<double>::max();
		for (int ei = 0; ei < existingNorm.size(); ++ei) {
			const double d = normalizedSpaceDistance(xNorm, existingNorm[ei]);
			nearest = std::min(nearest, d);
			neighbors.append(qMakePair(d, ei));
		}
		if (nearest == std::numeric_limits<double>::max())
			nearest = 1.0;

		std::sort(neighbors.begin(), neighbors.end(), [](const QPair<double, int>& a,
		                                                 const QPair<double, int>& b) {
			return a.first < b.first;
		});

		const int kk = std::max(0, std::min(scoring.neighborK, neighbors.size()));
		double localResidual = 0.0;
		int residualCount = 0;
		for (int ni = 0; ni < kk; ++ni) {
			const int ri = neighbors[ni].second;
			if (ri >= 0 && ri < existingResidualNorm.size()) {
				localResidual += existingResidualNorm[ri];
				++residualCount;
			}
		}
		const double localResidualNorm =
		    residualCount > 0 ? localResidual / static_cast<double>(residualCount) : 0.0;
		const bool residualActive = scoring.beta > 0.0
		                            && std::any_of(existingResidualNorm.begin(), existingResidualNorm.end(),
		                                           [](double r) { return r > 1e-12; });
		const double alpha = scoring.alpha;
		const double beta = residualActive ? scoring.beta : 0.0;
		const double denom = std::max(alpha + beta, 1e-12);

		ScoredIndividual si;
		si.ind = ind;
		si.minDistNorm = nearest;
		si.localResidualNorm = residualActive ? localResidualNorm : 0.0;
		si.alphaUsed = alpha;
		si.betaUsed = beta;
		si.neighborCount = kk;
		si.score = (alpha * si.minDistNorm + beta * si.localResidualNorm) / denom;
		if (cfg && basePoint) {
			const GearDesignPoint hashDp =
			    designPointForCaseHash(ind, *cfg, *basePoint, fixedMeshSizeMm, runMeshAuto,
			                           basePoint->commonWidth);
			si.caseHash = GearOptResultDatabase::caseHash(hashDp);
		}
		scored.append(si);
	}

	std::sort(scored.begin(), scored.end(), [](const ScoredIndividual& a, const ScoredIndividual& b) {
		return a.score > b.score;
	});

	QSet<QString> selectedCaseHashes;
	QVector<QVector<double>> selectedNorm;
	Population out;
	for (const ScoredIndividual& si : scored) {
		if (out.size() >= k)
			break;

		GearDesignPoint candDp = si.ind.toDesignPoint();
		if (cfg && basePoint) {
			candDp = designPointForCaseHash(si.ind, *cfg, *basePoint, fixedMeshSizeMm, runMeshAuto,
			                                basePoint->commonWidth);
		}
		QString reliefReason;
		if (!validateReliefDesign(candDp, &reliefReason)) {
			logInvalidReliefDesign(candDp, reliefReason);
			continue;
		}
		const QVector<double> candNorm =
		    normalizeByBounds(surrogateInputVars(candDp), lo, hi);
		double minDistToRealOrSelected = minNormalizedDistanceTo(candNorm, existingNorm);
		for (const QVector<double>& sel : selectedNorm) {
			minDistToRealOrSelected =
			    std::min(minDistToRealOrSelected, normalizedSpaceDistance(candNorm, sel));
		}
		if (minDistToRealOrSelected < minDesignDist) {
			logMsg(QStringLiteral("[Infill] skip near-duplicate minDist=%1 < %2 | %3")
			           .arg(minDistToRealOrSelected, 0, 'g', 6)
			           .arg(minDesignDist, 0, 'g', 6)
			           .arg(formatDesignVarsLine(candDp)));
			continue;
		}

		if (cfg && basePoint) {
			const GearDesignPoint dp = candDp;
			const QString caseH = GearOptResultDatabase::caseHash(dp);
			if (failedCaseHashes.contains(caseH)) {
				logMsg(QStringLiteral("[Infill] skip failed case_hash=%1").arg(caseH));
				continue;
			}
			if (selectedCaseHashes.contains(caseH)) {
				logMsg(QStringLiteral("[Infill] skip duplicate in this round: %1 | %2")
				           .arg(caseH, formatDesignVarsLine(dp)));
				continue;
			}
			selectedCaseHashes.insert(caseH);
		}

		selectedNorm.append(candNorm);
		out.append(si.ind);
		logMsg(QStringLiteral(
		           "[Infill] selected rank=%1 case_hash=%2 minDistNorm=%3 localResidualNorm=%4 "
		           "score=%5 k=%6 alpha=%7 beta=%8 | %9")
		           .arg(si.ind.rank)
		           .arg(si.caseHash.isEmpty() ? QStringLiteral("n/a") : si.caseHash)
		           .arg(si.minDistNorm, 0, 'g', 6)
		           .arg(si.localResidualNorm, 0, 'g', 6)
		           .arg(si.score, 0, 'g', 6)
		           .arg(si.neighborCount)
		           .arg(si.alphaUsed, 0, 'g', 6)
		           .arg(si.betaUsed, 0, 'g', 6)
		           .arg(formatDesignVarsLine(candDp)));
	}

	if (out.size() < k) {
		logMsg(QStringLiteral("[Infill] warning: requested %1 points, selected %2")
		                          .arg(k)
		                          .arg(out.size()));
	}

	return out;
}

} // namespace

Population GearInfillSelector::selectSparseParetoPoints(const Population& surrogatePareto,
                                                        const QVector<SurrogateSample>& existingSamples,
                                                        int k)
{
	QVector<double> lo;
	QVector<double> hi;
	if (!existingSamples.isEmpty()) {
		const int dim = existingSamples.first().x.size();
		lo.fill(0.0, dim);
		hi.fill(1.0, dim);
		for (int i = 0; i < dim; ++i) {
			lo[i] = std::numeric_limits<double>::max();
			hi[i] = -std::numeric_limits<double>::max();
		}
		for (const SurrogateSample& s : existingSamples) {
			for (int i = 0; i < dim && i < s.x.size(); ++i) {
				lo[i] = std::min(lo[i], s.x[i]);
				hi[i] = std::max(hi[i], s.x[i]);
			}
		}
	}
	return selectImpl(surrogatePareto, existingSamples, lo, hi,
	                  nullptr, nullptr, 0.0, false, {}, {}, {}, {}, distanceOnlyScoring(), k);
}

Population GearInfillSelector::selectSparseParetoPoints(const Population& surrogatePareto,
                                                        const QVector<SurrogateSample>& existingSamples,
                                                        const GearOptConfig& cfg,
                                                        const GearDesignPoint& basePoint,
                                                        double fixedMeshSizeMm,
                                                        bool runMeshAuto,
                                                        const QSet<QString>& existingCaseHashes,
                                                        int k)
{
	return selectImpl(surrogatePareto, existingSamples,
	                  inputBoundsLower(cfg), inputBoundsUpper(cfg),
	                  &cfg, &basePoint, fixedMeshSizeMm, runMeshAuto,
	                  existingCaseHashes, {}, {}, {}, distanceOnlyScoring(), k);
}

Population GearInfillSelector::selectSparseParetoPoints(const Population& surrogatePareto,
                                                        const QVector<SurrogateSample>& existingSamples,
                                                        const QVector<double>& sampleResiduals,
                                                        const InfillScoringConfig& scoring,
                                                        const GearOptConfig& cfg,
                                                        const GearDesignPoint& basePoint,
                                                        double fixedMeshSizeMm,
                                                        bool runMeshAuto,
                                                        const QSet<QString>& existingCaseHashes,
                                                        const QSet<QString>& existingDesignHashes,
                                                        const QSet<QString>& failedCaseHashes,
                                                        int k)
{
	return selectImpl(surrogatePareto, existingSamples,
	                  inputBoundsLower(cfg), inputBoundsUpper(cfg),
	                  &cfg, &basePoint, fixedMeshSizeMm, runMeshAuto,
	                  existingCaseHashes, existingDesignHashes, failedCaseHashes,
	                  sampleResiduals, scoring, k);
}

QString GearInfillSelector::formatDesignVarsForLog(const GearDesignPoint& dp)
{
	return formatDesignVarsLine(dp);
}

} // namespace GearAutoOpt
