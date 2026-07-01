#include "GearSurrogateVerboseLog.h"

#include "GearAutoOpt/data/GearOptGeometryBridge.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"

#include <QDir>
#include <QFileInfo>
#include <algorithm>
#include <cmath>
#include <limits>

namespace GearAutoOpt {

namespace {

QString naOrNum(double v, int prec = 6)
{
	if (v < 0.0 || !std::isfinite(v))
		return QStringLiteral("NA");
	return QString::number(v, 'g', prec);
}

QString pctOrNa(double relErr)
{
	if (relErr < 0.0 || !std::isfinite(relErr))
		return QStringLiteral("NA");
	return QStringLiteral("%1%").arg(relErr * 100.0, 0, 'f', 2);
}

QString boundsTag(const Bounds& b, bool enabled)
{
	return QStringLiteral("[%1,%2] enabled=%3")
	    .arg(b.lower, 0, 'g', 6)
	    .arg(b.upper, 0, 'g', 6)
	    .arg(enabled ? QStringLiteral("true") : QStringLiteral("false"));
}

SampleMetricRange rangeOf(const QVector<double>& vals)
{
	SampleMetricRange r;
	if (vals.isEmpty())
		return r;
	double sum = 0.0;
	r.minVal = vals[0];
	r.maxVal = vals[0];
	for (double v : vals) {
		if (!std::isfinite(v))
			continue;
		r.minVal = std::min(r.minVal, v);
		r.maxVal = std::max(r.maxVal, v);
		sum += v;
	}
	r.meanVal = sum / static_cast<double>(vals.size());
	return r;
}

QString inferCcxFailureStage(const GearDesignPoint& dp)
{
	if (dp.status == PointStatus::Infeasible || dp.status == PointStatus::Invalid)
		return QStringLiteral("geometry");
	const QString e = dp.errorMsg.toLower();
	if (e.contains(QStringLiteral("gmsh")) || e.contains(QStringLiteral("mesh"))
	    || e.contains(QStringLiteral("timeout")))
		return QStringLiteral("mesh");
	if (e.contains(QStringLiteral("frd")) || e.contains(QStringLiteral("parse"))
	    || e.contains(QStringLiteral("post")))
		return QStringLiteral("postprocess");
	return QStringLiteral("ccx");
}

QString csvPath(const QString& runDir, const QString& name)
{
	return QFileInfo::exists(QDir(runDir).filePath(name)) ? QDir(runDir).filePath(name)
	                                                      : QStringLiteral("missing");
}

QVector<double> inputBoundsLower(const GearOptConfig& cfg)
{
	return {cfg.x1Bound.lower,  cfg.x2Bound.lower,  cfg.ca1Bound.lower, cfg.lca1Bound.lower,
	        cfg.ca2Bound.lower, cfg.lca2Bound.lower, cfg.hubRatioBound.lower};
}

QVector<double> inputBoundsUpper(const GearOptConfig& cfg)
{
	return {cfg.x1Bound.upper,  cfg.x2Bound.upper,  cfg.ca1Bound.upper, cfg.lca1Bound.upper,
	        cfg.ca2Bound.upper, cfg.lca2Bound.upper, cfg.hubRatioBound.upper};
}

double clamp01(double v)
{
	return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
}

double normalizedDist(const QVector<double>& a, const QVector<double>& b)
{
	const int n = std::min(a.size(), b.size());
	if (n <= 0)
		return 1.0;
	double sum = 0.0;
	for (int i = 0; i < n; ++i) {
		const double d = a[i] - b[i];
		sum += d * d;
	}
	return clamp01(std::sqrt(sum) / std::sqrt(static_cast<double>(n)));
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

} // namespace

bool surrogateVerboseLogEnabled(const GearOptConfig& cfg)
{
	return cfg.solver.surrogateVerbose;
}

TrainMetricRanges computeTrainMetricRanges(const QVector<SurrogateSample>& samples)
{
	TrainMetricRanges tr;
	QVector<double> cpress, edge, sigma, u, mass, cv;
	for (const SurrogateSample& s : samples) {
		if (s.cpressMax > 0.0)
			cpress.append(s.cpressMax);
		if (s.edgeLoadRatio > 0.0)
			edge.append(s.edgeLoadRatio);
		if (s.sigmaMax > 0.0)
			sigma.append(s.sigmaMax);
		if (s.uMax >= 0.0)
			u.append(s.uMax);
		if (s.mass > 0.0)
			mass.append(s.mass);
		if (s.cpressCV > 0.0)
			cv.append(s.cpressCV);
	}
	tr.cpress         = rangeOf(cpress);
	tr.edgeLoadRatio  = rangeOf(edge);
	tr.sigmaMax       = rangeOf(sigma);
	tr.uMax           = rangeOf(u);
	tr.mass           = rangeOf(mass);
	tr.cpressCV       = rangeOf(cv);
	return tr;
}

void emitSurrogateRunSummary(const GearOptConfig& cfg,
                             const QString& runId,
                             const GearDesignPoint& baseDp,
                             double fixedWidthMm,
                             double fixedMeshSizeMm,
                             double fixedRootMeshSizeMm,
                             int fixedZLayers,
                             const QString& baseCaseHash,
                             const std::function<void(const QString&)>& logFn)
{
	if (!surrogateVerboseLogEnabled(cfg) || !logFn)
		return;
	const DesignVariableFlags& dv = cfg.designVars;
	logFn(QStringLiteral("[GearOpt][RunSummary]"));
	logFn(QStringLiteral("run_id=%1").arg(runId));
	logFn(QStringLiteral("mode=surrogate_assisted"));
	logFn(QStringLiteral("base_case_hash=%1").arg(baseCaseHash));
	logFn(QStringLiteral("base case:"));
	logFn(QStringLiteral("m=%1").arg(baseDp.module, 0, 'g', 8));
	logFn(QStringLiteral("z1=%1").arg(baseDp.z1));
	logFn(QStringLiteral("z2=%1").arg(baseDp.z2));
	logFn(QStringLiteral("alpha=%1").arg(baseDp.alpha, 0, 'g', 8));
	logFn(QStringLiteral("width=%1").arg(fixedWidthMm, 0, 'g', 8));
	logFn(QStringLiteral("torque=%1").arg(baseDp.torque_Nm, 0, 'g', 8));
	logFn(QStringLiteral("meshSize=%1").arg(fixedMeshSizeMm, 0, 'g', 8));
	logFn(QStringLiteral("rootSize=%1").arg(fixedRootMeshSizeMm, 0, 'g', 8));
	logFn(QStringLiteral("zLayers=%1").arg(fixedZLayers));
	logFn(QStringLiteral("contact=%1").arg(baseDp.enableContact ? QStringLiteral("on")
	                                                            : QStringLiteral("off")));
	logFn(QStringLiteral("material=%1").arg(baseDp.materialName.isEmpty() ? QStringLiteral("STEEL")
	                                                                    : baseDp.materialName));
	logFn(QStringLiteral("surrogate input dim=%1").arg(kSurrogateInputDim));
	logFn(QStringLiteral("surrogate vars:"));
	logFn(QStringLiteral("x1=%1").arg(boundsTag(cfg.x1Bound, dv.x1)));
	logFn(QStringLiteral("x2=%1").arg(boundsTag(cfg.x2Bound, dv.x2)));
	logFn(QStringLiteral("ca1=%1").arg(boundsTag(cfg.ca1Bound, dv.ca1)));
	logFn(QStringLiteral("lca1=%1").arg(boundsTag(cfg.lca1Bound, dv.lca1)));
	logFn(QStringLiteral("ca2=%1").arg(boundsTag(cfg.ca2Bound, dv.ca2)));
	logFn(QStringLiteral("lca2=%1").arg(boundsTag(cfg.lca2Bound, dv.lca2)));
	logFn(QStringLiteral("hubRatio=%1").arg(boundsTag(cfg.hubRatioBound, dv.hubRatio)));
	logFn(QStringLiteral("objectives:"));
	logFn(QStringLiteral("primary=cpressMax_MPa"));
	logFn(QStringLiteral("secondary=edgeLoadRatio"));
	logFn(QStringLiteral("recorded metrics:"));
	logFn(QStringLiteral("sigmaMax_MPa,uMax_mm,mass_kg,cpressCV"));
}

void emitSurrogateDbSummary(const GearOptConfig& cfg,
                            const QString& baseCaseHash,
                            double fixedWidthMm,
                            int minRequiredSamples,
                            const SurrogateSampleLoadStats& mergedStats,
                            int duplicateSkipped,
                            const std::function<void(const QString&)>& logFn)
{
	if (!surrogateVerboseLogEnabled(cfg) || !logFn)
		return;

	auto& globalDb = GearOptResultDatabase::global();
	auto& runDb    = GearOptResultDatabase::runSession();

	SurrogateSampleLoadStats gStats;
	SurrogateSampleLoadStats rStats;
	if (globalDb.isOpen())
		globalDb.loadValidatedSamples(baseCaseHash, fixedWidthMm, 0, &gStats);
	if (runDb.isOpen())
		runDb.loadValidatedSamples(baseCaseHash, fixedWidthMm, 0, &rStats);

	const int gVerified = globalDb.isOpen()
	                          ? globalDb.countValidatedSamplesForBaseCase(baseCaseHash, fixedWidthMm)
	                          : 0;
	const int rVerified = runDb.isOpen()
	                          ? runDb.countValidatedSamplesForBaseCase(baseCaseHash, fixedWidthMm)
	                          : 0;

	logFn(QStringLiteral("[GearOpt][DBSummary]"));
	logFn(QStringLiteral("global path=%1")
	          .arg(globalDb.isOpen() ? globalDb.databasePath()
	                                 : GearOptResultDatabase::defaultGlobalDatabasePath()));
	logFn(QStringLiteral("run path=%1")
	          .arg(runDb.isOpen() ? runDb.databasePath()
	                              : GearOptResultDatabase::databasePathInRunDir(QString())));
	logFn(QStringLiteral("global total=%1").arg(globalDb.isOpen() ? globalDb.countValidResults() : 0));
	logFn(QStringLiteral("run total=%1").arg(rStats.totalSamples));
	logFn(QStringLiteral("same baseCase global total=%1").arg(gStats.totalSamples));
	logFn(QStringLiteral("same baseCase run total=%1").arg(rStats.totalSamples));
	logFn(QStringLiteral("same baseCase+width verified global=%1").arg(gVerified));
	logFn(QStringLiteral("same baseCase+width verified run=%1").arg(rVerified));
	logFn(QStringLiteral("merged valid samples=%1").arg(mergedStats.validSamples));
	logFn(QStringLiteral("failed_skipped=%1").arg(mergedStats.failedSkipped));
	logFn(QStringLiteral("duplicate_skipped=%1").arg(duplicateSkipped));
	logFn(QStringLiteral("min required samples=%1").arg(minRequiredSamples));
}

void emitInitialLhsLog(int required,
                       int existing,
                       int toGenerate,
                       int lhsBatch,
                       const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;
	logFn(QStringLiteral("[GearOpt][InitialLHS]"));
	logFn(QStringLiteral("required=%1").arg(required));
	logFn(QStringLiteral("existing=%1").arg(existing));
	logFn(QStringLiteral("to_generate=%1").arg(toGenerate));
	logFn(QStringLiteral("lhs_batch=%1").arg(lhsBatch));
	logFn(QStringLiteral("reason=insufficient_training_samples"));
}

void emitRoundStartLog(int round,
                       int samplesLoaded,
                       int newSamplesSinceLastRound,
                       int failedSkipped,
                       int trainValid,
                       int infillTarget,
                       const QString& phase,
                       int exploitCount,
                       int exploreCount,
                       const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;
	logFn(QStringLiteral("[GearOpt][RoundStart]"));
	logFn(QStringLiteral("round=%1").arg(round));
	logFn(QStringLiteral("samples_loaded=%1").arg(samplesLoaded));
	logFn(QStringLiteral("new_samples_since_last_round=%1").arg(newSamplesSinceLastRound));
	logFn(QStringLiteral("failed_skipped=%1").arg(failedSkipped));
	logFn(QStringLiteral("train_valid=%1").arg(trainValid));
	logFn(QStringLiteral("train_invalid=%1").arg(failedSkipped));
	logFn(QStringLiteral("infill_target=%1").arg(infillTarget));
	logFn(QStringLiteral("phase=%1").arg(phase));
	logFn(QStringLiteral("exploitCount=%1").arg(exploitCount));
	logFn(QStringLiteral("exploreCount=%1").arg(exploreCount));
}

void emitRbfTrainingLog(int round,
                        bool trained,
                        int trainSamples,
                        const QVector<SurrogateSample>& samples,
                        const GearSurrogateModel& model,
                        const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;

	double cpressMin = -1.0, cpressMax = -1.0;
	double edgeMin = -1.0, edgeMax = -1.0;
	for (const SurrogateSample& s : samples) {
		if (s.cpressMax > 0.0) {
			cpressMin = cpressMin < 0.0 ? s.cpressMax : std::min(cpressMin, s.cpressMax);
			cpressMax = cpressMax < 0.0 ? s.cpressMax : std::max(cpressMax, s.cpressMax);
		}
		if (s.edgeLoadRatio > 0.0) {
			edgeMin = edgeMin < 0.0 ? s.edgeLoadRatio : std::min(edgeMin, s.edgeLoadRatio);
			edgeMax = edgeMax < 0.0 ? s.edgeLoadRatio : std::max(edgeMax, s.edgeLoadRatio);
		}
	}

	const double looCpress = model.trainingMaxRelativeError(QStringLiteral("cpressMax_MPa"));
	const double looEdge   = model.trainingMaxRelativeError(QStringLiteral("edgeLoadRatio"));

	QString warning = QStringLiteral("none");
	if (trained) {
		if (looEdge >= 0.10)
			warning = QStringLiteral("high_edge_error");
		else if (looCpress >= 0.10)
			warning = QStringLiteral("high_cpress_error");
	} else {
		warning = QStringLiteral("training_failed");
	}

	logFn(QStringLiteral("[GearOpt][RBF]"));
	logFn(QStringLiteral("round=%1").arg(round));
	logFn(QStringLiteral("trained=%1").arg(trained ? QStringLiteral("true") : QStringLiteral("false")));
	logFn(QStringLiteral("train_samples=%1").arg(trainSamples));
	logFn(QStringLiteral("input_dim=7"));
	logFn(QStringLiteral("targets=cpressMax_MPa,edgeLoadRatio"));
	logFn(QStringLiteral("cpress_train_range=[%1,%2]")
	          .arg(naOrNum(cpressMin))
	          .arg(naOrNum(cpressMax)));
	logFn(QStringLiteral("edge_train_range=[%1,%2]").arg(naOrNum(edgeMin)).arg(naOrNum(edgeMax)));
	logFn(QStringLiteral("cpress_LOO_max_rel_err=%1").arg(pctOrNa(looCpress)));
	logFn(QStringLiteral("edge_LOO_max_rel_err=%1").arg(pctOrNa(looEdge)));
	logFn(QStringLiteral("warning=%1").arg(warning));

	const TrainMetricRanges tr = computeTrainMetricRanges(samples);
	logFn(QStringLiteral("[GearOpt][TrainRange]"));
	logFn(QStringLiteral("cpress min=%1 max=%2 mean=%3")
	          .arg(naOrNum(tr.cpress.minVal))
	          .arg(naOrNum(tr.cpress.maxVal))
	          .arg(naOrNum(tr.cpress.meanVal)));
	logFn(QStringLiteral("edgeLoadRatio min=%1 max=%2 mean=%3")
	          .arg(naOrNum(tr.edgeLoadRatio.minVal))
	          .arg(naOrNum(tr.edgeLoadRatio.maxVal))
	          .arg(naOrNum(tr.edgeLoadRatio.meanVal)));
	logFn(QStringLiteral("sigmaMax min=%1 max=%2 mean=%3")
	          .arg(naOrNum(tr.sigmaMax.minVal))
	          .arg(naOrNum(tr.sigmaMax.maxVal))
	          .arg(naOrNum(tr.sigmaMax.meanVal)));
	logFn(QStringLiteral("uMax min=%1 max=%2 mean=%3")
	          .arg(naOrNum(tr.uMax.minVal))
	          .arg(naOrNum(tr.uMax.maxVal))
	          .arg(naOrNum(tr.uMax.meanVal)));
	logFn(QStringLiteral("mass min=%1 max=%2 mean=%3")
	          .arg(naOrNum(tr.mass.minVal))
	          .arg(naOrNum(tr.mass.maxVal))
	          .arg(naOrNum(tr.mass.meanVal)));
	logFn(QStringLiteral("cpressCV min=%1 max=%2 mean=%3")
	          .arg(naOrNum(tr.cpressCV.minVal))
	          .arg(naOrNum(tr.cpressCV.maxVal))
	          .arg(naOrNum(tr.cpressCV.meanVal)));
}

void emitSurrogateNsgaLog(int round,
                          int population,
                          int generations,
                          const Population& surrogatePareto,
                          double predHv,
                          const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;

	double bestCpress = std::numeric_limits<double>::max();
	double minCpress  = std::numeric_limits<double>::max();
	double maxCpress  = -1.0;
	double bestEdge   = std::numeric_limits<double>::max();
	double minEdge    = std::numeric_limits<double>::max();
	double maxEdge    = -1.0;

	for (const Individual& ind : surrogatePareto) {
		if (ind.objs.size() < 2)
			continue;
		const double c = ind.objs[0];
		const double e = ind.objs[1];
		if (c > 0.0 && std::isfinite(c)) {
			bestCpress = std::min(bestCpress, c);
			minCpress  = std::min(minCpress, c);
			maxCpress  = std::max(maxCpress, c);
		}
		if (e > 0.0 && std::isfinite(e)) {
			bestEdge = std::min(bestEdge, e);
			minEdge  = std::min(minEdge, e);
			maxEdge  = std::max(maxEdge, e);
		}
	}

	logFn(QStringLiteral("[GearOpt][SurrogateNSGA]"));
	logFn(QStringLiteral("round=%1").arg(round));
	logFn(QStringLiteral("population=%1").arg(population));
	logFn(QStringLiteral("generations=%1").arg(generations));
	logFn(QStringLiteral("pareto_size=%1").arg(surrogatePareto.size()));
	logFn(QStringLiteral("pred_cpress_best=%1").arg(naOrNum(bestCpress)));
	logFn(QStringLiteral("pred_cpress_min=%1").arg(naOrNum(minCpress)));
	logFn(QStringLiteral("pred_cpress_max=%1").arg(naOrNum(maxCpress)));
	logFn(QStringLiteral("pred_edge_best=%1").arg(naOrNum(bestEdge)));
	logFn(QStringLiteral("pred_edge_min=%1").arg(naOrNum(minEdge)));
	logFn(QStringLiteral("pred_edge_max=%1").arg(naOrNum(maxEdge)));
	logFn(QStringLiteral("pred_hv=%1").arg(naOrNum(predHv)));

	Population sorted = surrogatePareto;
	std::sort(sorted.begin(), sorted.end(), [](const Individual& a, const Individual& b) {
		if (a.objs.isEmpty() || b.objs.isEmpty())
			return false;
		return a.objs[0] < b.objs[0];
	});

	logFn(QStringLiteral("[GearOpt][PredParetoTop]"));
	const int topN = std::min(5, sorted.size());
	for (int i = 0; i < topN; ++i) {
		const Individual& ind = sorted[i];
		GearDesignPoint dp  = ind.toDesignPoint();
		const double pc     = ind.objs.size() > 0 ? ind.objs[0] : -1.0;
		const double pe     = ind.objs.size() > 1 ? ind.objs[1] : -1.0;
		logFn(QStringLiteral(
		          "rank=%1 pred_cpress=%2 pred_edge=%3 x1=%4 x2=%5 ca1=%6 lca1=%7 ca2=%8 lca2=%9 "
		          "hubRatio=%10")
		          .arg(i + 1)
		          .arg(naOrNum(pc))
		          .arg(naOrNum(pe))
		          .arg(dp.x1, 0, 'g', 8)
		          .arg(dp.x2, 0, 'g', 8)
		          .arg(dp.ca1, 0, 'g', 8)
		          .arg(dp.lca1, 0, 'g', 8)
		          .arg(dp.ca2, 0, 'g', 8)
		          .arg(dp.lca2, 0, 'g', 8)
		          .arg(dp.hubRatio, 0, 'g', 8));
	}
}

void emitInfillPlanLog(int round,
                       int target,
                       const QString& phase,
                       int exploitCount,
                       int exploreCount,
                       const QString& sourceExploit,
                       double minDesignDistNorm,
                       double alpha,
                       double beta,
                       int neighborK,
                       const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;
	logFn(QStringLiteral("[GearOpt][InfillPlan]"));
	logFn(QStringLiteral("round=%1").arg(round));
	logFn(QStringLiteral("target=%1").arg(target));
	logFn(QStringLiteral("phase=%1").arg(phase));
	logFn(QStringLiteral("exploitCount=%1").arg(exploitCount));
	logFn(QStringLiteral("exploreCount=%1").arg(exploreCount));
	logFn(QStringLiteral("source_exploit=%1").arg(sourceExploit));
	logFn(QStringLiteral("source_explore=globalLHSMaximin"));
	logFn(QStringLiteral("minDesignDistNorm=%1").arg(minDesignDistNorm, 0, 'g', 6));
	logFn(QStringLiteral("alpha=%1").arg(alpha, 0, 'g', 6));
	logFn(QStringLiteral("beta=%1").arg(beta, 0, 'g', 6));
	logFn(QStringLiteral("neighborK=%1").arg(neighborK));
}

void emitInfillFilterLog(const InfillFilterStats& stats,
                         const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;
	logFn(QStringLiteral("[GearOpt][InfillFilter]"));
	logFn(QStringLiteral("candidate_total=%1").arg(stats.candidateTotal));
	logFn(QStringLiteral("valid_after_relief=%1").arg(stats.validAfterRelief));
	logFn(QStringLiteral("skip_failed=%1").arg(stats.skipFailed));
	logFn(QStringLiteral("skip_known_case=%1").arg(stats.skipKnownCase));
	logFn(QStringLiteral("skip_known_design=%1").arg(stats.skipKnownDesign));
	logFn(QStringLiteral("skip_near_duplicate=%1").arg(stats.skipNearDuplicate));
	logFn(QStringLiteral("skip_too_close_to_selected=%1").arg(stats.skipTooCloseToSelected));
	logFn(QStringLiteral("selected=%1").arg(stats.selected));
}

void emitInfillSelectedLog(int round,
                           const QString& type,
                           int idx,
                           const GearDesignPoint& dp,
                           double predCpress,
                           double predEdge,
                           double minDistNorm,
                           double localResidualNorm,
                           double score,
                           const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;
	const QString caseH   = GearOptResultDatabase::caseHash(dp);
	const QString designH = GearOptResultDatabase::designHash(dp);
	const bool isExplore  = type == QStringLiteral("explore");
	logFn(QStringLiteral("[GearOpt][InfillSelected]"));
	logFn(QStringLiteral("type=%1").arg(type));
	logFn(QStringLiteral("idx=%1").arg(idx));
	logFn(QStringLiteral("round=%1").arg(round));
	logFn(QStringLiteral("case_hash=%1").arg(caseH));
	logFn(QStringLiteral("design_hash=%1").arg(designH));
	logFn(QStringLiteral("pred_cpress=%1").arg(isExplore ? QStringLiteral("NA") : naOrNum(predCpress)));
	logFn(QStringLiteral("pred_edge=%1").arg(isExplore ? QStringLiteral("NA") : naOrNum(predEdge)));
	logFn(QStringLiteral("minDistNorm=%1").arg(naOrNum(minDistNorm)));
	logFn(QStringLiteral("localResidualNorm=%1")
	          .arg(isExplore ? QStringLiteral("NA") : naOrNum(localResidualNorm)));
	logFn(QStringLiteral("score=%1").arg(isExplore ? QStringLiteral("NA") : naOrNum(score)));
	logFn(QStringLiteral("x1=%1").arg(dp.x1, 0, 'g', 8));
	logFn(QStringLiteral("x2=%1").arg(dp.x2, 0, 'g', 8));
	logFn(QStringLiteral("ca1=%1").arg(dp.ca1, 0, 'g', 8));
	logFn(QStringLiteral("lca1=%1").arg(dp.lca1, 0, 'g', 8));
	logFn(QStringLiteral("ca2=%1").arg(dp.ca2, 0, 'g', 8));
	logFn(QStringLiteral("lca2=%1").arg(dp.lca2, 0, 'g', 8));
	logFn(QStringLiteral("hubRatio=%1").arg(dp.hubRatio, 0, 'g', 8));
}

void emitCcxStartLog(const SurrogateCcxLogOptions& opts,
                     int idx,
                     const GearDesignPoint& dp)
{
	if (!opts.enabled || !opts.logFn)
		return;
	const QString type = idx >= 0 && idx < opts.infillTypes.size() ? opts.infillTypes[idx]
	                                                               : QStringLiteral("unknown");
	const double predC = idx >= 0 && idx < opts.predCpress.size() ? opts.predCpress[idx] : -1.0;
	const double predE = idx >= 0 && idx < opts.predEdge.size() ? opts.predEdge[idx] : -1.0;
	opts.logFn(QStringLiteral("[GearOpt][CCXStart]"));
	opts.logFn(QStringLiteral("round=%1").arg(opts.round));
	opts.logFn(QStringLiteral("idx=%1").arg(idx));
	opts.logFn(QStringLiteral("type=%1").arg(type));
	opts.logFn(QStringLiteral("case_hash=%1").arg(GearOptResultDatabase::caseHash(dp)));
	opts.logFn(QStringLiteral("workDir=%1").arg(dp.runDir));
	opts.logFn(QStringLiteral("pred_cpress=%1").arg(naOrNum(predC)));
	opts.logFn(QStringLiteral("pred_edge=%1").arg(naOrNum(predE)));
}

void emitCcxDoneLog(const SurrogateCcxLogOptions& opts,
                    int idx,
                    const GearDesignPoint& dp,
                    bool cached,
                    double predCpress,
                    double predEdge)
{
	if (!opts.enabled || !opts.logFn)
		return;
	const QString type = idx >= 0 && idx < opts.infillTypes.size() ? opts.infillTypes[idx]
	                                                               : QStringLiteral("unknown");
	const bool converged = dp.status == PointStatus::Done && dp.cpressMax_MPa > 0.0;
	QString status;
	if (cached)
		status = QStringLiteral("cache_reused");
	else if (dp.status == PointStatus::Infeasible || dp.status == PointStatus::Invalid)
		status = QStringLiteral("invalid");
	else if (dp.status == PointStatus::Failed)
		status = QStringLiteral("failed");
	else
		status = QStringLiteral("done");

	const double absCpress =
	    predCpress >= 0.0 && dp.cpressMax_MPa > 0.0 ? std::abs(predCpress - dp.cpressMax_MPa) : -1.0;
	const double relCpress =
	    predCpress >= 0.0 && dp.cpressMax_MPa > 1e-12
	        ? std::abs(predCpress - dp.cpressMax_MPa) / dp.cpressMax_MPa
	        : -1.0;
	const double absEdge =
	    predEdge >= 0.0 && dp.edgeLoadRatio > 0.0 ? std::abs(predEdge - dp.edgeLoadRatio) : -1.0;
	const double relEdge = predEdge >= 0.0 && dp.edgeLoadRatio > 1e-12
	                           ? std::abs(predEdge - dp.edgeLoadRatio) / dp.edgeLoadRatio
	                           : -1.0;

	opts.logFn(QStringLiteral("[GearOpt][CCXDone]"));
	opts.logFn(QStringLiteral("round=%1").arg(opts.round));
	opts.logFn(QStringLiteral("idx=%1").arg(idx));
	opts.logFn(QStringLiteral("type=%1").arg(type));
	opts.logFn(QStringLiteral("case_hash=%1").arg(GearOptResultDatabase::caseHash(dp)));
	opts.logFn(QStringLiteral("status=%1").arg(status));
	opts.logFn(QStringLiteral("converged=%1").arg(converged ? QStringLiteral("true")
	                                                        : QStringLiteral("false")));
	opts.logFn(QStringLiteral("time_sec=%1").arg(naOrNum(dp.solverTime)));
	opts.logFn(QStringLiteral("nodes=%1").arg(dp.nodeCount));
	opts.logFn(QStringLiteral("elements=%1").arg(dp.elementCount));
	opts.logFn(QStringLiteral("true_cpress=%1").arg(naOrNum(dp.cpressMax_MPa)));
	opts.logFn(QStringLiteral("true_edge=%1").arg(naOrNum(dp.edgeLoadRatio)));
	opts.logFn(QStringLiteral("true_sigma=%1").arg(naOrNum(dp.sigmaMax)));
	opts.logFn(QStringLiteral("true_u=%1").arg(naOrNum(dp.uMax)));
	opts.logFn(QStringLiteral("mass=%1").arg(naOrNum(dp.mass)));
	opts.logFn(QStringLiteral("cpressCV=%1").arg(naOrNum(dp.cpressCV)));
	opts.logFn(QStringLiteral("abs_err_cpress=%1").arg(naOrNum(absCpress)));
	opts.logFn(QStringLiteral("rel_err_cpress=%1").arg(pctOrNa(relCpress)));
	opts.logFn(QStringLiteral("abs_err_edge=%1").arg(naOrNum(absEdge)));
	opts.logFn(QStringLiteral("rel_err_edge=%1").arg(pctOrNa(relEdge)));
}

void emitCcxFailLog(const SurrogateCcxLogOptions& opts,
                    int idx,
                    const GearDesignPoint& dp)
{
	if (!opts.enabled || !opts.logFn)
		return;
	if (dp.status == PointStatus::Done && dp.cpressMax_MPa > 0.0)
		return;
	const QString type = idx >= 0 && idx < opts.infillTypes.size() ? opts.infillTypes[idx]
	                                                               : QStringLiteral("unknown");
	opts.logFn(QStringLiteral("[GearOpt][CCXFail]"));
	opts.logFn(QStringLiteral("round=%1").arg(opts.round));
	opts.logFn(QStringLiteral("idx=%1").arg(idx));
	opts.logFn(QStringLiteral("type=%1").arg(type));
	opts.logFn(QStringLiteral("case_hash=%1").arg(GearOptResultDatabase::caseHash(dp)));
	opts.logFn(QStringLiteral("stage=%1").arg(inferCcxFailureStage(dp)));
	opts.logFn(QStringLiteral("reason=%1").arg(dp.errorMsg.isEmpty() ? pointStatusToString(dp.status)
	                                                                 : dp.errorMsg.left(240)));
	opts.logFn(QStringLiteral("gmsh_timeout=%1").arg(QStringLiteral("NA")));
	opts.logFn(QStringLiteral("ccx_exit_code=%1").arg(QStringLiteral("NA")));
	opts.logFn(QStringLiteral("workDir=%1").arg(dp.runDir));
}

void emitPredTrueSummaryLog(int round,
                            int newCcx,
                            int cacheReused,
                            int failed,
                            const QVector<double>& predCpress,
                            const QVector<double>& trueCpress,
                            const QVector<double>& predEdge,
                            const QVector<double>& trueEdge,
                            const QVector<SurrogateSample>& roundNewSamples,
                            const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;

	auto meanMaxRel = [](const QVector<double>& pred,
	                     const QVector<double>& truth,
	                     double& meanRel,
	                     double& maxRel) {
		meanRel = -1.0;
		maxRel  = -1.0;
		const int n = std::min(pred.size(), truth.size());
		if (n <= 0)
			return;
		double sum = 0.0;
		maxRel       = 0.0;
		int cnt      = 0;
		for (int i = 0; i < n; ++i) {
			if (truth[i] <= 1e-12)
				continue;
			const double rel = std::abs(pred[i] - truth[i]) / std::abs(truth[i]);
			sum += rel;
			maxRel = std::max(maxRel, rel);
			++cnt;
		}
		if (cnt > 0)
			meanRel = sum / static_cast<double>(cnt);
	};

	double meanCpress = -1.0, maxCpress = -1.0;
	double meanEdge = -1.0, maxEdge = -1.0;
	meanMaxRel(predCpress, trueCpress, meanCpress, maxCpress);
	meanMaxRel(predEdge, trueEdge, meanEdge, maxEdge);

	const SurrogateSample* bestCpress = nullptr;
	const SurrogateSample* bestEdge   = nullptr;
	for (const SurrogateSample& s : roundNewSamples) {
		if (s.cpressMax > 0.0 && (!bestCpress || s.cpressMax < bestCpress->cpressMax))
			bestCpress = &s;
		if (s.edgeLoadRatio > 0.0
		    && (!bestEdge || s.edgeLoadRatio < bestEdge->edgeLoadRatio))
			bestEdge = &s;
	}

	QString bestDesign = QStringLiteral("NA");
	if (bestCpress && !bestCpress->x.isEmpty()) {
		bestDesign =
		    QStringLiteral("x1=%1 x2=%2 ca1=%3 lca1=%4 ca2=%5 lca2=%6 hubRatio=%7 case_hash=%8")
		        .arg(bestCpress->x.value(0), 0, 'g', 6)
		        .arg(bestCpress->x.value(1), 0, 'g', 6)
		        .arg(bestCpress->x.value(2), 0, 'g', 6)
		        .arg(bestCpress->x.value(3), 0, 'g', 6)
		        .arg(bestCpress->x.value(4), 0, 'g', 6)
		        .arg(bestCpress->x.value(5), 0, 'g', 6)
		        .arg(bestCpress->x.value(6), 0, 'g', 6)
		        .arg(bestCpress->caseHash);
	}

	logFn(QStringLiteral("[GearOpt][PredTrueSummary]"));
	logFn(QStringLiteral("round=%1").arg(round));
	logFn(QStringLiteral("new_ccx=%1").arg(newCcx));
	logFn(QStringLiteral("cache_reused=%1").arg(cacheReused));
	logFn(QStringLiteral("failed=%1").arg(failed));
	logFn(QStringLiteral("mean_rel_err_cpress=%1").arg(pctOrNa(meanCpress)));
	logFn(QStringLiteral("max_rel_err_cpress=%1").arg(pctOrNa(maxCpress)));
	logFn(QStringLiteral("mean_rel_err_edge=%1").arg(pctOrNa(meanEdge)));
	logFn(QStringLiteral("max_rel_err_edge=%1").arg(pctOrNa(maxEdge)));
	logFn(QStringLiteral("best_true_cpress_this_round=%1")
	          .arg(bestCpress ? naOrNum(bestCpress->cpressMax) : QStringLiteral("NA")));
	logFn(QStringLiteral("best_true_edge_this_round=%1")
	          .arg(bestEdge ? naOrNum(bestEdge->edgeLoadRatio) : QStringLiteral("NA")));
	logFn(QStringLiteral("best_true_design_this_round=%1").arg(bestDesign));
}

void emitTrueParetoLog(int round,
                       int validatedSamples,
                       int trueParetoSize,
                       const QVector<SurrogateSample>& samples,
                       double ccxHv,
                       const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;
	const SurrogateSample* bestCpress = nullptr;
	const SurrogateSample* bestEdge   = nullptr;
	const SurrogateSample* bestSigma  = nullptr;
	for (const SurrogateSample& s : samples) {
		if (s.cpressMax > 0.0 && (!bestCpress || s.cpressMax < bestCpress->cpressMax))
			bestCpress = &s;
		if (s.edgeLoadRatio > 0.0 && (!bestEdge || s.edgeLoadRatio < bestEdge->edgeLoadRatio))
			bestEdge = &s;
		if (s.sigmaMax > 0.0 && (!bestSigma || s.sigmaMax < bestSigma->sigmaMax))
			bestSigma = &s;
	}
	logFn(QStringLiteral("[GearOpt][TruePareto]"));
	logFn(QStringLiteral("round=%1").arg(round));
	logFn(QStringLiteral("validated_samples=%1").arg(validatedSamples));
	logFn(QStringLiteral("true_pareto_size=%1").arg(trueParetoSize));
	logFn(QStringLiteral("best_cpress=%1")
	          .arg(bestCpress ? naOrNum(bestCpress->cpressMax) : QStringLiteral("NA")));
	logFn(QStringLiteral("best_edge=%1")
	          .arg(bestEdge ? naOrNum(bestEdge->edgeLoadRatio) : QStringLiteral("NA")));
	logFn(QStringLiteral("best_sigma=%1")
	          .arg(bestSigma ? naOrNum(bestSigma->sigmaMax) : QStringLiteral("NA")));
	logFn(QStringLiteral("ccx_hv=%1").arg(naOrNum(ccxHv)));
}

void emitConvergenceLog(int round,
                        double rmaeCpressAll,
                        double rmaeCpressNew,
                        double rmaeSigmaAll,
                        double rmaeSigmaNew,
                        double rmaeUmaxAll,
                        const QString& stopReason,
                        bool willStop,
                        const std::function<void(const QString&)>& logFn)
{
	if (!logFn)
		return;
	auto pct = [](double v) -> QString {
		if (v < 0.0)
			return QStringLiteral("NA");
		return QStringLiteral("%1%").arg(v * 100.0, 0, 'f', 2);
	};
	QString reason = QStringLiteral("continue");
	if (willStop) {
		if (stopReason == QStringLiteral("user_stop"))
			reason = QStringLiteral("stop_requested");
		else if (stopReason == QStringLiteral("rmae_threshold"))
			reason = QStringLiteral("rmae_converged");
		else if (stopReason == QStringLiteral("max_generations"))
			reason = QStringLiteral("max_round");
		else
			reason = stopReason;
	}
	logFn(QStringLiteral("[GearOpt][Convergence]"));
	logFn(QStringLiteral("round=%1").arg(round));
	logFn(QStringLiteral("rmae_cpress_all=%1").arg(pct(rmaeCpressAll)));
	logFn(QStringLiteral("rmae_cpress_new=%1").arg(pct(rmaeCpressNew)));
	logFn(QStringLiteral("rmae_sigma_all=%1").arg(pct(rmaeSigmaAll)));
	logFn(QStringLiteral("rmae_sigma_new=%1").arg(pct(rmaeSigmaNew)));
	logFn(QStringLiteral("rmae_umax_all=%1").arg(pct(rmaeUmaxAll)));
	logFn(QStringLiteral("rmae_umax_new=%1").arg(QStringLiteral("NA")));
	logFn(QStringLiteral("stop_threshold=0.05"));
	logFn(QStringLiteral("stop_reason=%1").arg(reason));
}

void emitFinalSummaryLog(const GearOptConfig& cfg,
                         const QString& runId,
                         const QString& runDir,
                         bool finished,
                         const QString& stopReason,
                         int totalValidatedSamples,
                         int newCcxSamples,
                         int cacheReused,
                         int failedSamples,
                         int finalParetoSize,
                         const QVector<SurrogateSample>& samples,
                         const std::function<void(const QString&)>& logFn)
{
	if (!surrogateVerboseLogEnabled(cfg) || !logFn)
		return;

	const SurrogateSample* bestCpress = nullptr;
	const SurrogateSample* bestEdge   = nullptr;
	const SurrogateSample* bestSigma  = nullptr;
	for (const SurrogateSample& s : samples) {
		if (s.cpressMax > 0.0 && (!bestCpress || s.cpressMax < bestCpress->cpressMax))
			bestCpress = &s;
		if (s.edgeLoadRatio > 0.0 && (!bestEdge || s.edgeLoadRatio < bestEdge->edgeLoadRatio))
			bestEdge = &s;
		if (s.sigmaMax > 0.0 && (!bestSigma || s.sigmaMax < bestSigma->sigmaMax))
			bestSigma = &s;
	}

	logFn(QStringLiteral("[GearOpt][FinalSummary]"));
	logFn(QStringLiteral("run_id=%1").arg(runId));
	logFn(QStringLiteral("finished=%1").arg(finished ? QStringLiteral("true") : QStringLiteral("false")));
	logFn(QStringLiteral("stop_reason=%1").arg(stopReason.isEmpty() ? QStringLiteral("unknown")
	                                                                : stopReason));
	logFn(QStringLiteral("total_validated_samples=%1").arg(totalValidatedSamples));
	logFn(QStringLiteral("new_ccx_samples=%1").arg(newCcxSamples));
	logFn(QStringLiteral("cache_reused=%1").arg(cacheReused));
	logFn(QStringLiteral("failed_samples=%1").arg(failedSamples));
	logFn(QStringLiteral("final_true_pareto_size=%1").arg(finalParetoSize));
	logFn(QStringLiteral("best_true_cpress=%1")
	          .arg(bestCpress ? naOrNum(bestCpress->cpressMax) : QStringLiteral("NA")));
	logFn(QStringLiteral("best_true_edge=%1")
	          .arg(bestEdge ? naOrNum(bestEdge->edgeLoadRatio) : QStringLiteral("NA")));
	logFn(QStringLiteral("best_true_sigma=%1")
	          .arg(bestSigma ? naOrNum(bestSigma->sigmaMax) : QStringLiteral("NA")));
	logFn(QStringLiteral("best_design:"));
	if (bestCpress && bestCpress->x.size() >= 7) {
		logFn(QStringLiteral("x1=%1").arg(bestCpress->x[0], 0, 'g', 8));
		logFn(QStringLiteral("x2=%1").arg(bestCpress->x[1], 0, 'g', 8));
		logFn(QStringLiteral("ca1=%1").arg(bestCpress->x[2], 0, 'g', 8));
		logFn(QStringLiteral("lca1=%1").arg(bestCpress->x[3], 0, 'g', 8));
		logFn(QStringLiteral("ca2=%1").arg(bestCpress->x[4], 0, 'g', 8));
		logFn(QStringLiteral("lca2=%1").arg(bestCpress->x[5], 0, 'g', 8));
		logFn(QStringLiteral("hubRatio=%1").arg(bestCpress->x[6], 0, 'g', 8));
	} else {
		logFn(QStringLiteral("x1=NA"));
	}
	logFn(QStringLiteral("csv_outputs:"));
	logFn(QStringLiteral("surrogate_metrics.csv=%1").arg(csvPath(runDir, QStringLiteral("surrogate_metrics.csv"))));
	logFn(QStringLiteral("ccx_metrics.csv=%1").arg(csvPath(runDir, QStringLiteral("ccx_metrics.csv"))));
	logFn(QStringLiteral("surrogate_rmae.csv=%1").arg(csvPath(runDir, QStringLiteral("surrogate_rmae.csv"))));
	logFn(QStringLiteral("pred_vs_true.csv=%1").arg(csvPath(runDir, QStringLiteral("pred_vs_true.csv"))));
	logFn(QStringLiteral("optimization_compare.csv=%1")
	          .arg(csvPath(runDir, QStringLiteral("result/optimization_compare.csv"))));
}

double computeMinDistNormToSamples(const GearDesignPoint& dp,
                                   const QVector<SurrogateSample>& samples,
                                   const GearOptConfig& cfg)
{
	const QVector<double> lo = inputBoundsLower(cfg);
	const QVector<double> hi = inputBoundsUpper(cfg);
	const QVector<double> xNorm =
	    normalizeByBounds(surrogateInputVars(dp), lo, hi);
	double best = 1.0;
	for (const SurrogateSample& s : samples) {
		if (s.x.isEmpty())
			continue;
		const QVector<double> exNorm = normalizeByBounds(s.x, lo, hi);
		best = std::min(best, normalizedDist(xNorm, exNorm));
	}
	return best;
}

} // namespace GearAutoOpt
