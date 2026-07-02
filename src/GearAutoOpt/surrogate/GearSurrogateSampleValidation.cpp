#include "GearSurrogateSampleValidation.h"

#include "GearAutoOpt/data/GearOptConfig.h"

#include <cmath>

namespace GearAutoOpt {

QStringList defaultSurrogateTargets();

namespace {

bool finitePositive(double v)
{
	return v > 0.0 && std::isfinite(v);
}

bool finiteNonNegative(double v)
{
	return v >= 0.0 && std::isfinite(v);
}

bool metricValidValue(double v, const QString& metricName)
{
	if (metricName == QLatin1String(kSurrogateMetricCpressMax)
	    || metricName == QLatin1String(kSurrogateMetricEdgeLoadRatio)
	    || metricName == QLatin1String(kSurrogateMetricSigmaMax)
	    || metricName == QLatin1String(kSurrogateMetricMass)
	    || metricName == QLatin1String(kSurrogateMetricCpressCV))
		return finitePositive(v);
	if (metricName == QLatin1String(kSurrogateMetricUMax))
		return finiteNonNegative(v);
	return false;
}

QStringList resolvedSurrogateTargets(const QStringList& surrogateTargets)
{
	return surrogateTargets.isEmpty() ? defaultSurrogateTargets() : surrogateTargets;
}

} // namespace

QStringList defaultSurrogateTargets()
{
	return {QString::fromLatin1(kSurrogateMetricCpressMax),
	        QString::fromLatin1(kSurrogateMetricEdgeLoadRatio)};
}

QStringList effectiveSurrogateTargets(const GearOptConfig& cfg)
{
	return cfg.effectiveSurrogateTargets();
}

double surrogateMetricValue(const GearDesignPoint& dp, const QString& metricName)
{
	if (metricName == QLatin1String(kSurrogateMetricCpressMax))
		return dp.cpressMax_MPa;
	if (metricName == QLatin1String(kSurrogateMetricEdgeLoadRatio))
		return dp.edgeLoadRatio;
	if (metricName == QLatin1String(kSurrogateMetricSigmaMax))
		return dp.sigmaMax;
	if (metricName == QLatin1String(kSurrogateMetricUMax))
		return dp.uMax;
	if (metricName == QLatin1String(kSurrogateMetricMass))
		return dp.mass;
	if (metricName == QLatin1String(kSurrogateMetricCpressCV))
		return dp.cpressCV;
	return -1.0;
}

double surrogateMetricValue(const SurrogateSample& s, const QString& metricName)
{
	if (metricName == QLatin1String(kSurrogateMetricCpressMax))
		return s.cpressMax;
	if (metricName == QLatin1String(kSurrogateMetricEdgeLoadRatio))
		return s.edgeLoadRatio;
	if (metricName == QLatin1String(kSurrogateMetricSigmaMax))
		return s.sigmaMax;
	if (metricName == QLatin1String(kSurrogateMetricUMax))
		return s.uMax;
	if (metricName == QLatin1String(kSurrogateMetricMass))
		return s.mass;
	if (metricName == QLatin1String(kSurrogateMetricCpressCV))
		return s.cpressCV;
	return -1.0;
}

bool surrogateTargetMetricValid(const GearDesignPoint& dp, const QString& metricName)
{
	return metricValidValue(surrogateMetricValue(dp, metricName), metricName);
}

bool surrogateTargetMetricValid(const SurrogateSample& s, const QString& metricName)
{
	return metricValidValue(surrogateMetricValue(s, metricName), metricName);
}

bool surrogateTargetsExtracted(const GearDesignPoint& dp, const QStringList& surrogateTargets)
{
	const QStringList targets = resolvedSurrogateTargets(surrogateTargets);
	for (const QString& metric : targets) {
		if (!surrogateTargetMetricValid(dp, metric))
			return false;
	}
	return !targets.isEmpty();
}

bool surrogateTrainingValid(const GearDesignPoint& dp, const QStringList& surrogateTargets)
{
	if (dp.status != PointStatus::Done)
		return false;
	return surrogateTargetsExtracted(dp, surrogateTargets);
}

bool surrogateTrainingValid(const SurrogateSample& s, const QStringList& surrogateTargets)
{
	const QStringList targets = resolvedSurrogateTargets(surrogateTargets);
	for (const QString& metric : targets) {
		if (!surrogateTargetMetricValid(s, metric))
			return false;
	}
	return !targets.isEmpty();
}

bool fullMetricsValid(const GearDesignPoint& dp)
{
	return dp.status == PointStatus::Done
	       && finitePositive(dp.cpressMax_MPa)
	       && finitePositive(dp.sigmaMax)
	       && finiteNonNegative(dp.uMax)
	       && finitePositive(dp.mass)
	       && finitePositive(dp.edgeLoadRatio)
	       && finitePositive(dp.cpressCV);
}

bool fullMetricsValid(const SurrogateSample& s)
{
	return finitePositive(s.cpressMax)
	       && finitePositive(s.sigmaMax)
	       && finiteNonNegative(s.uMax)
	       && finitePositive(s.mass)
	       && finitePositive(s.edgeLoadRatio)
	       && finitePositive(s.cpressCV);
}

QString surrogateTargetMetricSqlClause(const QString& metricName)
{
	if (metricName == QLatin1String(kSurrogateMetricCpressMax))
		return QStringLiteral(" cpressMax_MPa > 0");
	if (metricName == QLatin1String(kSurrogateMetricEdgeLoadRatio))
		return QStringLiteral(" edgeLoadRatio IS NOT NULL AND edgeLoadRatio > 0");
	if (metricName == QLatin1String(kSurrogateMetricSigmaMax))
		return QStringLiteral(" sigmaMax_MPa > 0");
	if (metricName == QLatin1String(kSurrogateMetricUMax))
		return QStringLiteral(" uMax_mm >= 0");
	if (metricName == QLatin1String(kSurrogateMetricMass))
		return QStringLiteral(" mass_kg > 0");
	if (metricName == QLatin1String(kSurrogateMetricCpressCV))
		return QStringLiteral(" cpressCV IS NOT NULL AND cpressCV > 0");
	return QString();
}

QString validatedSurrogateSampleWhereSql(const QStringList& surrogateTargets)
{
	const QStringList targets = resolvedSurrogateTargets(surrogateTargets);
	QString sql = QStringLiteral(
	    " status = 'done'"
	    " AND verified_by_ccx = 1"
	    " AND is_valid = 1");
	for (const QString& metric : targets) {
		const QString clause = surrogateTargetMetricSqlClause(metric);
		if (!clause.isEmpty())
			sql += QStringLiteral(" AND") + clause;
	}
	return sql;
}

SurrogateResultFlags computeSurrogateResultFlags(const GearDesignPoint& dp,
                                                 const QStringList& surrogateTargets)
{
	SurrogateResultFlags flags;
	flags.fullMetricsValid = fullMetricsValid(dp) ? 1 : 0;
	if (dp.status == PointStatus::Failed || dp.status == PointStatus::Invalid
	    || dp.status == PointStatus::Infeasible) {
		return flags;
	}
	const bool extracted = surrogateTargetsExtracted(dp, surrogateTargets);
	flags.verifiedByCcx = (dp.status == PointStatus::Done && extracted) ? 1 : 0;
	flags.converged     = flags.verifiedByCcx;
	flags.isValid       = surrogateTrainingValid(dp, surrogateTargets) ? 1 : 0;
	return flags;
}

} // namespace GearAutoOpt
