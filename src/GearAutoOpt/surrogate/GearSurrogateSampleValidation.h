#ifndef _GEARAUTOOPT_GEAR_SURROGATE_SAMPLE_VALIDATION_H_
#define _GEARAUTOOPT_GEAR_SURROGATE_SAMPLE_VALIDATION_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/surrogate/GearSurrogateModel.h"

#include <QString>
#include <QStringList>

namespace GearAutoOpt {

struct GearOptConfig;

inline constexpr const char kSurrogateMetricCpressMax[]    = "cpressMax_MPa";
inline constexpr const char kSurrogateMetricEdgeLoadRatio[] = "edgeLoadRatio";
inline constexpr const char kSurrogateMetricSigmaMax[]     = "sigmaMax_MPa";
inline constexpr const char kSurrogateMetricUMax[]         = "uMax_mm";
inline constexpr const char kSurrogateMetricMass[]         = "mass_kg";
inline constexpr const char kSurrogateMetricCpressCV[]     = "cpressCV";

GEARAUTOOPTAPI QStringList defaultSurrogateTargets();
GEARAUTOOPTAPI QStringList effectiveSurrogateTargets(const GearOptConfig& cfg);

GEARAUTOOPTAPI double surrogateMetricValue(const GearDesignPoint& dp, const QString& metricName);
GEARAUTOOPTAPI double surrogateMetricValue(const SurrogateSample& s, const QString& metricName);

/// 单个指标是否满足代理目标有效性（仅用于 cfg.surrogateTargets 中的项）。
GEARAUTOOPTAPI bool surrogateTargetMetricValid(const GearDesignPoint& dp, const QString& metricName);
GEARAUTOOPTAPI bool surrogateTargetMetricValid(const SurrogateSample& s, const QString& metricName);

/// CCX 已完成且当前代理目标均已提取（不要求 status=done）。
GEARAUTOOPTAPI bool surrogateTargetsExtracted(const GearDesignPoint& dp,
                                              const QStringList& surrogateTargets);

/// RBF 训练样本有效：status=done 且所有 surrogateTargets 指标有效。
GEARAUTOOPTAPI bool surrogateTrainingValid(const GearDesignPoint& dp,
                                           const QStringList& surrogateTargets);
GEARAUTOOPTAPI bool surrogateTrainingValid(const SurrogateSample& s,
                                           const QStringList& surrogateTargets);

/// 论文统计 / 完整结果导出：六项指标全部有效。
GEARAUTOOPTAPI bool fullMetricsValid(const GearDesignPoint& dp);
GEARAUTOOPTAPI bool fullMetricsValid(const SurrogateSample& s);

GEARAUTOOPTAPI QString surrogateTargetMetricSqlClause(const QString& metricName);
GEARAUTOOPTAPI QString validatedSurrogateSampleWhereSql(const QStringList& surrogateTargets);

struct SurrogateResultFlags {
	int verifiedByCcx{0};
	int converged{0};
	int isValid{0};
	int fullMetricsValid{0};
};

GEARAUTOOPTAPI SurrogateResultFlags computeSurrogateResultFlags(const GearDesignPoint& dp,
                                                                const QStringList& surrogateTargets);

} // namespace GearAutoOpt

#endif
