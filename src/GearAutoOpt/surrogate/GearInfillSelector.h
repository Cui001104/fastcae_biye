#ifndef _GEARAUTOOPT_GEAR_INFILL_SELECTOR_H_
#define _GEARAUTOOPT_GEAR_INFILL_SELECTOR_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/opt/NSGA2.h"
#include "GearAutoOpt/surrogate/GearSurrogateModel.h"

#include <QSet>

namespace GearAutoOpt {

struct GEARAUTOOPTAPI InfillScoringConfig {
	double alpha = 0.6;
	double beta = 0.4;
	int    neighborK = 3;
	/// 归一化设计空间最小距离；低于此值视为重复 infill 并跳过。
	double minDesignDistNorm = 1e-4;
};

class GEARAUTOOPTAPI GearInfillSelector {
public:
	static Population selectSparseParetoPoints(const Population& surrogatePareto,
	                                           const QVector<SurrogateSample>& existingSamples,
	                                           int k);

	/// 在代理 Pareto 前沿上按 maximin 距离选 k 个 infill 点；排除已有 case_hash 与本轮重复。
	static Population selectSparseParetoPoints(const Population& surrogatePareto,
	                                           const QVector<SurrogateSample>& existingSamples,
	                                           const GearOptConfig& cfg,
	                                           const GearDesignPoint& basePoint,
	                                           double fixedMeshSizeMm,
	                                           bool runMeshAuto,
	                                           const QSet<QString>& existingCaseHashes,
	                                           int k);

	/// Residual-aware infill: score = alpha * minDistNorm + beta * localResidualNorm.
	/// sampleResiduals is aligned with existingSamples and is kept in memory only.
	static Population selectSparseParetoPoints(const Population& surrogatePareto,
	                                           const QVector<SurrogateSample>& existingSamples,
	                                           const QVector<double>& sampleResiduals,
	                                           const InfillScoringConfig& scoring,
	                                           const GearOptConfig& cfg,
	                                           const GearDesignPoint& basePoint,
	                                           double fixedMeshSizeMm,
	                                           bool runMeshAuto,
	                                           const QSet<QString>& existingCaseHashes,
	                                           const QSet<QString>& existingDesignHashes,
	                                           int k);

	/// infill 日志：7 维代理设计变量。
	static QString formatDesignVarsForLog(const GearDesignPoint& dp);
};

} // namespace GearAutoOpt

#endif
