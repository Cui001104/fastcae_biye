#ifndef _GEARAUTOOPT_GEAR_INFILL_SELECTOR_H_
#define _GEARAUTOOPT_GEAR_INFILL_SELECTOR_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/opt/NSGA2.h"
#include "GearAutoOpt/surrogate/GearSurrogateModel.h"

#include <QSet>
#include <functional>

namespace GearAutoOpt {

struct GEARAUTOOPTAPI InfillFilterStats {
	int candidateTotal         = 0;
	int validAfterRelief       = 0;
	int skipFailed             = 0;
	int skipKnownCase          = 0;
	int skipKnownDesign        = 0;
	int skipNearDuplicate      = 0;
	int skipTooCloseToSelected = 0;
	int selected               = 0;
};

struct GEARAUTOOPTAPI InfillSelectionMeta {
	double minDistNorm        = -1.0;
	double localResidualNorm  = -1.0;
	double score              = -1.0;
};

struct GEARAUTOOPTAPI InfillScoringConfig {
	double alpha = 0.6;
	double beta = 0.4;
	int    neighborK = 3;
	/// 归一化设计空间最小距离；低于此值视为重复 infill 并跳过。
	double minDesignDistNorm = 0.03;
	/// 可选：将 skip/selected 日志转发到 UI（未设置时写 qDebug）。
	std::function<void(const QString&)> logFn;
	/// 可选：填充过滤统计（仅日志，不改变选点结果）。
	InfillFilterStats* outFilterStats = nullptr;
	/// 可选：与选中个体一一对应的打分元数据（仅日志）。
	QVector<InfillSelectionMeta>* outSelectionMeta = nullptr;
};

/// 全局探索补点：LHS 候选 + Maximin Distance（不看代理目标/残差）。
struct GEARAUTOOPTAPI GlobalExplorationConfig {
	int    explorationCandidateCount = 2000;
	double minDesignDistNorm         = 0.03;
	std::function<void(const QString&)> logFn;
	InfillFilterStats* outFilterStats = nullptr;
	QVector<InfillSelectionMeta>* outSelectionMeta = nullptr;
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
	                                           const QSet<QString>& failedCaseHashes,
	                                           int k);

	/// 全局探索补点：LHS 候选中按到已有 CCX 样本的最大最小归一化距离选 k 个。
	/// excludeNearPoints：本轮已选 exploitation 点（过滤 near-duplicate）。
	static Population selectGlobalExplorationPoints(
	    const QVector<SurrogateSample>& existingSamples,
	    const GlobalExplorationConfig& exploreCfg,
	    const GearOptConfig& cfg,
	    const GearDesignPoint& basePoint,
	    double fixedMeshSizeMm,
	    bool runMeshAuto,
	    double fixedWidthMm,
	    const QSet<QString>& existingCaseHashes,
	    const QSet<QString>& existingDesignHashes,
	    const QSet<QString>& failedCaseHashes,
	    const Population& excludeNearPoints,
	    int k,
	    int seed);

	/// infill 日志：7 维代理设计变量。
	static QString formatDesignVarsForLog(const GearDesignPoint& dp);
};

} // namespace GearAutoOpt

#endif
