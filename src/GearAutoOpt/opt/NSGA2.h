// UTF-8 BOM
#ifndef _GEARAUTOOPT_NSGA2_H_
#define _GEARAUTOOPT_NSGA2_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearOptConfig.h"

#include <QVector>
#include <utility>
#include <random>

namespace GearAutoOpt {

// ================================================================
//   Individual / Population 数据结构
// ================================================================

/// 变量索引常量（与 GearDesignPoint 字段一一对应，顺序固定）
enum VarIdx {
    VAR_MODULE   = 0,
    VAR_Z1       = 1,
    VAR_Z2       = 2,
    VAR_ALPHA    = 3,
    VAR_X1       = 4,
    VAR_X2       = 5,
    VAR_CA1      = 6,
    VAR_LCA1     = 7,
    VAR_CA2      = 8,
    VAR_LCA2     = 9,
    VAR_WIDTH    = 10,
    VAR_HUBRATIO = 11,
    VAR_COUNT    = 12
};

/// NSGA-II 个体。
struct GEARAUTOOPTAPI Individual {
    QVector<double> vars;               ///< VAR_COUNT 个设计变量
    QVector<double> objs;               ///< 目标值（sigmaMax, mass[, ratioErr]）
    double   constraintViolation = 0.0; ///< 总约束违反量（可行个体 = 0）
    int      rank               = 0;   ///< 非支配层（1 = Pareto 最优）
    double   crowdingDist       = 0.0;
    bool     evaluated          = false;

    /// 转为 GearDesignPoint（vars → dp 字段）
    GearDesignPoint toDesignPoint(int id = -1, int generation = 0) const;
    /// 从 GearDesignPoint 反向构造（dp 字段 → vars）
    static Individual fromDesignPoint(const GearDesignPoint& dp, const GearOptConfig& cfg);
};

using Population = QVector<Individual>;

/// 返回 cfg 中 12 个设计变量的 Bounds 列表（顺序 = VarIdx 顺序）
GEARAUTOOPTAPI QVector<Bounds> boundsVector(const GearOptConfig& cfg);

// ================================================================
//  拉丁超立方初始采样
// ================================================================

GEARAUTOOPTAPI Population initLatin(int N, const GearOptConfig& cfg, int seed = 42);

// ================================================================
//   快速非支配排序
// ================================================================

/// 对种群排序并写回每个 ind.rank；返回每层前沿的个体下标。
GEARAUTOOPTAPI QVector<QVector<int>> fastNonDominatedSort(Population& pop);

// ================================================================
//   拥挤度计算
// ================================================================

/// 对 front（pop 下标列表）中的个体计算拥挤度，写回 ind.crowdingDist。
GEARAUTOOPTAPI void assignCrowdingDistance(Population& pop, const QVector<int>& front);

// ================================================================
//   SBX 交叉 + 多项式变异
// ================================================================

GEARAUTOOPTAPI std::pair<Individual, Individual>
sbxCrossover(const Individual& p1, const Individual& p2,
             const GearOptConfig& cfg, std::mt19937& rng);

GEARAUTOOPTAPI void
polyMutate(Individual& ind, const GearOptConfig& cfg, std::mt19937& rng,
           double mutProb = -1.0);  ///< mutProb < 0 → 1/n

// ================================================================
//   Deb 约束支配
// ================================================================

/// a Deb-dominates b：可行优先 → 违反量小优先 → 正常 Pareto 支配
GEARAUTOOPTAPI bool debDominates(const Individual& a, const Individual& b);

// ================================================================
//   约束修复
// ================================================================

/// 修复明显不可行的几何约束；顺便更新 constraintViolation。
GEARAUTOOPTAPI void repairAndEval(Individual& ind, const GearOptConfig& cfg);

// ================================================================
//   NSGA-II 主循环
// ================================================================

/// 从父代生成等量子代（未评估）。
GEARAUTOOPTAPI Population evolve(const Population& parents,
                                  const GearOptConfig& cfg, std::mt19937& rng);

/// 合并父代与子代后，按非支配排序 + 拥挤度截取 N 个（精英选择）。
GEARAUTOOPTAPI Population selectNextGen(Population combined, int N);

// ================================================================
//   Pareto 归档 + 超体积收敛判据
// ================================================================

/// 从已评估种群中提取 Pareto 最优个体集合（rank == 1）。
GEARAUTOOPTAPI Population extractPareto(const Population& pop);

/// 两目标超体积（reference point = [ref0, ref1]）。
/// 简单排序法 O(N log N)，仅支持双目标。
GEARAUTOOPTAPI double hypervolume2D(const Population& pareto,
                                     double ref0, double ref1);

/// 检查超体积历史是否收敛（连续 windowSize 代变化 < tol）。
GEARAUTOOPTAPI bool checkConverged(const QVector<double>& hvHistory,
                                    double tol = 1e-3, int windowSize = 3);

} // namespace GearAutoOpt
#endif
