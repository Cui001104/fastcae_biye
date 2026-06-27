// UTF-8 BOM
#include "NSGA2.h"

#include "GearAutoOpt/data/GearOptGeometryBridge.h"

#include <QDebug>
#include <QMap>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

namespace GearAutoOpt {

// ================================================================
// 辅助函数
// ================================================================

static double closestDiscrete(double val, const QVector<double>& dv) {
    if (dv.isEmpty()) return val;
    double best = dv[0];
    double bestDist = std::abs(val - best);
    for (int i = 1; i < dv.size(); ++i) {
        double d = std::abs(val - dv[i]);
        if (d < bestDist) { bestDist = d; best = dv[i]; }
    }
    return best;
}

static double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static double snapBounds(double val, const Bounds& b) {
    if (!b.discreteValues.isEmpty())
        return closestDiscrete(val, b.discreteValues);
    double v = clamp(val, b.lower, b.upper);
    if (b.isInteger) v = std::round(v);
    return v;
}

static QString fmtGearOptDpFields(const GearDesignPoint& dp)
{
    return QStringLiteral("module=%1, z1=%2, z2=%3, alpha=%4, x1=%5, x2=%6, ca1=%7, lca1=%8, ca2=%9, lca2=%10, commonWidth=%11, hubRatio=%12")
        .arg(dp.module, 0, 'g', 8)
        .arg(dp.z1)
        .arg(dp.z2)
        .arg(dp.alpha, 0, 'g', 8)
        .arg(dp.x1, 0, 'g', 8)
        .arg(dp.x2, 0, 'g', 8)
        .arg(dp.ca1, 0, 'g', 8)
        .arg(dp.lca1, 0, 'g', 8)
        .arg(dp.ca2, 0, 'g', 8)
        .arg(dp.lca2, 0, 'g', 8)
        .arg(dp.commonWidth, 0, 'g', 8)
        .arg(dp.hubRatio, 0, 'g', 8);
}

// ================================================================
//  Individual 转换
// ================================================================

QVector<Bounds> boundsVector(const GearOptConfig& cfg) {
    return {
        cfg.moduleBound,
        cfg.z1Bound,
        cfg.z2Bound,
        cfg.alphaBound,
        cfg.x1Bound,
        cfg.x2Bound,
        cfg.ca1Bound,
        cfg.lca1Bound,
        cfg.ca2Bound,
        cfg.lca2Bound,
        cfg.widthBound,
        cfg.hubRatioBound,
    };
}

GearDesignPoint Individual::toDesignPoint(int id, int generation) const {
    GearDesignPoint dp;
    dp.id         = id;
    dp.generation = generation;
    if (vars.size() >= VAR_COUNT) {
        dp.module   = vars[VAR_MODULE];
        dp.z1       = qRound(vars[VAR_Z1]);
        dp.z2       = qRound(vars[VAR_Z2]);
        dp.alpha    = vars[VAR_ALPHA];
        dp.x1       = vars[VAR_X1];
        dp.x2       = vars[VAR_X2];
        dp.ca1      = vars[VAR_CA1];
        dp.lca1     = vars[VAR_LCA1];
        dp.ca2      = vars[VAR_CA2];
        dp.lca2     = vars[VAR_LCA2];
        dp.commonWidth = vars[VAR_COMMON_WIDTH];
        dp.hubRatio    = vars[VAR_HUBRATIO];
    }
    dp.syncPairGearWidth();
    return dp;
}

Individual Individual::fromDesignPoint(const GearDesignPoint& dp, const GearOptConfig&) {
    Individual ind;
    ind.vars.resize(VAR_COUNT);
    ind.vars[VAR_MODULE]   = dp.module;
    ind.vars[VAR_Z1]       = dp.z1;
    ind.vars[VAR_Z2]       = dp.z2;
    ind.vars[VAR_ALPHA]    = dp.alpha;
    ind.vars[VAR_X1]       = dp.x1;
    ind.vars[VAR_X2]       = dp.x2;
    ind.vars[VAR_CA1]      = dp.ca1;
    ind.vars[VAR_LCA1]     = dp.lca1;
    ind.vars[VAR_CA2]      = dp.ca2;
    ind.vars[VAR_LCA2]     = dp.lca2;
    ind.vars[VAR_COMMON_WIDTH] = dp.commonWidth;
    ind.vars[VAR_HUBRATIO]     = dp.hubRatio;

    if (dp.cpressMax_MPa > 0.0 && dp.edgeLoadRatio > 0.0) {
        ind.objs.append(dp.cpressMax_MPa);
        ind.objs.append(dp.edgeLoadRatio);
        ind.evaluated = true;
    }
    return ind;
}

// ================================================================
// 修复 + 约束评估（先于采样使用，所以放在前面）
// ================================================================

void repairAndEval(Individual& ind, const GearOptConfig& cfg) {
    if (ind.vars.size() < VAR_COUNT) ind.vars.resize(VAR_COUNT);
    const QVector<Bounds> bv = boundsVector(cfg);

    // 1. Snap 所有变量到合法范围（离散/整数/clip）
    for (int i = 0; i < VAR_COUNT; ++i)
        ind.vars[i] = snapBounds(ind.vars[i], bv[i]);

    // 2. 求几何可行性（toDesignPoint → isFeasible）
    GearDesignPoint dp = ind.toDesignPoint();
    QString msg;
    const bool feasible = dp.isFeasible(&msg);

    if (!feasible) {
        // 最小修复：若 x1 导致根切，尝试提升 x1 到下根切极限
        // x_min = (17 - z) / 17 (近似，Maag 公式)
        const double xMin1 = (17.0 - dp.z1) / 17.0;
        const double xMin2 = (17.0 - dp.z2) / 17.0;
        if (ind.vars[VAR_X1] < xMin1)
            ind.vars[VAR_X1] = snapBounds(xMin1, bv[VAR_X1]);
        if (ind.vars[VAR_X2] < xMin2)
            ind.vars[VAR_X2] = snapBounds(xMin2, bv[VAR_X2]);

        // 重新 clip
        for (int i = 0; i < VAR_COUNT; ++i)
            ind.vars[i] = snapBounds(ind.vars[i], bv[i]);

        dp = ind.toDesignPoint();
        dp.isFeasible(&msg);
    }

    // 3. 约束违反量 = 软违反加总（不可行但尽量量化）
    double cv = 0.0;
    if (!dp.isFeasible()) {
        // 简单惩罚：齿顶厚 / 模数（< minSaOverM 违反）
        const double haCoeff = 1.0 + dp.x1;
        const double sa = 2.0 * dp.module * (haCoeff - std::tan(dp.alpha * M_PI / 180.0) * (haCoeff - dp.x1));
        const double saOverM = sa / dp.module;
        if (saOverM < cfg.constraints.minSaOverM)
            cv += cfg.constraints.minSaOverM - saOverM;
        // 根切（使用 x_min 估计）
        const double xMin = (17.0 - dp.z1) / 17.0;
        if (dp.x1 < xMin) cv += xMin - dp.x1;
        if (cv == 0.0) cv = 0.001; // 有不可行但未识别，给一个非零惩罚
    }
    ind.constraintViolation = cv;
}

// ================================================================
//   拉丁超立方初始采样
// ================================================================

Population initLatin(int N, const GearOptConfig& cfg, int seed) {
    std::mt19937 rng(static_cast<unsigned>(seed));
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    const QVector<Bounds> bv = boundsVector(cfg);
    Population pop(N);
    for (auto& ind : pop) ind.vars.resize(VAR_COUNT);

    for (int vi = 0; vi < VAR_COUNT; ++vi) {
        // 生成 [0,N) 的随机排列
        QVector<int> order(N);
        std::iota(order.begin(), order.end(), 0);
        std::shuffle(order.begin(), order.end(), rng);

        const Bounds& b = bv[vi];
        const double lo = b.lower, hi = b.upper;

        for (int j = 0; j < N; ++j) {
            double t = (order[j] + uni(rng)) / N; // in [0, 1)
            double val = lo + t * (hi - lo);
            val = snapBounds(val, b);
            pop[j].vars[vi] = val;
        }
    }

    // 修复所有个体
    for (auto& ind : pop)
        repairAndEval(ind, cfg);

    if (cfg.useOptimizationBase) {
        pop[0] = Individual::fromDesignPoint(cfg.optimizationBase, cfg);
        repairAndEval(pop[0], cfg);
    }

    for (int j = 0; j < N; ++j) {
        const GearDesignPoint dp = pop[j].toDesignPoint(j, 0);
        checkGearOptDesignPointBounds(dp, cfg);
        const char* tag = (cfg.useOptimizationBase && j == 0) ? "BASE" : "SAMPLE";
        qDebug().noquote() << QStringLiteral("[GearOpt][InitPop] id=%1 %2 %3")
                                  .arg(j)
                                  .arg(QLatin1String(tag))
                                  .arg(fmtGearOptDpFields(dp));
    }

    return pop;
}

// ================================================================
//   快速非支配排序
// ================================================================

QVector<QVector<int>> fastNonDominatedSort(Population& pop) {
    const int n = pop.size();
    QVector<QVector<int>> S(n);  // S[i] = i 支配的个体下标集合
    QVector<int> np(n, 0);       // np[i] = 支配 i 的个体数
    QVector<QVector<int>> fronts;

    QVector<int> front0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            if (debDominates(pop[i], pop[j])) {
                S[i].append(j);
            } else if (debDominates(pop[j], pop[i])) {
                np[i]++;
            }
        }
        if (np[i] == 0) {
            pop[i].rank = 1;
            front0.append(i);
        }
    }
    fronts.append(front0);

    int layer = 0;
    while (!fronts[layer].isEmpty()) {
        QVector<int> nextFront;
        for (int i : fronts[layer]) {
            for (int j : S[i]) {
                np[j]--;
                if (np[j] == 0) {
                    pop[j].rank = layer + 2;
                    nextFront.append(j);
                }
            }
        }
        fronts.append(nextFront);
        layer++;
    }
    // 最后一层是空的，去掉
    if (!fronts.isEmpty() && fronts.last().isEmpty())
        fronts.removeLast();

    return fronts;
}

// ================================================================
//   拥挤度计算
// ================================================================

void assignCrowdingDistance(Population& pop, const QVector<int>& front) {
    const int sz = front.size();
    if (sz == 0) return;
    for (int idx : front) pop[idx].crowdingDist = 0.0;
    if (sz <= 2) {
        for (int idx : front) pop[idx].crowdingDist = std::numeric_limits<double>::infinity();
        return;
    }

    const int nObj = pop[front[0]].objs.size();
    for (int m = 0; m < nObj; ++m) {
        // 按目标 m 排序
        QVector<int> sorted = front;
        std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
            return pop[a].objs[m] < pop[b].objs[m];
        });
        pop[sorted.first()].crowdingDist  = std::numeric_limits<double>::infinity();
        pop[sorted.last()].crowdingDist   = std::numeric_limits<double>::infinity();
        const double range = pop[sorted.last()].objs[m] - pop[sorted.first()].objs[m];
        if (range < 1e-12) continue;
        for (int i = 1; i < sz - 1; ++i) {
            pop[sorted[i]].crowdingDist +=
                (pop[sorted[i+1]].objs[m] - pop[sorted[i-1]].objs[m]) / range;
        }
    }
}

// ================================================================
//   SBX 交叉 + 多项式变异
// ================================================================

std::pair<Individual, Individual>
sbxCrossover(const Individual& p1, const Individual& p2,
             const GearOptConfig& cfg, std::mt19937& rng) {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    const double eta = cfg.nsga2.sbxEta;
    const QVector<Bounds> bv = boundsVector(cfg);

    Individual c1 = p1, c2 = p2;
    for (int i = 0; i < VAR_COUNT; ++i) {
        if (uni(rng) > cfg.nsga2.crossoverRate) continue;
        double x1 = p1.vars[i], x2 = p2.vars[i];
        if (std::abs(x1 - x2) < 1e-12) continue;
        if (x1 > x2) std::swap(x1, x2);

        const Bounds& b = bv[i];
        const double lo = b.lower, hi = b.upper;
        const double u = uni(rng);
        double beta;
        if (u <= 0.5) {
            beta = std::pow(2.0 * u, 1.0 / (eta + 1.0));
        } else {
            beta = std::pow(1.0 / (2.0 * (1.0 - u)), 1.0 / (eta + 1.0));
        }
        c1.vars[i] = 0.5 * ((1.0 + beta) * x1 + (1.0 - beta) * x2);
        c2.vars[i] = 0.5 * ((1.0 - beta) * x1 + (1.0 + beta) * x2);
        c1.vars[i] = snapBounds(c1.vars[i], b);
        c2.vars[i] = snapBounds(c2.vars[i], b);
        Q_UNUSED(lo); Q_UNUSED(hi);
    }
    c1.evaluated = false; c1.objs.clear();
    c2.evaluated = false; c2.objs.clear();
    return {c1, c2};
}

void polyMutate(Individual& ind, const GearOptConfig& cfg, std::mt19937& rng,
                double mutProb) {
    if (mutProb < 0) mutProb = 1.0 / VAR_COUNT;
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    const double eta = cfg.nsga2.mutEta;
    const QVector<Bounds> bv = boundsVector(cfg);

    for (int i = 0; i < VAR_COUNT; ++i) {
        if (uni(rng) > mutProb) continue;
        const Bounds& b = bv[i];
        const double range = b.upper - b.lower;
        if (range < 1e-12) continue;
        const double u = uni(rng);
        double delta;
        if (u < 0.5) {
            delta = std::pow(2.0 * u, 1.0 / (eta + 1.0)) - 1.0;
        } else {
            delta = 1.0 - std::pow(2.0 * (1.0 - u), 1.0 / (eta + 1.0));
        }
        ind.vars[i] = snapBounds(ind.vars[i] + delta * range, b);
    }
    ind.evaluated = false;
    ind.objs.clear();
}

// ================================================================
//   Deb 约束支配
// ================================================================

bool debDominates(const Individual& a, const Individual& b) {
    const bool aFeas = (a.constraintViolation <= 0.0);
    const bool bFeas = (b.constraintViolation <= 0.0);

    if (aFeas && !bFeas) return true;
    if (!aFeas && bFeas) return false;
    if (!aFeas && !bFeas) return a.constraintViolation < b.constraintViolation;

    // 都可行：Pareto 支配
    const int n = std::min(a.objs.size(), b.objs.size());
    bool allLeq = true, oneLt = false;
    for (int i = 0; i < n; ++i) {
        if (a.objs[i] > b.objs[i]) { allLeq = false; break; }
        if (a.objs[i] < b.objs[i]) oneLt = true;
    }
    return allLeq && oneLt;
}

// ================================================================
//   evolve() + selectNextGen()
// ================================================================

// 二元锦标赛选手：rank 小 > rank 大；同 rank 时 crowding 大优先
static const Individual& tournament(const Individual& a, const Individual& b,
                                     std::mt19937& rng) {
    // 50% 随机打平，保持多样性
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    if (a.rank < b.rank) return a;
    if (b.rank < a.rank) return b;
    if (a.crowdingDist > b.crowdingDist) return a;
    if (b.crowdingDist > a.crowdingDist) return b;
    return (uni(rng) < 0.5) ? a : b;
}

Population evolve(const Population& parents, const GearOptConfig& cfg, std::mt19937& rng) {
    const int N = parents.size();
    std::uniform_int_distribution<int> pickIdx(0, N - 1);
    Population offspring;
    offspring.reserve(N);

    while (offspring.size() < N) {
        // 二元锦标赛 × 2
        const Individual& p1 = tournament(parents[pickIdx(rng)], parents[pickIdx(rng)], rng);
        const Individual& p2 = tournament(parents[pickIdx(rng)], parents[pickIdx(rng)], rng);
        auto [c1, c2] = sbxCrossover(p1, p2, cfg, rng);
        polyMutate(c1, cfg, rng);
        polyMutate(c2, cfg, rng);
        repairAndEval(c1, cfg);
        repairAndEval(c2, cfg);
        offspring.append(c1);
        if (offspring.size() < N) offspring.append(c2);
    }
    return offspring;
}

Population selectNextGen(Population combined, int N) {
    auto fronts = fastNonDominatedSort(combined);
    for (const auto& front : fronts)
        assignCrowdingDistance(combined, front);

    // 按 (rank ASC, crowdingDist DESC) 排序
    QVector<int> indices(combined.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        if (combined[a].rank != combined[b].rank)
            return combined[a].rank < combined[b].rank;
        return combined[a].crowdingDist > combined[b].crowdingDist;
    });

    Population next;
    next.reserve(N);
    for (int i = 0; i < N && i < indices.size(); ++i)
        next.append(combined[indices[i]]);
    return next;
}

// ================================================================
//   Pareto 归档 + 超体积
// ================================================================

Population extractPareto(const Population& pop) {
    Population pareto;
    for (const auto& ind : pop)
        if (ind.rank == 1 && ind.evaluated) pareto.append(ind);
    return pareto;
}

double hypervolume2D(const Population& pareto, double ref0, double ref1) {
    if (pareto.isEmpty()) return 0.0;
    // 按第一目标排序
    QVector<const Individual*> sorted;
    for (const auto& ind : pareto) {
        if (ind.objs.size() >= 2) sorted.append(&ind);
    }
    std::sort(sorted.begin(), sorted.end(), [](const Individual* a, const Individual* b) {
        return a->objs[0] < b->objs[0];
    });

    double hv = 0.0;
    double prevX = ref0;
    for (const auto* ind : sorted) {
        if (ind->objs[0] >= ref0) continue;
        if (ind->objs[1] >= ref1) continue;
        hv += (ref0 - ind->objs[0]) * (ref1 - prevX);
        // 累计宽度（按第二目标方向）
        // 简化版：用扫描线法（按 f1 排序，累计矩形面积）
        Q_UNUSED(prevX);
        // 重新实现：标准扫描线
        break; // 以下用正确实现
    }

    // 正确的双目标超体积（扫描线，f1 升序，f2 需考虑）
    hv = 0.0;
    double lastY = ref1;
    for (const auto* ind : sorted) {
        if (ind->objs[1] >= ref1) continue;
        const double width = ref0 - ind->objs[0];
        if (width <= 0) continue;
        const double height = lastY - ind->objs[1];
        if (height > 0) hv += width * height;
        if (ind->objs[1] < lastY) lastY = ind->objs[1];
    }
    return hv;
}

bool checkConverged(const QVector<double>& hvHistory, double tol, int windowSize) {
    if (hvHistory.size() < windowSize) return false;
    const int n = hvHistory.size();
    for (int i = n - windowSize + 1; i < n; ++i) {
        if (std::abs(hvHistory[i] - hvHistory[i-1]) > tol) return false;
    }
    return true;
}

} // namespace GearAutoOpt
