// UTF-8 BOM
#include "GearAutoOptManager.h"
#include "GearOptCaseRunner.h"

#include <QDir>
#include <QDateTime>

namespace GearAutoOpt {

GearAutoOptManager::GearAutoOptManager(QObject* parent)
    : QObject(parent)
{}

GearAutoOptManager::~GearAutoOptManager() = default;

QList<GearDesignPoint> GearAutoOptManager::paretoFront() const {
    QList<GearDesignPoint> all;
    for (const auto& gen : _allPoints)
        for (const auto& dp : gen)
            if (dp.status == PointStatus::Done) all.append(dp);

    QList<GearDesignPoint> pareto;
    for (const auto& a : all) {
        bool dominated = false;
        for (const auto& b : all) {
            if (b.sigmaMax <= a.sigmaMax && b.mass <= a.mass &&
                (b.sigmaMax < a.sigmaMax || b.mass < a.mass)) {
                dominated = true; break;
            }
        }
        if (!dominated) pareto.append(a);
    }
    return pareto;
}

void GearAutoOptManager::evaluatePopulation(Population& pop, int generation) {
    const int N = pop.size();
    QList<GearDesignPoint> genPoints;

    for (int i = 0; i < N; ++i) {
        if (_stopRequested) break;

        Individual& ind = pop[i];
        if (ind.evaluated) continue;  // 精英保留个体无需重复评估

        GearDesignPoint dp = ind.toDesignPoint(i, generation);

        const QString workDir = QString("%1/%2_%3")
            .arg(_runDir).arg(generation).arg(i);
        QDir().mkpath(workDir);
        dp.runDir = workDir;

        GearOptCaseRunner runner;
        runner.setConfig(_cfg);
        runner.setWorkDir(workDir);
        if (_cfg.solver.meshSize > 0.0)
            runner.setMeshSize(_cfg.solver.meshSize);
        runner.setThreads(_cfg.solver.threads);
        connect(&runner, &GearOptCaseRunner::log,
                [this](const QString& msg) { emit log(msg); });

        runner.runOne(dp);

        if (dp.status == PointStatus::Done && dp.sigmaMax >= 0) {
            ind.objs.clear();
            ind.objs.append(dp.sigmaMax);
            ind.objs.append(dp.mass >= 0 ? dp.mass : 0.0);
            if (_cfg.objectives.minRatioErr && dp.z1 > 0 && dp.z2 > 0) {
                const double ratioActual = static_cast<double>(dp.z2) / dp.z1;
                ind.objs.append(std::abs(ratioActual - _cfg.objectives.targetRatio));
            }
            ind.evaluated = true;
            ind.constraintViolation = 0.0;
        } else {
            // 求解失败：大惩罚目标值 + 约束违反量
            ind.objs = {1e8, 1e8};
            if (_cfg.objectives.minRatioErr) ind.objs.append(1e8);
            ind.constraintViolation = 1.0;
            ind.evaluated = true;
        }

        genPoints.append(dp);
        emit pointFinished(generation, i, dp);

        emit log(QString("[gen%1/%2] id=%3 σ=%4 MPa m=%5 kg status=%6")
                 .arg(generation).arg(_cfg.nsga2.maxGenerations).arg(i)
                 .arg(dp.sigmaMax, 0, 'g', 5)
                 .arg(dp.mass, 0, 'g', 4)
                 .arg(pointStatusToString(dp.status)));
    }
    _allPoints.append(genPoints);
}

void GearAutoOptManager::start() {
    emit log(QStringLiteral("GearAutoOptManager: start"));
    _stopRequested = false;
    _allPoints.clear();
    // 若 config 指定了结果根目录则覆盖 setRunDir() 传来的值
    if (!_cfg.solver.runBaseDir.isEmpty())
        _runDir = _cfg.solver.runBaseDir;
    QDir().mkpath(_runDir);

    const int N    = _cfg.nsga2.populationSize;
    const int Gmax = _cfg.nsga2.maxGenerations;
    std::mt19937 rng(static_cast<unsigned>(_cfg.nsga2.randomSeed));

    emit log(QString("init: Latin hypercube N=%1 seed=%2")
             .arg(N).arg(_cfg.nsga2.randomSeed));
    _population = initLatin(N, _cfg, _cfg.nsga2.randomSeed);
    evaluatePopulation(_population, 0);

    if (_stopRequested) { emit finished(false); return; }

    auto fronts = fastNonDominatedSort(_population);
    for (const auto& front : fronts)
        assignCrowdingDistance(_population, front);

    QVector<double> hvHistory;
    Population pareto0 = extractPareto(_population);
    double hv0 = hypervolume2D(pareto0, _cfg.constraints.sigmaAllow * 2.0,
                                _cfg.solver.density * 1000.0 * 3.14 * 1000.0);
    hvHistory.append(hv0);
    emit generationFinished(0, pareto0.size(), hv0);

    // G=1..Gmax
    for (int g = 1; g <= Gmax && !_stopRequested; ++g) {
        emit log(QString("--- generation %1 / %2 ---").arg(g).arg(Gmax));

        Population offspring = evolve(_population, _cfg, rng);
        evaluatePopulation(offspring, g);

        if (_stopRequested) break;

        Population combined = _population + offspring;
        _population = selectNextGen(combined, N);

        Population pareto = extractPareto(_population);
        // 参考点取 2× 许用应力和估算最大质量，留足余量
        const double refSigma = _cfg.constraints.sigmaAllow * 2.0;
        const double refMass  = 5.0; // kg，齿宽 30mm 下约 1 kg，留裕量
        const double hv = hypervolume2D(pareto, refSigma, refMass);
        hvHistory.append(hv);

        emit generationFinished(g, pareto.size(), hv);
        emit log(QString("gen%1: paretoSize=%2 hv=%3")
                 .arg(g).arg(pareto.size()).arg(hv, 0, 'g', 6));

        if (checkConverged(hvHistory, _cfg.nsga2.hypervolumeTol,
                           _cfg.nsga2.convergenceWindow)) {
            emit log(QStringLiteral("converged (hypervolume stable)"));
            break;
        }
    }

    emit finished(!_stopRequested);
}

} // namespace GearAutoOpt
