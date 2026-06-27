// UTF-8 BOM
#include "GearAutoOptManager.h"
#include "GearOptCaseRunner.h"

#include "GearAutoOpt/data/GearLogLevel.h"
#include "GearAutoOpt/data/GearOptGeometryBridge.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"
#include "GearAutoOpt/solver/CCXResultParser.h"
#include "GearAutoOpt/surrogate/GearInfillSelector.h"
#include "GearAutoOpt/surrogate/GearSurrogateMetrics.h"

#include <QDebug>
#include <QDir>
#include <QDateTime>
#include <QFileInfo>
#include <QHash>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <vector>

namespace GearAutoOpt {

GearAutoOptManager::GearAutoOptManager(QObject* parent)
    : QObject(parent)
{}

GearAutoOptManager::~GearAutoOptManager() = default;

GearDesignPoint GearAutoOptManager::configuredBasePoint() const
{
    GearDesignPoint dp = _cfg.useOptimizationBase ? _cfg.optimizationBase : GearDesignPoint();
    dp.applyRunSimDefaults(_cfg, _fixedMeshSizeMm, _runMeshAuto);
    return dp;
}

static bool isValidSurrogateTrainingSample(const SurrogateSample& s, const GearOptConfig& cfg)
{
    if (s.cpressMax <= 0.0 || !std::isfinite(s.cpressMax))
        return false;
    if (s.edgeLoadRatio <= 0.0 || !std::isfinite(s.edgeLoadRatio))
        return false;
    if (s.cpressCV <= 0.0 || !std::isfinite(s.cpressCV))
        return false;
    if (cfg.objectives.minSigmaMax && (s.sigmaMax <= 0.0 || !std::isfinite(s.sigmaMax)))
        return false;
    return true;
}

static std::pair<double, double> paretoObjectiveMax(const Population& pareto)
{
    double maxCpress = -1.0;
    double maxEdge   = -1.0;
    for (const Individual& ind : pareto) {
        if (!ind.evaluated || ind.objs.size() < 2)
            continue;
        if (ind.objs[0] > 0.0 && std::isfinite(ind.objs[0]))
            maxCpress = std::max(maxCpress, ind.objs[0]);
        if (ind.objs[1] > 0.0 && std::isfinite(ind.objs[1]))
            maxEdge = std::max(maxEdge, ind.objs[1]);
    }
    return {maxCpress, maxEdge};
}

static QVector<SurrogateSample> loadMergedValidatedSamples(const QString& baseCaseH,
                                                           double fixedWidthMm,
                                                           const GearOptConfig& cfg)
{
    QHash<QString, SurrogateSample> uniq;
    auto ingestDb = [&](GearOptResultDatabase& db) {
        if (!db.isOpen())
            return;
        for (const SurrogateSample& s : db.loadValidatedSamples(baseCaseH, fixedWidthMm)) {
            if (!isValidSurrogateTrainingSample(s, cfg))
                continue;
            const QString key = s.designHash.isEmpty()
                                    ? QStringLiteral("idx_%1").arg(uniq.size())
                                    : s.designHash;
            uniq.insert(key, s);
        }
    };
    ingestDb(GearOptResultDatabase::global());
    ingestDb(GearOptResultDatabase::runSession());
    return uniq.values().toVector();
}

static void fixPopulationCommonWidth(Population& pop, double widthMm)
{
    for (Individual& ind : pop) {
        if (ind.vars.size() < VAR_COUNT)
            ind.vars.resize(VAR_COUNT);
        ind.vars[VAR_COMMON_WIDTH] = widthMm;
    }
}

static int primaryObjectiveCount(const GearOptConfig& cfg)
{
    int n = 0;
    if (cfg.objectives.minCpressMax)
        ++n;
    if (cfg.objectives.minEdgeLoadRatio)
        ++n;
    if (cfg.objectives.minCpressCV)
        ++n;
    if (cfg.objectives.minSigmaMax)
        ++n;
    if (cfg.objectives.minMass)
        ++n;
    return n > 0 ? n : 2;
}

static void appendPrimaryObjectives(QVector<double>& objs,
                                    const GearOptConfig& cfg,
                                    double cpress,
                                    double edgeLoadRatio,
                                    double cpressCV,
                                    double sigma,
                                    double mass)
{
    constexpr double kPenalty = 1.0e6;
    objs.clear();
    if (cfg.objectives.minCpressMax)
        objs.append(cpress);
    if (cfg.objectives.minEdgeLoadRatio)
        objs.append(edgeLoadRatio > 0.0 ? edgeLoadRatio : kPenalty);
    if (cfg.objectives.minCpressCV)
        objs.append(cpressCV > 0.0 ? cpressCV : kPenalty);
    if (cfg.objectives.minSigmaMax)
        objs.append(sigma > 0.0 ? sigma : 0.0);
    if (cfg.objectives.minMass)
        objs.append(mass >= 0.0 ? mass : 0.0);
    if (objs.isEmpty()) {
        objs.append(cpress);
        objs.append(edgeLoadRatio > 0.0 ? edgeLoadRatio : kPenalty);
    }
}

static bool dominatesByObjectives(const GearDesignPoint& a,
                                const GearDesignPoint& b,
                                const Objectives& obj)
{
    const bool useDefault = !obj.minCpressMax && !obj.minSigmaMax && !obj.minMass;
    bool       allLe      = true;
    bool       anyStrict  = false;
    auto       consider   = [&](bool enabled, double av, double bv) {
        if (!enabled)
            return;
        if (bv > av)
            allLe = false;
        else if (bv < av)
            anyStrict = true;
    };
    consider(useDefault || obj.minCpressMax, a.cpressMax_MPa, b.cpressMax_MPa);
    consider(useDefault || obj.minEdgeLoadRatio, a.edgeLoadRatio, b.edgeLoadRatio);
    consider(!useDefault && obj.minCpressCV, a.cpressCV, b.cpressCV);
    consider(useDefault || obj.minSigmaMax, a.sigmaMax, b.sigmaMax);
    consider(!useDefault && obj.minMass, a.mass, b.mass);
    return allLe && anyStrict;
}

static void logEnabledObjectives(const GearOptConfig& cfg,
                                 const std::function<void(const QString&)>& emitLog)
{
    QStringList enabled;
    if (cfg.objectives.minCpressMax)
        enabled << QStringLiteral("cpressMax_MPa");
    if (cfg.objectives.minEdgeLoadRatio)
        enabled << QStringLiteral("edgeLoadRatio");
    if (cfg.objectives.minCpressCV)
        enabled << QStringLiteral("cpressCV");
    if (cfg.objectives.minSigmaMax)
        enabled << QStringLiteral("sigmaMax_MPa");
    if (cfg.objectives.minMass)
        enabled << QStringLiteral("mass_total_kg");
    if (enabled.isEmpty()) {
        enabled << QStringLiteral("cpressMax_MPa") << QStringLiteral("edgeLoadRatio");
    }
    emitLog(QStringLiteral("[GearOpt][Objectives] enabled: %1").arg(enabled.join(QStringLiteral(", "))));
    emitLog(QStringLiteral("[GearOpt][Objectives] primary objective: cpressMax_MPa"));
    emitLog(QStringLiteral("[GearOpt][Objectives] secondary objective: edgeLoadRatio"));
    emitLog(QStringLiteral("[GearOpt][Objectives] recorded metrics: sigmaMax_MPa, uMax_mm, mass_kg, cpressCV"));
    if (!cfg.objectives.minMass) {
        emitLog(QStringLiteral(
            "[GearOpt][Objectives] mass objective disabled; mass is recorded only"));
    }
}

static Population populationFromValidatedSamples(const QVector<SurrogateSample>& samples,
                                                 const GearOptConfig& cfg)
{
    Population pop;
    pop.reserve(samples.size());
    for (const SurrogateSample& s : samples) {
        if (!isValidSurrogateTrainingSample(s, cfg))
            continue;
        Individual ind;
        ind.objs.append(s.cpressMax);
        ind.objs.append(s.edgeLoadRatio);
        ind.evaluated = true;
        pop.append(ind);
    }
    if (!pop.isEmpty())
        fastNonDominatedSort(pop);
    return pop;
}

static void logSurrogateFixedObjectives(const std::function<void(const QString&)>& emitLog)
{
    emitLog(QStringLiteral("[GearOpt][Objectives] enabled: cpressMax_MPa, edgeLoadRatio"));
    emitLog(QStringLiteral("[GearOpt][Objectives] primary objective: cpressMax_MPa"));
    emitLog(QStringLiteral("[GearOpt][Objectives] secondary objective: edgeLoadRatio"));
    emitLog(QStringLiteral("[GearOpt][Objectives] recorded metrics: sigmaMax_MPa, uMax_mm, mass_kg, cpressCV"));
    emitLog(QStringLiteral("[GearOpt][Surrogate] targets: cpressMax_MPa, edgeLoadRatio"));
}

static QVector<double> computeSurrogateSampleResidualsFixed(const QVector<SurrogateSample>& samples,
                                                            const GearSurrogateModel& model)
{
    QVector<double> residuals;
    residuals.reserve(samples.size());
    for (const SurrogateSample& s : samples) {
        const SurrogatePrediction pred = model.predict(s.x);
        double residualSum = 0.0;
        int    residualN   = 0;
        if (s.cpressMax > 0.0 && pred.cpressMaxPred >= 0.0) {
            residualSum += std::abs(s.cpressMax - pred.cpressMaxPred);
            ++residualN;
        }
        if (s.edgeLoadRatio > 0.0 && pred.edgeLoadRatioPred >= 0.0) {
            residualSum += std::abs(s.edgeLoadRatio - pred.edgeLoadRatioPred);
            ++residualN;
        }
        residuals.append(residualN > 0 ? residualSum / static_cast<double>(residualN) : 0.0);
    }
    return residuals;
}

static QString sampleDesignSummary(const SurrogateSample& s)
{
    const auto v = [&](int i) -> double {
        return i >= 0 && i < s.x.size() ? s.x[i] : 0.0;
    };
    return QStringLiteral("x1=%1 x2=%2 ca1=%3 lca1=%4 ca2=%5 lca2=%6 hubRatio=%7 case_hash=%8")
        .arg(v(0), 0, 'g', 6)
        .arg(v(1), 0, 'g', 6)
        .arg(v(2), 0, 'g', 6)
        .arg(v(3), 0, 'g', 6)
        .arg(v(4), 0, 'g', 6)
        .arg(v(5), 0, 'g', 6)
        .arg(v(6), 0, 'g', 6)
        .arg(s.caseHash.isEmpty() ? QStringLiteral("n/a") : s.caseHash);
}

static void logBestRealSoFar(const QVector<SurrogateSample>& samples,
                             int round,
                             const GearOptConfig& cfg,
                             const std::function<void(const QString&)>& emitLog)
{
    const SurrogateSample* bestCpress = nullptr;
    const SurrogateSample* bestSigma = nullptr;
    for (const SurrogateSample& s : samples) {
        if (!isValidSurrogateTrainingSample(s, cfg))
            continue;
        if (!bestCpress || s.cpressMax < bestCpress->cpressMax)
            bestCpress = &s;
        if (s.sigmaMax > 0.0 && (!bestSigma || s.sigmaMax < bestSigma->sigmaMax))
            bestSigma = &s;
    }

    emitLog(QStringLiteral("[Surrogate][BestReal] round=%1 best_real_cpress_so_far=%2 MPa")
                .arg(round)
                .arg(bestCpress ? QString::number(bestCpress->cpressMax, 'g', 8)
                                : QStringLiteral("N/A")));
    emitLog(QStringLiteral("[Surrogate][BestReal] round=%1 best_real_sigma_so_far=%2 MPa")
                .arg(round)
                .arg(bestSigma ? QString::number(bestSigma->sigmaMax, 'g', 8)
                               : QStringLiteral("N/A")));
    emitLog(QStringLiteral("[Surrogate][BestReal] round=%1 best_real_design=%2")
                .arg(round)
                .arg(bestCpress ? sampleDesignSummary(*bestCpress) : QStringLiteral("N/A")));
}

void GearAutoOptManager::openResultDatabases()
{
    if (!_cfg.solver.runBaseDir.isEmpty())
        _runDir = _cfg.solver.runBaseDir;
    QDir().mkpath(_runDir);

    auto& globalDb = GearOptResultDatabase::global();
    if (!globalDb.isOpen()) {
        if (!globalDb.openDatabase(GearOptResultDatabase::defaultGlobalDatabasePath())) {
            emit log(QStringLiteral("[GearOpt][DB] warning: global database unavailable"));
        }
    }
	if (globalDb.isOpen()) {
		emit log(QStringLiteral("[Save] global database: %1").arg(globalDb.databasePath()));
		const int backfilled = globalDb.backfillMissingCpressDistributionMetrics(_fixedZLayers > 0 ? _fixedZLayers : 11);
		if (backfilled > 0) {
			emit log(QStringLiteral("[GearOpt][DB] global backfilled cpress distribution metrics: %1")
			             .arg(backfilled));
		}
		emit log(QStringLiteral("[GearOpt][DB] global cached samples: ")
		         + QString::number(globalDb.countValidResults()));
	}

    const QString runDbPath = GearOptResultDatabase::databasePathInRunDir(_runDir);
    auto&         runDb     = GearOptResultDatabase::runSession();
    if (!runDb.openDatabase(runDbPath)) {
        emit log(QStringLiteral("[GearOpt][DB] warning: run database not available"));
	} else {
		emit log(QStringLiteral("[Save] run database: %1").arg(runDb.databasePath()));
		emit log(QStringLiteral("[GearOpt][DB] run_id = ") + _runId);
		const int backfilled = runDb.backfillMissingCpressDistributionMetrics(_fixedZLayers > 0 ? _fixedZLayers : 11);
		if (backfilled > 0) {
			emit log(QStringLiteral("[GearOpt][DB] run backfilled cpress distribution metrics: %1")
			             .arg(backfilled));
		}
	}
}

QList<GearDesignPoint> GearAutoOptManager::paretoFront() const {
    QList<GearDesignPoint> all;
    for (const auto& gen : _allPoints)
        for (const auto& dp : gen) {
            if (dp.status != PointStatus::Done || dp.cpressMax_MPa <= 0.0)
                continue;
            if (_cfg.objectives.minSigmaMax && dp.sigmaMax <= 0.0)
                continue;
            all.append(dp);
        }

    QList<GearDesignPoint> pareto;
    for (const auto& a : all) {
        bool dominated = false;
        for (const auto& b : all) {
            if (dominatesByObjectives(a, b, _cfg.objectives)) {
                dominated = true;
                break;
            }
        }
        if (!dominated) pareto.append(a);
    }
    return pareto;
}

void GearAutoOptManager::evaluatePopulation(Population& pop, int generation) {
    evaluatePopulationByCcx(pop, generation);
}

void GearAutoOptManager::evaluatePopulationByCcx(
    Population& pop,
    int generation,
    CcxEvalSummary* summary,
    const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback) {
    const int N = pop.size();
    QList<GearDesignPoint> genPoints;
    if (summary) {
        summary->newCcxCount      = 0;
        summary->reusedCacheCount = 0;
        summary->cacheHitByIndex  = QVector<bool>(N, false);
    }
    //储存参数

    for (int i = 0; i < N; ++i) {
        if (_stopRequested) break;
        //如果点击了停止 停止优化

        Individual& ind = pop[i];
        if (ind.evaluated) continue;  // 精英保留个体无需重复评估

        GearDesignPoint dp = ind.toDesignPoint(i, generation);

        checkGearOptDesignPointBounds(dp, _cfg);

        QString reliefReason;
        if (!validateReliefDesign(dp, &reliefReason)) {
            logInvalidReliefDesign(dp, reliefReason);
            dp.status   = PointStatus::Infeasible;
            dp.errorMsg = reliefReason;
            dp.runId    = _runId;
            dp.rank     = ind.rank;
            dp.crowdingDistance = ind.crowdingDist;
            dp.isPareto = (ind.rank == 1) ? 1 : 0;
            const int nObj = primaryObjectiveCount(_cfg) + (_cfg.objectives.minRatioErr ? 1 : 0);
            ind.objs                  = QVector<double>(nObj, 1e8);
            ind.constraintViolation   = 1.0;
            ind.evaluated             = true;
            genPoints.append(dp);
            emit pointFinished(generation, i, dp);
            if (pointCallback)
                pointCallback(i, dp, false);
            emit log(QString("[gen%1/%2] id=%3 skipped invalid relief status=infeasible")
                         .arg(generation).arg(_cfg.nsga2.maxGenerations).arg(i));
            continue;
        }

        const QString workDir = QString("%1/%2_%3")
            .arg(_runDir).arg(generation).arg(i);
        QDir().mkpath(workDir);
        dp.runDir = workDir;

        dp.applyRunSimDefaults(_cfg, _fixedMeshSizeMm, _runMeshAuto);
        dp.runDir  = workDir;
        dp.runId   = _runId;
        dp.rank    = ind.rank;
        dp.crowdingDistance = ind.crowdingDist;
        dp.isPareto = (ind.rank == 1) ? 1 : 0;

        const QString caseH = GearOptResultDatabase::caseHash(dp);

        auto tryLoadCache = [&](GearOptResultDatabase& db, const char* label) -> bool {
            int cachedRowId = -1;
            if (!db.isOpen() || !db.findIdByCaseHash(caseH, &cachedRowId))
                return false;
            qDebug() << "[GearOpt][DB]" << label << "duplicate case id=" << cachedRowId;
            GearDesignPoint cached = dp;
            if (!db.loadResultByCaseHash(caseH, cached))
                return false;
            cached.generation       = generation;
            cached.runDir           = workDir;
            cached.runId            = _runId;
            cached.rank             = ind.rank;
            cached.crowdingDistance = ind.crowdingDist;
            cached.isPareto         = (ind.rank == 1) ? 1 : 0;
            dp                      = cached;
            emit log(QStringLiteral("[GearOpt][DB] %1 cache hit (cpress=%2 MPa)")
                         .arg(QString::fromLatin1(label))
                         .arg(dp.cpressMax_MPa, 0, 'g', 5));
            return true;
        };

        const bool skippedByCache =
            tryLoadCache(GearOptResultDatabase::global(), "global")
            || tryLoadCache(GearOptResultDatabase::runSession(), "run");

        if (summary)
            summary->cacheHitByIndex[i] = skippedByCache;

        if (!skippedByCache) {
            if (summary)
                summary->newCcxCount++;
            GearOptCaseRunner runner;
            runner.setConfig(_cfg);
            runner.setWorkDir(workDir);
            GearMeshParams meshParams;
            meshParams.globalSize = _fixedMeshSizeMm;
            meshParams.rootSize   = _fixedRootMeshSizeMm;
            meshParams.zLayers    = _fixedZLayers;
            runner.setMeshParams(meshParams);
            runner.setLogLevel(_cfg.solver.debugMode ? GearLogLevel::Debug : GearLogLevel::Normal);
            runner.setThreads(_cfg.solver.threads);
            connect(&runner, &GearOptCaseRunner::log,
                    [this](const QString& msg) { emit log(msg); });
                //跑自动化流程
            runner.runOne(dp);
        } else if (summary) {
            summary->reusedCacheCount++;
        }

        dp.fillResultArtifactPaths();
        auto insertToDb = [&](GearOptResultDatabase& db) {
            if (!db.isOpen())
                return;
            db.insertDesignPointResult(_runId, dp, generation, i, ind.rank, ind.crowdingDist,
                                       dp.isPareto, caseH);
        };
        insertToDb(GearOptResultDatabase::global());
        insertToDb(GearOptResultDatabase::runSession());
        if (dp.status == PointStatus::Done && dp.cpressActiveNodes > 0 && QFileInfo::exists(dp.frdPath)) {
            const CpressDistributionMetrics metrics =
                parseCpressDistributionMetrics(dp.frdPath, dp.cpressBinCount > 0 ? dp.cpressBinCount : (_fixedZLayers > 0 ? _fixedZLayers : 11));
            if (!GearOptResultDatabase::appendCpressWidthBinsCsv(_runDir, _runId, generation, i, caseH, metrics)) {
                emit log(QStringLiteral("[GearOpt][CPress] warning: failed to append cpress_width_bins.csv under %1")
                             .arg(_runDir));
            }
        }

        if (GearOptResultDatabase::global().isOpen()) {
            const int validN = GearOptResultDatabase::global().countValidResults();
            if (validN >= 500 && !_surrogateThresholdLogged) {
                _surrogateThresholdLogged = true;
                emit log(QStringLiteral(
                    "[GearOpt][Surrogate] global valid samples >= 500, surrogate can be enabled later."));
            }
        }

        const bool sigmaOk = !_cfg.objectives.minSigmaMax || dp.sigmaMax > 0.0;
        const bool edgeOk = !_cfg.objectives.minEdgeLoadRatio || dp.edgeLoadRatio > 0.0;
        const bool cvOk = !_cfg.objectives.minCpressCV || dp.cpressCV > 0.0;
        if (dp.status == PointStatus::Done && dp.cpressMax_MPa > 0.0 && sigmaOk && edgeOk && cvOk) {
            appendPrimaryObjectives(ind.objs, _cfg, dp.cpressMax_MPa, dp.edgeLoadRatio,
                                    dp.cpressCV, dp.sigmaMax, dp.mass);
            if (_cfg.objectives.minRatioErr && dp.z1 > 0 && dp.z2 > 0) {
                const double ratioActual = static_cast<double>(dp.z2) / dp.z1;
                ind.objs.append(std::abs(ratioActual - _cfg.objectives.targetRatio));
            }
            ind.evaluated = true;
            ind.constraintViolation = 0.0;
        } else {
            // 求解失败或主目标缺失：大惩罚目标值 + 约束违反量
            const int nObj = primaryObjectiveCount(_cfg) + (_cfg.objectives.minRatioErr ? 1 : 0);
            ind.objs = QVector<double>(nObj, 1e8);
            ind.constraintViolation = 1.0;
            ind.evaluated = true;
            if (dp.status == PointStatus::Done && dp.cpressMax_MPa <= 0.0)
                emit log(QStringLiteral("[GearOpt] Skip sample because cpressMax_MPa is missing."));
            else if (dp.status == PointStatus::Done && !edgeOk)
                emit log(QStringLiteral("[GearOpt] Skip sample because edgeLoadRatio is missing."));
            else if (dp.status == PointStatus::Done && !cvOk)
                emit log(QStringLiteral("[GearOpt] Skip sample because cpressCV is missing."));
            else if (dp.status == PointStatus::Done && !sigmaOk)
                emit log(QStringLiteral("[GearOpt] Skip sample because sigmaMax_MPa is missing."));
        }

        genPoints.append(dp);
        emit pointFinished(generation, i, dp);
        if (pointCallback)
            pointCallback(i, dp, skippedByCache);

        emit log(QString("[gen%1/%2] id=%3 cpress=%4 MPa σ=%5 MPa m=%6 kg status=%7%8")
                 .arg(generation).arg(_cfg.nsga2.maxGenerations).arg(i)
                 .arg(dp.cpressMax_MPa, 0, 'g', 5)
                 .arg(dp.sigmaMax, 0, 'g', 5)
                 .arg(dp.mass, 0, 'g', 4)
                 .arg(pointStatusToString(dp.status))
                 .arg(dp.status != PointStatus::Done && !dp.errorMsg.isEmpty()
                      ? QStringLiteral(" | ") + dp.errorMsg.left(160)
                      : QString()));
    }
    _allPoints.append(genPoints);
}

static bool ccxCaseConverged(const GearOptConfig& cfg, const GearDesignPoint& dp)
{
    const bool sigmaOk = !cfg.objectives.minSigmaMax || dp.sigmaMax > 0.0;
    const bool edgeOk  = !cfg.objectives.minEdgeLoadRatio || dp.edgeLoadRatio > 0.0;
    const bool cvOk    = !cfg.objectives.minCpressCV || dp.cpressCV > 0.0;
    return dp.status == PointStatus::Done && dp.cpressMax_MPa > 0.0 && sigmaOk && edgeOk && cvOk;
}

static void applyCcxResultToIndividual(Individual& ind,
                                       const GearOptConfig& cfg,
                                       GearDesignPoint& dp,
                                       const std::function<void(const QString&)>& emitLog)
{
    const bool sigmaOk = !cfg.objectives.minSigmaMax || dp.sigmaMax > 0.0;
    const bool edgeOk  = !cfg.objectives.minEdgeLoadRatio || dp.edgeLoadRatio > 0.0;
    const bool cvOk    = !cfg.objectives.minCpressCV || dp.cpressCV > 0.0;
    if (dp.status == PointStatus::Done && dp.cpressMax_MPa > 0.0 && sigmaOk && edgeOk && cvOk) {
        appendPrimaryObjectives(ind.objs, cfg, dp.cpressMax_MPa, dp.edgeLoadRatio,
                                dp.cpressCV, dp.sigmaMax, dp.mass);
        if (cfg.objectives.minRatioErr && dp.z1 > 0 && dp.z2 > 0) {
            const double ratioActual = static_cast<double>(dp.z2) / dp.z1;
            ind.objs.append(std::abs(ratioActual - cfg.objectives.targetRatio));
        }
        ind.evaluated           = true;
        ind.constraintViolation = 0.0;
    } else {
        const int nObj = primaryObjectiveCount(cfg) + (cfg.objectives.minRatioErr ? 1 : 0);
        ind.objs                  = QVector<double>(nObj, 1e8);
        ind.constraintViolation   = 1.0;
        ind.evaluated             = true;
        if (dp.status == PointStatus::Done && dp.cpressMax_MPa <= 0.0)
            emitLog(QStringLiteral("[GearOpt] Skip sample because cpressMax_MPa is missing."));
        else if (dp.status == PointStatus::Done && !edgeOk)
            emitLog(QStringLiteral("[GearOpt] Skip sample because edgeLoadRatio is missing."));
        else if (dp.status == PointStatus::Done && !cvOk)
            emitLog(QStringLiteral("[GearOpt] Skip sample because cpressCV is missing."));
        else if (dp.status == PointStatus::Done && !sigmaOk)
            emitLog(QStringLiteral("[GearOpt] Skip sample because sigmaMax_MPa is missing."));
    }
}

void GearAutoOptManager::evaluateInfillByCcx(
    Population& pop,
    int generation,
    CcxEvalSummary* summary,
    const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback) {
    if (!_cfg.solver.parallelCcxEnabled || _cfg.solver.parallelCcxJobs <= 1) {
        evaluatePopulationByCcx(pop, generation, summary, pointCallback);
        return;
    }
    evaluateInfillByCcxParallel(pop, generation, summary, pointCallback);
}

void GearAutoOptManager::evaluateInfillByCcxParallel(
    Population& pop,
    int generation,
    CcxEvalSummary* summary,
    const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback) {
    const int N = pop.size();
    const int jobs = std::clamp(_cfg.solver.parallelCcxJobs, 1, 8);
    const int threadsPerJob = std::clamp(_cfg.solver.ccxThreadsPerJob, 1, 16);
    const int totalThreads  = jobs * threadsPerJob;

    emit log(QStringLiteral("[GearOpt][Parallel] enabled=true jobs=%1 threadsPerJob=%2 totalThreads=%3")
                 .arg(jobs)
                 .arg(threadsPerJob)
                 .arg(totalThreads));

    if (summary) {
        summary->newCcxCount      = 0;
        summary->reusedCacheCount = 0;
        summary->cacheHitByIndex  = QVector<bool>(N, false);
    }

    struct InfillCasePrep {
        int             index = 0;
        Individual      ind;
        GearDesignPoint dp;
        QString         caseHash;
        QString         workDir;
        bool            skippedByCache = false;
    };

    QVector<InfillCasePrep> preps;
    preps.reserve(N);

    for (int i = 0; i < N; ++i) {
        if (_stopRequested)
            break;

        Individual& ind = pop[i];
        if (ind.evaluated)
            continue;

        InfillCasePrep prep;
        prep.index = i;
        prep.ind   = ind;

        GearDesignPoint dp = ind.toDesignPoint(i, generation);
        checkGearOptDesignPointBounds(dp, _cfg);

        const QString workDir = QStringLiteral("%1/gen%2_id%3")
                                    .arg(_runDir)
                                    .arg(generation)
                                    .arg(i);

        QString reliefReason;
        if (!validateReliefDesign(dp, &reliefReason)) {
            logInvalidReliefDesign(dp, reliefReason);
            dp.status   = PointStatus::Infeasible;
            dp.errorMsg = reliefReason;
            dp.runDir   = workDir;
            dp.runId    = _runId;
            dp.rank     = ind.rank;
            dp.crowdingDistance = ind.crowdingDist;
            dp.isPareto = (ind.rank == 1) ? 1 : 0;
            prep.ind    = ind;
            prep.dp     = dp;
            prep.caseHash = GearOptResultDatabase::caseHash(dp);
            prep.workDir  = workDir;
            prep.skippedByCache = false;
            if (summary)
                summary->cacheHitByIndex[i] = false;
            preps.append(prep);
            continue;
        }

        QDir().mkpath(workDir);
        dp.runDir = workDir;

        dp.applyRunSimDefaults(_cfg, _fixedMeshSizeMm, _runMeshAuto);
        dp.runDir             = workDir;
        dp.runId              = _runId;
        dp.rank               = ind.rank;
        dp.crowdingDistance   = ind.crowdingDist;
        dp.isPareto           = (ind.rank == 1) ? 1 : 0;

        const QString caseH = GearOptResultDatabase::caseHash(dp);

        auto tryLoadCache = [&](GearOptResultDatabase& db, const char* label) -> bool {
            int cachedRowId = -1;
            if (!db.isOpen() || !db.findIdByCaseHash(caseH, &cachedRowId))
                return false;
            qDebug() << "[GearOpt][DB]" << label << "duplicate case id=" << cachedRowId;
            GearDesignPoint cached = dp;
            if (!db.loadResultByCaseHash(caseH, cached))
                return false;
            cached.generation       = generation;
            cached.runDir           = workDir;
            cached.runId            = _runId;
            cached.rank             = ind.rank;
            cached.crowdingDistance = ind.crowdingDist;
            cached.isPareto         = (ind.rank == 1) ? 1 : 0;
            dp                      = cached;
            emit log(QStringLiteral("[GearOpt][DB] %1 cache hit (cpress=%2 MPa)")
                         .arg(QString::fromLatin1(label))
                         .arg(dp.cpressMax_MPa, 0, 'g', 5));
            return true;
        };

        prep.skippedByCache =
            tryLoadCache(GearOptResultDatabase::global(), "global")
            || tryLoadCache(GearOptResultDatabase::runSession(), "run");

        if (summary)
            summary->cacheHitByIndex[i] = prep.skippedByCache;

        if (prep.skippedByCache) {
            if (summary)
                summary->reusedCacheCount++;
        } else if (summary) {
            summary->newCcxCount++;
        }

        prep.dp       = dp;
        prep.caseHash = caseH;
        prep.workDir  = workDir;
        preps.append(prep);
    }

    struct ParallelCcxOutcome {
        int             index = 0;
        GearDesignPoint dp;
        bool            skippedByCache = false;
        qint64          elapsedMs      = 0;
        bool            ccxRan         = false;
    };

    QVector<ParallelCcxOutcome> outcomes(N);

    QVector<int> runQueue;
    runQueue.reserve(preps.size());
    for (const InfillCasePrep& prep : preps) {
        ParallelCcxOutcome out;
        out.index           = prep.index;
        out.dp              = prep.dp;
        out.skippedByCache  = prep.skippedByCache;
        outcomes[prep.index] = out;
        if (!prep.skippedByCache && prep.dp.status != PointStatus::Infeasible)
            runQueue.append(prep.index);
    }

    QHash<int, InfillCasePrep> prepByIndex;
    for (const InfillCasePrep& prep : preps)
        prepByIndex.insert(prep.index, prep);

    const GearOptConfig cfgCopy     = _cfg;
    const QString       runIdCopy   = _runId;
    const double        meshSizeCopy = _fixedMeshSizeMm;
    const double        rootMeshCopy = _fixedRootMeshSizeMm;
    const int           zLayersCopy  = _fixedZLayers;

    int batchNum = 0;
    for (int batchStart = 0; batchStart < runQueue.size() && !_stopRequested; batchStart += jobs) {
        const int batchEnd = std::min(batchStart + jobs, static_cast<int>(runQueue.size()));
        ++batchNum;
        emit log(QStringLiteral("[GearOpt][Parallel] batch %1 started, cases=%2")
                     .arg(batchNum)
                     .arg(batchEnd - batchStart));

        std::vector<std::future<ParallelCcxOutcome>> futures;
        futures.reserve(batchEnd - batchStart);

        for (int qi = batchStart; qi < batchEnd; ++qi) {
            const int idx = runQueue[qi];
            if (!prepByIndex.contains(idx))
                continue;

            const InfillCasePrep prep = prepByIndex.value(idx);
            futures.push_back(std::async(std::launch::async, [prep, cfgCopy, runIdCopy, meshSizeCopy,
                                                              rootMeshCopy, zLayersCopy, threadsPerJob,
                                                              generation]() {
                ParallelCcxOutcome out;
                out.index          = prep.index;
                out.skippedByCache = false;
                out.ccxRan         = true;
                out.dp             = prep.dp;

                const auto t0 = std::chrono::steady_clock::now();
                try {
                    GearOptCaseRunner runner(nullptr);
                    runner.setConfig(cfgCopy);
                    runner.setWorkDir(prep.workDir);
                    GearMeshParams meshParams;
                    meshParams.globalSize = meshSizeCopy;
                    meshParams.rootSize   = rootMeshCopy;
                    meshParams.zLayers    = zLayersCopy;
                    runner.setMeshParams(meshParams);
                    runner.setLogLevel(cfgCopy.solver.debugMode ? GearLogLevel::Debug
                                                                 : GearLogLevel::Normal);
                    runner.setThreads(threadsPerJob);
                    runner.runOne(out.dp);
                    out.dp.fillResultArtifactPaths();
                } catch (const std::exception& ex) {
                    out.dp.status   = PointStatus::Failed;
                    out.dp.errorMsg = QString::fromUtf8(ex.what());
                } catch (...) {
                    out.dp.status   = PointStatus::Failed;
                    out.dp.errorMsg = QStringLiteral("parallel CCX worker unknown exception");
                }
                out.dp.runId            = runIdCopy;
                out.dp.generation       = generation;
                out.dp.runDir           = prep.workDir;
                out.dp.rank             = prep.ind.rank;
                out.dp.crowdingDistance = prep.ind.crowdingDist;
                out.dp.isPareto         = (prep.ind.rank == 1) ? 1 : 0;
                const auto t1 = std::chrono::steady_clock::now();
                out.elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                return out;
            }));
        }

        for (auto& fut : futures) {
            ParallelCcxOutcome out = fut.get();
            outcomes[out.index]      = out;
            const bool converged     = ccxCaseConverged(cfgCopy, out.dp);
            emit log(QStringLiteral("[GearOpt][Parallel] case id=%1 finished converged=%2 time=%3s")
                         .arg(out.index)
                         .arg(converged ? QStringLiteral("true") : QStringLiteral("false"))
                         .arg(out.elapsedMs / 1000));
        }

        emit log(QStringLiteral("[GearOpt][Parallel] batch %1 done").arg(batchNum));
    }

    emit log(QStringLiteral("[GearOpt][Parallel] writing DB in main thread"));

    QList<GearDesignPoint> genPoints;
    std::sort(preps.begin(), preps.end(),
              [](const InfillCasePrep& a, const InfillCasePrep& b) { return a.index < b.index; });

    for (const InfillCasePrep& prep : preps) {
        if (_stopRequested)
            break;

        const int i               = prep.index;
        Individual& ind           = pop[i];
        GearDesignPoint dp        = outcomes[i].dp;
        dp.fillResultArtifactPaths();

        auto insertToDb = [&](GearOptResultDatabase& db) {
            if (!db.isOpen())
                return;
            db.insertDesignPointResult(_runId, dp, generation, i, ind.rank, ind.crowdingDist,
                                       dp.isPareto, prep.caseHash);
        };
        insertToDb(GearOptResultDatabase::global());
        insertToDb(GearOptResultDatabase::runSession());

        if (dp.status == PointStatus::Done && dp.cpressActiveNodes > 0
            && QFileInfo::exists(dp.frdPath)) {
            const CpressDistributionMetrics metrics = parseCpressDistributionMetrics(
                dp.frdPath,
                dp.cpressBinCount > 0 ? dp.cpressBinCount
                                      : (_fixedZLayers > 0 ? _fixedZLayers : 11));
            if (!GearOptResultDatabase::appendCpressWidthBinsCsv(_runDir, _runId, generation, i,
                                                                 prep.caseHash, metrics)) {
                emit log(QStringLiteral(
                             "[GearOpt][CPress] warning: failed to append cpress_width_bins.csv under %1")
                             .arg(_runDir));
            }
        }

        applyCcxResultToIndividual(ind, _cfg, dp, [this](const QString& msg) { emit log(msg); });

        emit pointFinished(generation, i, dp);
        if (pointCallback)
            pointCallback(i, dp, prep.skippedByCache);

        emit log(QString("[gen%1/%2] id=%3 cpress=%4 MPa σ=%5 MPa m=%6 kg status=%7%8")
                     .arg(generation)
                     .arg(_cfg.nsga2.maxGenerations)
                     .arg(i)
                     .arg(dp.cpressMax_MPa, 0, 'g', 5)
                     .arg(dp.sigmaMax, 0, 'g', 5)
                     .arg(dp.mass, 0, 'g', 4)
                     .arg(pointStatusToString(dp.status))
                     .arg(dp.status != PointStatus::Done && !dp.errorMsg.isEmpty()
                              ? QStringLiteral(" | ") + dp.errorMsg.left(160)
                              : prep.skippedByCache ? QStringLiteral(" (cache)") : QString()));

        genPoints.append(dp);
    }
    _allPoints.append(genPoints);

    if (GearOptResultDatabase::global().isOpen()) {
        const int validN = GearOptResultDatabase::global().countValidResults();
        if (validN >= 500 && !_surrogateThresholdLogged) {
            _surrogateThresholdLogged = true;
            emit log(QStringLiteral(
                "[GearOpt][Surrogate] global valid samples >= 500, surrogate can be enabled later."));
        }
    }
}

void GearAutoOptManager::evaluatePopulationBySurrogate(Population& pop,
                                                       int generation,
                                                       const GearSurrogateModel& model)
{
    constexpr double kEdgePenalty = 1.0e6;
    const int N = pop.size();
    for (int i = 0; i < N; ++i) {
        if (_stopRequested)
            break;

        Individual& ind = pop[i];
        if (ind.evaluated)
            continue;

        GearDesignPoint dp = ind.toDesignPoint(i, generation);
        if (ind.vars.size() < VAR_COUNT)
            ind.vars.resize(VAR_COUNT);
        ind.vars[VAR_COMMON_WIDTH] = _fixedCommonWidthMm;
        dp.commonWidth = _fixedCommonWidthMm;
        checkGearOptDesignPointBounds(dp, _cfg);
        dp.applyRunSimDefaults(_cfg, _fixedMeshSizeMm, _runMeshAuto);
        dp.runId = _runId;
        dp.rank = ind.rank;
        dp.crowdingDistance = ind.crowdingDist;
        dp.isPareto = (ind.rank == 1) ? 1 : 0;

        QString reliefReason;
        if (!validateReliefDesign(dp, &reliefReason)) {
            logInvalidReliefDesign(dp, reliefReason);
            ind.objs.clear();
            ind.objs.append(kEdgePenalty);
            ind.objs.append(kEdgePenalty);
            ind.evaluated = true;
            ind.constraintViolation = 1.0;
            if (ind.vars.size() < VAR_COUNT)
                ind.vars.resize(VAR_COUNT);
            ind.vars[VAR_CA1]  = dp.ca1;
            ind.vars[VAR_LCA1] = dp.lca1;
            ind.vars[VAR_CA2]  = dp.ca2;
            ind.vars[VAR_LCA2] = dp.lca2;
            continue;
        }
        if (ind.vars.size() < VAR_COUNT)
            ind.vars.resize(VAR_COUNT);
        ind.vars[VAR_CA1]  = dp.ca1;
        ind.vars[VAR_LCA1] = dp.lca1;
        ind.vars[VAR_CA2]  = dp.ca2;
        ind.vars[VAR_LCA2] = dp.lca2;

        const SurrogatePrediction pred = model.predict(surrogateInputVars(dp));
        ind.objs.clear();
        if (model.isReady()) {
            ind.objs.append(pred.cpressMaxPred);
            if (pred.edgeLoadRatioPred > 0.0 && std::isfinite(pred.edgeLoadRatioPred))
                ind.objs.append(pred.edgeLoadRatioPred);
            else
                ind.objs.append(kEdgePenalty);
        } else {
            ind.objs.append(kEdgePenalty);
            ind.objs.append(kEdgePenalty);
        }
        ind.evaluated = true;
        ind.constraintViolation =
            (ind.objs.size() >= 2 && (ind.objs[0] <= 0.0 || ind.objs[1] >= kEdgePenalty))
                ? 1.0
                : 0.0;

        if (model.isReady() && pred.cpressMaxPred > 0.0) {
            dp.cpressMax_MPa = pred.cpressMaxPred;
            dp.edgeLoadRatio = pred.edgeLoadRatioPred;
            dp.cpressCV = pred.cpressCVPred;
            dp.sigmaMax = pred.sigmaMaxPred;
            dp.uMax = pred.uMaxPred;
            dp.sigmaMax_total = pred.sigmaMaxPred;
            dp.uMax_total = pred.uMaxPred;
            dp.status = PointStatus::Done;
        } else {
            dp.status = PointStatus::Failed;
            dp.errorMsg = QStringLiteral("surrogate prediction unavailable");
        }

        SurrogatePredictionRow predRow;
        predRow.generation = generation;
        predRow.individualId = i;
        predRow.x1 = dp.x1;
        predRow.x2 = dp.x2;
        predRow.ca1 = dp.ca1;
        predRow.lca1 = dp.lca1;
        predRow.ca2 = dp.ca2;
        predRow.lca2 = dp.lca2;
        predRow.hubRatio = dp.hubRatio;
        predRow.predCpressMax_MPa = pred.cpressMaxPred;
        predRow.predEdgeLoadRatio = pred.edgeLoadRatioPred;
        appendSurrogatePredictionsCsv(_runDir, predRow);

        Q_UNUSED(dp);
    }
}

void GearAutoOptManager::start() {
    emit log(QStringLiteral("GearAutoOptManager: start"));
    _stopRequested = false;
    _allPoints.clear();
    logEnabledObjectives(_cfg, [this](const QString& msg) { emit log(msg); });
    // 若 config 指定了结果根目录则覆盖 setRunDir() 传来的值
    if (!_cfg.solver.runBaseDir.isEmpty())
        _runDir = _cfg.solver.runBaseDir;
    QDir().mkpath(_runDir);

    _surrogateThresholdLogged = false;
    _runId = GearOptResultDatabase::generateRunId(_cfg.nsga2.randomSeed);

    auto& globalDb = GearOptResultDatabase::global();
    if (!globalDb.isOpen()) {
        if (!globalDb.openDatabase(GearOptResultDatabase::defaultGlobalDatabasePath())) {
            emit log(QStringLiteral("[GearOpt][DB] warning: global database unavailable"));
        }
    }
    if (globalDb.isOpen()) {
        emit log(QStringLiteral("[Save] global database: %1").arg(globalDb.databasePath()));
        const int backfilled = globalDb.backfillMissingCpressDistributionMetrics(_cfg.solver.meshZLayers > 0 ? _cfg.solver.meshZLayers : 11);
        if (backfilled > 0) {
            emit log(QStringLiteral("[GearOpt][DB] global backfilled cpress distribution metrics: %1")
                         .arg(backfilled));
        }
        emit log(QStringLiteral("[GearOpt][DB] global cached samples: ")
                 + QString::number(globalDb.countValidResults()));
    }

    const QString runDbPath = GearOptResultDatabase::databasePathInRunDir(_runDir);
    auto&         runDb     = GearOptResultDatabase::runSession();
    if (!runDb.openDatabase(runDbPath)) {
        emit log(QStringLiteral("[GearOpt][DB] warning: run database not available"));
    } else {
        emit log(QStringLiteral("[Save] run database: %1").arg(runDb.databasePath()));
        emit log(QStringLiteral("[GearOpt][DB] run_id = ") + _runId);
        const int backfilled = runDb.backfillMissingCpressDistributionMetrics(_cfg.solver.meshZLayers > 0 ? _cfg.solver.meshZLayers : 11);
        if (backfilled > 0) {
            emit log(QStringLiteral("[GearOpt][DB] run backfilled cpress distribution metrics: %1")
                         .arg(backfilled));
        }
    }

    _fixedMeshSizeMm     = _cfg.solver.meshSize;
    _fixedRootMeshSizeMm = _cfg.solver.meshRootSizeMm;
    _fixedZLayers        = _cfg.solver.meshZLayers;
    _runMeshAuto         = _fixedMeshSizeMm <= 0.0;
    qDebug() << "[GearOpt][Config] meshSize_mm =" << _fixedMeshSizeMm;
    qDebug() << "[GearOpt][Config] meshRootSize_mm =" << _fixedRootMeshSizeMm;
    qDebug() << "[GearOpt][Config] meshZLayers =" << _fixedZLayers;
    qDebug() << "[GearOpt][Config] meshAuto =" << (_runMeshAuto ? "true" : "false");
    qDebug() << "[GearOpt][Config] torque_Nm =" << _cfg.solver.torque;
    qDebug() << "[GearOpt][Config] material = STEEL";
    qDebug() << "[GearOpt][Config] contactStiffness =" << _cfg.solver.contactStiffness;
    qDebug() << "[GearOpt][Config] enableContact =" << _cfg.solver.enableContact;
    emit log(QStringLiteral("[GearOpt][Config] meshSize_mm=%1 meshRootSize_mm=%2 meshZLayers=%3 meshAuto=%4 torque_Nm=%5 contactStiffness=%6 enableContact=%7")
                 .arg(_fixedMeshSizeMm, 0, 'g', 6)
                 .arg(_fixedRootMeshSizeMm, 0, 'g', 6)
                 .arg(_fixedZLayers)
                 .arg(_runMeshAuto ? QStringLiteral("true") : QStringLiteral("false"))
                 .arg(_cfg.solver.torque, 0, 'g', 6)
                 .arg(_cfg.solver.contactStiffness, 0, 'g', 6)
                 .arg(_cfg.solver.enableContact ? QStringLiteral("true") : QStringLiteral("false")));

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
    syncGenerationParetoToDatabase(0, _population);

    QVector<double> hvHistory;
    const double refCpress = _cfg.constraints.sigmaAllow * 2.0;
    const double refSigma  = _cfg.constraints.sigmaAllow * 2.0;
    Population pareto0 = extractPareto(_population);
    double hv0 = hypervolume2D(pareto0, refCpress, refSigma);
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

        auto frontsG = fastNonDominatedSort(_population);
        for (const auto& front : frontsG)
            assignCrowdingDistance(_population, front);
        syncGenerationParetoToDatabase(g, _population);

        Population pareto = extractPareto(_population);
        const double hv = hypervolume2D(pareto, refCpress, refSigma);
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

    syncFinalParetoToDatabase();
    GearOptResultDatabase::runSession().closeDatabase();
    emit log(QStringLiteral("primary_objective = cpressMax_MPa"));
    emit log(QStringLiteral("secondary_objective = edgeLoadRatio"));
    emit log(QStringLiteral("recorded_metrics = sigmaMax_MPa, uMax_mm, mass_kg, cpressCV"));
    emit finished(!_stopRequested);
}

void GearAutoOptManager::startSurrogateAssisted()
{
    emit log(QStringLiteral("GearAutoOptManager: startSurrogateAssisted"));
    _stopRequested = false;
    _allPoints.clear();
    logSurrogateFixedObjectives([this](const QString& msg) { emit log(msg); });
    _surrogateThresholdLogged = false;
    _runId = GearOptResultDatabase::generateRunId(_cfg.nsga2.randomSeed);
    _fixedMeshSizeMm     = _cfg.solver.meshSize;
    _fixedRootMeshSizeMm = _cfg.solver.meshRootSizeMm;
    _fixedZLayers        = _cfg.solver.meshZLayers;
    _runMeshAuto = _fixedMeshSizeMm <= 0.0;
    openResultDatabases();

    auto& globalDb = GearOptResultDatabase::global();
    if (!globalDb.isOpen()) {
        emit finished(false);
        return;
    }

    const int N = _cfg.nsga2.populationSize;
    const int Gmax = _cfg.nsga2.maxGenerations;
    const int minSamples = minSurrogateSampleCount(N);
    const int infillCount = std::min(5, std::max(3, std::max(1, N / 10)));
    constexpr double kRmaeStopEps = 0.05;
    const double hvFallbackCpress = _cfg.constraints.sigmaAllow * 2.0;
    constexpr double hvFallbackEdge = 2.0;
    constexpr double kHvRefMargin = 1.05;

    const GearDesignPoint baseDp = configuredBasePoint();
    _fixedCommonWidthMm = baseDp.commonWidth;
    const QString baseCaseH = GearOptResultDatabase::baseCaseHash(baseDp);
    emit log(QStringLiteral("[GearOpt][Surrogate] base_case_hash=%1").arg(baseCaseH));
    emit log(QStringLiteral("[GearOpt][Surrogate] base case: m=%1 z1=%2 z2=%3 alpha=%4 width=%5 mm torque=%6 N·m meshAuto=%7 meshSize=%8 contact=%9")
                 .arg(baseDp.module, 0, 'g', 6)
                 .arg(baseDp.z1)
                 .arg(baseDp.z2)
                 .arg(baseDp.alpha, 0, 'g', 6)
                 .arg(_fixedCommonWidthMm, 0, 'g', 6)
                 .arg(baseDp.torque_Nm, 0, 'g', 6)
                 .arg(baseDp.meshAuto ? QStringLiteral("true") : QStringLiteral("false"))
                 .arg(baseDp.meshSize_mm, 0, 'g', 6)
                 .arg(baseDp.enableContact ? QStringLiteral("on") : QStringLiteral("off")));
    emit log(QStringLiteral("[GearOpt][Surrogate] RBF input dim=%1 (width fixed, not in surrogate vars)")
                 .arg(kSurrogateInputDim));
    emit log(QStringLiteral("[GearOpt][Surrogate] min CCX samples for RBF = %1 (same baseCase + width)")
                 .arg(minSamples));

    const int globalTotal = globalDb.countValidResults();
    const int baseCountGlobal =
        globalDb.countValidatedSamplesForBaseCase(baseCaseH, _fixedCommonWidthMm);
    emit log(QStringLiteral("[GearOpt][Surrogate] DB total done=%1, same baseCase+width verified=%2")
                 .arg(globalTotal)
                 .arg(baseCountGlobal));

    QVector<SurrogateSample> samples =
        loadMergedValidatedSamples(baseCaseH, _fixedCommonWidthMm, _cfg);
    emit log(QStringLiteral("[GearOpt][Surrogate] merged validated samples (width=%1 mm): %2")
                 .arg(_fixedCommonWidthMm, 0, 'g', 6)
                 .arg(samples.size()));

    auto supplementValidatedSamples = [&](int generationBase) -> bool {
        constexpr int kMaxAttempts = 20;
        constexpr int kMaxStagnantAttempts = 6;
        int stagnantAttempts = 0;

        for (int attempt = 1;
             samples.size() < minSamples && attempt <= kMaxAttempts && !_stopRequested;
             ++attempt) {
            const int before = samples.size();
            const int need = minSamples - before;
            const int batchN = std::max(
                need, std::min(std::max(1, N), std::max(3, need * 2)));
            const int seed = _cfg.nsga2.randomSeed
                             + generationBase * 1009
                             + attempt * 7919;

            emit log(QStringLiteral("[GearOpt][Surrogate] LHS supplement attempt=%1: samples=%2/%3, request=%4")
                         .arg(attempt)
                         .arg(before)
                         .arg(minSamples)
                         .arg(batchN));

            _population = initLatin(batchN, _cfg, seed);
            fixPopulationCommonWidth(_population, _fixedCommonWidthMm);

            CcxEvalSummary supplementStats;
            evaluatePopulationByCcx(_population, generationBase + attempt, &supplementStats);
            if (_stopRequested)
                return false;

            samples = loadMergedValidatedSamples(baseCaseH, _fixedCommonWidthMm, _cfg);
            const int newValidated = std::max(0, samples.size() - before);
            const int duplicateOrFailed = std::max(0, batchN - newValidated - supplementStats.reusedCacheCount);
            emit log(QStringLiteral("[GearOpt][Surrogate] LHS supplement result: new_validated=%1 cache_hit=%2 duplicate_or_failed=%3 total=%4/%5")
                         .arg(newValidated)
                         .arg(supplementStats.reusedCacheCount)
                         .arg(duplicateOrFailed)
                         .arg(samples.size())
                         .arg(minSamples));

            if (newValidated <= 0)
                ++stagnantAttempts;
            else
                stagnantAttempts = 0;

            if (stagnantAttempts >= kMaxStagnantAttempts) {
                emit log(QStringLiteral("[GearOpt][Surrogate] LHS supplement stopped: no new validated samples in %1 consecutive attempts")
                             .arg(kMaxStagnantAttempts));
                break;
            }
        }

        return samples.size() >= minSamples;
    };

    if (samples.size() < minSamples) {
        emit log(QStringLiteral("[GearOpt][Surrogate] samples %1 < %2, LHS+CCX will supplement until RBF minimum is satisfied")
                     .arg(samples.size())
                     .arg(minSamples));
        if (!supplementValidatedSamples(0)) {
            GearOptResultDatabase::runSession().closeDatabase();
            emit finished(false);
            return;
        }
        if (samples.size() < minSamples) {
            emit log(QStringLiteral("[GearOpt][Surrogate] still below RBF minimum (%1/%2); surrogate training is skipped")
                         .arg(samples.size())
                         .arg(minSamples));
            GearOptResultDatabase::runSession().closeDatabase();
            emit finished(false);
            return;
        }
    }

    for (int round = 1; round <= std::max(1, Gmax) && !_stopRequested; ++round) {
        if (samples.size() < minSamples) {
            emit log(QStringLiteral("[GearOpt][Surrogate] round %1 skipped training: samples=%2 < minSamplesForRBF=%3")
                         .arg(round)
                         .arg(samples.size())
                         .arg(minSamples));
            if (!supplementValidatedSamples(round * 1000)) {
                emit log(QStringLiteral("[GearOpt][Surrogate] round %1 terminated before RBF training due to insufficient validated samples")
                             .arg(round));
                break;
            }
        }
        if (samples.size() < minSamples) {
            emit log(QStringLiteral("[GearOpt][Surrogate] RBF training is not allowed with %1/%2 validated samples")
                         .arg(samples.size())
                         .arg(minSamples));
            break;
        }

        GearSurrogateModel model;
        if (!model.train(samples)) {
            emit log(QStringLiteral("[GearOpt][Surrogate] model training failed; need more valid cpressMax samples"));
            break;
        }

        emit log(QStringLiteral("[GearOpt][Surrogate] round %1: RBF trained with %2 samples")
                     .arg(round)
                     .arg(samples.size()));
        {
            double edgeMin = samples.first().edgeLoadRatio;
            double edgeMax = samples.first().edgeLoadRatio;
            for (const SurrogateSample& s : samples) {
                edgeMin = std::min(edgeMin, s.edgeLoadRatio);
                edgeMax = std::max(edgeMax, s.edgeLoadRatio);
            }
            const double looEdge = model.trainingMaxRelativeError(QStringLiteral("edgeLoadRatio"));
            const double looCpress = model.trainingMaxRelativeError(QStringLiteral("cpressMax_MPa"));
            emit log(QStringLiteral("[GearOpt][Surrogate] round %1: edgeLoadRatio train range [%2, %3], LOO max rel err edge=%4% cpress=%5%")
                         .arg(round)
                         .arg(edgeMin, 0, 'g', 6)
                         .arg(edgeMax, 0, 'g', 6)
                         .arg(looEdge >= 0.0 ? looEdge * 100.0 : -1.0, 0, 'g', 4)
                         .arg(looCpress >= 0.0 ? looCpress * 100.0 : -1.0, 0, 'g', 4));
            if (looEdge >= 0.10) {
                emit log(QStringLiteral(
                    "[Surrogate] warning: edgeLoadRatio surrogate error is high; final ranking requires CCX validation."));
            }
        }

        const QVector<double> sampleResiduals =
            computeSurrogateSampleResidualsFixed(samples, model);
        InfillScoringConfig infillScoring;
        infillScoring.alpha = 0.6;
        infillScoring.beta = 0.4;
        infillScoring.neighborK = 3;
        infillScoring.minDesignDistNorm = 1e-4;

        std::mt19937 rng(static_cast<unsigned>(_cfg.nsga2.randomSeed + round));
        Population surrogatePop = initLatin(N, _cfg, _cfg.nsga2.randomSeed + round);
        fixPopulationCommonWidth(surrogatePop, _fixedCommonWidthMm);
        evaluatePopulationBySurrogate(surrogatePop, round, model);
        auto fronts0 = fastNonDominatedSort(surrogatePop);
        for (const auto& front : fronts0)
            assignCrowdingDistance(surrogatePop, front);

        for (int g = 1; g <= std::max(1, Gmax) && !_stopRequested; ++g) {
            Population offspring = evolve(surrogatePop, _cfg, rng);
            fixPopulationCommonWidth(offspring, _fixedCommonWidthMm);
            evaluatePopulationBySurrogate(offspring, round * 100 + g, model);
            Population combined = surrogatePop + offspring;
            surrogatePop = selectNextGen(combined, N);
            fixPopulationCommonWidth(surrogatePop, _fixedCommonWidthMm);
        }

        auto fronts = fastNonDominatedSort(surrogatePop);
        for (const auto& front : fronts)
            assignCrowdingDistance(surrogatePop, front);
        const Population surrogatePareto = extractPareto(surrogatePop);
        emit log(QStringLiteral("[GearOpt][Surrogate] round %1: surrogate Pareto size=%2")
                     .arg(round)
                     .arg(surrogatePareto.size()));

        const auto predParetoMax = paretoObjectiveMax(surrogatePareto);
        const HypervolumeReference2D predHvRef = computeDynamicHypervolumeReference(
            samples, predParetoMax.first, predParetoMax.second, kHvRefMargin,
            hvFallbackCpress, hvFallbackEdge);
        const double predHv =
            hypervolume2D(surrogatePareto, predHvRef.refCpressMax, predHvRef.refEdgeLoadRatio);
        emit log(QStringLiteral("[Surrogate] HV ref round=%1 ref_cpress=%2 ref_edge=%3 (max*1.05)")
                     .arg(round)
                     .arg(predHvRef.refCpressMax, 0, 'g', 8)
                     .arg(predHvRef.refEdgeLoadRatio, 0, 'g', 8));
        const int predParetoSize = surrogatePareto.size();
        SurrogateHvRow hvRow;
        hvRow.round      = round;
        hvRow.predHv     = predHv;
        hvRow.predPareto = predParetoSize;
        if (!appendSurrogateHvCsv(_runDir, hvRow)) {
            emit log(QStringLiteral("[Surrogate] failed to append surrogate_metrics.csv under %1")
                         .arg(_runDir));
        } else {
            emit log(QStringLiteral("[Surrogate] wrote surrogate_metrics.csv round=%1 PredHV=%2 PredPareto=%3")
                         .arg(round)
                         .arg(predHv, 0, 'g', 4)
                         .arg(predParetoSize));
        }

        QSet<QString> knownCaseHashes;
        QSet<QString> knownDesignHashes;
        auto mergeKnownHashes = [&](GearOptResultDatabase& db) {
            if (db.isOpen()) {
                knownCaseHashes.unite(
                    db.knownCaseHashesForBaseCase(baseCaseH, _fixedCommonWidthMm));
                knownDesignHashes.unite(
                    db.knownDesignHashesForBaseCase(baseCaseH, _fixedCommonWidthMm));
            }
        };
        mergeKnownHashes(GearOptResultDatabase::global());
        mergeKnownHashes(GearOptResultDatabase::runSession());
        for (const SurrogateSample& s : samples) {
            if (!s.caseHash.isEmpty())
                knownCaseHashes.insert(s.caseHash);
            if (!s.designHash.isEmpty())
                knownDesignHashes.insert(s.designHash);
        }

        auto caseHashForIndividual = [&](const Individual& ind) -> QString {
            GearDesignPoint dp = ind.toDesignPoint();
            if (_cfg.useOptimizationBase) {
                dp.module = baseDp.module;
                dp.z1     = baseDp.z1;
                dp.z2     = baseDp.z2;
                dp.alpha  = baseDp.alpha;
            }
            dp.commonWidth = _fixedCommonWidthMm;
            applySurrogateInputVars(dp, surrogateInputVars(dp));
            checkGearOptDesignPointBounds(dp, _cfg);
            dp.applyRunSimDefaults(_cfg, _fixedMeshSizeMm, _runMeshAuto);
            return GearOptResultDatabase::caseHash(dp);
        };

        Population infill = GearInfillSelector::selectSparseParetoPoints(
            surrogatePareto, samples, sampleResiduals, infillScoring, _cfg, baseDp,
            _fixedMeshSizeMm, _runMeshAuto, knownCaseHashes, knownDesignHashes, infillCount);
        if (infill.size() < infillCount) {
            QSet<QString> fallbackKnownHashes = knownCaseHashes;
            QSet<QString> fallbackKnownDesignHashes = knownDesignHashes;
            for (const Individual& ind : infill)
                fallbackKnownHashes.insert(caseHashForIndividual(ind));

            const int remaining = infillCount - infill.size();
            Population fallback = GearInfillSelector::selectSparseParetoPoints(
                surrogatePop, samples, sampleResiduals, infillScoring, _cfg, baseDp,
                _fixedMeshSizeMm, _runMeshAuto, fallbackKnownHashes, fallbackKnownDesignHashes,
                remaining);
            if (!fallback.isEmpty()) {
                emit log(QStringLiteral("[GearOpt][Surrogate] round %1: Pareto infill selected=%2, sparse fallback added=%3")
                             .arg(round)
                             .arg(infill.size())
                             .arg(fallback.size()));
                infill += fallback;
            }
        }

        {
            Population validInfill;
            validInfill.reserve(infill.size());
            for (const Individual& ind : infill) {
                GearDesignPoint dp = ind.toDesignPoint();
                if (_cfg.useOptimizationBase) {
                    dp.module = baseDp.module;
                    dp.z1     = baseDp.z1;
                    dp.z2     = baseDp.z2;
                    dp.alpha  = baseDp.alpha;
                }
                dp.commonWidth = _fixedCommonWidthMm;
                applySurrogateInputVars(dp, surrogateInputVars(dp));
                QString reliefReason;
                if (!validateReliefDesign(dp, &reliefReason)) {
                    logInvalidReliefDesign(dp, reliefReason);
                    continue;
                }
                validInfill.append(ind);
            }
            if (validInfill.size() != infill.size()) {
                emit log(QStringLiteral("[GearOpt][Surrogate] round %1: filtered %2 invalid relief infill candidates")
                             .arg(round)
                             .arg(infill.size() - validInfill.size()));
            }
            infill = validInfill;
        }

        if (infill.isEmpty()) {
            emit log(QStringLiteral("[GearOpt][Surrogate] no non-duplicate sparse infill points; stop"));
            break;
        }
        if (infill.size() < std::min(3, infillCount)) {
            emit log(QStringLiteral("[GearOpt][Surrogate] warning: infill selected only %1 points after sparse fallback")
                         .arg(infill.size()));
        }
        const int infillSelected = infill.size();
        emit log(QStringLiteral("[GearOpt][Surrogate] round %1: infill selected=%2 (target %3)")
                     .arg(round)
                     .arg(infillSelected)
                     .arg(infillCount));
        for (int ii = 0; ii < infill.size(); ++ii) {
            GearDesignPoint dp = infill[ii].toDesignPoint();
            if (_cfg.useOptimizationBase) {
                dp.module = baseDp.module;
                dp.z1     = baseDp.z1;
                dp.z2     = baseDp.z2;
                dp.alpha  = baseDp.alpha;
            }
            dp.commonWidth = _fixedCommonWidthMm;
            applySurrogateInputVars(dp, surrogateInputVars(dp));
            checkGearOptDesignPointBounds(dp, _cfg);
            dp.applyRunSimDefaults(_cfg, _fixedMeshSizeMm, _runMeshAuto);
            emit log(QStringLiteral("[Infill] #%1 %2 case_hash=%3")
                         .arg(ii)
                         .arg(GearInfillSelector::formatDesignVarsForLog(dp))
                         .arg(GearOptResultDatabase::caseHash(dp)));
        }

        QVector<double> predCpress;
        QVector<double> predEdge;
        QVector<double> predSigma;
        QVector<double> predU;
        for (Individual& ind : infill) {
            GearDesignPoint dp = ind.toDesignPoint();
            if (_cfg.useOptimizationBase) {
                dp.module = baseDp.module;
                dp.z1     = baseDp.z1;
                dp.z2     = baseDp.z2;
                dp.alpha  = baseDp.alpha;
            }
            dp.commonWidth = _fixedCommonWidthMm;
            applySurrogateInputVars(dp, surrogateInputVars(dp));
            checkGearOptDesignPointBounds(dp, _cfg);
            const SurrogatePrediction pred = model.predict(surrogateInputVars(dp));
            predCpress.append(pred.cpressMaxPred);
            predEdge.append(pred.edgeLoadRatioPred);
            predSigma.append(pred.sigmaMaxPred);
            predU.append(pred.uMaxPred);
            ind.evaluated = false;
            ind.objs.clear();
        }

        CcxEvalSummary ccxStats;
        QVector<double> predCpressAll;
        QVector<double> predCpressNew;
        QVector<double> predSigmaAll;
        QVector<double> predSigmaNew;
        QVector<double> predUAll;
        QVector<double> trueCpressAll;
        QVector<double> trueCpressNew;
        QVector<double> trueSigmaAll;
        QVector<double> trueSigmaNew;
        QVector<double> trueUAll;

        auto appendPredVsTrue = [&](int pointIndex, const GearDesignPoint& dp, bool cached) {
            Q_UNUSED(cached);
            if (pointIndex < 0 || pointIndex >= predCpress.size())
                return;
            if (!ccxCaseConverged(_cfg, dp))
                return;
            if (dp.status != PointStatus::Done || dp.cpressMax_MPa <= 0.0)
                return;

            predCpressAll.append(predCpress[pointIndex]);
            trueCpressAll.append(dp.cpressMax_MPa);
            predSigmaAll.append(predSigma[pointIndex]);
            trueSigmaAll.append(dp.sigmaMax);
            predUAll.append(predU[pointIndex]);
            trueUAll.append(dp.uMax);

            const double cpressErr = dp.cpressMax_MPa > 1e-12
                                         ? std::abs(predCpress[pointIndex] - dp.cpressMax_MPa)
                                               / std::abs(dp.cpressMax_MPa)
                                         : -1.0;
            const double predEdgeVal = pointIndex < predEdge.size() ? predEdge[pointIndex] : -1.0;
            const double edgeErr =
                dp.edgeLoadRatio > 1e-12 && predEdgeVal >= 0.0
                    ? std::abs(predEdgeVal - dp.edgeLoadRatio) / std::abs(dp.edgeLoadRatio)
                    : -1.0;

            SurrogateInfillValidationRow row;
            row.round = round;
            row.id = pointIndex;
            row.caseHash = GearOptResultDatabase::caseHash(dp);
            row.predCpressMax_MPa = predCpress[pointIndex];
            row.trueCpressMax_MPa = dp.cpressMax_MPa;
            row.cpressErrorPercent = cpressErr >= 0.0 ? cpressErr * 100.0 : -1.0;
            row.predEdgeLoadRatio = predEdgeVal;
            row.trueEdgeLoadRatio = dp.edgeLoadRatio;
            row.edgeLoadRatioErrorPercent = edgeErr >= 0.0 ? edgeErr * 100.0 : -1.0;
            row.trueSigmaMax_MPa = dp.sigmaMax;
            row.trueUMax_mm = dp.uMax;
            row.trueMass_kg = dp.mass;
            row.trueCpressCV = dp.cpressCV;
            if (!appendSurrogateInfillValidationCsv(_runDir, QVector<SurrogateInfillValidationRow>{row})) {
                emit log(QStringLiteral("[Surrogate] failed to append pred_vs_true.csv under %1")
                             .arg(_runDir));
            }
            emit log(QStringLiteral("[Surrogate][PredTrue] id=%1 pred_cpress=%2 true_cpress=%3 pred_edgeRatio=%4 true_edgeRatio=%5 cpress_error=%6% edgeRatio_error=%7%")
                         .arg(pointIndex)
                         .arg(predCpress[pointIndex], 0, 'g', 8)
                         .arg(dp.cpressMax_MPa, 0, 'g', 8)
                         .arg(predEdgeVal, 0, 'g', 8)
                         .arg(dp.edgeLoadRatio, 0, 'g', 8)
                         .arg(row.cpressErrorPercent >= 0.0 ? row.cpressErrorPercent : -1.0, 0, 'g', 6)
                         .arg(row.edgeLoadRatioErrorPercent >= 0.0 ? row.edgeLoadRatioErrorPercent : -1.0, 0, 'g', 6));

            if (!cached) {
                predCpressNew.append(predCpress[pointIndex]);
                trueCpressNew.append(dp.cpressMax_MPa);
                if (predSigma[pointIndex] > 0.0 && dp.sigmaMax > 0.0) {
                    predSigmaNew.append(predSigma[pointIndex]);
                    trueSigmaNew.append(dp.sigmaMax);
                }
            }
        };

        fixPopulationCommonWidth(infill, _fixedCommonWidthMm);
        emit log(QStringLiteral("[GearOpt][Surrogate] round %1: validating %2 sparse points with CCX")
                     .arg(round)
                     .arg(infill.size()));
        evaluateInfillByCcx(infill, 1000 + round, &ccxStats, appendPredVsTrue);
        if (_stopRequested)
            break;

        const double rmaeCpressAll = computeRmae(predCpressAll, trueCpressAll);
        const double rmaeCpressNew = computeRmae(predCpressNew, trueCpressNew);
        const double rmaeSigma     = computeRmae(predSigmaAll, trueSigmaAll);
        const double rmaeSigmaNew  = computeRmae(predSigmaNew, trueSigmaNew);
        const double rmaeU         = computeRmae(predUAll, trueUAll);

        samples = loadMergedValidatedSamples(baseCaseH, _fixedCommonWidthMm, _cfg);
        logBestRealSoFar(samples, round, _cfg, [this](const QString& msg) { emit log(msg); });

        const Population ccxPop = populationFromValidatedSamples(samples, _cfg);
        const Population ccxPareto = extractPareto(ccxPop);
        const auto ccxParetoMax = paretoObjectiveMax(ccxPareto);
        const HypervolumeReference2D ccxHvRef = computeDynamicHypervolumeReference(
            samples, ccxParetoMax.first, ccxParetoMax.second, kHvRefMargin,
            hvFallbackCpress, hvFallbackEdge);
        const double ccxHv =
            hypervolume2D(ccxPareto, ccxHvRef.refCpressMax, ccxHvRef.refEdgeLoadRatio);
        const int ccxSampleCount = samples.size();

        QString stopReason;
        if (_stopRequested)
            stopReason = QStringLiteral("user_stop");
        else if (ccxStats.newCcxCount > 0 && rmaeCpressNew >= 0.0 && rmaeSigmaNew >= 0.0
                 && rmaeCpressNew < kRmaeStopEps && rmaeSigmaNew < kRmaeStopEps)
            stopReason = QStringLiteral("rmae_threshold");
        else if (round >= std::max(1, Gmax))
            stopReason = QStringLiteral("max_generations");

        CcxHvRow ccxRow;
        ccxRow.round      = round;
        ccxRow.ccxSamples = ccxSampleCount;
        ccxRow.ccxHv      = ccxHv;
        if (!appendCcxHvCsv(_runDir, ccxRow)) {
            emit log(QStringLiteral("[Surrogate] failed to append ccx_metrics.csv under %1")
                         .arg(_runDir));
        }

        SurrogateRmaeRow rmaeRow;
        rmaeRow.round            = round;
        rmaeRow.sampleCount      = ccxSampleCount;
        rmaeRow.infillSelected   = infillSelected;
        rmaeRow.newCcxCount      = ccxStats.newCcxCount;
        rmaeRow.reusedCacheCount = ccxStats.reusedCacheCount;
        rmaeRow.rmaeCpressAll    = rmaeCpressAll;
        rmaeRow.rmaeCpressNew    = rmaeCpressNew;
        rmaeRow.rmaeSigma        = rmaeSigma;
        rmaeRow.rmaeUmax         = rmaeU;
        rmaeRow.stopReason       = stopReason;
        rmaeRow.createdAt        = QDateTime::currentDateTime();
        appendSurrogateRmaeCsv(_runDir, rmaeRow);

        const auto fmtRmaePct = [](double rmae) -> QString {
            if (rmae < 0.0)
                return QStringLiteral("N/A");
            return QStringLiteral("%1%").arg(rmae * 100.0, 0, 'f', 1);
        };
        emit log(QStringLiteral("[Surrogate] round=%1 samples=%2 infill=%3 newCCX=%4 reusedCache=%5")
                     .arg(round)
                     .arg(ccxSampleCount)
                     .arg(infillSelected)
                     .arg(ccxStats.newCcxCount)
                     .arg(ccxStats.reusedCacheCount));
        emit log(QStringLiteral("[Surrogate] RMAE_cpress_all=%1 RMAE_cpress_new=%2 RMAE_sigma_all=%3 RMAE_sigma_new=%4")
                     .arg(fmtRmaePct(rmaeCpressAll))
                     .arg(fmtRmaePct(rmaeCpressNew))
                     .arg(fmtRmaePct(rmaeSigma))
                     .arg(fmtRmaePct(rmaeSigmaNew)));
        emit log(QStringLiteral("[Surrogate] HV round=%1 PredHV=%2 PredPareto=%3 ref_cpress=%4 ref_edge=%5 | CCXSamples=%6 CCXHV=%7")
                     .arg(round)
                     .arg(predHv, 0, 'g', 4)
                     .arg(predParetoSize)
                     .arg(predHvRef.refCpressMax, 0, 'g', 6)
                     .arg(predHvRef.refEdgeLoadRatio, 0, 'g', 6)
                     .arg(ccxSampleCount)
                     .arg(ccxHv, 0, 'g', 4));

        if (ccxStats.newCcxCount > 0 && rmaeCpressNew >= 0.0 && rmaeSigmaNew >= 0.0
            && rmaeCpressNew < kRmaeStopEps && rmaeSigmaNew < kRmaeStopEps) {
            emit log(QStringLiteral("[GearOpt][Surrogate] RMAE (new CCX) below threshold; stop"));
            break;
        }

        emit generationFinished(round, predParetoSize, predHv);
    }

    syncFinalParetoToDatabase();
    GearOptResultDatabase::runSession().closeDatabase();
    emit log(QStringLiteral("primary_objective = cpressMax_MPa"));
    emit log(QStringLiteral("secondary_objective = edgeLoadRatio"));
    emit log(QStringLiteral("recorded_metrics = sigmaMax_MPa, uMax_mm, mass_kg, cpressCV"));
    emit finished(!_stopRequested);
}

void GearAutoOptManager::syncGenerationParetoToDatabase(int generation, const Population& pop)
{
    auto updateDb = [&](GearOptResultDatabase& db) {
        if (!db.isOpen())
            return;
        for (int i = 0; i < pop.size(); ++i) {
            const int isPareto = (pop[i].rank == 1) ? 1 : 0;
            db.updateParetoFlag(_runId, generation, i, isPareto, pop[i].rank, pop[i].crowdingDist);
        }
    };
    updateDb(GearOptResultDatabase::runSession());
    updateDb(GearOptResultDatabase::global());
}

void GearAutoOptManager::syncFinalParetoToDatabase()
{
    const QList<GearDesignPoint> pareto = paretoFront();
    QSet<QString> keys;
    for (const GearDesignPoint& p : pareto) {
        keys.insert(QStringLiteral("%1_%2").arg(p.generation).arg(p.id));
    }
    for (const QList<GearDesignPoint>& gen : _allPoints) {
        for (const GearDesignPoint& dp : gen) {
            const bool onFront =
                keys.contains(QStringLiteral("%1_%2").arg(dp.generation).arg(dp.id));
            auto updateOne = [&](GearOptResultDatabase& db) {
                if (!db.isOpen())
                    return;
                db.updateParetoFlag(_runId, dp.generation, dp.id, onFront ? 1 : 0, dp.rank,
                                    dp.crowdingDistance);
            };
            updateOne(GearOptResultDatabase::runSession());
            updateOne(GearOptResultDatabase::global());
        }
    }
}

} // namespace GearAutoOpt
