// UTF-8 BOM
#include "GearAutoOptManager.h"
#include "GearOptCaseRunner.h"
#include "GearOptMainThreadRunner.h"

#include "GearAutoOpt/data/GearLogLevel.h"
#include "GearAutoOpt/data/GearOptGeometryBridge.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"
#include "GearAutoOpt/solver/CCXResultParser.h"
#include "GearAutoOpt/surrogate/GearInfillSelector.h"
#include "GearAutoOpt/surrogate/GearSurrogateMetrics.h"
#include "GearAutoOpt/surrogate/GearSurrogateSampleValidation.h"
#include "GearSurrogateVerboseLog.h"

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

static void connectMainThreadRunnerLogs(GearAutoOptManager* mgr)
{
	if (GearOptMainThreadRunner* mt = GearOptMainThreadRunner::instance()) {
		QObject::connect(mt, &GearOptMainThreadRunner::logMessage, mgr, &GearAutoOptManager::log,
		                 Qt::QueuedConnection);
	}
}

GearDesignPoint GearAutoOptManager::configuredBasePoint() const
{
    GearDesignPoint dp = _cfg.useOptimizationBase ? _cfg.optimizationBase : GearDesignPoint();
    dp.applyRunSimDefaults(_cfg, _fixedMeshSizeMm, _runMeshAuto);
    return dp;
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
                                                           const GearOptConfig& cfg,
                                                           SurrogateSampleLoadStats* mergedStats = nullptr)
{
    QHash<QString, SurrogateSample> uniq;
    SurrogateSampleLoadStats globalStats;
    SurrogateSampleLoadStats runStats;
    int duplicateSkipped = 0;
    const QStringList targets = cfg.effectiveSurrogateTargets();
    auto ingestDb = [&](GearOptResultDatabase& db, SurrogateSampleLoadStats* dbStats) {
        if (!db.isOpen())
            return;
        for (const SurrogateSample& s :
             db.loadValidatedSamples(baseCaseH, fixedWidthMm, 0, dbStats, targets)) {
            if (!surrogateTrainingValid(s, targets))
                continue;
            const QString key = !s.caseHash.isEmpty()
                                    ? s.caseHash
                                    : (!s.designHash.isEmpty()
                                           ? s.designHash
                                           : QStringLiteral("idx_%1").arg(uniq.size()));
            if (uniq.contains(key))
                ++duplicateSkipped;
            uniq.insert(key, s);
        }
    };
    ingestDb(GearOptResultDatabase::global(), &globalStats);
    ingestDb(GearOptResultDatabase::runSession(), &runStats);
    if (mergedStats) {
        mergedStats->totalSamples     = globalStats.totalSamples + runStats.totalSamples;
        mergedStats->validSamples     = uniq.size();
        mergedStats->failedSkipped    = std::max(0, mergedStats->totalSamples - mergedStats->validSamples);
        mergedStats->duplicateSkipped = duplicateSkipped;
    }
    return uniq.values().toVector();
}

static QList<GearDesignPoint> designPointsFromValidatedSamples(
    const QVector<SurrogateSample>& samples,
    const GearDesignPoint& baseDp,
    const GearOptConfig& cfg,
    double fixedWidthMm)
{
    QList<GearDesignPoint> out;
    out.reserve(samples.size());
    int idx = 0;
    const QStringList targets = cfg.effectiveSurrogateTargets();
    for (const SurrogateSample& s : samples) {
        if (!surrogateTrainingValid(s, targets))
            continue;
        GearDesignPoint dp = baseDp;
        if (!s.x.isEmpty())
            applySurrogateInputVars(dp, s.x);
        dp.commonWidth     = fixedWidthMm;
        dp.cpressMax_MPa   = s.cpressMax;
        dp.edgeLoadRatio   = s.edgeLoadRatio;
        dp.cpressCV        = s.cpressCV;
        dp.sigmaMax        = s.sigmaMax;
        dp.uMax            = s.uMax;
        dp.mass            = s.mass;
        dp.status          = PointStatus::Done;
        dp.generation      = 0;
        dp.id              = idx++;
        out.append(dp);
    }
    return out;
}

static void refreshSurrogateValidatedCache(QList<GearDesignPoint>& cache,
                                           const QVector<SurrogateSample>& samples,
                                           const GearDesignPoint& baseDp,
                                           const GearOptConfig& cfg,
                                           double fixedWidthMm)
{
    cache = designPointsFromValidatedSamples(samples, baseDp, cfg, fixedWidthMm);
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

static QList<GearDesignPoint> paretoFrontFromDesignPoints(const QList<GearDesignPoint>& all,
                                                          const Objectives& obj)
{
    QList<GearDesignPoint> pareto;
    for (const GearDesignPoint& a : all) {
        if (a.status != PointStatus::Done || a.cpressMax_MPa <= 0.0)
            continue;
        if (obj.minSigmaMax && a.sigmaMax <= 0.0)
            continue;
        bool dominated = false;
        for (const GearDesignPoint& b : all) {
            if (dominatesByObjectives(a, b, obj)) {
                dominated = true;
                break;
            }
        }
        if (!dominated)
            pareto.append(a);
    }
    return pareto;
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
    emitLog(QStringLiteral("[GearOpt][Surrogate] targets: %1")
                .arg(cfg.effectiveSurrogateTargets().join(QStringLiteral(", "))));
}

static Population populationFromValidatedSamples(const QVector<SurrogateSample>& samples,
                                                 const GearOptConfig& cfg)
{
    Population pop;
    pop.reserve(samples.size());
    const QStringList targets = cfg.effectiveSurrogateTargets();
    for (const SurrogateSample& s : samples) {
        if (!surrogateTrainingValid(s, targets))
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
    const QStringList targets = cfg.effectiveSurrogateTargets();
    for (const SurrogateSample& s : samples) {
        if (!surrogateTrainingValid(s, targets))
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
    if (!_surrogateValidatedPoints.isEmpty())
        return paretoFrontFromDesignPoints(_surrogateValidatedPoints, _cfg.objectives);

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
    const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback,
    const SurrogateCcxLogOptions* ccxLog) {
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

        const QString workDir = gearOptCaseWorkDir(_runDir, generation, i);
        QDir().mkpath(workDir);
        dp.runDir = workDir;

        dp.applyRunSimDefaults(_cfg, _fixedMeshSizeMm, _runMeshAuto);
        dp.runDir  = workDir;
        dp.runId   = _runId;
        dp.rank    = ind.rank;
        dp.crowdingDistance = ind.crowdingDist;
        dp.isPareto = (ind.rank == 1) ? 1 : 0;

        if (ccxLog && ccxLog->enabled)
            emitCcxStartLog(*ccxLog, i, dp);

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
            GearOptCasePrepRequest prepReq;
            prepReq.dp         = dp;
            prepReq.workDir    = workDir;
            prepReq.cfg        = _cfg;
            prepReq.meshParams = {_fixedMeshSizeMm, _fixedRootMeshSizeMm, _fixedZLayers};
            prepReq.logLevel   = _cfg.solver.debugMode ? GearLogLevel::Debug : GearLogLevel::Normal;
            prepReq.threads    = _cfg.solver.threads;
            if (GearOptMainThreadRunner::runPreCcxBlocking(&prepReq)) {
                dp = prepReq.dp;
                GearOptCaseRunner ccxRunner;
                ccxRunner.setConfig(_cfg);
                ccxRunner.setWorkDir(workDir);
                ccxRunner.setMeshParams(prepReq.meshParams);
                ccxRunner.setLogLevel(prepReq.logLevel);
                ccxRunner.setThreads(_cfg.solver.threads);
                connect(&ccxRunner, &GearOptCaseRunner::log,
                        [this](const QString& msg) { emit log(msg); });
                if (ccxRunner.runCcxOnlySteps(dp))
                    dp.status = PointStatus::Done;
            } else {
                dp = prepReq.dp;
            }
        } else if (summary) {
            summary->reusedCacheCount++;
        }

        dp.fillResultArtifactPaths();
        auto insertToDb = [&](GearOptResultDatabase& db, const char* label) {
            if (!db.isOpen())
                return;
            const bool ok = db.insertDesignPointResult(_runId, dp, generation, i, ind.rank, ind.crowdingDist,
                                       dp.isPareto, caseH, _cfg.effectiveSurrogateTargets());
            emit log(QStringLiteral("[GearOpt][Case] case_id=%1 case_hash=%2 workDir=%3 dbUpdateStatus=%4 db=%5")
                         .arg(i)
                         .arg(caseH)
                         .arg(workDir)
                         .arg(ok ? QStringLiteral("insert_ok") : QStringLiteral("insert_failed"))
                         .arg(QString::fromLatin1(label)));
        };
        insertToDb(GearOptResultDatabase::global(), "global");
        insertToDb(GearOptResultDatabase::runSession(), "run");
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

        if (ccxLog && ccxLog->enabled) {
            const double predC = i < ccxLog->predCpress.size() ? ccxLog->predCpress[i] : -1.0;
            const double predE = i < ccxLog->predEdge.size() ? ccxLog->predEdge[i] : -1.0;
            if (dp.status == PointStatus::Done && dp.cpressMax_MPa > 0.0)
                emitCcxDoneLog(*ccxLog, i, dp, skippedByCache, predC, predE);
            else
                emitCcxFailLog(*ccxLog, i, dp);
        }

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
    const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback,
    const SurrogateCcxLogOptions* ccxLog) {
    if (!_cfg.solver.parallelCcxEnabled || _cfg.solver.parallelCcxJobs <= 1) {
        evaluatePopulationByCcx(pop, generation, summary, pointCallback, ccxLog);
        return;
    }
    evaluateInfillByCcxParallel(pop, generation, summary, pointCallback, ccxLog);
}

void GearAutoOptManager::evaluateInfillByCcxParallel(
    Population& pop,
    int generation,
    CcxEvalSummary* summary,
    const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback,
    const SurrogateCcxLogOptions* ccxLog) {
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

        const QString workDir = gearOptCaseWorkDir(_runDir, generation, i);

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

    const GearOptConfig cfgCopy      = _cfg;
    const QString       runIdCopy    = _runId;
    const double        meshSizeCopy = _fixedMeshSizeMm;
    const double        rootMeshCopy = _fixedRootMeshSizeMm;
    const int           zLayersCopy  = _fixedZLayers;
    const GearLogLevel  logLevelCopy =
        _cfg.solver.debugMode ? GearLogLevel::Debug : GearLogLevel::Normal;
    const GearMeshParams meshParamsCopy = {meshSizeCopy, rootMeshCopy, zLayersCopy};

    emit log(QStringLiteral("[GearOpt][Parallel] geometry prep on main thread (serial), cases=%1")
                 .arg(runQueue.size()));

    QVector<int> ccxQueue;
    ccxQueue.reserve(runQueue.size());
    for (int idx : runQueue) {
        if (!prepByIndex.contains(idx))
            continue;
        InfillCasePrep prep = prepByIndex.value(idx);

        GearOptCasePrepRequest prepReq;
        prepReq.dp         = prep.dp;
        prepReq.workDir    = prep.workDir;
        prepReq.cfg        = cfgCopy;
        prepReq.meshParams = meshParamsCopy;
        prepReq.logLevel   = logLevelCopy;
        prepReq.threads    = _cfg.solver.threads;

        ParallelCcxOutcome& out = outcomes[idx];
        if (GearOptMainThreadRunner::runPreCcxBlocking(&prepReq)) {
            out.dp     = prepReq.dp;
            ccxQueue.append(idx);
        } else {
            out.dp            = prepReq.dp;
            out.skippedByCache = false;
            out.ccxRan         = false;
            emit log(QStringLiteral("[GearOpt][Parallel] case id=%1 geometry/mesh/inp failed: %2")
                         .arg(idx)
                         .arg(out.dp.errorMsg.left(200)));
        }
    }

    const GearOptConfig cfgCopyCcx     = cfgCopy;
    const QString       runIdCopyCcx   = runIdCopy;
    const GearLogLevel  logLevelCopyCcx = logLevelCopy;

    if (!ccxQueue.isEmpty()) {
        emit log(QStringLiteral("[Parallel] Submit %1 CCX jobs...")
                     .arg(ccxQueue.size()));
    }

    int batchNum = 0;
    for (int batchStart = 0; batchStart < ccxQueue.size() && !_stopRequested; batchStart += jobs) {
        const int batchEnd = std::min(batchStart + jobs, static_cast<int>(ccxQueue.size()));
        ++batchNum;
        emit log(QStringLiteral("[GearOpt][Parallel] CCX batch %1 started, cases=%2")
                     .arg(batchNum)
                     .arg(batchEnd - batchStart));

        std::vector<std::future<ParallelCcxOutcome>> futures;
        futures.reserve(batchEnd - batchStart);

        for (int qi = batchStart; qi < batchEnd; ++qi) {
            const int idx = ccxQueue[qi];
            if (!prepByIndex.contains(idx))
                continue;

            const InfillCasePrep prep = prepByIndex.value(idx);
            // 主线程 prep 后 nodeCount/elementCount/mass 在 outcomes[idx].dp，勿用 prep.dp（prep 前快照）
            const GearDesignPoint prepDp = outcomes[idx].dp;
            futures.push_back(std::async(std::launch::async, [prep, prepDp, cfgCopyCcx, runIdCopyCcx, meshParamsCopy,
                                                              logLevelCopyCcx, threadsPerJob, generation]() {
                ParallelCcxOutcome out;
                out.index          = prep.index;
                out.skippedByCache = false;
                out.ccxRan         = true;
                out.dp             = prepDp;
                out.dp.runDir      = prep.workDir;

                const auto t0 = std::chrono::steady_clock::now();
                try {
                    GearOptCaseRunner runner(nullptr);
                    runner.setConfig(cfgCopyCcx);
                    runner.setWorkDir(prep.workDir);
                    runner.setMeshParams(meshParamsCopy);
                    runner.setLogLevel(logLevelCopyCcx);
                    runner.setThreads(threadsPerJob);
                    if (runner.runCcxOnlySteps(out.dp))
                        out.dp.status = PointStatus::Done;
                    out.dp.fillResultArtifactPaths();
                } catch (const std::exception& ex) {
                    out.dp.status   = PointStatus::Failed;
                    out.dp.errorMsg = QString::fromUtf8(ex.what());
                } catch (...) {
                    out.dp.status   = PointStatus::Failed;
                    out.dp.errorMsg = QStringLiteral("parallel CCX worker unknown exception");
                }
                out.dp.runId            = runIdCopyCcx;
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
            const bool converged     = ccxCaseConverged(cfgCopyCcx, out.dp);
            emit log(QStringLiteral("[GearOpt][Parallel] case id=%1 finished converged=%2 time=%3s")
                         .arg(out.index)
                         .arg(converged ? QStringLiteral("true") : QStringLiteral("false"))
                         .arg(out.elapsedMs / 1000));
        }

        emit log(QStringLiteral("[GearOpt][Parallel] CCX batch %1 done").arg(batchNum));
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

        auto insertToDb = [&](GearOptResultDatabase& db, const char* label) {
            if (!db.isOpen())
                return;
            const bool ok = db.insertDesignPointResult(_runId, dp, generation, i, ind.rank, ind.crowdingDist,
                                       dp.isPareto, prep.caseHash, _cfg.effectiveSurrogateTargets());
            emit log(QStringLiteral("[GearOpt][Case] case_id=%1 case_hash=%2 workDir=%3 dbUpdateStatus=%4 db=%5")
                         .arg(i)
                         .arg(prep.caseHash)
                         .arg(dp.runDir)
                         .arg(ok ? QStringLiteral("insert_ok") : QStringLiteral("insert_failed"))
                         .arg(QString::fromLatin1(label)));
        };
        insertToDb(GearOptResultDatabase::global(), "global");
        insertToDb(GearOptResultDatabase::runSession(), "run");

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

        if (ccxLog && ccxLog->enabled) {
            const double predC = i < ccxLog->predCpress.size() ? ccxLog->predCpress[i] : -1.0;
            const double predE = i < ccxLog->predEdge.size() ? ccxLog->predEdge[i] : -1.0;
            if (dp.status == PointStatus::Done && dp.cpressMax_MPa > 0.0)
                emitCcxDoneLog(*ccxLog, i, dp, prep.skippedByCache, predC, predE);
            else
                emitCcxFailLog(*ccxLog, i, dp);
        }

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
    connectMainThreadRunnerLogs(this);
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
    const auto vLog = [this](const QString& msg) {
        if (surrogateVerboseLogEnabled(_cfg))
            emit log(msg);
    };

    emit log(QStringLiteral("GearAutoOptManager: startSurrogateAssisted"));
    connectMainThreadRunnerLogs(this);
    _stopRequested = false;
    _allPoints.clear();
    _surrogateValidatedPoints.clear();
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

    emitSurrogateRunSummary(_cfg, _runId, baseDp, _fixedCommonWidthMm, _fixedMeshSizeMm,
                              _fixedRootMeshSizeMm, _fixedZLayers, baseCaseH, vLog);

    SurrogateSampleLoadStats sampleStats;
    QVector<SurrogateSample> samples =
        loadMergedValidatedSamples(baseCaseH, _fixedCommonWidthMm, _cfg, &sampleStats);
    emitSurrogateDbSummary(_cfg, baseCaseH, _fixedCommonWidthMm, minSamples, sampleStats,
                           sampleStats.duplicateSkipped, vLog);

    const int initialExisting = samples.size();
    int sessionNewCcx      = 0;
    int sessionCacheReused = 0;
    int sessionFailedCcx   = 0;
    QString finalStopReason;

    auto infillDesignPoint = [&](const Individual& ind) -> GearDesignPoint {
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
        return dp;
    };

    auto supplementValidatedSamples = [&](int generationBase, bool logAsInitial) -> bool {
        constexpr int kMaxAttempts = 20;
        constexpr int kMaxStagnantAttempts = 6;
        int stagnantAttempts = 0;

        for (int attempt = 1;
             samples.size() < minSamples && attempt <= kMaxAttempts && !_stopRequested;
             ++attempt) {
            const int before = samples.size();
            const int need = minSamples - before;
            const int batchN = std::max(1, need);
            const int seed = _cfg.nsga2.randomSeed
                             + generationBase * 1009
                             + attempt * 7919;

            if (logAsInitial && attempt == 1) {
                emitInitialLhsLog(minSamples, before, need, batchN, vLog);
            } else if (!surrogateVerboseLogEnabled(_cfg)) {
                emit log(QStringLiteral("[GearOpt][Surrogate] LHS supplement attempt=%1: samples=%2/%3, request=%4")
                             .arg(attempt)
                             .arg(before)
                             .arg(minSamples)
                             .arg(batchN));
            }

            _population = initLatin(batchN, _cfg, seed);
            fixPopulationCommonWidth(_population, _fixedCommonWidthMm);

            CcxEvalSummary supplementStats;
            evaluateInfillByCcx(_population, generationBase + attempt, &supplementStats);
            sessionNewCcx += supplementStats.newCcxCount;
            sessionCacheReused += supplementStats.reusedCacheCount;
            if (_stopRequested)
                return false;

            samples = loadMergedValidatedSamples(baseCaseH, _fixedCommonWidthMm, _cfg, &sampleStats);
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

        if (logAsInitial && samples.size() >= minSamples && !_stopRequested) {
            emit log(QStringLiteral("[Parallel] All initial samples finished."));
        }

        return samples.size() >= minSamples;
    };

    if (samples.size() < minSamples) {
        if (!supplementValidatedSamples(0, true)) {
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

    emit log(QStringLiteral("[Surrogate] Training surrogate..."));

    for (int round = 1; round <= std::max(1, Gmax) && !_stopRequested; ++round) {
        const int samplesBeforeReload = samples.size();
        samples = loadMergedValidatedSamples(baseCaseH, _fixedCommonWidthMm, _cfg, &sampleStats);
        refreshSurrogateValidatedCache(_surrogateValidatedPoints, samples, baseDp, _cfg,
                                       _fixedCommonWidthMm);

        int exploreCountPlan = 0;
        QString infillPhasePlan;
        if (round <= 5) {
            exploreCountPlan = static_cast<int>(std::ceil(infillCount * 2.0 / 3.0));
            infillPhasePlan  = QStringLiteral("early");
        } else if (round <= 12) {
            exploreCountPlan = static_cast<int>(std::ceil(infillCount * 1.0 / 3.0));
            infillPhasePlan  = QStringLiteral("middle");
        } else {
            exploreCountPlan = 0;
            infillPhasePlan  = QStringLiteral("late");
        }
        const int exploitCountPlan = std::max(0, infillCount - exploreCountPlan);

        emitRoundStartLog(round, samples.size(), samples.size() - samplesBeforeReload,
                          sampleStats.failedSkipped, samples.size(), infillCount, infillPhasePlan,
                          exploitCountPlan, exploreCountPlan, vLog);

        if (samples.size() < minSamples) {
            emit log(QStringLiteral("[GearOpt][Surrogate] round %1 skipped training: samples=%2 < minSamplesForRBF=%3")
                         .arg(round)
                         .arg(samples.size())
                         .arg(minSamples));
            if (!supplementValidatedSamples(round * 1000, false)) {
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
        const bool trained = model.train(samples, _cfg.effectiveSurrogateTargets());
        emitRbfTrainingLog(round, trained, samples.size(), samples, model, vLog);
        if (!trained) {
            emit log(QStringLiteral("[GearOpt][Surrogate] model training failed; need more valid cpressMax samples"));
            finalStopReason = QStringLiteral("rbf_training_failed");
            break;
        }

        const QVector<double> sampleResiduals =
            computeSurrogateSampleResidualsFixed(samples, model);
        InfillScoringConfig infillScoring;
        infillScoring.alpha = 0.6;
        infillScoring.beta = 0.4;
        infillScoring.neighborK = 3;
        infillScoring.minDesignDistNorm = 0.03;
        infillScoring.logFn             = [this](const QString& msg) {
            if (!surrogateVerboseLogEnabled(_cfg))
                emit log(msg);
        };

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

        const auto predParetoMax = paretoObjectiveMax(surrogatePareto);
        const HypervolumeReference2D predHvRef = computeDynamicHypervolumeReference(
            samples, predParetoMax.first, predParetoMax.second, kHvRefMargin,
            hvFallbackCpress, hvFallbackEdge);
        const double predHv =
            hypervolume2D(surrogatePareto, predHvRef.refCpressMax, predHvRef.refEdgeLoadRatio);
        emitSurrogateNsgaLog(round, N, Gmax, surrogatePareto, predHv, vLog);
        const int predParetoSize = surrogatePareto.size();
        SurrogateHvRow hvRow;
        hvRow.round      = round;
        hvRow.predHv     = predHv;
        hvRow.predPareto = predParetoSize;
        if (!appendSurrogateHvCsv(_runDir, hvRow)) {
            emit log(QStringLiteral("[Surrogate] failed to append surrogate_metrics.csv under %1")
                         .arg(_runDir));
        }

        QSet<QString> knownCaseHashes;
        QSet<QString> knownDesignHashes;
        QSet<QString> failedCaseHashes;
        auto mergeKnownHashes = [&](GearOptResultDatabase& db) {
            if (db.isOpen()) {
                knownCaseHashes.unite(
                    db.knownCaseHashesForBaseCase(baseCaseH, _fixedCommonWidthMm));
                knownDesignHashes.unite(
                    db.knownDesignHashesForBaseCase(baseCaseH, _fixedCommonWidthMm));
                failedCaseHashes.unite(
                    db.failedCaseHashesForBaseCase(baseCaseH, _fixedCommonWidthMm));
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

        auto mergeInfillKnownHashes = [&](QSet<QString>& caseHashes, QSet<QString>& designHashes,
                                          const Population& pop) {
            for (const Individual& ind : pop) {
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
                caseHashes.insert(GearOptResultDatabase::caseHash(dp));
                designHashes.insert(GearOptResultDatabase::designHash(dp));
            }
        };

        auto selectExploitPoints = [&](int count, const Population& paretoSource,
                                     QSet<QString> caseHashes, QSet<QString> designHashes,
                                     QString* sourceOut,
                                     InfillFilterStats* filterOut,
                                     QVector<InfillSelectionMeta>* metaOut) -> Population {
            if (count <= 0)
                return {};
            InfillScoringConfig scoring = infillScoring;
            scoring.outFilterStats   = filterOut;
            scoring.outSelectionMeta = metaOut;
            Population out = GearInfillSelector::selectSparseParetoPoints(
                paretoSource, samples, sampleResiduals, scoring, _cfg, baseDp,
                _fixedMeshSizeMm, _runMeshAuto, caseHashes, designHashes, failedCaseHashes, count);
            if (sourceOut)
                *sourceOut = QStringLiteral("surrogatePareto");
            if (out.size() < count) {
                mergeInfillKnownHashes(caseHashes, designHashes, out);
                const int remaining = count - out.size();
                InfillFilterStats fallbackStats;
                InfillScoringConfig fallbackScoring = infillScoring;
                fallbackScoring.outFilterStats   = filterOut ? &fallbackStats : nullptr;
                fallbackScoring.outSelectionMeta = metaOut;
                Population fallback = GearInfillSelector::selectSparseParetoPoints(
                    surrogatePop, samples, sampleResiduals, fallbackScoring, _cfg, baseDp,
                    _fixedMeshSizeMm, _runMeshAuto, caseHashes, designHashes, failedCaseHashes,
                    remaining);
                if (!fallback.isEmpty()) {
                    if (sourceOut)
                        *sourceOut = QStringLiteral("surrogatePopFallback");
                    if (filterOut) {
                        filterOut->candidateTotal += fallbackStats.candidateTotal;
                        filterOut->validAfterRelief += fallbackStats.validAfterRelief;
                        filterOut->skipFailed += fallbackStats.skipFailed;
                        filterOut->skipKnownCase += fallbackStats.skipKnownCase;
                        filterOut->skipKnownDesign += fallbackStats.skipKnownDesign;
                        filterOut->skipNearDuplicate += fallbackStats.skipNearDuplicate;
                        filterOut->skipTooCloseToSelected += fallbackStats.skipTooCloseToSelected;
                        filterOut->selected += fallbackStats.selected;
                    }
                    out += fallback;
                }
            }
            return out;
        };

        const int exploreCount = exploreCountPlan;
        const int exploitCount = exploitCountPlan;
        const QString infillPhase = infillPhasePlan;

        QString exploitSource = QStringLiteral("surrogatePareto");
        InfillFilterStats exploitFilterStats;
        InfillFilterStats exploreFilterStats;
        QVector<InfillSelectionMeta> exploitMeta;
        QVector<InfillSelectionMeta> exploreMeta;

        Population exploitPoints = selectExploitPoints(exploitCount, surrogatePareto, knownCaseHashes,
                                                      knownDesignHashes, &exploitSource,
                                                      &exploitFilterStats, &exploitMeta);

        emitInfillPlanLog(round, infillCount, infillPhase, exploitCount, exploreCount, exploitSource,
                          infillScoring.minDesignDistNorm, infillScoring.alpha, infillScoring.beta,
                          infillScoring.neighborK, vLog);

        Population explorePoints;
        if (exploreCount > 0) {
            GlobalExplorationConfig exploreCfg;
            exploreCfg.explorationCandidateCount = 2000;
            exploreCfg.minDesignDistNorm         = infillScoring.minDesignDistNorm;
            exploreCfg.logFn = [this](const QString& msg) {
                if (!surrogateVerboseLogEnabled(_cfg))
                    emit log(msg);
            };
            exploreCfg.outFilterStats   = &exploreFilterStats;
            exploreCfg.outSelectionMeta = &exploreMeta;
            const int exploreSeed = _cfg.nsga2.randomSeed + round * 31337;
            explorePoints         = GearInfillSelector::selectGlobalExplorationPoints(
                samples, exploreCfg, _cfg, baseDp, _fixedMeshSizeMm, _runMeshAuto,
                _fixedCommonWidthMm, knownCaseHashes, knownDesignHashes, failedCaseHashes,
                exploitPoints, exploreCount, exploreSeed);
        }

        emitInfillFilterLog(exploitFilterStats, vLog);
        if (exploreCount > 0)
            emitInfillFilterLog(exploreFilterStats, vLog);

        Population infill = exploitPoints + explorePoints;

        if (infill.size() < infillCount) {
            int need = infillCount - infill.size();
            QSet<QString> supplementCaseHashes = knownCaseHashes;
            QSet<QString> supplementDesignHashes = knownDesignHashes;
            mergeInfillKnownHashes(supplementCaseHashes, supplementDesignHashes, infill);

            if (need > 0) {
                InfillFilterStats extraFilter;
                QVector<InfillSelectionMeta> extraMeta;
                QString extraSource;
                Population extraExploit =
                    selectExploitPoints(need, surrogatePareto, supplementCaseHashes,
                                        supplementDesignHashes, &extraSource, &extraFilter, &extraMeta);
                if (!extraExploit.isEmpty())
                    infill += extraExploit;
            }

            need = infillCount - infill.size();
            if (need > 0) {
                GlobalExplorationConfig exploreCfg;
                exploreCfg.explorationCandidateCount = 2000;
                exploreCfg.minDesignDistNorm         = infillScoring.minDesignDistNorm;
                exploreCfg.logFn = [this](const QString& msg) {
                    if (!surrogateVerboseLogEnabled(_cfg))
                        emit log(msg);
                };
                const int exploreSeed = _cfg.nsga2.randomSeed + round * 31337 + 17;
                Population extraExplore = GearInfillSelector::selectGlobalExplorationPoints(
                    samples, exploreCfg, _cfg, baseDp, _fixedMeshSizeMm, _runMeshAuto,
                    _fixedCommonWidthMm, supplementCaseHashes, supplementDesignHashes,
                    failedCaseHashes, infill, need, exploreSeed);
                if (!extraExplore.isEmpty())
                    infill += extraExplore;
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

        QSet<QString> exploitCaseHashes;
        for (const Individual& ind : exploitPoints)
            exploitCaseHashes.insert(GearOptResultDatabase::caseHash(infillDesignPoint(ind)));

        QVector<QString> infillTypes;
        infillTypes.reserve(infill.size());
        QVector<double> predCpress;
        QVector<double> predEdge;
        QVector<double> predSigma;
        QVector<double> predU;

        int exploitIdx = 0;
        int exploreIdx = 0;
        for (int ii = 0; ii < infill.size(); ++ii) {
            GearDesignPoint dp = infillDesignPoint(infill[ii]);
            const QString caseH = GearOptResultDatabase::caseHash(dp);
            const bool isExplore = !exploitCaseHashes.contains(caseH);
            const QString type =
                isExplore ? QStringLiteral("explore") : QStringLiteral("exploit");
            const SurrogatePrediction pred = model.predict(surrogateInputVars(dp));
            predCpress.append(pred.cpressMaxPred);
            predEdge.append(pred.edgeLoadRatioPred);
            predSigma.append(pred.sigmaMaxPred);
            predU.append(pred.uMaxPred);
            infillTypes.append(type);

            double minDist  = -1.0;
            double localRes = -1.0;
            double score    = -1.0;
            if (!isExplore) {
                if (exploitIdx < exploitMeta.size()) {
                    minDist  = exploitMeta[exploitIdx].minDistNorm;
                    localRes = exploitMeta[exploitIdx].localResidualNorm;
                    score    = exploitMeta[exploitIdx].score;
                } else {
                    minDist = computeMinDistNormToSamples(dp, samples, _cfg);
                }
                ++exploitIdx;
            } else {
                if (exploreIdx < exploreMeta.size())
                    minDist = exploreMeta[exploreIdx].minDistNorm;
                else
                    minDist = computeMinDistNormToSamples(dp, samples, _cfg);
                ++exploreIdx;
            }

            emitInfillSelectedLog(round, type, ii, dp,
                                  isExplore ? -1.0 : pred.cpressMaxPred,
                                  isExplore ? -1.0 : pred.edgeLoadRatioPred,
                                  minDist, localRes, score, vLog);

            infill[ii].evaluated = false;
            infill[ii].objs.clear();
        }

        CcxEvalSummary ccxStats;
        QVector<double> predCpressAll;
        QVector<double> predCpressNew;
        QVector<double> predSigmaAll;
        QVector<double> predSigmaNew;
        QVector<double> predUAll;
        QVector<double> predEdgeAll;
        QVector<double> predEdgeNew;
        QVector<double> trueCpressAll;
        QVector<double> trueCpressNew;
        QVector<double> trueSigmaAll;
        QVector<double> trueSigmaNew;
        QVector<double> trueUAll;
        QVector<double> trueEdgeAll;
        QVector<double> trueEdgeNew;
        int roundFailedCcx = 0;

        auto appendPredVsTrue = [&](int pointIndex, const GearDesignPoint& dp, bool cached) {
            if (pointIndex < 0 || pointIndex >= predCpress.size())
                return;
            if (dp.status != PointStatus::Done || dp.cpressMax_MPa <= 0.0) {
                if (!cached)
                    ++roundFailedCcx;
                return;
            }
            if (!ccxCaseConverged(_cfg, dp)) {
                if (!cached)
                    ++roundFailedCcx;
                return;
            }

            predCpressAll.append(predCpress[pointIndex]);
            trueCpressAll.append(dp.cpressMax_MPa);
            predSigmaAll.append(predSigma[pointIndex]);
            trueSigmaAll.append(dp.sigmaMax);
            predUAll.append(predU[pointIndex]);
            trueUAll.append(dp.uMax);
            const double predEdgeVal = pointIndex < predEdge.size() ? predEdge[pointIndex] : -1.0;
            if (predEdgeVal >= 0.0 && dp.edgeLoadRatio > 0.0) {
                predEdgeAll.append(predEdgeVal);
                trueEdgeAll.append(dp.edgeLoadRatio);
            }

            const double cpressErr = dp.cpressMax_MPa > 1e-12
                                         ? std::abs(predCpress[pointIndex] - dp.cpressMax_MPa)
                                               / std::abs(dp.cpressMax_MPa)
                                         : -1.0;
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
            appendSurrogateInfillValidationCsv(_runDir, QVector<SurrogateInfillValidationRow>{row});

            if (!cached) {
                predCpressNew.append(predCpress[pointIndex]);
                trueCpressNew.append(dp.cpressMax_MPa);
                if (predEdgeVal >= 0.0 && dp.edgeLoadRatio > 0.0) {
                    predEdgeNew.append(predEdgeVal);
                    trueEdgeNew.append(dp.edgeLoadRatio);
                }
                if (predSigma[pointIndex] > 0.0 && dp.sigmaMax > 0.0) {
                    predSigmaNew.append(predSigma[pointIndex]);
                    trueSigmaNew.append(dp.sigmaMax);
                }
            }
        };

        SurrogateCcxLogOptions ccxLogOpts;
        ccxLogOpts.enabled     = surrogateVerboseLogEnabled(_cfg);
        ccxLogOpts.round       = round;
        ccxLogOpts.infillTypes = infillTypes;
        ccxLogOpts.predCpress  = predCpress;
        ccxLogOpts.predEdge    = predEdge;
        ccxLogOpts.logFn       = vLog;

        fixPopulationCommonWidth(infill, _fixedCommonWidthMm);
        evaluateInfillByCcx(infill, 1000 + round, &ccxStats, appendPredVsTrue, &ccxLogOpts);
        sessionNewCcx += ccxStats.newCcxCount;
        sessionCacheReused += ccxStats.reusedCacheCount;
        sessionFailedCcx += roundFailedCcx;
        if (_stopRequested)
            break;

        emitPredTrueSummaryLog(round, ccxStats.newCcxCount, ccxStats.reusedCacheCount, roundFailedCcx,
                               predCpressNew.isEmpty() ? predCpressAll : predCpressNew,
                               trueCpressNew.isEmpty() ? trueCpressAll : trueCpressNew,
                               predEdgeNew.isEmpty() ? predEdgeAll : predEdgeNew,
                               trueEdgeNew.isEmpty() ? trueEdgeAll : trueEdgeNew,
                               samples, vLog);

        const double rmaeCpressAll = computeRmae(predCpressAll, trueCpressAll);
        const double rmaeCpressNew = computeRmae(predCpressNew, trueCpressNew);
        const double rmaeSigma     = computeRmae(predSigmaAll, trueSigmaAll);
        const double rmaeSigmaNew  = computeRmae(predSigmaNew, trueSigmaNew);
        const double rmaeU         = computeRmae(predUAll, trueUAll);

        samples = loadMergedValidatedSamples(baseCaseH, _fixedCommonWidthMm, _cfg, &sampleStats);
        refreshSurrogateValidatedCache(_surrogateValidatedPoints, samples, baseDp, _cfg,
                                       _fixedCommonWidthMm);

        const Population ccxPop = populationFromValidatedSamples(samples, _cfg);
        const Population ccxPareto = extractPareto(ccxPop);
        const int ccxParetoSize = ccxPareto.size();
        const auto ccxParetoMax = paretoObjectiveMax(ccxPareto);
        const HypervolumeReference2D ccxHvRef = computeDynamicHypervolumeReference(
            samples, ccxParetoMax.first, ccxParetoMax.second, kHvRefMargin,
            hvFallbackCpress, hvFallbackEdge);
        const double ccxHv =
            hypervolume2D(ccxPareto, ccxHvRef.refCpressMax, ccxHvRef.refEdgeLoadRatio);
        const int ccxSampleCount = samples.size();

        QString stopReason;
        bool willStop = false;
        if (_stopRequested) {
            stopReason = QStringLiteral("user_stop");
            willStop   = true;
        } else if (ccxStats.newCcxCount > 0 && rmaeCpressNew >= 0.0 && rmaeSigmaNew >= 0.0
                   && rmaeCpressNew < kRmaeStopEps && rmaeSigmaNew < kRmaeStopEps) {
            stopReason = QStringLiteral("rmae_threshold");
            willStop   = true;
        } else if (round >= std::max(1, Gmax)) {
            stopReason = QStringLiteral("max_generations");
            willStop   = true;
        }
        finalStopReason = stopReason;

        emitTrueParetoLog(round, ccxSampleCount, ccxParetoSize, samples, ccxHv, vLog);
        emitConvergenceLog(round, rmaeCpressAll, rmaeCpressNew, rmaeSigma, rmaeSigmaNew, rmaeU,
                           stopReason, willStop, vLog);

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

        if (willStop) {
            if (stopReason == QStringLiteral("rmae_threshold"))
                emit log(QStringLiteral("[GearOpt][Surrogate] RMAE (new CCX) below threshold; stop"));
            break;
        }

        emit generationFinished(round, ccxParetoSize, ccxHv);
    }

    _allPoints.clear();
    if (!_surrogateValidatedPoints.isEmpty())
        _allPoints.append(_surrogateValidatedPoints);

    syncFinalParetoToDatabase();
    GearOptResultDatabase::runSession().closeDatabase();

    samples = loadMergedValidatedSamples(baseCaseH, _fixedCommonWidthMm, _cfg, &sampleStats);
    emitFinalSummaryLog(_cfg, _runId, _runDir, !_stopRequested, finalStopReason, samples.size(),
                          sessionNewCcx, sessionCacheReused, sessionFailedCcx, paretoFront().size(),
                          samples, vLog);
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
