#ifndef _GEARAUTOOPT_GEAR_SURROGATE_VERBOSE_LOG_H_
#define _GEARAUTOOPT_GEAR_SURROGATE_VERBOSE_LOG_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"
#include "GearAutoOpt/opt/NSGA2.h"
#include "GearAutoOpt/surrogate/GearInfillSelector.h"
#include "GearAutoOpt/surrogate/GearSurrogateModel.h"

#include <functional>

namespace GearAutoOpt {

struct GEARAUTOOPTAPI SampleMetricRange {
	double minVal  = -1.0;
	double maxVal  = -1.0;
	double meanVal = -1.0;
};

struct GEARAUTOOPTAPI TrainMetricRanges {
	SampleMetricRange cpress;
	SampleMetricRange edgeLoadRatio;
	SampleMetricRange sigmaMax;
	SampleMetricRange uMax;
	SampleMetricRange mass;
	SampleMetricRange cpressCV;
};

struct GEARAUTOOPTAPI SurrogateCcxLogOptions {
	bool              enabled = false;
	int               round   = 0;
	QVector<QString>  infillTypes;
	QVector<double>   predCpress;
	QVector<double>   predEdge;
	std::function<void(const QString&)> logFn;
};

GEARAUTOOPTAPI bool surrogateVerboseLogEnabled(const GearOptConfig& cfg);

GEARAUTOOPTAPI TrainMetricRanges computeTrainMetricRanges(const QVector<SurrogateSample>& samples);

GEARAUTOOPTAPI void emitSurrogateRunSummary(const GearOptConfig& cfg,
                                            const QString& runId,
                                            const GearDesignPoint& baseDp,
                                            double fixedWidthMm,
                                            double fixedMeshSizeMm,
                                            double fixedRootMeshSizeMm,
                                            int fixedZLayers,
                                            const QString& baseCaseHash,
                                            const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitSurrogateDbSummary(const GearOptConfig& cfg,
                                           const QString& baseCaseHash,
                                           double fixedWidthMm,
                                           int minRequiredSamples,
                                           const SurrogateSampleLoadStats& mergedStats,
                                           int duplicateSkipped,
                                           const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitInitialLhsLog(int required,
                                      int existing,
                                      int toGenerate,
                                      int lhsBatch,
                                      const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitRoundStartLog(int round,
                                      int samplesLoaded,
                                      int newSamplesSinceLastRound,
                                      int failedSkipped,
                                      int trainValid,
                                      int infillTarget,
                                      const QString& phase,
                                      int exploitCount,
                                      int exploreCount,
                                      const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitRbfTrainingLog(int round,
                                       bool trained,
                                       int trainSamples,
                                       const QVector<SurrogateSample>& samples,
                                       const GearSurrogateModel& model,
                                       const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitSurrogateNsgaLog(int round,
                                         int population,
                                         int generations,
                                         const Population& surrogatePareto,
                                         double predHv,
                                         const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitInfillPlanLog(int round,
                                      int target,
                                      const QString& phase,
                                      int exploitCount,
                                      int exploreCount,
                                      const QString& sourceExploit,
                                      double minDesignDistNorm,
                                      double alpha,
                                      double beta,
                                      int neighborK,
                                      const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitInfillFilterLog(const InfillFilterStats& stats,
                                        const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitInfillSelectedLog(int round,
                                          const QString& type,
                                          int idx,
                                          const GearDesignPoint& dp,
                                          double predCpress,
                                          double predEdge,
                                          double minDistNorm,
                                          double localResidualNorm,
                                          double score,
                                          const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitCcxStartLog(const SurrogateCcxLogOptions& opts,
                                    int idx,
                                    const GearDesignPoint& dp);

GEARAUTOOPTAPI void emitCcxDoneLog(const SurrogateCcxLogOptions& opts,
                                   int idx,
                                   const GearDesignPoint& dp,
                                   bool cached,
                                   double predCpress,
                                   double predEdge);

GEARAUTOOPTAPI void emitCcxFailLog(const SurrogateCcxLogOptions& opts,
                                   int idx,
                                   const GearDesignPoint& dp);

GEARAUTOOPTAPI void emitPredTrueSummaryLog(int round,
                                           int newCcx,
                                           int cacheReused,
                                           int failed,
                                           const QVector<double>& predCpress,
                                           const QVector<double>& trueCpress,
                                           const QVector<double>& predEdge,
                                           const QVector<double>& trueEdge,
                                           const QVector<SurrogateSample>& roundNewSamples,
                                           const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitTrueParetoLog(int round,
                                      int validatedSamples,
                                      int trueParetoSize,
                                      const QVector<SurrogateSample>& samples,
                                      double ccxHv,
                                      const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitConvergenceLog(int round,
                                       double rmaeCpressAll,
                                       double rmaeCpressNew,
                                       double rmaeSigmaAll,
                                       double rmaeSigmaNew,
                                       double rmaeUmaxAll,
                                       const QString& stopReason,
                                       bool willStop,
                                       const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI void emitFinalSummaryLog(const GearOptConfig& cfg,
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
                                        const std::function<void(const QString&)>& logFn);

GEARAUTOOPTAPI double computeMinDistNormToSamples(const GearDesignPoint& dp,
                                                  const QVector<SurrogateSample>& samples,
                                                  const GearOptConfig& cfg);

} // namespace GearAutoOpt

#endif
