#ifndef _GEARAUTOOPT_GEAR_SURROGATE_METRICS_H_
#define _GEARAUTOOPT_GEAR_SURROGATE_METRICS_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QDateTime>
#include <QString>
#include <QVector>

namespace GearAutoOpt {

GEARAUTOOPTAPI double computeRmae(const QVector<double>& predValues,
                                  const QVector<double>& trueValues);

/// 代理搜索过程：runDir/surrogate_metrics.csv → Round, PredHV, PredPareto
struct GEARAUTOOPTAPI SurrogateHvRow {
	int    round       = 0;
	double predHv      = -1.0;
	int    predPareto  = 0;
};

/// 真实 CCX 收敛：runDir/ccx_metrics.csv → Round, CCXSamples, CCXHV
struct GEARAUTOOPTAPI CcxHvRow {
	int    round       = 0;
	int    ccxSamples  = 0;
	double ccxHv       = -1.0;
};

/// RMAE 验证：runDir/surrogate_rmae.csv
struct GEARAUTOOPTAPI SurrogateRmaeRow {
	int         round           = 0;
	int         sampleCount     = 0;
	int         infillSelected  = 0;
	int         newCcxCount     = 0;
	int         reusedCacheCount = 0;
	double      rmaeCpressAll   = -1.0;
	double      rmaeCpressNew   = -1.0;
	double      rmaeSigma       = -1.0;
	double      rmaeUmax        = -1.0;
	QString     stopReason;
	QDateTime   createdAt;
};

/// 代理搜索过程预测导出：runDir/surrogate_predictions.csv
struct GEARAUTOOPTAPI SurrogatePredictionRow {
	int    generation          = 0;
	int    individualId        = 0;
	double x1                  = 0.0;
	double x2                  = 0.0;
	double ca1                 = 0.0;
	double lca1                = 0.0;
	double ca2                 = 0.0;
	double lca2                = 0.0;
	double hubRatio            = 0.0;
	double predCpressMax_MPa   = -1.0;
	double predEdgeLoadRatio   = -1.0;
};

/// 每轮 infill 验证样本导出：代理预测与 CCX 真实结果。
struct GEARAUTOOPTAPI SurrogateInfillValidationRow {
	int     round                      = 0;
	int     id                         = 0;
	QString caseHash;
	double  predCpressMax_MPa          = -1.0;
	double  trueCpressMax_MPa          = -1.0;
	double  cpressErrorPercent         = -1.0;
	double  predEdgeLoadRatio          = -1.0;
	double  trueEdgeLoadRatio          = -1.0;
	double  edgeLoadRatioErrorPercent  = -1.0;
	double  trueSigmaMax_MPa           = -1.0;
	double  trueUMax_mm                = -1.0;
	double  trueMass_kg                = -1.0;
	double  trueCpressCV               = -1.0;
};

GEARAUTOOPTAPI bool appendSurrogateHvCsv(const QString& runDir, const SurrogateHvRow& row);
GEARAUTOOPTAPI bool appendCcxHvCsv(const QString& runDir, const CcxHvRow& row);
GEARAUTOOPTAPI bool appendSurrogateRmaeCsv(const QString& runDir, const SurrogateRmaeRow& row);
GEARAUTOOPTAPI bool appendSurrogatePredictionsCsv(const QString& runDir,
                                                  const SurrogatePredictionRow& row);
GEARAUTOOPTAPI bool appendSurrogateInfillValidationCsv(
    const QString& runDir,
    const QVector<SurrogateInfillValidationRow>& rows);

} // namespace GearAutoOpt

#endif
