#ifndef _GEARAUTOOPT_GEAR_SURROGATE_MODEL_H_
#define _GEARAUTOOPT_GEAR_SURROGATE_MODEL_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"

#include <QString>
#include <QStringList>
#include <QVector>

namespace GearAutoOpt {

struct GEARAUTOOPTAPI SurrogateSample {
	QVector<double> x;
	double cpressMax = -1.0;
	double edgeLoadRatio = -1.0;
	double cpressCV  = -1.0;
	double sigmaMax  = -1.0;
	double uMax      = -1.0;
	double mass      = -1.0;
	QString designHash;
	QString caseHash;
};

struct GEARAUTOOPTAPI SurrogatePrediction {
	double cpressMaxPred = -1.0;
	double edgeLoadRatioPred = -1.0;
	double cpressCVPred  = -1.0;
	double sigmaMaxPred  = -1.0;
	double uMaxPred      = -1.0;
	double cpressMaxStd  = 0.0;
	double edgeLoadRatioStd = 0.0;
	double cpressCVStd   = 0.0;
	double sigmaMaxStd   = 0.0;
	double uMaxStd       = 0.0;
};

struct GEARAUTOOPTAPI HypervolumeReference2D {
	double refCpressMax      = 1200.0;
	double refEdgeLoadRatio  = 2.0;
};

/// 动态 HV 参考点：max(训练样本, optional extraMax*) * margin。
GEARAUTOOPTAPI HypervolumeReference2D computeDynamicHypervolumeReference(
    const QVector<SurrogateSample>& samples,
    double                          extraMaxCpress    = -1.0,
    double                          extraMaxEdge      = -1.0,
    double                          margin            = 1.05,
    double                          fallbackRefCpress = 1200.0,
    double                          fallbackRefEdge   = 2.0);

GEARAUTOOPTAPI QVector<double> surrogateInputVars(const GearDesignPoint& dp);
GEARAUTOOPTAPI void applySurrogateInputVars(GearDesignPoint& dp, const QVector<double>& vars);

/// RBF 输入维度（x1,x2,ca1,lca1,ca2,lca2,hubRatio）；齿宽为 baseCase 固定参数。
static constexpr int kSurrogateInputDim = 7;

/// 启动代理辅助前同一 baseCase 下至少需要的 CCX 验证样本数。
GEARAUTOOPTAPI int minSurrogateSampleCount(int populationSize = 30);

class GEARAUTOOPTAPI GearSurrogateModel {
public:
	bool train(const QVector<SurrogateSample>& samples,
	           const QStringList& surrogateTargets = QStringList());
	SurrogatePrediction predict(const QVector<double>& vars) const;
	bool isReady() const { return _ready; }
	int inputDimension() const { return _dim; }
	/// 训练样本上的最大相对误差（用于诊断 RBF 是否可用）。
	double trainingMaxRelativeError(const QString& target = QStringLiteral("edgeLoadRatio")) const;

private:
	QVector<double> solveRbfWeights(const QVector<double>& y) const;
	QVector<double> normalized(const QVector<double>& x) const;
	double kernel(const QVector<double>& a, const QVector<double>& b) const;
	double nearestNormalizedDistance(const QVector<double>& xNorm) const;

	bool _ready = false;
	int _dim = 0;
	double _epsilon = 1.0;
	QVector<SurrogateSample> _samples;
	QVector<QVector<double>> _xNorm;
	QVector<double> _center;
	QVector<double> _scale;
	QVector<double> _cpressWeights;
	QVector<double> _edgeLoadRatioWeights;
	QVector<double> _cpressCVWeights;
	QVector<double> _sigmaWeights;
	QVector<double> _uWeights;
	double _cpressStdScale = 0.0;
	double _edgeLoadRatioStdScale = 0.0;
	double _cpressCVStdScale = 0.0;
	double _sigmaStdScale = 0.0;
	double _uStdScale = 0.0;
	double _edgeYMean = 0.0;
	double _edgeYStd = 1.0;
	double _edgeYMin = 0.0;
	double _edgeYMax = 0.0;
	double _cpressYMean = 0.0;
	double _cpressYStd = 1.0;
	double _cpressYMin = 0.0;
	double _cpressYMax = 0.0;
};

} // namespace GearAutoOpt

#endif
