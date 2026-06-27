#include "GearSurrogateModel.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace GearAutoOpt {

namespace {

double sqr(double v)
{
	return v * v;
}

double vectorDistance(const QVector<double>& a, const QVector<double>& b)
{
	const int n = std::min(a.size(), b.size());
	double sum = 0.0;
	for (int i = 0; i < n; ++i)
		sum += sqr(a[i] - b[i]);
	return std::sqrt(sum);
}

double sampleStd(const QVector<double>& y)
{
	if (y.size() < 2)
		return 0.0;
	double mean = 0.0;
	for (double v : y)
		mean += v;
	mean /= static_cast<double>(y.size());
	double ss = 0.0;
	for (double v : y)
		ss += sqr(v - mean);
	return std::sqrt(ss / static_cast<double>(y.size() - 1));
}

double sampleMean(const QVector<double>& y)
{
	if (y.isEmpty())
		return 0.0;
	double mean = 0.0;
	for (double v : y)
		mean += v;
	return mean / static_cast<double>(y.size());
}

double sampleMin(const QVector<double>& y)
{
	if (y.isEmpty())
		return 0.0;
	double best = y[0];
	for (double v : y)
		best = std::min(best, v);
	return best;
}

double sampleMax(const QVector<double>& y)
{
	if (y.isEmpty())
		return 0.0;
	double best = y[0];
	for (double v : y)
		best = std::max(best, v);
	return best;
}

QVector<double> normalizeTargets(const QVector<double>& y, double mean, double std)
{
	QVector<double> out;
	out.reserve(y.size());
	if (std < 1e-12) {
		for (double v : y)
			out.append(v - mean);
		return out;
	}
	for (double v : y)
		out.append((v - mean) / std);
	return out;
}

double denormalizeTarget(double yn, double mean, double std)
{
	return std < 1e-12 ? mean + yn : mean + yn * std;
}

double clipToRange(double v, double lo, double hi)
{
	if (!std::isfinite(v))
		return v;
	if (lo <= hi)
		return std::max(lo, std::min(hi, v));
	return v;
}

} // namespace

int minSurrogateSampleCount(int populationSize)
{
	const int base = std::max({20, kSurrogateInputDim + 1, 25});
	if (populationSize <= 0)
		return base;
	return std::max(base, populationSize);
}

QVector<double> surrogateInputVars(const GearDesignPoint& dp)
{
	return {
	    dp.x1,
	    dp.x2,
	    dp.ca1,
	    dp.lca1,
	    dp.ca2,
	    dp.lca2,
	    dp.hubRatio,
	};
}

void applySurrogateInputVars(GearDesignPoint& dp, const QVector<double>& vars)
{
	if (vars.size() > 0) dp.x1 = vars[0];
	if (vars.size() > 1) dp.x2 = vars[1];
	if (vars.size() > 2) dp.ca1 = vars[2];
	if (vars.size() > 3) dp.lca1 = vars[3];
	if (vars.size() > 4) dp.ca2 = vars[4];
	if (vars.size() > 5) dp.lca2 = vars[5];
	if (vars.size() > 6) dp.hubRatio = vars[6];
	dp.syncPairGearWidth();
}

HypervolumeReference2D computeDynamicHypervolumeReference(const QVector<SurrogateSample>& samples,
                                                          double extraMaxCpress,
                                                          double extraMaxEdge,
                                                          double margin,
                                                          double fallbackRefCpress,
                                                          double fallbackRefEdge)
{
	double maxCpress = extraMaxCpress > 0.0 ? extraMaxCpress : 0.0;
	double maxEdge   = extraMaxEdge > 0.0 ? extraMaxEdge : 0.0;
	for (const SurrogateSample& s : samples) {
		if (s.cpressMax > 0.0 && std::isfinite(s.cpressMax))
			maxCpress = std::max(maxCpress, s.cpressMax);
		if (s.edgeLoadRatio > 0.0 && std::isfinite(s.edgeLoadRatio))
			maxEdge = std::max(maxEdge, s.edgeLoadRatio);
	}

	HypervolumeReference2D ref;
	ref.refCpressMax = maxCpress > 0.0 ? maxCpress * margin : fallbackRefCpress;
	ref.refEdgeLoadRatio = maxEdge > 0.0 ? maxEdge * margin : fallbackRefEdge;
	return ref;
}

bool GearSurrogateModel::train(const QVector<SurrogateSample>& samples)
{
	_ready = false;
	_samples.clear();
	_xNorm.clear();
	_cpressWeights.clear();
	_edgeLoadRatioWeights.clear();
	_cpressCVWeights.clear();
	_sigmaWeights.clear();
	_uWeights.clear();
	_center.clear();
	_scale.clear();
	_dim = 0;

	for (const SurrogateSample& s : samples) {
		if (s.x.isEmpty() || s.cpressMax <= 0.0 || !std::isfinite(s.cpressMax)
		    || s.edgeLoadRatio <= 0.0 || !std::isfinite(s.edgeLoadRatio))
			continue;
		if (_dim == 0)
			_dim = s.x.size();
		if (s.x.size() != _dim)
			continue;
		_samples.append(s);
	}
	if (_dim <= 0)
		return false;

	if (_samples.size() < static_cast<size_t>(minSurrogateSampleCount(0)))
		return false;

	_center.fill(0.0, _dim);
	for (const SurrogateSample& s : _samples) {
		for (int i = 0; i < _dim; ++i)
			_center[i] += s.x[i];
	}
	for (double& v : _center)
		v /= static_cast<double>(_samples.size());

	_scale.fill(0.0, _dim);
	for (const SurrogateSample& s : _samples) {
		for (int i = 0; i < _dim; ++i)
			_scale[i] = std::max(_scale[i], std::abs(s.x[i] - _center[i]));
	}
	for (double& v : _scale) {
		if (v < 1e-12)
			v = 1.0;
	}

	for (const SurrogateSample& s : _samples)
		_xNorm.append(normalized(s.x));

	QVector<double> cpressY;
	QVector<double> edgeY;
	QVector<double> cvY;
	QVector<double> sigmaY;
	QVector<double> uY;
	cpressY.reserve(_samples.size());
	edgeY.reserve(_samples.size());
	cvY.reserve(_samples.size());
	sigmaY.reserve(_samples.size());
	uY.reserve(_samples.size());
	for (const SurrogateSample& s : _samples) {
		cpressY.append(s.cpressMax);
		edgeY.append(s.edgeLoadRatio);
		cvY.append(s.cpressCV > 0.0 ? s.cpressCV : 0.0);
		sigmaY.append(s.sigmaMax > 0.0 ? s.sigmaMax : 0.0);
		uY.append(s.uMax >= 0.0 ? s.uMax : 0.0);
	}

	_cpressStdScale = sampleStd(cpressY);
	_edgeLoadRatioStdScale = sampleStd(edgeY);
	_cpressCVStdScale = sampleStd(cvY);
	_sigmaStdScale = sampleStd(sigmaY);
	_uStdScale = sampleStd(uY);
	_edgeYMean = sampleMean(edgeY);
	_edgeYStd = _edgeLoadRatioStdScale;
	_edgeYMin = sampleMin(edgeY);
	_edgeYMax = sampleMax(edgeY);
	_cpressYMean = sampleMean(cpressY);
	_cpressYStd = _cpressStdScale;
	_cpressYMin = sampleMin(cpressY);
	_cpressYMax = sampleMax(cpressY);
	_cpressWeights = solveRbfWeights(normalizeTargets(cpressY, _cpressYMean, _cpressYStd));
	_edgeLoadRatioWeights = solveRbfWeights(normalizeTargets(edgeY, _edgeYMean, _edgeYStd));
	_cpressCVWeights = solveRbfWeights(cvY);
	_sigmaWeights = solveRbfWeights(sigmaY);
	_uWeights = solveRbfWeights(uY);
	_ready = !_cpressWeights.isEmpty() && !_edgeLoadRatioWeights.isEmpty();
	return _ready;
}

SurrogatePrediction GearSurrogateModel::predict(const QVector<double>& vars) const
{
	SurrogatePrediction p;
	if (!_ready || vars.size() != _dim)
		return p;

	const QVector<double> xNorm = normalized(vars);
	auto eval = [&](const QVector<double>& weights) {
		if (weights.isEmpty())
			return -1.0;
		double y = 0.0;
		for (int i = 0; i < _xNorm.size(); ++i)
			y += weights[i] * kernel(xNorm, _xNorm[i]);
		return y;
	};

	p.cpressMaxPred = clipToRange(
	    denormalizeTarget(eval(_cpressWeights), _cpressYMean, _cpressYStd),
	    _cpressYMin, _cpressYMax);
	p.edgeLoadRatioPred = clipToRange(
	    denormalizeTarget(eval(_edgeLoadRatioWeights), _edgeYMean, _edgeYStd),
	    _edgeYMin, _edgeYMax);
	p.cpressCVPred = eval(_cpressCVWeights);
	p.sigmaMaxPred = eval(_sigmaWeights);
	p.uMaxPred = eval(_uWeights);

	const double d = nearestNormalizedDistance(xNorm);
	p.cpressMaxStd = _cpressStdScale * d;
	p.edgeLoadRatioStd = _edgeLoadRatioStdScale * d;
	p.cpressCVStd = _cpressCVStdScale * d;
	p.sigmaMaxStd = _sigmaStdScale * d;
	p.uMaxStd = _uStdScale * d;
	return p;
}

QVector<double> GearSurrogateModel::solveRbfWeights(const QVector<double>& y) const
{
	const int n = _xNorm.size();
	if (y.size() != n || n <= 0)
		return {};

	QVector<QVector<double>> a(n, QVector<double>(n + 1, 0.0));
	for (int r = 0; r < n; ++r) {
		for (int c = 0; c < n; ++c)
			a[r][c] = kernel(_xNorm[r], _xNorm[c]);
		a[r][r] += 1e-4;
		a[r][n] = y[r];
	}

	for (int col = 0; col < n; ++col) {
		int pivot = col;
		double best = std::abs(a[col][col]);
		for (int r = col + 1; r < n; ++r) {
			const double v = std::abs(a[r][col]);
			if (v > best) {
				best = v;
				pivot = r;
			}
		}
		if (best < 1e-14)
			return {};
		if (pivot != col)
			std::swap(a[pivot], a[col]);

		const double div = a[col][col];
		for (int c = col; c <= n; ++c)
			a[col][c] /= div;

		for (int r = 0; r < n; ++r) {
			if (r == col)
				continue;
			const double f = a[r][col];
			if (std::abs(f) < 1e-20)
				continue;
			for (int c = col; c <= n; ++c)
				a[r][c] -= f * a[col][c];
		}
	}

	QVector<double> w(n);
	for (int i = 0; i < n; ++i)
		w[i] = a[i][n];
	return w;
}

QVector<double> GearSurrogateModel::normalized(const QVector<double>& x) const
{
	QVector<double> out;
	out.reserve(_dim);
	for (int i = 0; i < _dim; ++i)
		out.append((x[i] - _center[i]) / _scale[i]);
	return out;
}

double GearSurrogateModel::kernel(const QVector<double>& a, const QVector<double>& b) const
{
	const double r = vectorDistance(a, b);
	return std::exp(-sqr(_epsilon * r));
}

double GearSurrogateModel::nearestNormalizedDistance(const QVector<double>& xNorm) const
{
	double best = std::numeric_limits<double>::max();
	for (const QVector<double>& x : _xNorm)
		best = std::min(best, vectorDistance(xNorm, x));
	return best == std::numeric_limits<double>::max() ? 0.0 : best;
}

double GearSurrogateModel::trainingMaxRelativeError(const QString& target) const
{
	if (!_ready || _samples.isEmpty())
		return -1.0;

	const bool edgeTarget = target == QStringLiteral("edgeLoadRatio");
	double maxRel = 0.0;
	for (const SurrogateSample& s : _samples) {
		const SurrogatePrediction pred = predict(s.x);
		const double truth = edgeTarget ? s.edgeLoadRatio : s.cpressMax;
		const double predVal = edgeTarget ? pred.edgeLoadRatioPred : pred.cpressMaxPred;
		if (truth > 1e-12 && predVal >= 0.0 && std::isfinite(predVal)) {
			maxRel = std::max(maxRel, std::abs(truth - predVal) / std::abs(truth));
		}
	}
	return maxRel;
}

} // namespace GearAutoOpt
