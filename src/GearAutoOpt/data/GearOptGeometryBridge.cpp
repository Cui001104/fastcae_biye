#include "GearOptGeometryBridge.h"
#include "GearDesignPoint.h"
#include "GearOptConfig.h"

#include "Geometry/geometryData.h"
#include "Geometry/geometryModelParaBase.h"
#include "Geometry/geometryParaGear.h"
#include "Geometry/geometrySet.h"

#include <QDebug>
#include <cmath>

namespace GearAutoOpt {

namespace {

static bool valueInDiscrete(double val, const Bounds& b, double eps = 1e-6)
{
	for (double d : b.discreteValues) {
		if (std::abs(val - d) <= eps) return true;
	}
	return false;
}

static void warnBound(const char* field, double value, const Bounds& b)
{
	if (!b.discreteValues.isEmpty()) {
		qWarning().noquote() << QStringLiteral("[GearOpt][BoundsCheck][WARN] field=%1 value=%2 bound=discrete(%3 values)")
		                            .arg(QLatin1String(field))
		                            .arg(value, 0, 'g', 12)
		                            .arg(b.discreteValues.size());
		return;
	}
	qWarning().noquote() << QStringLiteral("[GearOpt][BoundsCheck][WARN] field=%1 value=%2 bound=[%3,%4]")
	                              .arg(QLatin1String(field))
	                              .arg(value, 0, 'g', 12)
	                              .arg(b.lower, 0, 'g', 12)
	                              .arg(b.upper, 0, 'g', 12);
}

} // namespace

GEARAUTOOPTAPI Geometry::GeometryParaGear* findCurrentGeometryParaGear()
{
	auto* gd = Geometry::GeometryData::getInstance();
	if (!gd) return nullptr;
	const int n = gd->getGeometrySetCount();
	for (int i = n - 1; i >= 0; --i) {
		auto* set = gd->getGeometrySetAt(i);
		if (!set) continue;
		auto* para = set->getParameter();
		if (!para || para->getParaType() != Geometry::GeometryParaCreateGear) continue;
		return dynamic_cast<Geometry::GeometryParaGear*>(para);
	}
	return nullptr;
}

GEARAUTOOPTAPI GearDesignPoint gearDesignPointFromGeometryParaGear(Geometry::GeometryParaGear& g)
{
	GearDesignPoint dp;
	dp.module = g.getModule();
	dp.z1 = g.getNumberOfTeeth();
	int z2 = g.getNumberOfSecondTeeth();
	if (z2 <= 0) z2 = dp.z1;
	dp.z2 = z2;
	qDebug().noquote() << QStringLiteral("[GearOpt][GeometryImport] from GeometryParaGear: z1=%1 z2=%2 (not clamped to defaultConfig z bounds)")
	                          .arg(dp.z1)
	                          .arg(dp.z2);
	dp.alpha       = g.getPressureAngle();
	dp.x1          = g.getProfileShiftCoefficient1();
	dp.x2          = g.getProfileShiftCoefficient2();
	dp.ca1         = g.getTipReliefAmount();
	dp.lca1        = g.getTipReliefLength();
	dp.ca2         = g.getTipReliefAmount2();
	dp.lca2        = g.getTipReliefLength2();
	const double w1 = g.getThickness();
	const double w2 = g.getThickness2();
	dp.commonWidth  = w1;
	if (std::fabs(w1 - w2) > 1e-6) {
		qWarning().noquote() << QStringLiteral(
		    "[GearOpt][GeometryImport] thickness1=%1 thickness2=%2 differ; use commonWidth=%3")
		                                .arg(w1, 0, 'g', 8)
		                                .arg(w2, 0, 'g', 8)
		                                .arg(dp.commonWidth, 0, 'g', 8);
	}
	dp.syncPairGearWidth();
	dp.hubRatio = 0.4; ///< GeometryParaGear 无轮毂比字段，与默认 GearDesignPoint 一致
	return dp;
}

GEARAUTOOPTAPI void logGearOptBasePointLine(const GearDesignPoint& dp)
{
	qDebug().noquote() << QStringLiteral("[GearOpt][BasePoint] module=%1, z1=%2, z2=%3, alpha=%4, x1=%5, x2=%6, ca1=%7, lca1=%8, ca2=%9, lca2=%10, width=%11, hubRatio=%12")
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

} // namespace GearAutoOpt
