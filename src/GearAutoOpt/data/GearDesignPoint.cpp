#include "GearDesignPoint.h"
#include "GearOptConfig.h"
#include "GearAutoOpt/solver/CCXSolverController.h"

#include <QDebug>
#include <QDir>
#include <QJsonObject>
#include <QJsonArray>
#include <QString>
#include <QStringList>
#include <cmath>

namespace GearAutoOpt {

QString pointStatusToString(PointStatus s)
{
	switch(s) {
	case PointStatus::Pending:    return QStringLiteral("pending");
	case PointStatus::Running:    return QStringLiteral("running");
	case PointStatus::Done:       return QStringLiteral("done");
	case PointStatus::Failed:     return QStringLiteral("failed");
	case PointStatus::Infeasible: return QStringLiteral("infeasible");
	}
	return QStringLiteral("pending");
}

PointStatus pointStatusFromString(const QString& s)
{
	const QString k = s.toLower();
	if(k == "pending")    return PointStatus::Pending;
	if(k == "running")    return PointStatus::Running;
	if(k == "done")       return PointStatus::Done;
	if(k == "failed")     return PointStatus::Failed;
	if(k == "infeasible") return PointStatus::Infeasible;
	return PointStatus::Pending;
}

GearDesignPoint::GearDesignPoint()
	: createdAt(QDateTime::currentDateTime())
	, updatedAt(createdAt)
{
}

QJsonObject GearDesignPoint::toJson() const
{
	QJsonObject obj;
	obj["id"]            = id;
	obj["generation"]    = generation;

	QJsonArray parents;
	for(int p : parentIds) parents.append(p);
	obj["parent_ids"]    = parents;

	obj["module"]        = module;
	obj["z1"]            = z1;
	obj["z2"]            = z2;
	obj["alpha"]         = alpha;
	obj["x1"]            = x1;
	obj["x2"]            = x2;
	obj["ca1"]           = ca1;
	obj["lca1"]          = lca1;
	obj["ca2"]           = ca2;
	obj["lca2"]          = lca2;
	obj["width"]         = commonWidth;
	obj["common_width"]  = commonWidth;
	obj["hub_ratio"]     = hubRatio;

	obj["mesh_size_mm"]      = meshSize_mm;
	obj["mesh_auto"]         = meshAuto;
	obj["mesh_method"]       = meshMethod;
	obj["element_order"]     = elementOrder;
	obj["node_count"]        = nodeCount;
	obj["element_count"]     = elementCount;
	obj["torque_Nm"]         = torque_Nm;
	obj["contact_stiffness"] = contactStiffness;
	obj["enable_contact"]    = enableContact;
	obj["friction_coeff"]    = frictionCoeff;
	obj["material_name"]     = materialName;
	obj["young_modulus_MPa"] = youngModulus_MPa;
	obj["poisson_ratio"]     = poissonRatio;
	obj["density"]           = density;
	obj["solver_path"]       = solverPath;

	obj["status"]        = pointStatusToString(status);
	obj["error_msg"]     = errorMsg;
	obj["run_dir"]       = runDir;

	obj["sigma_max"]          = sigmaMax;
	obj["u_max"]              = uMax;
	obj["mass"]               = mass;
	obj["sigma_max_total"]    = sigmaMax_total;
	obj["sigma_max_gear1"]    = sigmaMax_gear1;
	obj["sigma_max_gear2"]    = sigmaMax_gear2;
	obj["sigma_max_gear1_elem"] = sigmaMax_gear1_elem;
	obj["sigma_max_gear1_ip"]   = sigmaMax_gear1_ip;
	obj["sigma_max_gear2_elem"] = sigmaMax_gear2_elem;
	obj["sigma_max_gear2_ip"]   = sigmaMax_gear2_ip;
	obj["u_max_total"]        = uMax_total;
	obj["u_max_gear1"]        = uMax_gear1;
	obj["u_max_gear2"]        = uMax_gear2;
	obj["mass_total"]         = mass_total;
	obj["mass_gear1"]         = mass_gear1;
	obj["mass_gear2"]         = mass_gear2;
	obj["cpress_max_MPa"]     = cpressMax_MPa;
	obj["cpress_mean_MPa"]    = cpressMean_MPa;
	obj["cpress_std_MPa"]     = cpressStd_MPa;
	obj["cpress_cv"]          = cpressCV;
	obj["contact_width_mm"]   = contactWidth_mm;
	obj["edge_load_ratio"]    = edgeLoadRatio;
	obj["cpress_edge_mean_MPa"] = cpressEdgeMean_MPa;
	obj["cpress_center_mean_MPa"] = cpressCenterMean_MPa;
	obj["cpress_active_nodes"] = cpressActiveNodes;
	obj["cpress_bin_count"]   = cpressBinCount;

	obj["solver"]        = solver;
	obj["solver_time"]   = solverTime;
	obj["created_at"]    = createdAt.toString(Qt::ISODate);
	obj["updated_at"]    = updatedAt.toString(Qt::ISODate);
	return obj;
}

GearDesignPoint GearDesignPoint::fromJson(const QJsonObject& obj)
{
	GearDesignPoint p;
	p.id            = obj.value("id").toInt(-1);
	p.generation    = obj.value("generation").toInt(0);

	const QJsonArray parents = obj.value("parent_ids").toArray();
	for(const auto& v : parents) p.parentIds.append(v.toInt());

	p.module        = obj.value("module").toDouble(2.5);
	p.z1            = obj.value("z1").toInt(26);
	p.z2            = obj.value("z2").toInt(26);
	p.alpha         = obj.value("alpha").toDouble(20.0);
	p.x1            = obj.value("x1").toDouble(0.0);
	p.x2            = obj.value("x2").toDouble(0.0);
	p.ca1           = obj.value("ca1").toDouble(0.0);
	p.lca1          = obj.value("lca1").toDouble(0.0);
	p.ca2           = obj.value("ca2").toDouble(0.0);
	p.lca2          = obj.value("lca2").toDouble(0.0);
	if (obj.contains(QStringLiteral("common_width")))
		p.commonWidth = obj.value(QStringLiteral("common_width")).toDouble(10.0);
	else if (obj.contains(QStringLiteral("pair_width")))
		p.commonWidth = obj.value(QStringLiteral("pair_width")).toDouble(10.0);
	else if (obj.contains(QStringLiteral("width")))
		p.commonWidth = obj.value(QStringLiteral("width")).toDouble(10.0);
	else
		p.commonWidth = 10.0;
	// 历史字段 width1/width2：取较大者并告警由调用方处理
	if (obj.contains(QStringLiteral("width1")) || obj.contains(QStringLiteral("width2"))) {
		const double w1 = obj.value(QStringLiteral("width1")).toDouble(p.commonWidth);
		const double w2 = obj.value(QStringLiteral("width2")).toDouble(p.commonWidth);
		p.commonWidth   = std::max(w1, w2);
	}
	p.syncPairGearWidth();
	p.hubRatio      = obj.value("hub_ratio").toDouble(0.4);

	p.meshSize_mm      = obj.value("mesh_size_mm").toDouble(0.0);
	p.meshAuto         = obj.value("mesh_auto").toBool(true);
	p.meshMethod       = obj.value("mesh_method").toString(QStringLiteral("gmsh"));
	p.elementOrder     = obj.value("element_order").toInt(1);
	p.nodeCount        = obj.value("node_count").toInt(0);
	p.elementCount     = obj.value("element_count").toInt(0);
	p.torque_Nm        = obj.value("torque_Nm").toDouble(1.0);
	p.contactStiffness = obj.value("contact_stiffness").toDouble(100.0);
	p.enableContact    = obj.value("enable_contact").toBool(false);
	p.frictionCoeff    = obj.value("friction_coeff").toDouble(0.0);
	p.materialName     = obj.value("material_name").toString(QStringLiteral("Steel"));
	p.youngModulus_MPa = obj.value("young_modulus_MPa").toDouble(2.06e5);
	p.poissonRatio     = obj.value("poisson_ratio").toDouble(0.30);
	p.density          = obj.value("density").toDouble(7.85e-9);
	p.solverPath       = obj.value("solver_path").toString();

	p.status        = pointStatusFromString(obj.value("status").toString("pending"));
	p.errorMsg      = obj.value("error_msg").toString();
	p.runDir        = obj.value("run_dir").toString();

	p.sigmaMax          = obj.value("sigma_max").toDouble(-1.0);
	p.uMax              = obj.value("u_max").toDouble(-1.0);
	p.mass              = obj.value("mass").toDouble(-1.0);
	p.sigmaMax_total    = obj.value("sigma_max_total").toDouble(p.sigmaMax);
	p.sigmaMax_gear1    = obj.value("sigma_max_gear1").toDouble(-1.0);
	p.sigmaMax_gear2    = obj.value("sigma_max_gear2").toDouble(-1.0);
	p.sigmaMax_gear1_elem = obj.value("sigma_max_gear1_elem").toInt(-1);
	p.sigmaMax_gear1_ip   = obj.value("sigma_max_gear1_ip").toInt(-1);
	p.sigmaMax_gear2_elem = obj.value("sigma_max_gear2_elem").toInt(-1);
	p.sigmaMax_gear2_ip   = obj.value("sigma_max_gear2_ip").toInt(-1);
	p.uMax_total        = obj.value("u_max_total").toDouble(p.uMax);
	p.uMax_gear1        = obj.value("u_max_gear1").toDouble(-1.0);
	p.uMax_gear2        = obj.value("u_max_gear2").toDouble(-1.0);
	p.mass_total        = obj.value("mass_total").toDouble(p.mass);
	p.mass_gear1        = obj.value("mass_gear1").toDouble(-1.0);
	p.mass_gear2        = obj.value("mass_gear2").toDouble(-1.0);
	p.cpressMax_MPa     = obj.value("cpress_max_MPa").toDouble(-1.0);
	p.cpressMean_MPa    = obj.value("cpress_mean_MPa").toDouble(-1.0);
	p.cpressStd_MPa     = obj.value("cpress_std_MPa").toDouble(-1.0);
	p.cpressCV          = obj.value("cpress_cv").toDouble(-1.0);
	p.contactWidth_mm   = obj.value("contact_width_mm").toDouble(-1.0);
	p.edgeLoadRatio     = obj.value("edge_load_ratio").toDouble(-1.0);
	p.cpressEdgeMean_MPa = obj.value("cpress_edge_mean_MPa").toDouble(-1.0);
	p.cpressCenterMean_MPa = obj.value("cpress_center_mean_MPa").toDouble(-1.0);
	p.cpressActiveNodes = obj.value("cpress_active_nodes").toInt(0);
	p.cpressBinCount    = obj.value("cpress_bin_count").toInt(0);
	p.syncLegacyResultFields();

	p.solver        = obj.value("solver").toString("ccx");
	p.solverTime    = obj.value("solver_time").toDouble(-1.0);
	if(obj.contains("created_at"))
		p.createdAt = QDateTime::fromString(obj.value("created_at").toString(), Qt::ISODate);
	if(obj.contains("updated_at"))
		p.updatedAt = QDateTime::fromString(obj.value("updated_at").toString(), Qt::ISODate);
	return p;
}

void GearDesignPoint::syncPairGearWidth()
{
	// 单字段 commonWidth 已表示副齿宽；保留接口供 LHS/NSGA 与几何步骤显式调用。
}

void GearDesignPoint::applyRunSimDefaults(const GearOptConfig& cfg,
                                          double fixedMeshSizeMm,
                                          bool   runMeshAuto)
{
	meshSize_mm      = fixedMeshSizeMm;
	meshAuto         = runMeshAuto;
	meshMethod       = QStringLiteral("gmsh");
	elementOrder     = 1;
	torque_Nm        = cfg.solver.torque;
	contactStiffness = cfg.solver.contactStiffness;
	enableContact    = cfg.solver.enableContact;
	frictionCoeff    = 0.0;
	materialName     = QStringLiteral("STEEL");
	youngModulus_MPa = cfg.solver.youngModulus > 0.0 ? cfg.solver.youngModulus : 206000.0;
	poissonRatio     = cfg.solver.poissonRatio > 0.0 ? cfg.solver.poissonRatio : 0.30;
	density          = cfg.solver.density > 0.0 ? cfg.solver.density : 7.85e-9;
	contactType      = QStringLiteral("surface_to_surface_penalty");
	staticStep       = QStringLiteral("STATIC");
	solver           = QStringLiteral("CalculiX");
	solverPath       = CCXSolverController::detectCcxPath();
	solverVersion.clear();
}

void GearDesignPoint::fillResultArtifactPaths()
{
	if (runDir.isEmpty())
		return;
	const QDir d(runDir);
	meshInpPath = d.filePath(QStringLiteral("mesh.inp"));
	jobInpPath  = d.filePath(QStringLiteral("job.inp"));
	datPath     = d.filePath(QStringLiteral("job.dat"));
	frdPath     = d.filePath(QStringLiteral("job.frd"));
}

void GearDesignPoint::syncLegacyResultFields()
{
	auto peak2 = [](double a, double b) -> double {
		if (a < 0 && b < 0)
			return -1.0;
		if (a < 0)
			return b;
		if (b < 0)
			return a;
		return std::max(a, b);
	};

	sigmaMax_total = peak2(sigmaMax_gear1, sigmaMax_gear2);
	uMax_total     = peak2(uMax_gear1, uMax_gear2);

	if (mass_gear1 >= 0 && mass_gear2 >= 0)
		mass_total = mass_gear1 + mass_gear2;
	else if (mass_gear1 >= 0)
		mass_total = mass_gear1;
	else if (mass_gear2 >= 0)
		mass_total = mass_gear2;
	else
		mass_total = -1.0;

	if (sigmaMax_total >= 0)
		sigmaMax = sigmaMax_total;
	if (uMax_total >= 0)
		uMax = uMax_total;
	if (mass_total >= 0)
		mass = mass_total;
}

bool GearDesignPoint::isFeasible(QString* msg) const
{
	auto fail = [&](const QString& m) {
		if(msg) *msg = m;
		return false;
	};

	if(module <= 0.0)              return fail("module must be > 0");
	if(z1 < 6 || z2 < 6)           return fail("tooth count must be >= 6");
	if(alpha <= 0.0 || alpha > 45) return fail("alpha out of [0, 45]");
	if (commonWidth <= 0.0)
		return fail(QStringLiteral("commonWidth must be > 0"));

	// 不根切（标准齿条刀具）：z >= 2*ha*/sin^2(alpha) - 2*x/sin^2(alpha)。
	// 这里取 ha* = 1（齿顶高系数），简化检查。
	const double phi    = alpha * M_PI / 180.0;
	const double sin2   = std::sin(phi) * std::sin(phi);
	if(sin2 > 1e-9) {
		const double zMin1 = 2.0 * (1.0 - x1) / sin2;
		const double zMin2 = 2.0 * (1.0 - x2) / sin2;
		if(z1 < zMin1) return fail(QString("undercut on gear 1 (z1=%1 < %2)").arg(z1).arg(zMin1));
		if(z2 < zMin2) return fail(QString("undercut on gear 2 (z2=%1 < %2)").arg(z2).arg(zMin2));
	}

	// 不变尖（齿顶厚 sa >= 0.25 m）— 这里只做粗略判定。
	// sa ≈ d_a * (s/d - inv(alpha_a) + inv(alpha)) ; 简化为基于 x 的下限阈值。
	if(x1 > 0.8 || x2 > 0.8) return fail("profile shift too large (>0.8 may cause tip-pointing)");

	return true;
}

namespace {

bool valueInDiscrete(double val, const Bounds& b, double eps = 1e-6)
{
	for (double d : b.discreteValues) {
		if (std::abs(val - d) <= eps)
			return true;
	}
	return false;
}

void warnBound(const char* field, double value, const Bounds& b)
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

void checkGearOptDesignPointBounds(const GearDesignPoint& dp, const GearOptConfig& cfg)
{
	const Bounds& mb = cfg.moduleBound;
	if (!mb.discreteValues.isEmpty()) {
		if (!valueInDiscrete(dp.module, mb))
			warnBound("module", dp.module, mb);
	} else if (dp.module < mb.lower - 1e-9 || dp.module > mb.upper + 1e-9) {
		warnBound("module", dp.module, mb);
	}

	const Bounds& bz1 = cfg.z1Bound;
	const int     z1Lo = static_cast<int>(std::lround(bz1.lower));
	const int     z1Hi = static_cast<int>(std::lround(bz1.upper));
	if (dp.z1 < z1Lo || dp.z1 > z1Hi)
		warnBound("z1", static_cast<double>(dp.z1), bz1);

	const Bounds& bz2 = cfg.z2Bound;
	const int     z2Lo = static_cast<int>(std::lround(bz2.lower));
	const int     z2Hi = static_cast<int>(std::lround(bz2.upper));
	if (dp.z2 < z2Lo || dp.z2 > z2Hi)
		warnBound("z2", static_cast<double>(dp.z2), bz2);

	const Bounds& ab = cfg.alphaBound;
	if (!ab.discreteValues.isEmpty()) {
		if (!valueInDiscrete(dp.alpha, ab))
			warnBound("alpha", dp.alpha, ab);
	} else if (dp.alpha < ab.lower - 1e-9 || dp.alpha > ab.upper + 1e-9) {
		warnBound("alpha", dp.alpha, ab);
	}

	if (dp.x1 < cfg.x1Bound.lower - 1e-9 || dp.x1 > cfg.x1Bound.upper + 1e-9)
		warnBound("x1", dp.x1, cfg.x1Bound);
	if (dp.x2 < cfg.x2Bound.lower - 1e-9 || dp.x2 > cfg.x2Bound.upper + 1e-9)
		warnBound("x2", dp.x2, cfg.x2Bound);
	if (dp.ca1 < cfg.ca1Bound.lower - 1e-9 || dp.ca1 > cfg.ca1Bound.upper + 1e-9)
		warnBound("ca1", dp.ca1, cfg.ca1Bound);
	if (dp.lca1 < cfg.lca1Bound.lower - 1e-9 || dp.lca1 > cfg.lca1Bound.upper + 1e-9)
		warnBound("lca1", dp.lca1, cfg.lca1Bound);
	if (dp.ca2 < cfg.ca2Bound.lower - 1e-9 || dp.ca2 > cfg.ca2Bound.upper + 1e-9)
		warnBound("ca2", dp.ca2, cfg.ca2Bound);
	if (dp.lca2 < cfg.lca2Bound.lower - 1e-9 || dp.lca2 > cfg.lca2Bound.upper + 1e-9)
		warnBound("lca2", dp.lca2, cfg.lca2Bound);
	if (dp.commonWidth < cfg.widthBound.lower - 1e-9 || dp.commonWidth > cfg.widthBound.upper + 1e-9)
		warnBound("commonWidth", dp.commonWidth, cfg.widthBound);
	if (dp.hubRatio < cfg.hubRatioBound.lower - 1e-9 || dp.hubRatio > cfg.hubRatioBound.upper + 1e-9)
		warnBound("hubRatio", dp.hubRatio, cfg.hubRatioBound);
}

} // namespace GearAutoOpt
