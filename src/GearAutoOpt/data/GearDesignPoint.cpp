#include "GearDesignPoint.h"

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
	obj["width"]         = width;
	obj["hub_ratio"]     = hubRatio;

	obj["status"]        = pointStatusToString(status);
	obj["error_msg"]     = errorMsg;
	obj["run_dir"]       = runDir;

	obj["sigma_max"]     = sigmaMax;
	obj["u_max"]         = uMax;
	obj["mass"]          = mass;

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
	p.width         = obj.value("width").toDouble(10.0);
	p.hubRatio      = obj.value("hub_ratio").toDouble(0.4);

	p.status        = pointStatusFromString(obj.value("status").toString("pending"));
	p.errorMsg      = obj.value("error_msg").toString();
	p.runDir        = obj.value("run_dir").toString();

	p.sigmaMax      = obj.value("sigma_max").toDouble(-1.0);
	p.uMax          = obj.value("u_max").toDouble(-1.0);
	p.mass          = obj.value("mass").toDouble(-1.0);

	p.solver        = obj.value("solver").toString("ccx");
	p.solverTime    = obj.value("solver_time").toDouble(-1.0);
	if(obj.contains("created_at"))
		p.createdAt = QDateTime::fromString(obj.value("created_at").toString(), Qt::ISODate);
	if(obj.contains("updated_at"))
		p.updatedAt = QDateTime::fromString(obj.value("updated_at").toString(), Qt::ISODate);
	return p;
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
	if(width <= 0.0)               return fail("width must be > 0");

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

} // namespace GearAutoOpt
