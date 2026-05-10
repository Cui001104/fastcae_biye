#ifndef _GEARAUTOOPT_GEARDESIGNPOINT_H_
#define _GEARAUTOOPT_GEARDESIGNPOINT_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QString>
#include <QDateTime>
#include <QList>

class QJsonObject;

namespace GearAutoOpt {

/// 工况状态。
enum class PointStatus {
	Pending = 0,    ///< 待求解
	Running,        ///< 求解中
	Done,           ///< 求解成功，σ_max 等已写入
	Failed,         ///< 求解失败 (CCX 崩溃 / 网格问题)
	Infeasible      ///< 几何/物理约束不通过 (根切等)，跳过求解
};

GEARAUTOOPTAPI QString  pointStatusToString(PointStatus s);
GEARAUTOOPTAPI PointStatus pointStatusFromString(const QString& s);

/// 一个齿轮设计点：设计变量 + 求解响应 + 元数据。
/// 一行对应 SQLite design_points 表中一条记录。
struct GEARAUTOOPTAPI GearDesignPoint {
	// ---- 数据库主键 ----
	int       id = -1;            ///< -1 = 未入库
	int       generation = 0;     ///< 优化代数；0 = 初始采样
	QList<int> parentIds;         ///< 来源父代 id（NSGA-II 交叉时填写）

	// ---- 设计变量 ----
	double module      = 2.5;     ///< 模数 m [mm]
	int    z1          = 26;      ///< 主齿轮齿数
	int    z2          = 26;      ///< 副齿轮齿数
	double alpha       = 20.0;    ///< 压力角 [deg]
	double x1          = 0.0;     ///< 主齿轮变位系数
	double x2          = 0.0;     ///< 副齿轮变位系数
	double ca1         = 0.0;     ///< 主齿轮齿顶修型量 [mm]
	double lca1        = 0.0;     ///< 主齿轮齿顶修型长度 [mm]
	double ca2         = 0.0;     ///< 副齿轮齿顶修型量 [mm]
	double lca2        = 0.0;     ///< 副齿轮齿顶修型长度 [mm]
	double width       = 10.0;    ///< 齿宽 b [mm]
	double hubRatio    = 0.4;     ///< 轮毂内径 / 分度圆直径

	// ---- 工况状态 ----
	PointStatus status = PointStatus::Pending;
	QString  errorMsg;            ///< Failed/Infeasible 时的原因
	QString  runDir;              ///< 求解工作目录绝对路径，如 runs/0_42/

	// ---- 求解响应 ----
	double sigmaMax    = -1.0;    ///< 齿根最大 von Mises [MPa]，-1 表示未求解
	double uMax        = -1.0;    ///< 最大位移 [mm]
	double mass        = -1.0;    ///< 主齿轮质量 [kg]

	// ---- 元数据 ----
	QString  solver = "ccx";      ///< 使用的求解器名
	double   solverTime = -1.0;   ///< 求解 wall time [s]
	QDateTime createdAt;          ///< 入库时间，构造时填当前时间
	QDateTime updatedAt;

	GearDesignPoint();

	/// 序列化到 QJsonObject。
	QJsonObject toJson() const;
	/// 从 QJsonObject 反序列化。
	static GearDesignPoint fromJson(const QJsonObject& obj);

	/// 简化的几何可行性检查（不根切、不变尖等）。返回 true 表示可送去求解。
	/// 若返回 false，msg 写入失败原因。
	bool isFeasible(QString* msg = nullptr) const;
};

} // namespace GearAutoOpt

#endif
