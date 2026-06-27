#ifndef _GEARAUTOOPT_GEARDESIGNPOINT_H_
#define _GEARAUTOOPT_GEARDESIGNPOINT_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QString>
#include <QDateTime>
#include <QList>

class QJsonObject;

namespace GearAutoOpt {

class GearOptConfig;

/// 齿顶修形量低于此阈值视为关闭修形。
constexpr double kCaEps = 1e-6;
/// 启用修形时修形长度下限 [mm]。
constexpr double kMinReliefLength = 0.1;

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
	int       id = -1;            ///< 本代个体序号 individual_id；-1 = 未赋值
	int       generation = 0;     ///< 优化代数 gen
	QString   runId;              ///< 本次优化 run_id（同一次优化共用）
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
	double commonWidth = 10.0;    ///< 齿轮副公共齿宽 b [mm]（GEAR1/GEAR2 相同）
	double hubRatio    = 0.4;     ///< 轮毂内径 / 分度圆直径
	double addendumCoeff   = 1.0;   ///< 齿顶高系数 ha*
	double dedendumCoeff   = 1.25;  ///< 齿根高系数 hf*
	double rootFilletCoeff = 0.38;  ///< 齿根圆角系数

	// ---- 仿真配置（与单次 run 的网格/材料/接触/求解器一致，便于结果可比）----
	double   meshSize_mm     = 0.0;   ///< Gmsh 全局尺寸；0 表示自动（0.5×module）
	bool     meshAuto        = true;
	QString  meshMethod      = QStringLiteral("gmsh");
	int      elementOrder    = 1;
	int      nodeCount       = 0;
	int      elementCount    = 0;
	double   torque_Nm       = 100.0;
	double   contactStiffness = 500.0;
	bool     enableContact   = false;
	QString  contactType     = QStringLiteral("surface_to_surface_penalty");
	double   frictionCoeff   = 0.0; ///< 预留，当前无摩擦 penalty contact
	QString  materialName    = QStringLiteral("STEEL");
	double   youngModulus_MPa = 206000.0;
	double   poissonRatio    = 0.30;
	double   density         = 7.85e-9; ///< [t/mm^3]
	QString  solverPath;
	QString  solverVersion;       ///< 可选，ccx 版本字符串
	QString  staticStep        = QStringLiteral("STATIC");
	QString  meshInpPath;
	QString  jobInpPath;
	QString  datPath;
	QString  frdPath;

	// ---- 优化标记（入库 / CSV）----
	int    isPareto           = 0;
	int    rank               = 0;
	double crowdingDistance   = 0.0;

	// ---- 工况状态 ----
	PointStatus status = PointStatus::Pending;
	QString  errorMsg;            ///< Failed/Infeasible 时的原因
	QString  runDir;              ///< 求解工作目录绝对路径，如 runs/0_42/

	// ---- 求解响应（兼容字段 = total）----
	double sigmaMax    = -1.0;    ///< ≡ sigmaMax_total [MPa]，最终步 von Mises max
	double uMax        = -1.0;    ///< ≡ uMax_total [mm]
	double mass        = -1.0;    ///< ≡ mass_total [kg]

	double sigmaMax_total = -1.0;
	double sigmaMax_gear1 = -1.0;
	double sigmaMax_gear2 = -1.0;
	int    sigmaMax_gear1_elem = -1;
	int    sigmaMax_gear1_ip   = -1;
	int    sigmaMax_gear2_elem = -1;
	int    sigmaMax_gear2_ip   = -1;
	double uMax_total     = -1.0;
	double uMax_gear1     = -1.0;
	double uMax_gear2     = -1.0;
	double mass_total     = -1.0;
	double mass_gear1     = -1.0;
	double mass_gear2     = -1.0;

	/// 最终 Step/Increment 的齿面接触压力 CPRESS 最大值 [MPa]；NSGA-II 主目标之一。
	double cpressMax_MPa  = -1.0;
	double cpressMean_MPa       = -1.0;
	double cpressStd_MPa        = -1.0;
	double cpressCV             = -1.0;
	double contactWidth_mm      = -1.0;
	double edgeLoadRatio        = -1.0;
	double cpressEdgeMean_MPa   = -1.0;
	double cpressCenterMean_MPa = -1.0;
	int    cpressActiveNodes    = 0;
	int    cpressBinCount       = 0;

	/// 将分齿轮结果汇总到 *_total，并回填 sigmaMax/uMax/mass 兼容字段。
	void syncLegacyResultFields();

	/// 确保啮合副两齿轮齿宽一致（当前为单字段，供采样/几何前显式调用）。
	void syncPairGearWidth();

	/// 从 GearOptConfig 与本次 run 固定的网格设置写入仿真配置（不覆盖 nodeCount/elementCount/enableContact）。
	void applyRunSimDefaults(const GearOptConfig& cfg,
	                         double fixedMeshSizeMm,
	                         bool   runMeshAuto);

	/// 根据 runDir 填写 mesh/job/dat/frd 绝对路径（求解后调用）。
	void fillResultArtifactPaths();

	// ---- 元数据 ----
	QString  solver = QStringLiteral("CalculiX");
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

/// 修形变量合法性：ca≈0 时强制 lca=0；ca>0 时要求 lca≥kMinReliefLength。
/// 返回 false 时 reason 写入原因；p 中已关闭的修形会被归一化为 ca=lca=0。
GEARAUTOOPTAPI bool validateReliefDesign(GearDesignPoint& p, QString* reason = nullptr);

/// 统一输出无效修形跳过日志。
GEARAUTOOPTAPI void logInvalidReliefDesign(const GearDesignPoint& p, const QString& reason);

} // namespace GearAutoOpt

#endif
