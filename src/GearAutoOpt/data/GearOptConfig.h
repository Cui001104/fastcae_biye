#ifndef _GEARAUTOOPT_GEAROPTCONFIG_H_
#define _GEARAUTOOPT_GEAROPTCONFIG_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearLogLevel.h"

#include <QString>
#include <QVector>

class QJsonObject;

namespace GearAutoOpt {

/// 单个连续变量的上下界。
struct GEARAUTOOPTAPI Bounds {
	double lower = 0.0;
	double upper = 1.0;
	bool   isInteger = false;        ///< 整数变量取整后再 clip
	QVector<double> discreteValues;  ///< 非空 → 离散变量，从该集合采样
};

/// 优化目标的开关 (true = 参与目标函数)。
struct GEARAUTOOPTAPI Objectives {
	bool   minEdgeLoadRatio = true;
	bool   minCpressCV = false;
	bool   minCpressMax = true;  ///< 最小化最大接触压力 cpressMax_MPa
	bool   minSigmaMax  = true;  ///< 最小化最大应力 sigmaMax_MPa
	bool   minMass      = false; ///< 最小化质量 mass_total_kg（固定齿宽时通常不参与优化）
	bool   minRatioErr = false;      ///< |i_actual - i_target|
	double targetRatio = 1.0;        ///< minRatioErr 启用时的目标传动比
};

/// 物理/工艺约束阈值。
struct GEARAUTOOPTAPI Constraints {
	double sigmaAllow      = 600.0;  ///< 许用应力 [MPa] (45# 钢约 600)
	double minToothCount   = 17;     ///< 单齿轮最少齿数
	double minSaOverM      = 0.25;   ///< 齿顶厚 / 模数 最小值
	double centerDistanceTol = 0.05; ///< |a' - a_target| 上限 [mm]，<=0 表示不约束
	double targetCenterDistance = -1.0;
};

/// NSGA-II 超参数。
struct GEARAUTOOPTAPI NSGA2Params {
	int    populationSize  = 30;     ///< 每代种群大小
	int    maxGenerations  = 20;
	double crossoverRate   = 0.9;
	double mutationRate    = 0.1;    ///< 每个个体变异概率
	double sbxEta          = 15.0;   ///< SBX 交叉分布指数
	double mutEta          = 20.0;   ///< 多项式变异分布指数
	int    randomSeed      = 42;
	double hypervolumeTol  = 1e-3;   ///< 连续 3 代 HV 变化 < tol → 停
	int    convergenceWindow = 3;
};

/// 工况求解相关。
struct GEARAUTOOPTAPI SolverParams {
	QString solver         = "ccx";   ///< "ccx" 或 "z88"
	double  torque         = 1.0;    ///< 输入扭矩 [N·m]（低载荷调试默认 1）
	double  youngModulus   = 2.06e5;  ///< [MPa] (45# 钢)
	double  poissonRatio   = 0.30;
	double  density        = 7.85e-9; ///< [t/mm^3]，CalculiX 单位制
	int     timeoutSeconds = 1200;   ///< CCX 单工况超时 [s]，默认 20 分钟
	bool    keepRunDir     = true;    ///< false 则求解后只保留 .dat
	double  meshSize       = 1.50;    ///< Gmsh 全局网格尺寸 [mm]；0 = 自动 max(0.5×module, meshAutoMinMm)
	double  meshRootSizeMm = 0.40;    ///< 齿根 Distance+Threshold SizeMin [mm]（网格无关性方案 B）
	int     meshZLayers    = 11;      ///< 齿宽方向 Transfinite 层数；-1 = 按齿宽自动
	double  meshAutoMinMm  = 2.0;     ///< meshSize=0 时自动尺寸下限 [mm]
	double  contactPressureP0 = 0.05; ///< EXPONENTIAL 接触 p0（验证模板 0.05）
	double  contactStiffness  = 5.0;  ///< EXPONENTIAL 接触 c0（验证模板 5）
	bool    enableContact  = true;    ///< 齿面 CONTACT PAIR + 位移控制
	bool    useRigidBody   = true;    ///< 双齿轮：*RIGID BODY + CENTER REF/ROT 节点
	bool    useCoupling    = false;   ///< 单齿轮旧路径 *COUPLING
	bool    useKinematicCoupling = true;
	/// CONTACT PAIR 的 ADJUST [mm]；0=双齿轮模板默认 1e-5
	double  contactAdjustMm  = 0.0;
	QString runBaseDir;               ///< 本次优化结果根目录（必选）；其下生成 GearOptResults.db 与 0_0/ 等工况子目录
	int     threads        = 0;       ///< OMP_NUM_THREADS；0 = 跟随系统
	/// infill CCX 验证：多工况并发（仅代理辅助 infill 阶段生效）。
	bool    parallelCcxEnabled = false;
	int     parallelCcxJobs    = 2;   ///< 同时运行的 CCX 工况数
	int     ccxThreadsPerJob     = 4;   ///< 每个 CCX 进程的 OMP 线程数
	/// CENTER1_ROT DOF3 位移幅值 [mm]（DOF1–2 固定 0）。
	double  ccxDriveDisplacementMm = 0.001;
	/// 已弃用（历史转角驱动，保留 JSON 兼容）
	double  ccxDriveRotationRad = 0.0001;
	bool    ccxUseRotationDrive  = false;
	int     ccxRigidHubSamples = 24;
	/// true：代理辅助 NSGA-II（RBF + 稀疏 CCX 验证）；false：全种群 CCX。
	bool    surrogateAssisted  = false;
	/// false = Normal 日志；true = Debug（网格/CCX 识别细节 + qDebug）。
	bool    debugMode          = false;
};

/// 设计变量是否参与 NSGA-II（未勾选则 bounds 固定为基准值）。
struct GEARAUTOOPTAPI DesignVariableFlags {
	bool module   = false;
	bool z1       = false;
	bool z2       = false;
	bool alpha    = false;
	bool x1       = true;
	bool x2       = true;
	bool ca1      = true;
	bool lca1     = true;
	bool ca2      = true;
	bool lca2     = true;
	bool width    = false;  ///< 齿宽固定为基准值，不参与 RBF/代理优化
	bool hubRatio = false;
};

/// 整套优化配置：可加载/保存到 gear_opt_config.json。
struct GEARAUTOOPTAPI GearOptConfig {
	QString      runName = "default";

	// 设计变量上下界（按 GearDesignPoint 字段对应）
	Bounds   moduleBound;       ///< 默认离散 {1.5, 2, 2.5, 3, 4}
	Bounds   z1Bound;           ///< 整数 [17, 40]
	Bounds   z2Bound;           ///< 整数 [25, 80]
	Bounds   alphaBound;        ///< 离散 {20, 22.5, 25}
	Bounds   x1Bound;           ///< [-0.3, 0.6]
	Bounds   x2Bound;
	Bounds   ca1Bound;          ///< 齿顶修形量 [mm]，默认 [0, 0.35]
	Bounds   lca1Bound;         ///< [0.1, 2*m] — ca=0 时 lca 归一化为 0
	Bounds   ca2Bound;
	Bounds   lca2Bound;
	Bounds   widthBound;        ///< [10, 30]
	Bounds   hubRatioBound;     ///< [0.3, 0.5]

	Objectives  objectives;
	Constraints constraints;
	NSGA2Params nsga2;
	SolverParams solver;

	DesignVariableFlags designVars;

	/// 当为 true 时，NSGA-II 初始种群第 0 个体固定为 optimizationBase，设计空间为局部 bounds。
	GearDesignPoint optimizationBase;
	bool          useOptimizationBase = false;

	GearOptConfig();

	QJsonObject toJson() const;
	static GearOptConfig fromJson(const QJsonObject& obj);

	bool saveToFile(const QString& path) const;
	bool loadFromFile(const QString& path);

	/// 标准齿轮变量空间的默认配置。
	static GearOptConfig defaultConfig();

	/// 以当前建模参数为基准的局部优化；designVars 控制各变量固定或局部搜索。
	static GearOptConfig fromBasePoint(const GearDesignPoint& basePoint,
	                                   const DesignVariableFlags& designVars = DesignVariableFlags());
};

} // namespace GearAutoOpt

#endif
