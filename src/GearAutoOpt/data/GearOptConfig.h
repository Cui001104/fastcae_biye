#ifndef _GEARAUTOOPT_GEAROPTCONFIG_H_
#define _GEARAUTOOPT_GEAROPTCONFIG_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

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
	bool   minSigmaMax = true;
	bool   minMass     = true;
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
	double  torque         = 100.0;   ///< 输入扭矩 [N·m]
	double  youngModulus   = 2.06e5;  ///< [MPa] (45# 钢)
	double  poissonRatio   = 0.30;
	double  density        = 7.85e-9; ///< [t/mm^3]，CalculiX 单位制
	int     timeoutSeconds = 600;
	bool    keepRunDir     = true;    ///< false 则求解后只保留 .dat
	double  meshSize       = 0.0;     ///< Gmsh 全局网格尺寸；0 = 自动（0.5×module）
	QString runBaseDir;               ///< 结果根目录；空 = runs/opt_<时间戳>
	int     threads        = 0;       ///< OMP_NUM_THREADS；0 = 跟随系统
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
	Bounds   ca1Bound;          ///< [0, 0.05]
	Bounds   lca1Bound;         ///< [0, 2*m] — 实际由 module 派生
	Bounds   ca2Bound;
	Bounds   lca2Bound;
	Bounds   widthBound;        ///< [10, 30]
	Bounds   hubRatioBound;     ///< [0.3, 0.5]

	Objectives  objectives;
	Constraints constraints;
	NSGA2Params nsga2;
	SolverParams solver;

	GearOptConfig();

	QJsonObject toJson() const;
	static GearOptConfig fromJson(const QJsonObject& obj);

	bool saveToFile(const QString& path) const;
	bool loadFromFile(const QString& path);

	/// 标准齿轮变量空间的默认配置。
	static GearOptConfig defaultConfig();
};

} // namespace GearAutoOpt

#endif
