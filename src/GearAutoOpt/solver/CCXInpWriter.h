#ifndef _GEARAUTOOPT_CCX_INP_WRITER_H_
#define _GEARAUTOOPT_CCX_INP_WRITER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QString>
#include <QVector>

namespace GearAutoOpt {

/// 单个集中力载荷（施加在某 NSET 上的一个自由度）。
struct GEARAUTOOPTAPI CLoadSpec {
	QString nset;        ///< 已在 mesh.inp 中定义的 NSET 名（大小写不敏感）
	int     dof = 2;     ///< 自由度：1=x, 2=y, 3=z
	double  value = 0.0; ///< 力大小，符号即方向 [N]（CalculiX 单位制 [t·mm/s^2]）
};

/// 单个边界条件（固定 NSET 的某段自由度区间）。
struct GEARAUTOOPTAPI BoundarySpec {
	QString nset;        ///< NSET 名
	int     dofStart = 1;
	int     dofEnd   = 3;  ///< [dofStart, dofEnd] 闭区间
};

/// CCXInpWriter 的输入上下文：单次 ccx 静力分析所需全部信息。
///
/// 不含网格本身——网格通过 `meshInpFile` 字段以 `*INCLUDE` 拉入。
/// mesh.inp 内须已存在 `volumeElset`、`boundaries[*].nset`、`cloads[*].nset` 这些名字。
struct GEARAUTOOPTAPI InpContext {
	// ---- 头部元信息（写入注释，不影响求解） ----
	QString runName     = "default";
	int     generation  = 0;
	int     pointId     = 0;

	// ---- 网格 ----
	QString meshInpFile = "mesh.inp";  ///< 相对 job.inp 的路径

	// ---- 材料 ----
	QString materialName = "STEEL";
	double  youngModulus = 2.06e5;     ///< [MPa]
	double  poisson      = 0.30;
	double  density      = 7.85e-9;    ///< [t/mm^3]，<=0 不写 *DENSITY 卡

	// ---- 单元节点关联 ----
	QString volumeElset  = "GEAR1";    ///< 体单元集，用于 *SOLID SECTION
	QString outputNset;                ///< *NODE PRINT 的 NSET，留空则用 volumeElset
	QString outputElset;               ///< *EL PRINT 的 ELSET，留空则用 volumeElset

	// ---- 边界条件 + 载荷 ----
	QVector<BoundarySpec> boundaries;
	QVector<CLoadSpec>    cloads;

	// ---- 输出选项 ----
	bool requestNodeFile = true;       ///< *NODE FILE U（写 .frd）
	bool requestElFile   = true;       ///< *EL FILE S
};

/// 把 ctx 渲染到 `{dir}/job.inp`。
///
/// dir 必须存在（不会自动创建）。已存在的 job.inp 会被覆盖。
/// 返回 true 表示文件写入成功。
GEARAUTOOPTAPI bool writeJobInp(const QString& dir, const InpContext& ctx);

/// 把 ctx 渲染为 inp 字符串（不落盘，主要用于单元测试 / 调试）。
GEARAUTOOPTAPI QString renderJobInp(const InpContext& ctx);

} // namespace GearAutoOpt

#endif
