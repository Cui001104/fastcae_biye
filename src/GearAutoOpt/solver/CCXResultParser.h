#ifndef _GEARAUTOOPT_CCX_RESULT_PARSER_H_
#define _GEARAUTOOPT_CCX_RESULT_PARSER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QHash>
#include <QString>
#include <QVector>

namespace GearAutoOpt {

/// 一条应力记录：CalculiX *EL PRINT,S 在 .dat 中输出的一行。
/// 8 个数：elem_id, integ_pt, sxx, syy, szz, sxy, sxz, syz
struct GEARAUTOOPTAPI StressSample {
	int    elemId   = 0;
	int    integPt  = 0;
	double sxx      = 0.0;
	double syy      = 0.0;
	double szz      = 0.0;
	double sxy      = 0.0;
	double sxz      = 0.0;
	double syz      = 0.0;

	/// 计算该样本的 von Mises 应力。
	double vonMises() const;
};

/// .dat 解析后的目标 elset 段结果。
struct GEARAUTOOPTAPI DatStressResult {
	QString elsetName;          ///< 命中的 elset 名（按入参规范化）
	int     sampleCount = 0;    ///< 段内样本（行）数
	double  maxVonMises = -1.0; ///< 段内最大 von Mises [MPa]，-1 = 未找到
	int     maxElemId   = -1;   ///< 最大 von Mises 所在 element id
	int     maxIntegPt  = -1;   ///< 对应积分点 id

	bool found() const { return sampleCount > 0; }
};

/// 解析 ccx 输出的 .dat 文件，定位某个 elset 段（"...for set <NAME> and time..."），
/// 遍历段内所有应力样本，计算并返回 von Mises 极值。
///
/// 同一个 elset 在多个 step / time 下可能多次出现，本函数返回**所有同名段合并后**
/// 的最大值（保留最严苛工况，符合静力分析里"取应力包络"的语义）。
///
/// elsetName 大小写不敏感。
GEARAUTOOPTAPI DatStressResult parseDat(const QString& path, const QString& elsetName);

/// 解析 .dat 中某个 elset 段的所有应力样本（不计算极值，调试用）。
GEARAUTOOPTAPI QVector<StressSample> parseDatSamples(const QString& path,
                                                      const QString& elsetName);

// =====================================================================
// .frd 备份解析
// =====================================================================

/// 一个节点位置（.frd 节点块）。
struct GEARAUTOOPTAPI NodeXYZ {
	double x = 0.0, y = 0.0, z = 0.0;
};

/// 一个节点位移（DISP 块的一行）。
struct GEARAUTOOPTAPI NodeDisp {
	double ux = 0.0, uy = 0.0, uz = 0.0;

	double magnitude() const;
};

/// 一个节点应力（STRESS 块的一行，CCX 顺序：SXX, SYY, SZZ, SXY, SYZ, SZX）。
struct GEARAUTOOPTAPI NodeStress {
	double sxx = 0.0, syy = 0.0, szz = 0.0;
	double sxy = 0.0, syz = 0.0, szx = 0.0;

	double vonMises() const;
};

/// .frd 整体解析结果。键全为 1-based node id（CalculiX 习惯）。
struct GEARAUTOOPTAPI FrdResult {
	QHash<int, NodeXYZ>    nodes;
	QHash<int, NodeDisp>   displacements;
	QHash<int, NodeStress> stresses;

	int  nodeCount() const         { return nodes.size(); }
	int  displCount() const        { return displacements.size(); }
	int  stressCount() const       { return stresses.size(); }

	/// 节点应力中最大 von Mises（找不到返回 -1）。
	double maxVonMises(int* outNodeId = nullptr) const;
	/// 节点位移中最大幅值（找不到返回 -1）。
	double maxDisplMagnitude(int* outNodeId = nullptr) const;
};

/// 解析整个 .frd 文件。文件不存在或格式损坏返回空 FrdResult。
GEARAUTOOPTAPI FrdResult parseFrd(const QString& path);

} // namespace GearAutoOpt

#endif
