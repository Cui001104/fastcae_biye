#ifndef _GEARAUTOOPT_MESH_CONVERTER_H_
#define _GEARAUTOOPT_MESH_CONVERTER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QHash>
#include <QSet>
#include <QString>
#include <QStringList>

namespace GearAutoOpt {

/// 转换报告：MeshConverter::convertGmshInpToCcxInp 完成后的统计信息。
struct GEARAUTOOPTAPI ConvertReport {
	int totalNodes        = 0;
	int totalVolumeElems  = 0;
	int totalSurfaceElems = 0;  ///< 被剥离的面单元数（CPS/S 类型）
	QStringList nsetNames;      ///< 写出的 nset_*.inp 对应名字
	QStringList elsetNames;     ///< 写出的 elset_*.inp 对应名字（不含被剥离面元的 ELSET）
	QHash<QString, int> nsetNodeCount;   ///< NSET 名 → 节点数
	QHash<QString, int> elsetElemCount;  ///< ELSET 名 → 单元数

	/// 简要文字描述，用于日志。
	QString toString() const;
};

/// 把 Gmsh -format inp 输出的 mesh.inp 转换为 CalculiX 友好的多文件结构。
///
/// 输入：Gmsh 在 Mesh.SaveGroupsOfNodes=1 下导出的单个 .inp（含 *Heading / *NODE /
/// *ELEMENT / *ELSET / *NSET 卡）。
///
/// 输出（dstDir 必须已存在）：
///   - mesh_clean.inp       去掉 surface element（CPS3/STRI3/S3/S4 等）后的整文件
///   - nset_<NAME>.inp      每个 *NSET 一个文件（小写文件名，名字保留原大小写）
///   - elset_<NAME>.inp     每个 *ELSET 一个文件（被剥离的 surface elset 仍然保留
///                          其 NSET 用作 BC 引用，但 ELSET 行单独写出供调试）
///
/// 返回 true 表示输入文件可读且已成功写入所有目标文件。
GEARAUTOOPTAPI bool convertGmshInpToCcxInp(
	const QString& srcInp,
	const QString& dstDir,
	ConvertReport* report = nullptr
);

/// 仅做剥离 surface element，原地覆盖 inpPath。返回剥离的元素行数。
/// （供 P0-T02 测试脚本里的同名 strip_surface_elements 替代使用。）
GEARAUTOOPTAPI int stripSurfaceElements(const QString& inpPath);

/// 直接从 mesh.inp 数指定 NSET 的节点数
/// NSET 名按大小写不敏感匹配；找不到返回 -1。
GEARAUTOOPTAPI int countNsetNodes(const QString& inpPath, const QString& nsetName);

/// 直接从 mesh.inp 数指定 ELSET 中所有单元的「去重节点数」。
/// 用于：NSET 节点数 ≈ 物理面 ELSET 上节点数。
GEARAUTOOPTAPI int countElsetNodes(const QString& inpPath, const QString& elsetName);

} // namespace GearAutoOpt

#endif
