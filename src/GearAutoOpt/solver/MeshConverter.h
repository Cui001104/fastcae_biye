#ifndef _GEARAUTOOPT_MESH_CONVERTER_H_
#define _GEARAUTOOPT_MESH_CONVERTER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QHash>
#include <QList>
#include <QMap>
#include <QPair>
#include <QSet>
#include <QString>
#include <QStringList>

namespace GearAutoOpt {

/// 设为 true 时在 mesh 丰富化 / SurfaceVerify 阶段导出 TOOTH_SURF 调试 VTK；默认关闭以加快流程。
constexpr bool kExportToothSurfDebugVtk = false;

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

struct GEARAUTOOPTAPI GearNodeXYZ {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

/// 齿面外边界面筛选用的几何上下文（与 KEY *SET_SEGMENT 同源面集上的二次筛选）。
struct GEARAUTOOPTAPI GearToothPickContext {
	double module         = 0.0;
	double rootR          = 0.0;
	double pitchR         = 0.0;
	double tipR           = 0.0;
	double hubR           = 0.0;
	double boreTol        = 0.0;
	double zMin           = 0.0;
	double zMax           = 0.0;
	double axisX          = 0.0;   ///< 该齿轮轴线在全局 XY 中的位置
	double axisY          = 0.0;
	int    teethCount     = 0;     ///< 齿数 z（主齿轮 z1）
	double loadToothCount = 3.0;   ///< 啮合区等效载荷齿数（TOOTH_OUTER NSET 角度窗）
	double targetAngleRad = 0.0;   ///< 目标啮合区中心角 [rad]；0 表示默认 π/2（+Y 正上方）
	/// GEAR*_TOOTH_SURF：以 centerToothIndex 为中心，±contactToothHalfSpan 齿（默认 7 齿位）。
	bool localContactSurface    = true;
	int  contactToothHalfSpan   = 3;
	/// true：最外侧两齿各保留半齿面；false（v1）：保留 2*halfSpan+1 个完整齿。
	bool useHalfToothTransition = false;
};

/// 双齿轮 mesh.inp 后处理参数（几何已由 GeoCommandCreateGear 定位，此处不重算中心距）。
struct GEARAUTOOPTAPI GearMeshEnrichParams {
	double module   = 0.0;
	int    z1       = 0;
	int    z2       = 0;
	double x1       = 0.0;
	double x2       = 0.0;
	double hubRatio = 0.4;
	double meshSize = 0.0;
	double axis1X   = 0.0;
	double axis1Y   = 0.0;
	double axis2X   = 0.0;
	double axis2Y   = 0.0;
	bool   twoGears = false;
};

/// enrichGearMeshInp 统计（用于 [GearOpt][Mesh] 日志）。
struct GEARAUTOOPTAPI DualGearMeshReport {
	int gear1NodeCount        = 0;
	int gear2NodeCount        = 0;
	int gear1ElementCount     = 0;
	int gear2ElementCount     = 0;
	int gear1HubNodeCount     = 0;
	int gear2HubNodeCount     = 0;
	int gear1ToothNodeCount   = 0;
	int gear2ToothNodeCount   = 0;
	int gear1ToothSurfCount   = 0;
	int gear2ToothSurfCount   = 0;
	bool gear1LocalContactSurf = false;
	bool gear2LocalContactSurf = false;
	int  gear1CenterToothIndex = 0;
	int  gear2CenterToothIndex = 0;
	QString gear1ToothRangeText;
	QString gear2ToothRangeText;
};

enum class GEARAUTOOPTAPI GearToothFacePickMode {
	ToothBand,       ///< 根圆～顶圆齿带（默认）
	RelaxedRadial,   ///< 略放宽径向，仍排除端面/孔
	ExteriorShell    ///< 外表面壳层：仅排除 hub/端面/孔，径向最宽
};

/// 把 Gmsh -format inp 输出的 mesh.inp 转换为 CalculiX 友好的多文件结构。
GEARAUTOOPTAPI bool convertGmshInpToCcxInp(
	const QString& srcInp,
	const QString& dstDir,
	ConvertReport* report = nullptr
);

GEARAUTOOPTAPI int stripSurfaceElements(const QString& inpPath);

GEARAUTOOPTAPI int countNsetNodes(const QString& inpPath, const QString& nsetName);

/// 读取 mesh.inp 中 *NSET 的节点 id；不存在或为空返回 false。
GEARAUTOOPTAPI bool loadNsetNodeIds(const QString& inpPath,
                                    const QString& nsetName,
                                    QSet<int>* outIds);

/// *SURFACE, NAME=…, TYPE=ELEMENT 中的面片行数；未找到该 surface 返回 -1。
GEARAUTOOPTAPI int countSurfaceFaces(const QString& inpPath, const QString& surfaceName);

GEARAUTOOPTAPI int countElsetNodes(const QString& inpPath, const QString& elsetName);

/// 读取 mesh.inp 中 *ELSET（或 *ELEMENT,ELSET=）对应体单元的全部节点 id。
GEARAUTOOPTAPI bool loadElsetNodeIds(const QString& inpPath,
                                     const QString& elsetName,
                                     QSet<int>* outIds);

/// 与 KEY `buildExportBodies` 一致：体单元各面键 count==1 → 外边界面；再按径向/端面/孔柱/啮合区角度窗筛齿面，收集角点 → *NSET。
GEARAUTOOPTAPI bool collectToothNodesFromSolidExteriorFaces(
	const QString& meshInpPath,
	const QMap<int, GearNodeXYZ>& nodePos,
	const QSet<int>& hubIds,
	const GearToothPickContext& ctx,
	GearToothFacePickMode mode,
	QList<int>* outNodes,
	QString* diag = nullptr);

/// 当 mesh.inp 中已有 GEAR1+GEAR2 体单元时，生成 HUB / TOOTH_OUTER / TOOTH_SURF 集合（可重复调用）。
/// TOOTH_SURF 为啮合区局部外周齿面（默认中心齿 ±3）；同时写出 master/slave 供 *CONTACT PAIR。
GEARAUTOOPTAPI bool enrichGearMeshInp(const QString& meshInpPath,
                                     const GearMeshEnrichParams& params,
                                     DualGearMeshReport* report = nullptr);

GEARAUTOOPTAPI bool meshInpHasElset(const QString& meshInpPath, const QString& elsetName);

GEARAUTOOPTAPI bool meshInpHasNset(const QString& meshInpPath, const QString& nsetName);

/// mesh.inp 含 GEAR1/GEAR2 且尚未生成 GEAR1_HUB 时返回 true。
GEARAUTOOPTAPI bool meshInpNeedsGearSetEnrich(const QString& meshInpPath);

/// 基于 job.inp（及 mesh.inp 回退）中 *SURFACE, TYPE=ELEMENT 行重建三角面并做法向校验。
struct GEARAUTOOPTAPI CcxSurfaceVerifyReport {
	QString surfaceName;
	int     faceCount      = 0;
	int     outwardCount   = 0;
	int     inwardCount    = 0;
	int     invalidElem    = 0;
	int     invalidSide    = 0;
	double  bboxMin[3]     = {0, 0, 0};
	double  bboxMax[3]     = {0, 0, 0};
	bool    hasBbox        = false;
	double  radiusMin      = 0.0;
	double  radiusMax      = 0.0;
	bool    hasRadiusRange = false;
	double  zMin           = 0.0;
	double  zMax           = 0.0;
	bool    hasZRange      = false;
	QString vtkPath;
};

GEARAUTOOPTAPI bool verifyCcxElementSurfaceFaces(const QString& meshInpPath,
                                                 const QString& jobInpPath,
                                                 const QString& surfaceName,
                                                 CcxSurfaceVerifyReport* report = nullptr,
                                                 QString*                  errorMsg = nullptr);

} // namespace GearAutoOpt

#endif
