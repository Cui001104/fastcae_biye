#include "GearOptCaseRunner.h"
#include "GearAutoOpt/data/GearOptLog.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"
#include "GearAutoOpt/solver/MeshConverter.h"
#include "GearAutoOpt/solver/CCXInpWriter.h"
#include "GearAutoOpt/solver/CCXSolverController.h"
#include "GearAutoOpt/solver/CCXResultParser.h"
#include "GearAutoOpt/data/GearOptGeometryBridge.h"

#include "GeometryCommand/GeoCommandCreateGear.h"
#include "Geometry/geometryData.h"
#include "Geometry/geometrySet.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QMap>
#include <QProcess>
#include <QSet>
#include <QStringList>
#include <QTextStream>

#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>

#include <algorithm>
#include <cmath>
#include <functional>

namespace GearAutoOpt {

namespace {

/// Gmsh 常把体元写在 `*ELEMENT,...,ELSET=Volume1`；job.inp / parseDat 必须与之一致。
QString volumeElsetFromMeshInp(const QString& inpPath)
{
	QFile f(inpPath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return QStringLiteral("BEAM");
	QTextStream ts(&f);
	while (!ts.atEnd()) {
		const QString line = ts.readLine().trimmed();
		if (!line.startsWith(QStringLiteral("*ELEMENT"), Qt::CaseInsensitive))
			continue;
		const QString u = line.toUpper();
		if (!u.contains(QStringLiteral("C3D4")) && !u.contains(QStringLiteral("C3D8"))
		    && !u.contains(QStringLiteral("C3D6")) && !u.contains(QStringLiteral("C3D10")))
			continue;
		const int idx = u.indexOf(QStringLiteral("ELSET="));
		if (idx < 0)
			continue;
		int j = idx + 6;
		while (j < line.size() && (line[j].isSpace() || line[j] == QLatin1Char('=')))
			++j;
		int k = j;
		while (k < line.size() && line[k] != QLatin1Char(',') && !line[k].isSpace())
			++k;
		if (k > j)
			return line.mid(j, k - j);
	}
	return QStringLiteral("BEAM");
}

static bool shapeAxisXY(const TopoDS_Shape& shape, double* ax, double* ay)
{
	if (!ax || !ay || shape.IsNull())
		return false;
	Bnd_Box box;
	BRepBndLib::Add(shape, box);
	if (box.IsVoid())
		return false;
	double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
	box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	*ax = 0.5 * (xmin + xmax);
	*ay = 0.5 * (ymin + ymax);
	return true;
}

/// 按 mesh 齿面 SURFACE（master/slave）与配置决定是否写 *CONTACT PAIR。
void configureCcxContact(InpContext& ctx, const QString& meshInpPath, bool configEnableContact)
{
	const int masterFaces = countSurfaceFaces(meshInpPath, QStringLiteral("master"));
	const int slaveFaces  = countSurfaceFaces(meshInpPath, QStringLiteral("slave"));
	const int c1          = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR1_TOOTH_SURF"));
	const int c2          = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR2_TOOTH_SURF"));

	GEAR_OPT_DEBUG << "[GearOpt][Contact] master face count =" << (masterFaces < 0 ? 0 : masterFaces);
	GEAR_OPT_DEBUG << "[GearOpt][Contact] slave face count =" << (slaveFaces < 0 ? 0 : slaveFaces);
	GEAR_OPT_DEBUG << "[GearOpt][Contact] GEAR1_TOOTH_SURF face count (debug) =" << (c1 < 0 ? 0 : c1);
	GEAR_OPT_DEBUG << "[GearOpt][Contact] GEAR2_TOOTH_SURF face count (debug) =" << (c2 < 0 ? 0 : c2);

	ctx.enableContact = configEnableContact && ctx.twoGearJob && masterFaces > 0 && slaveFaces > 0;

	if (!ctx.enableContact) {
		const char* reason = "disabled in config";
		if (configEnableContact) {
			reason = "not two gear job";
			if (ctx.twoGearJob) {
				if (masterFaces < 0 || slaveFaces < 0)
					reason = "master/slave surface missing";
				else if (masterFaces == 0 || slaveFaces == 0)
					reason = "master/slave face count is zero";
			}
		}
		GEAR_OPT_DEBUG << "[GearOpt][Contact] contact disabled:" << reason;
	}

	GEAR_OPT_DEBUG << "[GearOpt][Contact] contact pair enabled ="
	         << (ctx.enableContact ? "true" : "false");
	GEAR_OPT_DEBUG << "[GearOpt][Contact] contact stiffness =" << ctx.contactStiffness;
	GEAR_OPT_DEBUG << "[GearOpt][Contact] contact adjust mm =" << ctx.contactAdjustMm;
}

/// CCX 2.10 固定 nboun_ 数组；CONTACT PAIR 的 ADJUST 会为大量接触点追加 SPC。
void clampCcxContactAdjustForNboun(InpContext& ctx, const QString& meshInpPath, int meshNodeCount,
                                   const std::function<void(const QString&)>& emitLogFn)
{
	if (ctx.contactAdjustMm <= 0.0 || !ctx.enableContact)
		return;

	const int masterFaces = countSurfaceFaces(meshInpPath, QStringLiteral("master"));
	const int slaveFaces  = countSurfaceFaces(meshInpPath, QStringLiteral("slave"));
	const int faces       = std::max(0, masterFaces) + std::max(0, slaveFaces);

	// 经验阈值：15 万节点或 150+ 接触面时 ADJUST 极易触发 bounadd: increase nboun_
	const bool tooLarge = meshNodeCount > 80000 || faces > 150;
	if (!tooLarge)
		return;

	const QString msg =
	    QStringLiteral("inp_write: contact ADJUST=%1 skipped (nodes=%2 faces=%3, CCX 2.10 nboun limit)")
	        .arg(ctx.contactAdjustMm, 0, 'g', 6)
	        .arg(meshNodeCount)
	        .arg(faces);
	if (emitLogFn && GearOptLog::isDebug())
		emitLogFn(msg);
	if (GearOptLog::isDebug())
		GEAR_OPT_DEBUG_NOQUOTE << "[GearOpt][Contact]" << msg;
	ctx.contactAdjustMm = 0.0;
}

/// 统计 mesh.inp 中 *NODE / *ELEMENT 数据行数（不含关键字行）。
void countMeshInpStats(const QString& inpPath, int* nodeCount, int* elementCount)
{
	if (nodeCount)
		*nodeCount = 0;
	if (elementCount)
		*elementCount = 0;

	QFile f(inpPath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return;

	enum class Section { None, Node, Element };
	Section section = Section::None;

	QTextStream ts(&f);
	while (!ts.atEnd()) {
		const QString line = ts.readLine().trimmed();
		if (line.isEmpty() || line.startsWith(QStringLiteral("**")))
			continue;
		if (line.startsWith(QStringLiteral("*"), Qt::CaseInsensitive)) {
			const QString u = line.toUpper();
			if (u.startsWith(QStringLiteral("*NODE"))
			    && !u.startsWith(QStringLiteral("*NODE PRINT"))
			    && !u.startsWith(QStringLiteral("*NODE FILE")))
				section = Section::Node;
			else if (u.startsWith(QStringLiteral("*ELEMENT")))
				section = Section::Element;
			else
				section = Section::None;
			continue;
		}
		if (section == Section::Node && nodeCount) {
			const QStringList parts = line.split(QLatin1Char(','));
			if (parts.size() >= 4)
				++(*nodeCount);
		} else if (section == Section::Element && elementCount) {
			const QStringList parts = line.split(QLatin1Char(','));
			if (parts.size() >= 3)
				++(*elementCount);
		}
	}
}

void recordMeshStatsOnDesignPoint(GearDesignPoint& dp, const QString& meshInpPath,
                                  double effectiveMeshSizeMm, bool meshAuto)
{
	countMeshInpStats(meshInpPath, &dp.nodeCount, &dp.elementCount);
	dp.meshSize_mm  = effectiveMeshSizeMm;
	dp.meshAuto     = meshAuto;
	dp.meshMethod   = QStringLiteral("gmsh");
	dp.elementOrder = 1;

	GEAR_OPT_DEBUG << "[GearOpt][Mesh] nodeCount =" << dp.nodeCount;
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] elementCount =" << dp.elementCount;
}

void recordContactOnDesignPoint(GearDesignPoint& dp, const InpContext& ctx)
{
	dp.enableContact    = ctx.enableContact;
	dp.contactStiffness = ctx.contactStiffness;
	dp.contactType      = QStringLiteral("surface_to_surface_penalty");
	dp.frictionCoeff    = 0.0;
	GEAR_OPT_DEBUG << "[GearOpt][Contact] enableContact ="
	         << (dp.enableContact ? "true" : "false");
	GEAR_OPT_DEBUG << "[GearOpt][Contact] contactStiffness =" << dp.contactStiffness;
}

struct MeshNodeCoord {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

bool hubAxisZMid(const QMap<int, MeshNodeCoord>& nodeMap, const QSet<int>& hubIds, double* zMid)
{
	if (!zMid || hubIds.isEmpty())
		return false;
	double sz = 0.0;
	int    n  = 0;
	for (int nid : hubIds) {
		const auto it = nodeMap.constFind(nid);
		if (it == nodeMap.constEnd())
			continue;
		sz += it->z;
		++n;
	}
	if (n <= 0)
		return false;
	*zMid = sz / static_cast<double>(n);
	return true;
}

bool appendInpNset(const QString& meshPath, const QString& name, const QList<int>& ids)
{
	if (ids.isEmpty())
		return false;
	QFile f(meshPath);
	if (!f.open(QIODevice::Append | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts << "\n** GearAutoOpt: " << name << "\n";
	ts << "*NSET, NSET=" << name << "\n";
	for (int i = 0; i < ids.size(); ++i) {
		ts << ids[i];
		if ((i + 1) % 16 == 0 || i == ids.size() - 1)
			ts << "\n";
		else
			ts << ", ";
	}
	return f.error() == QFile::NoError;
}

int maxNodeIdInMap(const QMap<int, MeshNodeCoord>& nodeMap)
{
	int maxId = 0;
	for (auto it = nodeMap.constBegin(); it != nodeMap.constEnd(); ++it)
		maxId = std::max(maxId, it.key());
	return maxId;
}

constexpr double kRootRefineDistMax   = 1.5;
constexpr double kRootRadiusTolMm     = 0.8;
constexpr double kRootZMarginMm       = 0.1;
constexpr double kZCurveDxyTolMm      = 1e-3;
constexpr double kZCurveDzMinRatio    = 0.8;
constexpr double kDedendumCoeffDefault = 1.25;

double gearRootRadiusMm(double module, int toothCount, double dedendumCoeff = kDedendumCoeffDefault)
{
	const double rf = module * static_cast<double>(toothCount) / 2.0 - dedendumCoeff * module;
	return rf > 0.0 ? rf : 0.1 * module;
}

double centerDistanceMm(const GearDesignPoint& dp)
{
	const double m    = dp.module;
	const int    Z1   = dp.z1;
	const int    Z2   = dp.z2;
	const double phi  = dp.alpha * M_PI / 180.0;
	const double xSum = dp.x1 + dp.x2;
	if (std::fabs(xSum) < 1e-12)
		return (Z1 + Z2) * m / 2.0;

	const double a             = (Z1 + Z2) * m / 2.0;
	const double invAlpha      = std::tan(phi) - phi;
	const double invAlphaPrime = invAlpha + 2.0 * xSum * std::tan(phi) / static_cast<double>(Z1 + Z2);

	double alphaPrime = phi;
	for (int i = 0; i < 100; ++i) {
		const double f      = std::tan(alphaPrime) - alphaPrime - invAlphaPrime;
		const double fPrime = 1.0 / (std::cos(alphaPrime) * std::cos(alphaPrime)) - 1.0;
		const double delta  = f / fPrime;
		alphaPrime -= delta;
		if (std::fabs(delta) < 1e-10)
			break;
	}
	return a * std::cos(phi) / std::cos(alphaPrime);
}

/// 与 GeoCommandCreateGear 一致：齿轮 1 在 (0,0)，齿轮 2 在 (0, a')。
void theoreticalGearAxisXY(const GearDesignPoint& dp, int gearIndex, double* axisX, double* axisY)
{
	if (!axisX || !axisY)
		return;
	*axisX = 0.0;
	*axisY = (gearIndex == 2) ? centerDistanceMm(dp) : 0.0;
}

int countShapeEdges(const TopoDS_Shape& shape)
{
	if (shape.IsNull())
		return 0;
	TopTools_IndexedMapOfShape edgeMap;
	TopExp::MapShapes(shape, TopAbs_EDGE, edgeMap);
	return edgeMap.Extent();
}

/// 按齿根半径 Rf 与 z∈[-margin, width+margin] 几何匹配 solid 边，返回 1-based Gmsh curve tag。
QVector<int> collectRootCurveTagsForGmsh(const TopoDS_Shape& solid, double cx, double cy, double Rf,
                                         double width)
{
	QVector<int> tags;
	if (solid.IsNull() || Rf <= 0.0)
		return tags;

	TopTools_IndexedMapOfShape edgeMap;
	TopExp::MapShapes(solid, TopAbs_EDGE, edgeMap);

	const double zMin = -kRootZMarginMm;
	const double zMax = width + kRootZMarginMm;

	for (int i = 1; i <= edgeMap.Extent(); ++i) {
		const TopoDS_Edge edge = TopoDS::Edge(edgeMap(i));
		BRepAdaptor_Curve curve(edge);
		const double uMid = 0.5 * (curve.FirstParameter() + curve.LastParameter());
		const gp_Pnt pt   = curve.Value(uMid);

		if (pt.Z() < zMin || pt.Z() > zMax)
			continue;

		const double dx = pt.X() - cx;
		const double dy = pt.Y() - cy;
		const double r  = std::sqrt(dx * dx + dy * dy);
		if (std::abs(r - Rf) < kRootRadiusTolMm)
			tags.append(i);
	}
	return tags;
}

struct ZCurveTagCollectResult {
	QVector<int> tags;
	double       rMin = 0.0;
	double       rMax = 0.0;
	double       rAvg = 0.0;
};

/// 遍历 solid 边，找出近似平行 Z 方向且径向在 (holeRadius, tipRadius) 内的边，返回 1-based Gmsh curve tag。
ZCurveTagCollectResult collectZCurveTagsForGmsh(const TopoDS_Shape& solid, double width, double cx,
                                                double cy, double holeRadius, double tipRadius)
{
	ZCurveTagCollectResult result;
	if (solid.IsNull() || width <= 0.0 || holeRadius <= 0.0 || tipRadius <= holeRadius)
		return result;

	TopTools_IndexedMapOfShape edgeMap;
	TopExp::MapShapes(solid, TopAbs_EDGE, edgeMap);

	const double dzMin = kZCurveDzMinRatio * width;
	double       rSum  = 0.0;
	bool         hasR  = false;

	for (int i = 1; i <= edgeMap.Extent(); ++i) {
		const TopoDS_Edge edge = TopoDS::Edge(edgeMap(i));
		BRepAdaptor_Curve curve(edge);
		const gp_Pnt p1 = curve.Value(curve.FirstParameter());
		const gp_Pnt p2 = curve.Value(curve.LastParameter());

		const double dz  = std::abs(p2.Z() - p1.Z());
		const double dxy = std::hypot(p2.X() - p1.X(), p2.Y() - p1.Y());
		if (dz < dzMin || dxy > kZCurveDxyTolMm)
			continue;

		const double mx  = 0.5 * (p1.X() + p2.X()) - cx;
		const double my  = 0.5 * (p1.Y() + p2.Y()) - cy;
		const double r   = std::hypot(mx, my);
		if (r <= holeRadius || r >= tipRadius)
			continue;

		result.tags.append(i);
		rSum += r;
		if (!hasR) {
			result.rMin = result.rMax = r;
			hasR        = true;
		} else {
			result.rMin = std::min(result.rMin, r);
			result.rMax = std::max(result.rMax, r);
		}
	}

	if (!result.tags.isEmpty())
		result.rAvg = rSum / static_cast<double>(result.tags.size());
	return result;
}

void mergeZCurveCollectResults(const ZCurveTagCollectResult& a, const ZCurveTagCollectResult& b,
                               ZCurveTagCollectResult* out)
{
	if (!out)
		return;
	if (a.tags.isEmpty()) {
		*out = b;
		return;
	}
	if (b.tags.isEmpty()) {
		*out = a;
		return;
	}
	*out = a;
	for (int t : b.tags)
		out->tags.append(t);
	const int nA = a.tags.size();
	const int nB = b.tags.size();
	out->rMin    = std::min(a.rMin, b.rMin);
	out->rMax    = std::max(a.rMax, b.rMax);
	out->rAvg    = (a.rAvg * static_cast<double>(nA) + b.rAvg * static_cast<double>(nB))
	            / static_cast<double>(nA + nB);
}

void warnRootCurveTagCount(const QString& gearLabel, int tagCount, int profileEdgeCount, int z)
{
	const int minExpected = std::max(1, z / 2);
	const int maxExpected = std::max(z * 4, profileEdgeCount > 0 ? profileEdgeCount * 4 : z * 4);

	if (tagCount < minExpected || tagCount > maxExpected) {
		qWarning().noquote()
		    << QStringLiteral("[GearOpt][Mesh] rootCurveTags %1 count=%2 out of range [%3,%4]"
		                      " (profileEdges=%5 z=%6)")
		           .arg(gearLabel)
		           .arg(tagCount)
		           .arg(minExpected)
		           .arg(maxExpected)
		           .arg(profileEdgeCount)
		           .arg(z);
	}
}

QString formatCurveTagList(const QVector<int>& tags)
{
	QStringList parts;
	parts.reserve(tags.size());
	for (int t : tags)
		parts << QString::number(t);
	return parts.join(QLatin1Char(','));
}

void writeGearGeoContent(QTextStream& ts, bool twoGears, double meshSize, double rootMeshSize,
                         const QVector<int>& rootCurveTags, bool useRootRefine, bool debugHeader,
                         int rootEdgesGear1, int rootEdgesGear2, int gear1EdgeCount,
                         const QVector<int>& zCurveTags, int zLayers)
{
	if (debugHeader) {
		ts << "// === GearAutoOpt root-mesh debug ===\n";
		ts << "// rootFilletEdges gear1=" << rootEdgesGear1 << " gear2=" << rootEdgesGear2 << "\n";
		ts << "// rootCurveTags count=" << rootCurveTags.size() << "\n";
		ts << "// zCurveTags count=" << zCurveTags.size() << "\n";
		ts << "// zLayers=" << zLayers << "\n";
		ts << "// gear1 solid edge count (TopExp::MapShapes)=" << gear1EdgeCount << "\n";
		ts << "// CurvesList={" << formatCurveTagList(rootCurveTags) << "}\n";
		ts << "// zCurvesList={" << formatCurveTagList(zCurveTags) << "}\n";
		ts << "// SizeMin=" << rootMeshSize << " SizeMax=" << meshSize
		   << " DistMax=" << kRootRefineDistMax << "\n";
		ts << "// useRootRefine=" << (useRootRefine ? 1 : 0) << "\n";
	} else {
		ts << "// generated by GearOptCaseRunner\n";
	}

	ts << "SetFactory(\"OpenCASCADE\");\n";
	ts << "Merge \"gear1.brep\";\n";
	if (twoGears) {
		ts << "Merge \"gear2.brep\";\n";
		ts << "Physical Volume(\"GEAR1\") = {1};\n";
		ts << "Physical Volume(\"GEAR2\") = {2};\n";
	} else {
		ts << "Physical Volume(\"BEAM\") = {1};\n";
	}

	if (useRootRefine) {
		const QString curveList = formatCurveTagList(rootCurveTags);
		ts << "// root refine: Distance + Threshold\n";
		ts << "Field[1] = Distance;\n";
		ts << "Field[1].CurvesList = {" << curveList << "};\n";
		ts << "Field[2] = Threshold;\n";
		ts << "Field[2].IField = 1;\n";
		ts << "Field[2].DistMin = 0;\n";
		ts << "Field[2].DistMax = " << QString::number(kRootRefineDistMax, 'g', 6) << ";\n";
		ts << "Field[2].SizeMin = " << QString::number(rootMeshSize, 'g', 6) << ";\n";
		ts << "Field[2].SizeMax = " << QString::number(meshSize, 'g', 6) << ";\n";
		ts << "Field[2].Sigmoid = 1;\n";
		ts << "Background Field = 2;\n";
		ts << "Mesh.MeshSizeMin = " << QString::number(rootMeshSize, 'g', 6) << ";\n";
	} else if (debugHeader) {
		ts << "// root refine disabled: rootCurveTags empty, fallback to global mesh only\n";
	}

	if (!zCurveTags.isEmpty() && zLayers > 0) {
		ts << "// z-direction transfinite layering\n";
		ts << "Transfinite Line {" << formatCurveTagList(zCurveTags) << "} = " << zLayers << ";\n";
	} else if (debugHeader && !zCurveTags.isEmpty() && zLayers <= 0) {
		ts << "// z-direction transfinite skipped: zLayers=" << zLayers << " (invalid)\n";
	}

	ts << "Mesh.MeshSizeMax = " << QString::number(meshSize, 'g', 6) << ";\n";
	ts << "Mesh.ElementOrder = 1;\n";
	ts << "Mesh.Algorithm3D = 1;\n";
	ts << "Mesh.SaveGroupsOfNodes = 1;\n";
}

bool writeGearGeoFile(const QString& geoPath, bool twoGears, double meshSize, double rootMeshSize,
                      const QVector<int>& rootCurveTags, bool useRootRefine, bool debugHeader,
                      int rootEdgesGear1, int rootEdgesGear2, int gear1EdgeCount,
                      const QVector<int>& zCurveTags, int zLayers)
{
	QFile geoFile(geoPath);
	if (!geoFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		return false;
	QTextStream ts(&geoFile);
	ts.setCodec("UTF-8");
	writeGearGeoContent(ts, twoGears, meshSize, rootMeshSize, rootCurveTags, useRootRefine, debugHeader,
	                    rootEdgesGear1, rootEdgesGear2, gear1EdgeCount, zCurveTags, zLayers);
	return geoFile.error() == QFile::NoError;
}

} // namespace

GearOptCaseRunner::GearOptCaseRunner(QObject* parent)
	: GearAutoCaseRunner(parent)
{}

GearOptCaseRunner::~GearOptCaseRunner() {
	cleanup();
}

void GearOptCaseRunner::runOne(GearDesignPoint& dp) {
	setLogLevel(_config.solver.debugMode ? GearLogLevel::Debug : GearLogLevel::Normal);
	emitLogNormal(QStringLiteral("[GearOpt] Start gen=%1 id=%2").arg(dp.generation).arg(dp.id));
	GearAutoCaseRunner::runOne(dp);
}

void GearOptCaseRunner::setMeshParams(const GearMeshParams& p) {
	_meshSize        = p.globalSize;
	_rootMeshSize    = p.rootSize > 0.0 ? p.rootSize : 0.40;
	_zLayersOverride = p.zLayers > 0 ? p.zLayers : -1;
}

int GearOptCaseRunner::computeAutoZLayers(double widthMm) {
	return std::clamp(static_cast<int>(std::ceil(widthMm / 1.0)) + 1, 10, 14);
}

void GearOptCaseRunner::cleanup() {
	auto* geoData = Geometry::GeometryData::getInstance();
	if (_set1) {
		geoData->removeTopGeometrySet(_set1);
		_set1 = nullptr;
	}
	if (_set2) {
		geoData->removeTopGeometrySet(_set2);
		_set2 = nullptr;
	}
	if (_geomCmd) {
		delete _geomCmd;
		_geomCmd = nullptr;
	}
}

bool GearOptCaseRunner::runGeometryStep(GearDesignPoint& dp) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][GeometryInput] gen=%1 id=%2 module=%3, z1=%4, z2=%5, alpha=%6, x1=%7, x2=%8, ca1=%9, lca1=%10, ca2=%11, lca2=%12, width=%13, hubRatio=%14")
	                              .arg(dp.generation)
	                              .arg(dp.id)
	                              .arg(dp.module, 0, 'g', 8)
	                              .arg(dp.z1)
	                              .arg(dp.z2)
	                              .arg(dp.alpha, 0, 'g', 8)
	                              .arg(dp.x1, 0, 'g', 8)
	                              .arg(dp.x2, 0, 'g', 8)
	                              .arg(dp.ca1, 0, 'g', 8)
	                              .arg(dp.lca1, 0, 'g', 8)
	                              .arg(dp.ca2, 0, 'g', 8)
	                              .arg(dp.lca2, 0, 'g', 8)
	                              .arg(dp.commonWidth, 0, 'g', 8)
	                              .arg(dp.hubRatio, 0, 'g', 8);

	checkGearOptDesignPointBounds(dp, _config);

	const int caseId = dp.id >= 0 ? dp.id : 0;
	const QString caseH = GearOptResultDatabase::caseHash(dp);
	emitLogNormal(QStringLiteral("[GearOpt][Geometry] case_id=%1 case_hash=%2 workDir=%3 geometryStepStarted=true")
	                  .arg(caseId)
	                  .arg(caseH)
	                  .arg(workDir()));

	// 上一次 run 的临时齿轮（如果还在）先清掉
	cleanup();

	auto* geoData = Geometry::GeometryData::getInstance();
	const int beforeCount = geoData->getGeometrySetCount();
	//创建一个齿轮命令
	auto* cmd = new Command::GeoCommandCreateGear(nullptr, nullptr);
	cmd->setName(QString("dp_g%1_p%2").arg(dp.generation).arg(dp.id)); //设置齿轮名称
	cmd->setNumberOfTeeth(dp.z1); //设置齿轮齿数
	cmd->setNumberOfSecondTeeth(dp.z2); //设置副齿轮齿数
	cmd->setModule(dp.module);
	cmd->setPressureAngle(dp.alpha); //设置压力角
	cmd->setAddendumCoefficient(1.0); //设置齿顶高系数
	cmd->setDedendumCoefficient(1.25); //设置齿根高系数
	cmd->setFilletCoefficient(0.38); //设置齿根圆角系数
	dp.addendumCoeff   = 1.0; //设置齿顶高系数
	dp.dedendumCoeff   = 1.25; //设置齿根高系数
	dp.rootFilletCoeff = 0.38; //设置齿根圆角系数
	dp.syncPairGearWidth();
	cmd->setThickness(dp.commonWidth); //设置齿轮厚度	
	cmd->setThickness2(dp.commonWidth); //设置副齿轮厚度
	GEAR_OPT_DEBUG << "[GearOpt][Geometry] commonWidth gear1/gear2 =" << dp.commonWidth;
	cmd->setExternalGear(true); //设置齿轮为外齿轮
	cmd->setTipReliefAmount(dp.ca1); //设置齿顶修型量
	cmd->setTipReliefLength(dp.lca1); //设置齿顶修型长度
	cmd->setTipReliefAmount2(dp.ca2); //设置副齿轮齿顶修型量
	cmd->setTipReliefLength2(dp.lca2); //设置副齿轮齿顶修型长度
	cmd->setprofileShiftCoefficient1(dp.x1); //设置主齿轮变位系数
	cmd->setprofileShiftCoefficient2(dp.x2); //设置副齿轮变位系数

	const bool ok = cmd->execute(); //执行齿轮命令
	if (!ok) {
		dp.errorMsg = QStringLiteral("GeoCommandCreateGear::execute() failed");
		emitLogNormal(QStringLiteral("[GearOpt][Geometry] case_id=%1 case_hash=%2 workDir=%3 geometryStepFinished=false createdShapeCount=0 shapeNames=")
		                  .arg(caseId)
		                  .arg(caseH)
		                  .arg(workDir()));
		delete cmd; //删除齿轮命令
		return false;
	}

	const int afterCount = geoData->getGeometrySetCount(); //获取几何集数量
	const int createdCount = afterCount - beforeCount;
	if (afterCount != beforeCount + 2) {
		dp.errorMsg = QString("expected geometry set count +2, got +%1")
		              .arg(createdCount);
		emitLogNormal(QStringLiteral("[GearOpt][Geometry] case_id=%1 case_hash=%2 workDir=%3 geometryStepFinished=false createdShapeCount=%4 shapeNames=")
		                  .arg(caseId)
		                  .arg(caseH)
		                  .arg(workDir())
		                  .arg(createdCount));
		delete cmd;
		return false;
	}

	// 取最后追加的两个为 _set1 / _set2（GeoCommandCreateGear::execute 末尾的
	// 顺序：先 set1 再 set2）
	_set1 = geoData->getGeometrySetAt(afterCount - 2);
	_set2 = geoData->getGeometrySetAt(afterCount - 1);
	_geomCmd = cmd;

	const QString shapeNames =
	    QStringLiteral("%1|%2")
	        .arg(_set1 ? _set1->getName() : QStringLiteral("?"))
	        .arg(_set2 ? _set2->getName() : QStringLiteral("?"));
	emitLogNormal(QStringLiteral("[GearOpt][Geometry] case_id=%1 case_hash=%2 workDir=%3 geometryStepFinished=true createdShapeCount=%4 shapeNames=%5")
	                  .arg(caseId)
	                  .arg(caseH)
	                  .arg(workDir())
	                  .arg(createdCount)
	                  .arg(shapeNames));

	emitLog(QString("geometry: 2 GeometrySets created (%1 / %2)")
	        .arg(_set1 ? _set1->getName() : QStringLiteral("?"))
	        .arg(_set2 ? _set2->getName() : QStringLiteral("?")));
	return true;
}

QString GearOptCaseRunner::detectGmshPath() {
	const QByteArray env = qgetenv("GMSH_PATH");
	if (!env.isEmpty()) {
		const QString p = QString::fromLocal8Bit(env);
		QFileInfo fi(p);
		if (fi.exists() && fi.isFile()) return QDir::cleanPath(fi.absoluteFilePath());
		return QString();
	}
	auto tryWithPrefix = [&](const QString& prefix) -> QString {
		for (int up = 0; up <= 4; ++up) {
			QString p = prefix;
			for (int i = 0; i < up; ++i) p += "/..";
			p += "/extlib/Gmsh/gmsh.exe";
			QFileInfo fi(p);
			if (fi.exists() && fi.isFile()) return QDir::cleanPath(fi.absoluteFilePath());
		}
		return QString();
	};
	QString hit = tryWithPrefix(QCoreApplication::applicationDirPath());
	if (!hit.isEmpty()) return hit;
	return tryWithPrefix(QDir::currentPath());
}

bool GearOptCaseRunner::runMeshStep(GearDesignPoint& dp) {
	const int caseId = dp.id >= 0 ? dp.id : 0;
	const QString caseH = GearOptResultDatabase::caseHash(dp);
	emitLogNormal(QStringLiteral("[GearOpt][Mesh] case_id=%1 case_hash=%2 workDir=%3 meshStepStarted=true")
	                  .arg(caseId)
	                  .arg(caseH)
	                  .arg(workDir()));

	if (!_set1) {
		dp.errorMsg = QStringLiteral("runMeshStep: _set1 is null (geometry step missing?)");
		emitLogNormal(QStringLiteral("[GearOpt][Mesh] case_id=%1 case_hash=%2 workDir=%3 meshStepFinished=false")
		                  .arg(caseId).arg(caseH).arg(workDir()));
		return false;
	}
	TopoDS_Shape* shape1Ptr = _set1->getShape();
	TopoDS_Shape* shape2Ptr = _set2 ? _set2->getShape() : nullptr;
	const bool    gear1Ok   = shape1Ptr && !shape1Ptr->IsNull();
	const bool    gear2Ok   = shape2Ptr && !shape2Ptr->IsNull();
	const bool    twoGears  = gear1Ok && gear2Ok;

	GEAR_OPT_DEBUG << "[GearOpt][Mesh] gear1 solid valid =" << gear1Ok;
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] gear2 solid valid =" << gear2Ok;
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] export two solids to gmsh =" << twoGears;

	if (!gear1Ok) {
		dp.errorMsg = QStringLiteral("runMeshStep: _set1->getShape() is null");
		return false;
	}

	const QString gmshPath = _gmshPathOverride.isEmpty()
	                          ? detectGmshPath()
	                          : _gmshPathOverride;
	if (gmshPath.isEmpty()) {
		dp.errorMsg = QStringLiteral("runMeshStep: gmsh.exe not found");
		return false;
	}

	if (workDir().isEmpty()) {
		dp.errorMsg = QStringLiteral("runMeshStep: workDir not set");
		return false;
	}
	QDir d(workDir());
	if (!d.exists() && !QDir().mkpath(workDir())) {
		dp.errorMsg = QString("runMeshStep: failed to create %1").arg(workDir());
		return false;
	}

	// BRep 是 gmsh Merge 直接识别的 OCC 原生格式
	const QString brep1Path = d.filePath(QStringLiteral("gear1.brep"));
	if (!BRepTools::Write(*shape1Ptr, brep1Path.toLocal8Bit().constData())) {
		dp.errorMsg = QStringLiteral("runMeshStep: BRepTools::Write gear1 failed");
		return false;
	}
	if (twoGears) {
		const QString brep2Path = d.filePath(QStringLiteral("gear2.brep"));
		if (!BRepTools::Write(*shape2Ptr, brep2Path.toLocal8Bit().constData())) {
			dp.errorMsg = QStringLiteral("runMeshStep: BRepTools::Write gear2 failed");
			return false;
		}
	}

	const bool   meshAuto = _meshSize <= 0.0;
	const double meshAutoMin =
	    (_config.solver.meshAutoMinMm > 0.0) ? _config.solver.meshAutoMinMm : 2.0;
	const double meshSize = meshAuto ? std::max(0.5 * dp.module, meshAutoMin) : _meshSize;
	dp.meshAuto     = meshAuto;
	dp.meshSize_mm  = meshSize;
	dp.meshMethod   = QStringLiteral("gmsh");
	dp.elementOrder = 1;

	const QString geoPath       = d.filePath(QStringLiteral("gear.geo"));
	const QString debugGeoPath  = d.filePath(QStringLiteral("gear_mesh_debug.geo"));
	const QString meshInp       = d.filePath(QStringLiteral("mesh.inp"));

	int rootEdgesGear1 = 0;
	int rootEdgesGear2 = 0;
	QVector<int> rootCurveTags;
	QVector<int> rootCurveTagsGear1;
	QVector<int> rootCurveTagsGear2;
	QVector<int> zCurveTags;
	QVector<int> zCurveTagsGear1;
	QVector<int> zCurveTagsGear2;
	const int gear1EdgeCount = countShapeEdges(*shape1Ptr);

	const double m           = dp.module;
	const double width       = dp.commonWidth;
	const double Rf1         = gearRootRadiusMm(m, dp.z1, dp.dedendumCoeff);
	const double Rf2         = gearRootRadiusMm(m, dp.z2, dp.dedendumCoeff);
	const double centerDist  = centerDistanceMm(dp);
	const double pitchR1     = dp.z1 * m / 2.0;
	const double pitchR2     = dp.z2 * m / 2.0;
	const double holeR1      = 0.4 * pitchR1;
	const double holeR2      = 0.4 * pitchR2;
	const double tipR1       = (dp.z1 / 2.0 + 1.0 + dp.x1) * m;
	const double tipR2       = (dp.z2 / 2.0 + 1.0 + dp.x2) * m;

	if (_geomCmd) {
		rootEdgesGear1 = _geomCmd->rootFilletEdgesGear1().size();
		rootEdgesGear2 = _geomCmd->rootFilletEdgesGear2().size();
	}

	rootCurveTagsGear1 = collectRootCurveTagsForGmsh(*shape1Ptr, 0.0, 0.0, Rf1, width);
	warnRootCurveTagCount(QStringLiteral("gear1"), rootCurveTagsGear1.size(), rootEdgesGear1, dp.z1);
	rootCurveTags = rootCurveTagsGear1;

	if (twoGears) {
		rootCurveTagsGear2 =
		    collectRootCurveTagsForGmsh(*shape2Ptr, 0.0, centerDist, Rf2, width);
		warnRootCurveTagCount(QStringLiteral("gear2"), rootCurveTagsGear2.size(), rootEdgesGear2,
		                      dp.z2);
		for (int t : rootCurveTagsGear2)
			rootCurveTags.append(gear1EdgeCount + t);
	}

	ZCurveTagCollectResult zCollect1 =
	    collectZCurveTagsForGmsh(*shape1Ptr, width, 0.0, 0.0, holeR1, tipR1);
	ZCurveTagCollectResult zCollect2;
	zCurveTagsGear1 = zCollect1.tags;
	zCurveTags      = zCurveTagsGear1;
	if (twoGears) {
		zCollect2       = collectZCurveTagsForGmsh(*shape2Ptr, width, 0.0, centerDist, holeR2, tipR2);
		zCurveTagsGear2 = zCollect2.tags;
		for (int t : zCurveTagsGear2)
			zCurveTags.append(gear1EdgeCount + t);
	}
	ZCurveTagCollectResult zCollectAll = zCollect1;
	if (twoGears)
		mergeZCurveCollectResults(zCollect1, zCollect2, &zCollectAll);

	const int zLayers = _zLayersOverride > 0 ? _zLayersOverride : computeAutoZLayers(width);

	if (zLayers <= 0) {
		dp.errorMsg = QStringLiteral("invalid mesh params: zLayers <= 0");
		emitLogNormal(QStringLiteral("[Mesh][Error] %1 (global=%2 root=%3)")
		                  .arg(dp.errorMsg)
		                  .arg(meshSize, 0, 'f', 2)
		                  .arg(_rootMeshSize, 0, 'f', 2));
		return false;
	}

	const bool    useRootRefine = !rootCurveTags.isEmpty();
	const QString curvesListStr = formatCurveTagList(rootCurveTags);

	emitLogNormal(QStringLiteral("[Mesh] global=%1 root=%2 zLayers=%3")
	                  .arg(meshSize, 0, 'f', 2)
	                  .arg(_rootMeshSize, 0, 'f', 2)
	                  .arg(zLayers));

	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] rootFilletEdges gear1=%1 gear2=%2")
	                              .arg(rootEdgesGear1)
	                              .arg(rootEdgesGear2);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] rootCurveTags gear1 count=%1 Rf=%2 center=(0,0)")
	                              .arg(rootCurveTagsGear1.size())
	                              .arg(Rf1, 0, 'g', 6);
	if (twoGears) {
		GEAR_OPT_DEBUG_NOQUOTE
		    << QStringLiteral("[GearOpt][Mesh] rootCurveTags gear2 count=%1 Rf=%2 center=(0,%3)")
		           .arg(rootCurveTagsGear2.size())
		           .arg(Rf2, 0, 'g', 6)
		           .arg(centerDist, 0, 'g', 6);
	}
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] rootCurveTags total count=%1")
	                              .arg(rootCurveTags.size());
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] rootCurveTags CurvesList={%1}")
	                              .arg(curvesListStr);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] gear1EdgeCount=%1 useRootRefine=%2")
	                              .arg(gear1EdgeCount)
	                              .arg(useRootRefine ? 1 : 0);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] zCurveTags count=%1 zLayers=%2 rAvg=%3 rMin=%4 rMax=%5")
	                              .arg(zCurveTags.size())
	                              .arg(zLayers)
	                              .arg(zCollectAll.rAvg, 0, 'g', 6)
	                              .arg(zCollectAll.tags.isEmpty() ? 0.0 : zCollectAll.rMin, 0, 'g', 6)
	                              .arg(zCollectAll.tags.isEmpty() ? 0.0 : zCollectAll.rMax, 0, 'g', 6);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] zCurveTags gear1=%1 gear2=%2 holeR=(%3,%4) tipR=(%5,%6)")
	                              .arg(zCurveTagsGear1.size())
	                              .arg(zCurveTagsGear2.size())
	                              .arg(holeR1, 0, 'g', 6)
	                              .arg(holeR2, 0, 'g', 6)
	                              .arg(tipR1, 0, 'g', 6)
	                              .arg(tipR2, 0, 'g', 6);

	emitLog(QStringLiteral("mesh: rootFilletEdges g1=%1 g2=%2 rootCurveTags g1=%3 g2=%4 total=%5 refine=%6")
	            .arg(rootEdgesGear1)
	            .arg(rootEdgesGear2)
	            .arg(rootCurveTagsGear1.size())
	            .arg(rootCurveTagsGear2.size())
	            .arg(rootCurveTags.size())
	            .arg(useRootRefine ? QStringLiteral("on") : QStringLiteral("off(global)")));
	if (useRootRefine) {
		emitLog(QStringLiteral("mesh: CurvesList={%1} SizeMin=%2 SizeMax=%3 DistMax=%4")
		            .arg(curvesListStr)
		            .arg(_rootMeshSize, 0, 'g', 4)
		            .arg(meshSize, 0, 'g', 4)
		            .arg(kRootRefineDistMax, 0, 'g', 4));
	} else {
		emitLog(QStringLiteral("mesh: rootCurveTags empty, fallback to global meshSize=%1")
		            .arg(meshSize, 0, 'g', 4));
	}
	emitLog(QStringLiteral("mesh: zCurveTags count=%1 zLayers=%2 rAvg=%3 rMin=%4 rMax=%5")
	            .arg(zCurveTags.size())
	            .arg(zLayers)
	            .arg(zCollectAll.rAvg, 0, 'g', 4)
	            .arg(zCollectAll.tags.isEmpty() ? 0.0 : zCollectAll.rMin, 0, 'g', 4)
	            .arg(zCollectAll.tags.isEmpty() ? 0.0 : zCollectAll.rMax, 0, 'g', 4));

	if (!writeGearGeoFile(geoPath, twoGears, meshSize, _rootMeshSize, rootCurveTags, useRootRefine, false,
	                      rootEdgesGear1, rootEdgesGear2, gear1EdgeCount, zCurveTags, zLayers)) {
		dp.errorMsg = QStringLiteral("runMeshStep: cannot write gear.geo");
		return false;
	}
	if (!writeGearGeoFile(debugGeoPath, twoGears, meshSize, _rootMeshSize, rootCurveTags, useRootRefine, true,
	                      rootEdgesGear1, rootEdgesGear2, gear1EdgeCount, zCurveTags, zLayers)) {
		dp.errorMsg = QStringLiteral("runMeshStep: cannot write gear_mesh_debug.geo");
		return false;
	}

	emitLogNormal(QStringLiteral("[Mesh][gear.geo] MeshSizeMax=%1 RootSize=%2 TransfiniteLayers=%3 path=%4")
	                  .arg(meshSize, 0, 'g', 6)
	                  .arg(_rootMeshSize, 0, 'g', 6)
	                  .arg(zLayers)
	                  .arg(geoPath));
	{
		QFile geoRead(geoPath);
		if (geoRead.open(QIODevice::ReadOnly | QIODevice::Text)) {
			const QString content = QString::fromUtf8(geoRead.readAll());
			for (const QString& line : content.split(QLatin1Char('\n'))) {
				const QString t = line.trimmed();
				if (t.startsWith(QStringLiteral("// zLayers="))
				    || t.contains(QStringLiteral("Mesh.MeshSizeMax"))
				    || t.contains(QStringLiteral("Mesh.MeshSizeMin"))
				    || t.contains(QStringLiteral("Field[2].SizeMin"))
				    || t.contains(QStringLiteral("Field[2].SizeMax"))) {
					emitLogNormal(QStringLiteral("[Mesh][gear.geo] %1").arg(t));
				}
			}
		}
	}

	emitLog(QStringLiteral("mesh: gmsh -3 -format inp gear.geo (mesh size %1, debug=%2)")
	            .arg(QString::number(meshSize, 'g', 4), QStringLiteral("gear_mesh_debug.geo")));
	QProcess proc;
	proc.setWorkingDirectory(workDir());
	proc.start(gmshPath, QStringList{QStringLiteral("-3"), QStringLiteral("-format"), QStringLiteral("inp"),
	                                 QStringLiteral("-o"), QStringLiteral("mesh.inp"), QStringLiteral("gear.geo")});
	if (!proc.waitForStarted(5000)) {
		dp.errorMsg = QStringLiteral("runMeshStep: gmsh failed to start");
		return false;
	}
	if (!proc.waitForFinished(180000)) {
		proc.kill();
		const QByteArray gmshOut = proc.readAllStandardOutput();
		const QByteArray gmshErr = proc.readAllStandardError();
		GearOptLog::writeText(d.filePath(QStringLiteral("gmsh_output.log")),
		                      QString::fromLocal8Bit(gmshOut + gmshErr));
		dp.errorMsg = QStringLiteral("runMeshStep: gmsh timeout (>180s)");
		emitLogNormal(QStringLiteral("[Mesh][Error] Gmsh failed, see gmsh_output.log"));
		return false;
	}
	const QByteArray gmshOut = proc.readAllStandardOutput();
	const QByteArray gmshErr = proc.readAllStandardError();
	GearOptLog::writeText(d.filePath(QStringLiteral("gmsh_output.log")),
	                      QString::fromLocal8Bit(gmshOut + gmshErr));
	if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0) {
		dp.errorMsg = QStringLiteral("runMeshStep: gmsh exit=%1 (see gmsh_output.log)")
		                  .arg(proc.exitCode());
		emitLogNormal(QStringLiteral("[Mesh][Error] Gmsh failed, see gmsh_output.log"));
		return false;
	}
	if (!QFile::exists(meshInp) || QFileInfo(meshInp).size() == 0) {
		dp.errorMsg = QStringLiteral("runMeshStep: mesh.inp missing or empty");
		return false;
	}

	// 若 Gmsh 仍写出面单元（S3/CPS 等），CCX 会与体单元混用导致失败；无面元时本函数几乎无操作。
	const int stripped = stripSurfaceElements(meshInp);
	if (stripped > 0) {
		emitLog(QStringLiteral("mesh: stripSurfaceElements removed %1 surface-element data rows (CCX)")
		              .arg(stripped));
	}

	if (meshInpHasElset(meshInp, QStringLiteral("GEAR1"))
	    && meshInpHasElset(meshInp, QStringLiteral("GEAR2"))) {
		GearMeshEnrichParams enrich;
			enrich.module   = dp.module;
			enrich.z1       = dp.z1;
			enrich.z2       = dp.z2;
			enrich.x1       = dp.x1;
			enrich.x2       = dp.x2;
			enrich.hubRatio = dp.hubRatio;
			enrich.meshSize = meshSize;
			if (twoGears) {
				shapeAxisXY(*shape1Ptr, &enrich.axis1X, &enrich.axis1Y);
				shapeAxisXY(*shape2Ptr, &enrich.axis2X, &enrich.axis2Y);
			}

			DualGearMeshReport meshRpt;
			if (!enrichGearMeshInp(meshInp, enrich, &meshRpt)) {
				dp.errorMsg = QStringLiteral(
				    "runMeshStep: enrichGearMeshInp failed (GEAR1_HUB/TOOTH/SURF from GEAR1/GEAR2)");
				return false;
			}
			GEAR_OPT_DEBUG << "[GearOpt][Mesh] gear1 node count =" << meshRpt.gear1NodeCount;
			GEAR_OPT_DEBUG << "[GearOpt][Mesh] gear2 node count =" << meshRpt.gear2NodeCount;
			GEAR_OPT_DEBUG << "[GearOpt][Mesh] gear1 element count =" << meshRpt.gear1ElementCount;
			GEAR_OPT_DEBUG << "[GearOpt][Mesh] gear2 element count =" << meshRpt.gear2ElementCount;
			emitLog(QStringLiteral("mesh: GEAR1_TOOTH_SURF faces=%1 local=%2 %3")
			              .arg(meshRpt.gear1ToothSurfCount)
			              .arg(meshRpt.gear1LocalContactSurf ? QStringLiteral("yes") : QStringLiteral("no"))
			              .arg(meshRpt.gear1ToothRangeText));
			emitLog(QStringLiteral("mesh: GEAR2_TOOTH_SURF faces=%1 local=%2 %3")
			              .arg(meshRpt.gear2ToothSurfCount)
			              .arg(meshRpt.gear2LocalContactSurf ? QStringLiteral("yes") : QStringLiteral("no"))
			              .arg(meshRpt.gear2ToothRangeText));
		emitLog(QStringLiteral("mesh: enriched GEAR1/GEAR2 → HUB/TOOTH_OUTER/TOOTH_SURF"));
	}

	recordMeshStatsOnDesignPoint(dp, meshInp, meshSize, meshAuto);

	emitLogNormal(QStringLiteral("[GearOpt][Mesh] case_id=%1 case_hash=%2 workDir=%3 meshStepFinished=true nodes=%4 elements=%5")
	                  .arg(caseId)
	                  .arg(caseH)
	                  .arg(workDir())
	                  .arg(dp.nodeCount)
	                  .arg(dp.elementCount));

	emitLogNormal(QStringLiteral("[Mesh] Done: nodes=%1 elements=%2")
	                  .arg(dp.nodeCount)
	                  .arg(dp.elementCount));

	emitLog(QStringLiteral("mesh: %1 (elset_hint=%2)")
	              .arg(meshInp, volumeElsetFromMeshInp(meshInp)));
	return true;
}

bool GearOptCaseRunner::runInpWriteStep(GearDesignPoint& dp) {
	const double m       = dp.module;
	const double pitchR  = dp.z1 * m / 2.0;
	const double hubR    = dp.hubRatio * pitchR;
	const double tipR    = (dp.z1 / 2.0 + 1.0 + dp.x1) * m;

	// _config.solver.torque 单位 N·m → N·mm
	const double torqueNmm = _config.solver.torque * 1000.0;
	const double Ft        = torqueNmm / pitchR;

	const QDir d(workDir());
	const QString meshPath = d.filePath(QStringLiteral("mesh.inp"));
	if (!QFile::exists(meshPath)) {
		dp.errorMsg = QStringLiteral("runInpWriteStep: mesh.inp not found");
		return false;
	}

	QMap<int, MeshNodeCoord> nodeMap;
	{
		QFile f(meshPath);
		if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
			dp.errorMsg = QStringLiteral("runInpWriteStep: cannot open mesh.inp");
			return false;
		}
		QTextStream ts(&f);
		bool inNode = false;
		while (!ts.atEnd()) {
			const QString line = ts.readLine().trimmed();
			if (line.isEmpty() || line.startsWith(QStringLiteral("**"))) continue;
			if (line.startsWith(QStringLiteral("*"), Qt::CaseInsensitive)) {
				inNode = line.startsWith(QStringLiteral("*NODE"), Qt::CaseInsensitive)
				         && !line.startsWith(QStringLiteral("*NODE PRINT"), Qt::CaseInsensitive)
				         && !line.startsWith(QStringLiteral("*NODE FILE"), Qt::CaseInsensitive);
				continue;
			}
			if (!inNode) continue;
			const QStringList parts = line.split(QLatin1Char(','));
			if (parts.size() < 4) continue;
			bool ok;
			const int id = parts[0].trimmed().toInt(&ok);
			if (!ok) continue;
			MeshNodeCoord nc;
			nc.x = parts[1].trimmed().toDouble();
			nc.y = parts[2].trimmed().toDouble();
			nc.z = parts[3].trimmed().toDouble();
			nodeMap.insert(id, nc);
		}
	}

	if (nodeMap.isEmpty()) {
		dp.errorMsg = QStringLiteral("runInpWriteStep: no nodes parsed from mesh.inp");
		return false;
	}

	emitLog(QStringLiteral("inp_write: mesh nodes=%1").arg(nodeMap.size()));

	const bool dualMesh = meshInpHasElset(meshPath, QStringLiteral("GEAR1"))
	                      && meshInpHasElset(meshPath, QStringLiteral("GEAR2"))
	                      && countNsetNodes(meshPath, QStringLiteral("GEAR1_HUB")) > 0;
	if (dualMesh) {
		const int hub1Nodes   = countNsetNodes(meshPath, QStringLiteral("GEAR1_HUB"));
		const int hub2Nodes   = countNsetNodes(meshPath, QStringLiteral("GEAR2_HUB"));
		if (hub1Nodes <= 0 || hub2Nodes <= 0) {
			dp.errorMsg = QStringLiteral(
			    "runInpWriteStep: dual mesh sets incomplete (GEAR1_HUB/GEAR2_HUB empty)");
			return false;
		}

		QSet<int> hub1Ids;
		QSet<int> hub2Ids;
		if (!loadNsetNodeIds(meshPath, QStringLiteral("GEAR1_HUB"), &hub1Ids) || hub1Ids.isEmpty()) {
			dp.errorMsg = QStringLiteral("runInpWriteStep: cannot load GEAR1_HUB node ids");
			return false;
		}
		if (!loadNsetNodeIds(meshPath, QStringLiteral("GEAR2_HUB"), &hub2Ids) || hub2Ids.isEmpty()) {
			dp.errorMsg = QStringLiteral("runInpWriteStep: cannot load GEAR2_HUB node ids");
			return false;
		}

		double cx1 = 0.0, cy1 = 0.0, zMid1 = 0.0;
		double cx2 = 0.0, cy2 = 0.0, zMid2 = 0.0;
		theoreticalGearAxisXY(dp, 1, &cx1, &cy1);
		theoreticalGearAxisXY(dp, 2, &cx2, &cy2);
		if (!hubAxisZMid(nodeMap, hub1Ids, &zMid1)) {
			dp.errorMsg = QStringLiteral("runInpWriteStep: cannot compute GEAR1 hub z mid");
			return false;
		}
		if (!hubAxisZMid(nodeMap, hub2Ids, &zMid2)) {
			dp.errorMsg = QStringLiteral("runInpWriteStep: cannot compute GEAR2 hub z mid");
			return false;
		}

		const int hub1SurfFaces = countSurfaceFaces(meshPath, QStringLiteral("GEAR1_HUB_SURF"));
		const int hub2SurfFaces = countSurfaceFaces(meshPath, QStringLiteral("GEAR2_HUB_SURF"));
		if (hub1SurfFaces <= 0 || hub2SurfFaces <= 0) {
			dp.errorMsg = QStringLiteral(
			    "runInpWriteStep: GEAR1/GEAR2_HUB_SURF missing (re-run mesh enrich)");
			return false;
		}

		const double torqueNm      = _config.solver.torque;
		const double driveDispMm =
		    (_config.solver.ccxDriveDisplacementMm > 0.0) ? _config.solver.ccxDriveDisplacementMm : 0.001;
		const int  maxNodeId = maxNodeIdInMap(nodeMap);
		const int  id1Ref    = maxNodeId + 1;
		const int  id2Ref    = maxNodeId + 2;
		const int  id1Rot    = maxNodeId + 3;
		const int  id2Rot    = maxNodeId + 4;
		const bool useRigid  = _config.solver.useRigidBody;
		const bool displacementDriven = useRigid;

		emitLog(QStringLiteral("inp_write: dual gear load path = %1")
		            .arg(useRigid ? QStringLiteral("*RIGID BODY + CENTER1_ROT displacement drive")
		                          : QStringLiteral("legacy *COUPLING (no torque CLOAD)")));
		emitLog(QStringLiteral("inp_write: CENTER1_REF coord=(%1,%2,%3) xy=theoretical axis z=hub avg")
		            .arg(cx1, 0, 'g', 8)
		            .arg(cy1, 0, 'g', 8)
		            .arg(zMid1, 0, 'g', 8));
		emitLog(QStringLiteral("inp_write: CENTER2_REF coord=(%1,%2,%3) xy=theoretical axis z=hub avg")
		            .arg(cx2, 0, 'g', 8)
		            .arg(cy2, 0, 'g', 8)
		            .arg(zMid2, 0, 'g', 8));
		emitLog(QStringLiteral("inp_write: CENTER1_REF id=%1 CENTER1_ROT id=%2 CENTER2_REF id=%3 CENTER2_ROT id=%4")
		            .arg(id1Ref)
		            .arg(id1Rot)
		            .arg(id2Ref)
		            .arg(id2Rot));
		emitLog(QStringLiteral("inp_write: GEAR1_HUB_SURF=%1 GEAR2_HUB_SURF=%2")
		            .arg(hub1SurfFaces)
		            .arg(hub2SurfFaces));
		emitLog(QStringLiteral("inp_write: GEAR1_TOOTH_OUTER equivalent CLOAD = disabled"));
		emitLog(QStringLiteral("inp_write: displacement-driven=%1 drive_disp_mm=%2 torque_Nm=%3 contactStiffness=%4")
		            .arg(displacementDriven ? QStringLiteral("true") : QStringLiteral("false"))
		            .arg(driveDispMm, 0, 'g', 8)
		            .arg(torqueNm, 0, 'g', 8)
		            .arg(_config.solver.contactStiffness, 0, 'g', 8));
		emitLog(QStringLiteral("inp_write: enableContact=%1")
		            .arg(_config.solver.enableContact ? QStringLiteral("true") : QStringLiteral("false")));

		InpContext ctx;
		ctx.runName        = QString("g%1_p%2").arg(dp.generation).arg(dp.id);
		ctx.generation     = dp.generation;
		ctx.pointId        = dp.id;
		ctx.meshInpFile    = QStringLiteral("mesh.inp");
		ctx.materialName   = QStringLiteral("STEEL");
		ctx.youngModulus   = _config.solver.youngModulus;
		ctx.poisson        = _config.solver.poissonRatio;
		ctx.density        = _config.solver.density;
		ctx.volumeElset    = QStringLiteral("GEAR1");
		ctx.volumeElsets   = {QStringLiteral("GEAR1"), QStringLiteral("GEAR2")};
		ctx.twoGearJob     = true;
		ctx.useRigidBody   = useRigid;
		ctx.useCoupling    = !useRigid && _config.solver.useCoupling;
		ctx.useKinematicCoupling = _config.solver.useKinematicCoupling;
		ctx.gearDriveMode  = _config.solver.gearDriveMode;
		ctx.cloads.clear();
		ctx.contactStiffness    = _config.solver.contactStiffness;
		ctx.contactPressureP0   = _config.solver.contactPressureP0;
		ctx.contactAdjustMm     = _config.solver.contactAdjustMm;
		configureCcxContact(ctx, meshPath, _config.solver.enableContact);
		ctx.requestContactOutput = false;
		ctx.requestNodeFile      = true;
		ctx.requestElFile        = true;

		const int tooth1Faces = countSurfaceFaces(meshPath, QStringLiteral("GEAR1_TOOTH_SURF"));
		const int tooth2Faces = countSurfaceFaces(meshPath, QStringLiteral("GEAR2_TOOTH_SURF"));
		const int masterFaces = countSurfaceFaces(meshPath, QStringLiteral("master"));
		const int slaveFaces  = countSurfaceFaces(meshPath, QStringLiteral("slave"));
		emitLog(QStringLiteral("inp_write: GEAR1_TOOTH_SURF faces=%1 GEAR2_TOOTH_SURF faces=%2 (debug)")
		            .arg(tooth1Faces)
		            .arg(tooth2Faces));
		emitLog(QStringLiteral("inp_write: contact surfaces master=%1 slave=%2 pair=slave,master")
		            .arg(masterFaces)
		            .arg(slaveFaces));
		emitLog(QStringLiteral("inp_write: contact %1")
		            .arg(ctx.enableContact ? QStringLiteral("enabled") : QStringLiteral("disabled")));

		if (useRigid) {
			ctx.rigidBody.torqueNmm = torqueNm * 1000.0;
			ctx.rigidBody.mentorStep2RotCloadZ =
			    mentorStep2CloadZFromTorqueNmm(ctx.rigidBody.torqueNmm);
			ctx.rigidBody.driveDispMm        = driveDispMm;
			ctx.rigidBody.displacementDriven = displacementDriven;
			ctx.rigidBody.gearWidthMm        = dp.commonWidth;
			ctx.rigidBody.gear1.hubNset     = QStringLiteral("GEAR1_HUB");
			ctx.rigidBody.gear1.hubSurfName = QStringLiteral("GEAR1_HUB_SURF");
			ctx.rigidBody.gear1.refNset     = QStringLiteral("CENTER1_REF");
			ctx.rigidBody.gear1.refNodeId   = id1Ref;
			ctx.rigidBody.gear1.rotNset     = QStringLiteral("CENTER1_ROT");
			ctx.rigidBody.gear1.rotNodeId   = id1Rot;
			ctx.rigidBody.gear1.cx          = cx1;
			ctx.rigidBody.gear1.cy          = cy1;
			ctx.rigidBody.gear1.zMid        = zMid1;
			ctx.rigidBody.gear2.hubNset     = QStringLiteral("GEAR2_HUB");
			ctx.rigidBody.gear2.hubSurfName = QStringLiteral("GEAR2_HUB_SURF");
			ctx.rigidBody.gear2.refNset     = QStringLiteral("CENTER2_REF");
			ctx.rigidBody.gear2.refNodeId   = id2Ref;
			ctx.rigidBody.gear2.rotNset     = QStringLiteral("CENTER2_ROT");
			ctx.rigidBody.gear2.rotNodeId   = id2Rot;
			ctx.rigidBody.gear2.cx          = cx2;
			ctx.rigidBody.gear2.cy          = cy2;
			ctx.rigidBody.gear2.zMid        = zMid2;
			emitLog(QStringLiteral("inp_write: mentor Step-2 CENTER2_ROT DOF3 Fz=%1 N (T/L_rot, L=%2 mm)")
			            .arg(ctx.rigidBody.mentorStep2RotCloadZ, 0, 'g', 8)
			            .arg(kGearMentorRotOffsetMm, 0, 'g', 4));
		} else if (ctx.useCoupling) {
			ctx.couplingDrive.torqueNmm         = torqueNm * 1000.0;
			ctx.couplingDrive.gear1.hubSurfName = QStringLiteral("GEAR1_HUB_SURF");
			ctx.couplingDrive.gear1.refNset     = QStringLiteral("CENTER1_REF");
			ctx.couplingDrive.gear1.refNodeId   = id1Ref;
			ctx.couplingDrive.gear1.cx          = cx1;
			ctx.couplingDrive.gear1.cy          = cy1;
			ctx.couplingDrive.gear1.zMid        = zMid1;
		}

		clampCcxContactAdjustForNboun(ctx, meshPath, nodeMap.size(),
		                              [this](const QString& s) { emitLog(s); });
		recordContactOnDesignPoint(dp, ctx);
		dp.torque_Nm        = _config.solver.torque;
		dp.materialName     = QStringLiteral("STEEL");
		dp.youngModulus_MPa = _config.solver.youngModulus;
		dp.poissonRatio     = _config.solver.poissonRatio;
		dp.density          = _config.solver.density;

		if (!writeJobInp(workDir(), ctx)) {
			dp.errorMsg = QStringLiteral("runInpWriteStep: writeJobInp failed (dual gear)");
			return false;
		}
		const QString jobInpPath = d.filePath(QStringLiteral("job.inp"));
		for (const QString& surfName :
		     {QStringLiteral("GEAR1_TOOTH_SURF"), QStringLiteral("GEAR2_TOOTH_SURF")}) {
			CcxSurfaceVerifyReport surfRpt;
			QString                surfErr;
			if (verifyCcxElementSurfaceFaces(meshPath, jobInpPath, surfName, &surfRpt, &surfErr)) {
				const QString vtkInfo = kExportToothSurfDebugVtk ? surfRpt.vtkPath : QStringLiteral("(vtk export off)");
				emitLog(QStringLiteral("inp_write: [SurfaceVerify] %1 faces=%2 outward=%3 inward=%4 vtk=%5")
				            .arg(surfName)
				            .arg(surfRpt.faceCount)
				            .arg(surfRpt.outwardCount)
				            .arg(surfRpt.inwardCount)
				            .arg(vtkInfo));
			} else {
				emitLog(QStringLiteral("inp_write: [SurfaceVerify] %1 failed: %2")
				            .arg(surfName, surfErr));
			}
		}
		for (const QString& raw : inpRigidBodyKeywordLines(jobInpPath)) {
			emitLog(QStringLiteral("inp_write: job.inp *RIGID BODY: %1").arg(raw.trimmed()));
		}

		const double pitchR2   = dp.z2 * m / 2.0;
		const double hubR1Geom = 0.4 * pitchR;
		const double hubR2Geom = 0.4 * pitchR2;
		const double tipR2     = (dp.z2 / 2.0 + 1.0 + dp.x2) * m;
		const double densityKgPerMm3 = _config.solver.density * 1000.0;
		dp.mass_gear1 = densityKgPerMm3 * M_PI * (tipR * tipR - hubR1Geom * hubR1Geom) * dp.commonWidth;
		dp.mass_gear2 =
		    densityKgPerMm3 * M_PI * (tipR2 * tipR2 - hubR2Geom * hubR2Geom) * dp.commonWidth;
		dp.syncLegacyResultFields();

		emitLog(QString("inp: dual rigid-body hub_surf=%1/%2 drive_disp_mm=%3 torque_Nm=%4 step2_Fz=%5 N mass~%6 kg")
		            .arg(hub1SurfFaces)
		            .arg(hub2SurfFaces)
		            .arg(driveDispMm, 0, 'g', 8)
		            .arg(torqueNm, 0, 'g', 8)
		            .arg(ctx.rigidBody.mentorStep2RotCloadZ, 0, 'g', 8)
		            .arg(dp.mass_total, 0, 'g', 4));
		return true;
	}

	// 轮毂（内孔圆柱面）：与 Gmsh 后处理一致，用 |r_xy − hubR| ≤ tol 筛圆柱面上的点，避免「r≤常数」把根圆附近
	// 大片体积节点当 hub 而与齿面载荷集相交。hubR = hubRatio×分度圆半径（内孔半径）。
	const double meshSizeEff =
	    (_config.solver.meshSize > 0.0) ? _config.solver.meshSize : (0.5 * m);
	const double boreTol = std::max(0.15 * meshSizeEff, 0.012 * std::max(hubR, 1e-6));

	QList<int> hubNodes;
	QSet<int> hubIds;
	for (auto it = nodeMap.constBegin(); it != nodeMap.constEnd(); ++it) {
		const MeshNodeCoord& nc = it.value();
		const double r = std::sqrt(nc.x * nc.x + nc.y * nc.y);
		if (std::fabs(r - hubR) <= boreTol) {
			const int nid = it.key();
			hubNodes.append(nid);
			hubIds.insert(nid);
		}
	}
	if (hubNodes.isEmpty()) {
		emitLog(QStringLiteral("inp_write: bore shell empty (|r−hubR|<=tol), fallback disk r<=hubR*1.1"));
		for (auto it = nodeMap.constBegin(); it != nodeMap.constEnd(); ++it) {
			const MeshNodeCoord& nc = it.value();
			const double r = std::sqrt(nc.x * nc.x + nc.y * nc.y);
			if (r <= hubR * 1.1) {
				const int nid = it.key();
				hubNodes.append(nid);
				hubIds.insert(nid);
			}
		}
	}

	// 齿面载荷：solid 外边界面（KEY 同源）→ 排除端面/孔/hub + 径向齿带 + 啮合区约 3 齿角度窗（默认 +Y）
	const double rootR = std::max(0.1 * m, pitchR - (1.25 - dp.x1) * m);

	QMap<int, GearNodeXYZ> nodePos;
	double                 zMin = 1e300;
	double                 zMax = -1e300;
	for (auto it = nodeMap.constBegin(); it != nodeMap.constEnd(); ++it) {
		const MeshNodeCoord& nc = it.value();
		GearNodeXYZ      g;
		g.x = nc.x;
		g.y = nc.y;
		g.z = nc.z;
		nodePos.insert(it.key(), g);
		zMin = std::min(zMin, nc.z);
		zMax = std::max(zMax, nc.z);
	}

	GearToothPickContext pickCtx;
	pickCtx.module  = m;
	pickCtx.rootR   = rootR;
	pickCtx.pitchR  = pitchR;
	pickCtx.tipR    = tipR;
	pickCtx.hubR    = hubR;
	pickCtx.boreTol = boreTol;
	pickCtx.zMin           = zMin;
	pickCtx.zMax           = zMax;
	pickCtx.teethCount     = dp.z1;
	pickCtx.loadToothCount = 3.0;
	pickCtx.targetAngleRad = 0.0; // 0 → MeshConverter 内默认 π/2（正上方啮合区）

	QList<int> toothNodes;
	QString    tdiag;

	auto tryPick = [&](GearToothFacePickMode mode, const QString& label) -> bool {
		if (!collectToothNodesFromSolidExteriorFaces(meshPath, nodePos, hubIds, pickCtx, mode, &toothNodes,
		                                             &tdiag)) {
			return false;
		}
		emitLog(QStringLiteral("inp_write: NSET_TOOTH from ext. faces (%1), n=%2").arg(label).arg(toothNodes.size()));
		return true;
	};

	if (!tryPick(GearToothFacePickMode::ToothBand, QStringLiteral("3 teeth @ +Y, tooth band"))) {
		if (!tryPick(GearToothFacePickMode::RelaxedRadial, QStringLiteral("3 teeth @ +Y, relaxed radial"))) {
			if (!tryPick(GearToothFacePickMode::ExteriorShell, QStringLiteral("3 teeth @ +Y, exterior shell"))) {
				emitLog(QStringLiteral("inp_write: boundary-face tooth failed (%1)").arg(tdiag));
			}
		}
	}

	const int minToothNodesForCload = std::max(500, nodeMap.size() / 200);
	if (!toothNodes.isEmpty() && toothNodes.size() < minToothNodesForCload) {
		emitLog(QStringLiteral("inp_write: NSET_TOOTH n=%1 < %2; widen boundary pick (RelaxedRadial → ExteriorShell)")
		              .arg(toothNodes.size())
		              .arg(minToothNodesForCload));
		if (toothNodes.size() < minToothNodesForCload
		    && collectToothNodesFromSolidExteriorFaces(meshPath, nodePos, hubIds, pickCtx,
		                                             GearToothFacePickMode::RelaxedRadial, &toothNodes, &tdiag)) {
			emitLog(QStringLiteral("inp_write: NSET_TOOTH relaxed radial, n=%1").arg(toothNodes.size()));
		}
		if (toothNodes.size() < minToothNodesForCload
		    && collectToothNodesFromSolidExteriorFaces(meshPath, nodePos, hubIds, pickCtx,
		                                             GearToothFacePickMode::ExteriorShell, &toothNodes, &tdiag)) {
			emitLog(QStringLiteral("inp_write: NSET_TOOTH exterior shell, n=%1").arg(toothNodes.size()));
		}
	}

	if (hubNodes.isEmpty()) {
		double rMin = 1e300;
		double rMax = 0.0;
		for (auto it = nodeMap.constBegin(); it != nodeMap.constEnd(); ++it) {
			const MeshNodeCoord& nc = it.value();
			const double r = std::sqrt(nc.x * nc.x + nc.y * nc.y);
			rMin = std::min(rMin, r);
			rMax = std::max(rMax, r);
		}
		dp.errorMsg = QString("runInpWriteStep: no hub nodes (hubR=%1 mm, tol=%2, total=%3 nodes, r_xy min/max=%4/%5)")
		              .arg(hubR, 0, 'g', 4)
		              .arg(boreTol, 0, 'g', 4)
		              .arg(nodeMap.size())
		              .arg(rMin, 0, 'g', 5)
		              .arg(rMax, 0, 'g', 5);
		return false;
	}
	if (toothNodes.isEmpty()) {
		dp.errorMsg = QStringLiteral("runInpWriteStep: no tooth nodes for load (boundary-face pick failed)");
		return false;
	}

	auto appendNset = [&](const QString& name, const QList<int>& ids) -> bool {
		QFile f(meshPath);
		if (!f.open(QIODevice::Append | QIODevice::Text)) return false;
		QTextStream ts(&f);
		ts << "*NSET, NSET=" << name << "\n";
		for (int i = 0; i < ids.size(); ++i) {
			ts << ids[i];
			if ((i + 1) % 16 == 0 || i == ids.size() - 1)
				ts << "\n";
			else
				ts << ", ";
		}
		return f.error() == QFile::NoError;
	};
	if (!appendNset(QStringLiteral("NSET_HUB"), hubNodes)) {
		dp.errorMsg = QStringLiteral("runInpWriteStep: failed to append NSET_HUB");
		return false;
	}
	if (!appendNset(QStringLiteral("NSET_TOOTH"), toothNodes)) {
		dp.errorMsg = QStringLiteral("runInpWriteStep: failed to append NSET_TOOTH");
		return false;
	}

	const QString volumeElset = volumeElsetFromMeshInp(meshPath);
	emitLog(QStringLiteral("inp_write: volume ELSET=%1 (for CCX solid section / .dat parse)").arg(volumeElset));

	InpContext ctx;
	ctx.runName     = QString("g%1_p%2").arg(dp.generation).arg(dp.id);
	ctx.generation  = dp.generation;
	ctx.pointId     = dp.id;
	ctx.meshInpFile = QStringLiteral("mesh.inp");
	ctx.materialName = QStringLiteral("STEEL");
	ctx.youngModulus = _config.solver.youngModulus;
	ctx.poisson      = _config.solver.poissonRatio;
	ctx.density      = _config.solver.density;
	ctx.volumeElset  = volumeElset;
	// Gmsh 常带 *NSET,NSET=BEAM（Physical 节点组）；体元卡上 ELSET 多为 Volume1。*NODE PRINT 须引用存在的 NSET。
	ctx.outputNset  = QStringLiteral("BEAM");
	ctx.outputElset = volumeElset;

	BoundarySpec hub;
	hub.nset     = QStringLiteral("NSET_HUB");
	hub.dofStart = 1;
	hub.dofEnd   = 3;
	ctx.boundaries.append(hub);

	// 分布切向力（-X 方向，对应+Y 位置的切向传动方向）
	CLoadSpec cload;
	cload.nset  = QStringLiteral("NSET_TOOTH");
	cload.dof   = 1;
	cload.value = -Ft / static_cast<double>(toothNodes.size());
	ctx.cloads.append(cload);

	ctx.contactStiffness   = _config.solver.contactStiffness;
	ctx.contactPressureP0  = _config.solver.contactPressureP0;
	ctx.contactAdjustMm    = _config.solver.contactAdjustMm;
	configureCcxContact(ctx, meshPath, _config.solver.enableContact);
	ctx.requestContactOutput = false;
	ctx.requestNodeFile      = true;
	ctx.requestElFile        = true;
	recordContactOnDesignPoint(dp, ctx);
	dp.torque_Nm        = _config.solver.torque;
	dp.materialName     = QStringLiteral("STEEL");
	dp.youngModulus_MPa = _config.solver.youngModulus;
	dp.poissonRatio     = _config.solver.poissonRatio;
	dp.density          = _config.solver.density;

	if (!writeJobInp(workDir(), ctx)) {
		dp.errorMsg = QStringLiteral("runInpWriteStep: writeJobInp failed");
		return false;
	}

	const double densityKgPerMm3 = _config.solver.density * 1000.0; // t/mm³ → kg/mm³
	dp.mass_gear1 = densityKgPerMm3 * M_PI * (tipR * tipR - hubR * hubR) * dp.commonWidth;
	dp.syncLegacyResultFields();

	emitLog(QString("inp: hub=%1 nodes, tooth=%2 nodes, Ft=%3 N, mass~%4 kg")
	        .arg(hubNodes.size()).arg(toothNodes.size())
	        .arg(Ft, 0, 'f', 1).arg(dp.mass, 0, 'g', 4));
	return true;
}

// QEventLoop 把异步的 processFinish 信号包成同步调用，让 runOne() 保持线性流程。
bool GearOptCaseRunner::runSolveStep(GearDesignPoint& dp) {
	const int caseId = dp.id >= 0 ? dp.id : 0;
	const QString caseH = GearOptResultDatabase::caseHash(dp);

	const QString ccxPath = CCXSolverController::detectCcxPath();
	if (ccxPath.isEmpty()) {
		dp.errorMsg = QStringLiteral("runSolveStep: ccx_MT.exe not found; set CCX_PATH or place at tools/calculix/ccx_MT.exe");
		emitLogNormal(QStringLiteral("[GearOpt][CCX] case_id=%1 case_hash=%2 workDir=%3 solverStarted=false solverFinished=false")
		                  .arg(caseId).arg(caseH).arg(workDir()));
		return false;
	}

	auto* ctl = new CCXSolverController(this);
	ctl->setExePath(ccxPath);
	ctl->setWorkDir(workDir());
	ctl->setJobName(QStringLiteral("job"));
	// 调试阶段暂不启用 CCX 超时 kill（恢复时改回 _config.solver.timeoutSeconds）
	// ctl->setTimeoutSeconds(_config.solver.timeoutSeconds);
	ctl->setTimeoutSeconds(0);
	const int threadsEff =
	    (_threads > 0) ? _threads : CCXSolverController::defaultThreadCount();
	ctl->setThreads(threadsEff);
	ctl->setLogLevel(logLevel());
	ctl->setOutputLogPath(QDir(workDir()).filePath(QStringLiteral("ccx_output.log")));

	bool           success = false;
	CCXFailureReason reason = CCXFailureReason::None;
	QString        firstError;

	QEventLoop loop;
	connect(ctl, &CCXSolverController::processFinish,
	        [&](int, CCXFailureReason r, const QString& err) {
		        reason = r;
		        firstError = err;
		        success = (r == CCXFailureReason::None);
		        loop.quit();
	        });

	const qint64 t0ms = QDateTime::currentMSecsSinceEpoch();
	emitLogNormal(QStringLiteral("[GearOpt][CCX] case_id=%1 case_hash=%2 workDir=%3 solverStarted=true")
	                  .arg(caseId).arg(caseH).arg(workDir()));
	if (!ctl->start()) {
		dp.errorMsg = QStringLiteral("runSolveStep: CCXSolverController::start() failed (check ccx_MT.exe path / job.inp)");
		emitLogNormal(QStringLiteral("[GearOpt][CCX] case_id=%1 case_hash=%2 workDir=%3 solverFinished=false")
		                  .arg(caseId).arg(caseH).arg(workDir()));
		emitLogNormal(QStringLiteral("[CCX][Error] Solver failed, see ccx_output.log"));
		return false;
	}
	loop.exec();
	dp.solverTime = (QDateTime::currentMSecsSinceEpoch() - t0ms) / 1000.0;
	dp.solver     = _config.solver.solver.isEmpty() ? QStringLiteral("ccx") : _config.solver.solver;
	dp.solverPath = ctl->exePath().isEmpty() ? ccxPath : ctl->exePath();

	ctl->deleteLater();

	if (!success) {
		dp.errorMsg = QString("CCX: %1%2")
		              .arg(failureReasonToString(reason))
		              .arg(firstError.isEmpty() ? QString() : QStringLiteral(" – ") + firstError);
		emitLogNormal(QStringLiteral("[GearOpt][CCX] case_id=%1 case_hash=%2 workDir=%3 solverFinished=false")
		                  .arg(caseId).arg(caseH).arg(workDir()));
		emitLogNormal(QStringLiteral("[CCX][Error] Solver failed, see ccx_output.log"));
		return false;
	}
	emitLogNormal(QStringLiteral("[GearOpt][CCX] case_id=%1 case_hash=%2 workDir=%3 solverFinished=true converged=true time=%4 s")
	                  .arg(caseId)
	                  .arg(caseH)
	                  .arg(workDir())
	                  .arg(dp.solverTime, 0, 'f', 1));
	emitLogNormal(QStringLiteral("[CCX] Done: converged=true time=%1 s")
	                  .arg(dp.solverTime, 0, 'f', 1));
	return true;
}

namespace {

bool meshHasDualGearElsets(const QString& meshInpPath)
{
	return QFile::exists(meshInpPath) && meshInpHasElset(meshInpPath, QStringLiteral("GEAR1"))
	       && meshInpHasElset(meshInpPath, QStringLiteral("GEAR2"));
}

/// 优先从 job.frd（*NODE FILE U / *EL FILE S）提取位移与 Mises 应力。
bool parseMetricsFromFrd(const QString& frdPath,
                         const QString& meshInpPath,
                         bool           dualGear,
                         GearDesignPoint& dp,
                         const std::function<void(const QString&)>& logFn)
{
	if (!QFile::exists(frdPath))
		return false;

	const FrdResult frd = parseFrd(frdPath);
	const bool      hasU = frd.hasDisplacement();
	const bool      hasS = frd.stressCount() > 0;
	if (!hasU && !hasS) {
		const QStringList ds = listFrdDatasetNames(frdPath);
		const QString     dsText =
		    ds.isEmpty() ? QStringLiteral("none") : ds.join(QLatin1Char(','));
		if (logFn)
			logFn(QStringLiteral("parse: FRD empty (no U/S blocks; datasets=%1)").arg(dsText));
		return false;
	}

	QSet<int> gear1Nodes;
	QSet<int> gear2Nodes;
	const bool hasG1 =
	    dualGear && loadElsetNodeIds(meshInpPath, QStringLiteral("GEAR1"), &gear1Nodes);
	const bool hasG2 =
	    dualGear && loadElsetNodeIds(meshInpPath, QStringLiteral("GEAR2"), &gear2Nodes);

	if (hasU) {
		int peakTotal = -1;
		dp.uMax_total = frd.maxDisplMagnitude(&peakTotal);
		if (dualGear) {
			int node1 = -1;
			int node2 = -1;
			if (hasG1)
				dp.uMax_gear1 = maxDisplMagnitudeForNodes(frd, gear1Nodes, &node1);
			if (hasG2)
				dp.uMax_gear2 = maxDisplMagnitudeForNodes(frd, gear2Nodes, &node2);
			if (logFn) {
				logFn(QStringLiteral("parse: [frd] GEAR1 uMax=%1 mm node=%2")
				          .arg(dp.uMax_gear1, 0, 'g', 8)
				          .arg(node1));
				logFn(QStringLiteral("parse: [frd] GEAR2 uMax=%1 mm node=%2")
				          .arg(dp.uMax_gear2, 0, 'g', 8)
				          .arg(node2));
			}
		} else {
			dp.uMax_gear1 = dp.uMax_total;
			if (logFn) {
				logFn(QStringLiteral("parse: [frd] uMax=%1 mm node=%2 (U rows=%3)")
				          .arg(dp.uMax_total, 0, 'g', 8)
				          .arg(peakTotal)
				          .arg(frd.displCount()));
			}
		}
	} else if (logFn) {
		const QStringList ds = listFrdDatasetNames(frdPath);
		logFn(QStringLiteral("parse: [frd] displacement U not found (datasets=%1)")
		          .arg(ds.isEmpty() ? QStringLiteral("none") : ds.join(QLatin1Char(','))));
	}

	if (hasS) {
		int nodeTotal = -1;
		dp.sigmaMax_total = maxVonMisesForNodes(frd, QSet<int>{}, &nodeTotal);
		if (dualGear) {
			int node1 = -1;
			int node2 = -1;
			if (hasG1)
				dp.sigmaMax_gear1 = maxVonMisesForNodes(frd, gear1Nodes, &node1);
			if (hasG2)
				dp.sigmaMax_gear2 = maxVonMisesForNodes(frd, gear2Nodes, &node2);
			if (logFn) {
				logFn(QStringLiteral("parse: [frd] GEAR1 sigmaMax=%1 MPa node=%2")
				          .arg(dp.sigmaMax_gear1, 0, 'g', 8)
				          .arg(node1));
				logFn(QStringLiteral("parse: [frd] GEAR2 sigmaMax=%1 MPa node=%2")
				          .arg(dp.sigmaMax_gear2, 0, 'g', 8)
				          .arg(node2));
			}
		} else {
			dp.sigmaMax_gear1 = dp.sigmaMax_total;
			if (logFn) {
				logFn(QStringLiteral("parse: [frd] sigmaMax=%1 MPa node=%2 (S rows=%3)")
				          .arg(dp.sigmaMax_total, 0, 'g', 8)
				          .arg(nodeTotal)
				          .arg(frd.stressCount()));
			}
		}
	} else if (logFn) {
		logFn(QStringLiteral("parse: [frd] stress S not found"));
	}

	dp.cpressMax_MPa = frd.cpressMax_MPa;
	if (logFn) {
		if (dp.cpressMax_MPa >= 0.0) {
			logFn(QStringLiteral("parse: [frd] cpressMax=%1 MPa")
			          .arg(dp.cpressMax_MPa, 0, 'g', 8));
		} else {
			logFn(QStringLiteral("parse: [frd] cpressMax unavailable (no CPRESS in CONTACT block)"));
		}
	}

	return hasU || hasS;
}

/// FRD 缺失或指标不全时，从 job.dat *EL PRINT 回退应力。
void parseStressFallbackFromDat(const QString& datPath,
                                bool           dualGear,
                                const QString& volElset,
                                GearDesignPoint& dp,
                                const std::function<void(const QString&)>& logFn)
{
	if (!QFile::exists(datPath))
		return;

	if (dualGear) {
		if (dp.sigmaMax_gear1 < 0) {
			const DatStressResult r1 = parseDat(datPath, QStringLiteral("GEAR1"));
			if (r1.found()) {
				dp.sigmaMax_gear1      = r1.maxVonMises;
				dp.sigmaMax_gear1_elem = r1.maxElemId;
				dp.sigmaMax_gear1_ip   = r1.maxIntegPt;
				if (logFn) {
					logFn(QStringLiteral("parse: [dat] GEAR1 sigmaMax=%1 MPa elem=%2 ip=%3")
					          .arg(dp.sigmaMax_gear1, 0, 'g', 8)
					          .arg(dp.sigmaMax_gear1_elem)
					          .arg(dp.sigmaMax_gear1_ip));
				}
			}
		}
		if (dp.sigmaMax_gear2 < 0) {
			const DatStressResult r2 = parseDat(datPath, QStringLiteral("GEAR2"));
			if (r2.found()) {
				dp.sigmaMax_gear2      = r2.maxVonMises;
				dp.sigmaMax_gear2_elem = r2.maxElemId;
				dp.sigmaMax_gear2_ip   = r2.maxIntegPt;
				if (logFn) {
					logFn(QStringLiteral("parse: [dat] GEAR2 sigmaMax=%1 MPa elem=%2 ip=%3")
					          .arg(dp.sigmaMax_gear2, 0, 'g', 8)
					          .arg(dp.sigmaMax_gear2_elem)
					          .arg(dp.sigmaMax_gear2_ip));
				}
			}
		}
	} else if (dp.sigmaMax_gear1 < 0) {
		DatStressResult res = parseDat(datPath, volElset);
		if (!res.found() && volElset.compare(QStringLiteral("BEAM"), Qt::CaseInsensitive) != 0)
			res = parseDat(datPath, QStringLiteral("BEAM"));
		if (res.found()) {
			dp.sigmaMax_gear1      = res.maxVonMises;
			dp.sigmaMax_gear1_elem = res.maxElemId;
			dp.sigmaMax_gear1_ip   = res.maxIntegPt;
			if (logFn) {
				logFn(QStringLiteral("parse: [dat] sigmaMax=%1 MPa elset=%2 elem=%3 ip=%4")
				          .arg(dp.sigmaMax_gear1, 0, 'g', 8)
				          .arg(volElset)
				          .arg(res.maxElemId)
				          .arg(res.maxIntegPt));
			}
		}
	}
}

} // namespace

// 优先读 job.frd（*NODE FILE U / *EL FILE S）；缺失或指标不全时回退 job.dat。
bool GearOptCaseRunner::runParseStep(GearDesignPoint& dp) {
	const QDir d(workDir());

	const QString datPath     = d.filePath(QStringLiteral("job.dat"));
	const QString frdPath     = d.filePath(QStringLiteral("job.frd"));
	const QString meshInpPath = d.filePath(QStringLiteral("mesh.inp"));
	const bool    dualGear    = meshHasDualGearElsets(meshInpPath);
	const QString volElset    = QFile::exists(meshInpPath) ? volumeElsetFromMeshInp(meshInpPath)
	                                                       : QStringLiteral("BEAM");

	const bool frdParsed = parseMetricsFromFrd(frdPath, meshInpPath, dualGear, dp,
	                                           [this](const QString& s) { emitLog(s); });
	if (!frdParsed)
		emitLog(QStringLiteral("parse: job.frd unavailable or empty — fallback to job.dat for stress"));

	parseStressFallbackFromDat(datPath, dualGear, volElset, dp,
	                           [this](const QString& s) { emitLog(s); });

	const int binCount = _config.solver.meshZLayers > 0 ? _config.solver.meshZLayers : 11;
	const CpressDistributionMetrics cpressDist =
	    parseCpressDistributionMetrics(frdPath, binCount);
	dp.cpressMean_MPa       = cpressDist.cpressMean_MPa;
	dp.cpressStd_MPa        = cpressDist.cpressStd_MPa;
	dp.cpressCV             = cpressDist.cpressCV;
	dp.contactWidth_mm      = cpressDist.contactWidth_mm;
	dp.edgeLoadRatio        = cpressDist.edgeLoadRatio;
	dp.cpressEdgeMean_MPa   = cpressDist.cpressEdgeMean_MPa;
	dp.cpressCenterMean_MPa = cpressDist.cpressCenterMean_MPa;
	dp.cpressActiveNodes    = cpressDist.cpressActiveNodes;
	dp.cpressBinCount       = cpressDist.cpressBinCount;
	if (dp.cpressActiveNodes <= 0) {
		emitLog(QStringLiteral("[Result][Warning] CPRESS distribution unavailable: no active contact nodes above threshold"));
	} else if (dp.cpressCenterMean_MPa <= 0.0) {
		emitLog(QStringLiteral("[Result][Warning] CPRESS edgeLoadRatio unavailable: centerMeanCPRESS <= 0"));
	}

	dp.syncLegacyResultFields();

	emitLogNormal(QStringLiteral("[Result] sigmaMax=%1 MPa uMax=%2 mm")
	                  .arg(dp.sigmaMax_total, 0, 'g', 8)
	                  .arg(dp.uMax_total, 0, 'g', 4));
	emitLogNormal(QStringLiteral("[Result] cpressMax=%1 MPa mean=%2 MPa std=%3 MPa CV=%4 edgeRatio=%5 contactWidth=%6 mm")
	                  .arg(dp.cpressMax_MPa, 0, 'g', 8)
	                  .arg(dp.cpressMean_MPa, 0, 'g', 8)
	                  .arg(dp.cpressStd_MPa, 0, 'g', 8)
	                  .arg(dp.cpressCV, 0, 'g', 8)
	                  .arg(dp.edgeLoadRatio, 0, 'g', 8)
	                  .arg(dp.contactWidth_mm, 0, 'g', 8));

	emitLog(QStringLiteral("parse: sigmaMax_total=%1 MPa uMax_total=%2 mm cpressMax=%3 MPa")
	            .arg(dp.sigmaMax_total, 0, 'g', 8)
	            .arg(dp.uMax_total, 0, 'g', 4)
	            .arg(dp.cpressMax_MPa, 0, 'g', 8));
	if (dualGear) {
		emitLog(QStringLiteral("parse: dual gear1 σ=%1 u=%2 | gear2 σ=%3 u=%4")
		            .arg(dp.sigmaMax_gear1, 0, 'g', 6)
		            .arg(dp.uMax_gear1, 0, 'g', 6)
		            .arg(dp.sigmaMax_gear2, 0, 'g', 6)
		            .arg(dp.uMax_gear2, 0, 'g', 6));
	}

	if (dp.uMax_total < 0) {
		dp.errorMsg = QStringLiteral(
		    "displacement parse failed: uMax unavailable (verify *NODE FILE\\nU in job.inp and DISP/U block in job.frd)");
		return false;
	}

	if (dp.sigmaMax < 0) {
		dp.errorMsg = QStringLiteral("runParseStep: σ_max not extracted (check job.frd / job.dat)");
		return false;
	}

	if (dp.cpressMax_MPa <= 0.0) {
		dp.errorMsg = QStringLiteral(
		    "runParseStep: cpressMax unavailable (verify CONTACT pair and CPRESS in job.frd)");
		return false;
	}

	dp.fillResultArtifactPaths();

	return true;
}

} // namespace GearAutoOpt
