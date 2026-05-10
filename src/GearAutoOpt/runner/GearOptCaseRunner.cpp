#include "GearOptCaseRunner.h"
#include "GearAutoOpt/solver/MeshConverter.h"
#include "GearAutoOpt/solver/CCXInpWriter.h"
#include "GearAutoOpt/solver/CCXSolverController.h"
#include "GearAutoOpt/solver/CCXResultParser.h"

#include "GeometryCommand/GeoCommandCreateGear.h"
#include "Geometry/geometryData.h"
#include "Geometry/geometrySet.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QMap>
#include <QProcess>
#include <QStringList>
#include <QTextStream>

#include <BRepTools.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

namespace GearAutoOpt {

GearOptCaseRunner::GearOptCaseRunner(QObject* parent)
	: GearAutoCaseRunner(parent)
{}

GearOptCaseRunner::~GearOptCaseRunner() {
	cleanup();
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
	// 上一次 run 的临时齿轮（如果还在）先清掉
	cleanup();

	auto* geoData = Geometry::GeometryData::getInstance();
	const int beforeCount = geoData->getGeometrySetCount();

	auto* cmd = new Command::GeoCommandCreateGear(nullptr, nullptr);
	cmd->setName(QString("dp_g%1_p%2").arg(dp.generation).arg(dp.id));
	cmd->setNumberOfTeeth(dp.z1);
	cmd->setNumberOfSecondTeeth(dp.z2);
	cmd->setModule(dp.module);
	cmd->setPressureAngle(dp.alpha);
	cmd->setAddendumCoefficient(1.0);
	cmd->setDedendumCoefficient(1.25);
	cmd->setFilletCoefficient(0.38);
	cmd->setThickness(dp.width);
	cmd->setExternalGear(true);
	cmd->setTipReliefAmount(dp.ca1);
	cmd->setTipReliefLength(dp.lca1);
	cmd->setTipReliefAmount2(dp.ca2);
	cmd->setTipReliefLength2(dp.lca2);
	cmd->setprofileShiftCoefficient1(dp.x1);
	cmd->setprofileShiftCoefficient2(dp.x2);

	const bool ok = cmd->execute();
	if (!ok) {
		dp.errorMsg = QStringLiteral("GeoCommandCreateGear::execute() failed");
		delete cmd;
		return false;
	}

	const int afterCount = geoData->getGeometrySetCount();
	if (afterCount != beforeCount + 2) {
		dp.errorMsg = QString("expected geometry set count +2, got +%1")
		              .arg(afterCount - beforeCount);
		delete cmd;
		return false;
	}

	// 取最后追加的两个为 _set1 / _set2（GeoCommandCreateGear::execute 末尾的
	// 顺序：先 set1 再 set2）
	_set1 = geoData->getGeometrySetAt(afterCount - 2);
	_set2 = geoData->getGeometrySetAt(afterCount - 1);
	_geomCmd = cmd;

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
	if (!_set1) {
		dp.errorMsg = QStringLiteral("runMeshStep: _set1 is null (geometry step missing?)");
		return false;
	}
	TopoDS_Shape* shapePtr = _set1->getShape();
	if (!shapePtr || shapePtr->IsNull()) {
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
	const QString brepPath = d.filePath(QStringLiteral("gear1.brep"));
	if (!BRepTools::Write(*shapePtr, brepPath.toLocal8Bit().constData())) {
		dp.errorMsg = QStringLiteral("runMeshStep: BRepTools::Write failed");
		return false;
	}

	const double meshSize = (_meshSize > 0.0) ? _meshSize : (0.5 * dp.module);
	const QString geoPath = d.filePath(QStringLiteral("gear.geo"));
	{
		QFile geoFile(geoPath);
		if (!geoFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
			dp.errorMsg = QStringLiteral("runMeshStep: cannot write gear.geo");
			return false;
		}
		QTextStream ts(&geoFile);
		ts.setCodec("UTF-8");
		ts << "// generated by GearOptCaseRunner\n";
		ts << "SetFactory(\"OpenCASCADE\");\n";
		ts << "Merge \"gear1.brep\";\n";
		ts << "Physical Volume(\"BEAM\") = {1};\n";
		ts << "Mesh.MeshSizeMax = " << QString::number(meshSize, 'g', 6) << ";\n";
		ts << "Mesh.ElementOrder = 1;\n";
		ts << "Mesh.Algorithm3D = 1;\n";
		ts << "Mesh.SaveGroupsOfNodes = 1;\n";
	}

	emitLog(QString("mesh: gmsh -3 -format inp gear.geo (mesh size %1)")
	        .arg(QString::number(meshSize, 'g', 4)));
	QProcess proc;
	proc.setWorkingDirectory(workDir());
	proc.start(gmshPath, QStringList{"-3", "-format", "inp",
	                                  "-o", "mesh.inp", "gear.geo"});
	if (!proc.waitForStarted(5000)) {
		dp.errorMsg = QStringLiteral("runMeshStep: gmsh failed to start");
		return false;
	}
	if (!proc.waitForFinished(180000)) {
		proc.kill();
		dp.errorMsg = QStringLiteral("runMeshStep: gmsh timeout (>180s)");
		return false;
	}
	if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0) {
		dp.errorMsg = QString("runMeshStep: gmsh exit=%1 stderr=%2")
		              .arg(proc.exitCode())
		              .arg(QString::fromLocal8Bit(proc.readAllStandardError()).left(200));
		return false;
	}

	const QString meshInp = d.filePath(QStringLiteral("mesh.inp"));
	if (!QFile::exists(meshInp) || QFileInfo(meshInp).size() == 0) {
		dp.errorMsg = QStringLiteral("runMeshStep: mesh.inp missing or empty");
		return false;
	}

	// 剥离 surface element，NSET 保留用作边界条件
	const int removed = stripSurfaceElements(meshInp);
	emitLog(QString("mesh: stripped %1 surface element rows").arg(removed));
	emitLog(QString("mesh: %1").arg(meshInp));
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

	struct NodeCoord { double x = 0, y = 0, z = 0; };
	QMap<int, NodeCoord> nodeMap;
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
			NodeCoord nc;
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

	// hub 节点：r ≤ hubR × 1.1（内孔面）；tooth 节点：r ≥ pitchR × 0.7 且在 +Y 60°~120° 扇区（啮合侧）
	QList<int> hubNodes, toothNodes;
	for (auto it = nodeMap.constBegin(); it != nodeMap.constEnd(); ++it) {
		const NodeCoord& nc = it.value();
		const double r = std::sqrt(nc.x * nc.x + nc.y * nc.y);
		if (r <= hubR * 1.1) {
			hubNodes.append(it.key());
		} else if (r >= pitchR * 0.7) {
			const double angle = std::atan2(nc.y, nc.x); // [-π, π]
			if (angle >= M_PI/ 3.0 && angle <= 2.0 * M_PI/ 3.0) {
				toothNodes.append(it.key());
			}
		}
	}

	if (hubNodes.isEmpty()) {
		dp.errorMsg = QString("runInpWriteStep: no hub nodes (hubR=%1 mm, total=%2 nodes)")
		              .arg(hubR, 0, 'g', 4).arg(nodeMap.size());
		return false;
	}
	if (toothNodes.isEmpty()) {
		dp.errorMsg = QStringLiteral("runInpWriteStep: no tooth nodes in +Y sector");
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

	InpContext ctx;
	ctx.runName     = QString("g%1_p%2").arg(dp.generation).arg(dp.id);
	ctx.generation  = dp.generation;
	ctx.pointId     = dp.id;
	ctx.meshInpFile = QStringLiteral("mesh.inp");
	ctx.materialName = QStringLiteral("STEEL");
	ctx.youngModulus = _config.solver.youngModulus;
	ctx.poisson      = _config.solver.poissonRatio;
	ctx.density      = _config.solver.density;
	ctx.volumeElset  = QStringLiteral("BEAM");

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

	if (!writeJobInp(workDir(), ctx)) {
		dp.errorMsg = QStringLiteral("runInpWriteStep: writeJobInp failed");
		return false;
	}

	const double densityKgPerMm3 = _config.solver.density * 1000.0; // t/mm³ → kg/mm³
	dp.mass = densityKgPerMm3 * M_PI* (tipR * tipR - hubR * hubR) * dp.width;

	emitLog(QString("inp: hub=%1 nodes, tooth=%2 nodes, Ft=%3 N, mass~%4 kg")
	        .arg(hubNodes.size()).arg(toothNodes.size())
	        .arg(Ft, 0, 'f', 1).arg(dp.mass, 0, 'g', 4));
	return true;
}

// QEventLoop 把异步的 processFinish 信号包成同步调用，让 runOne() 保持线性流程。
bool GearOptCaseRunner::runSolveStep(GearDesignPoint& dp) {
	const QString ccxPath = CCXSolverController::detectCcxPath();
	if (ccxPath.isEmpty()) {
		dp.errorMsg = QStringLiteral("runSolveStep: ccx.exe not found; set CCX_PATH or place at tools/calculix/ccx.exe");
		return false;
	}

	auto* ctl = new CCXSolverController(this);
	ctl->setExePath(ccxPath);
	ctl->setWorkDir(workDir());
	ctl->setJobName(QStringLiteral("job"));
	ctl->setTimeoutSeconds(_config.solver.timeoutSeconds);
	ctl->setThreads(_threads);

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
	connect(ctl, &CCXSolverController::sendMessage,
	        [this](const QString& msg) { emitLog(msg); });

	const qint64 t0ms = QDateTime::currentMSecsSinceEpoch();
	if (!ctl->start()) {
		dp.errorMsg = QStringLiteral("runSolveStep: CCXSolverController::start() failed (check ccx.exe path / job.inp)");
		return false;
	}
	loop.exec();
	dp.solverTime = (QDateTime::currentMSecsSinceEpoch() - t0ms) / 1000.0;
	ctl->deleteLater();

	if (!success) {
		dp.errorMsg = QString("CCX: %1%2")
		              .arg(failureReasonToString(reason))
		              .arg(firstError.isEmpty() ? QString() : QStringLiteral(" – ") + firstError);
		return false;
	}
	emitLog(QString("solve: done in %1 s").arg(dp.solverTime, 0, 'f', 1));
	return true;
}

// 优先读 .dat（积分点应力精度高），.frd 补位移，也作为 .dat 缺失时的应力备选。
bool GearOptCaseRunner::runParseStep(GearDesignPoint& dp) {
	const QDir d(workDir());

	// .dat 积分点 von Mises
	const QString datPath = d.filePath(QStringLiteral("job.dat"));
	if (QFile::exists(datPath)) {
		const DatStressResult res = parseDat(datPath, QStringLiteral("BEAM"));
		if (res.found()) {
			dp.sigmaMax = res.maxVonMises;
			emitLog(QString("parse: σ_max=%1 MPa (.dat, %2 samples)")
			        .arg(dp.sigmaMax, 0, 'g', 6).arg(res.sampleCount));
		}
	}

	// .frd 节点位移 + 备份节点应力
	const QString frdPath = d.filePath(QStringLiteral("job.frd"));
	if (QFile::exists(frdPath)) {
		const FrdResult frd = parseFrd(frdPath);
		dp.uMax = frd.maxDisplMagnitude();
		if (dp.sigmaMax < 0 && frd.nodeCount() > 0) {
			dp.sigmaMax = frd.maxVonMises();
			emitLog(QString("parse: σ_max=%1 MPa (.frd node stress)")
			        .arg(dp.sigmaMax, 0, 'g', 6));
		}
		emitLog(QString("parse: u_max=%1 mm").arg(dp.uMax, 0, 'g', 4));
	}

	if (dp.sigmaMax < 0) {
		dp.errorMsg = QStringLiteral("runParseStep: σ_max not extracted (check .dat/.frd)");
		return false;
	}
	return true;
}

} // namespace GearAutoOpt
