#include "MeshIndependenceRunner.h"
#include "GearOptCaseRunner.h"
#include "GearAutoOpt/data/GearLogLevel.h"
#include "GearAutoOpt/data/GearOptGeometryBridge.h"

#include <QDir>
#include <QFile>
#include <QTextStream>

#include <cmath>

namespace GearAutoOpt {

namespace {

QString csvField(double v)
{
	if (v < 0.0 || !std::isfinite(v))
		return QString();
	return QString::number(v, 'g', 8);
}

double errorPercent(double cur, double prev)
{
	if (prev <= 0.0 || cur < 0.0 || !std::isfinite(cur) || !std::isfinite(prev))
		return -1.0;
	return std::abs(cur - prev) / prev * 100.0;
}

} // namespace

MeshIndependenceRunner::MeshIndependenceRunner(const GearOptConfig& baseCfg, QObject* parent)
	: QObject(parent)
	, _baseCfg(baseCfg)
{}

QVector<MeshIndependenceCase> MeshIndependenceRunner::defaultCases() const
{
	QVector<MeshIndependenceCase> cases;
	cases.append({QStringLiteral("A"), 2.00, 0.50});
	cases.append({QStringLiteral("B"), 1.50, 0.40});
	cases.append({QStringLiteral("C"), 1.25, 0.35});
	if (_includeCaseD)
		cases.append({QStringLiteral("D"), 1.00, 0.35});
	return cases;
}

GearOptConfig MeshIndependenceRunner::makeCaseConfig(const MeshIndependenceCase& c) const
{
	GearOptConfig cfg = _baseCfg;
	cfg.solver.meshSize = c.globalSize;
	return cfg;
}

GearDesignPoint MeshIndependenceRunner::baseDesignPoint() const
{
	if (_baseCfg.useOptimizationBase && _baseCfg.optimizationBase.z1 > 0)
		return _baseCfg.optimizationBase;

	if (Geometry::GeometryParaGear* g = findCurrentGeometryParaGear())
		return gearDesignPointFromGeometryParaGear(*g);

	return _baseCfg.optimizationBase;
}

MeshIndependenceResult MeshIndependenceRunner::runSingleCase(const MeshIndependenceCase& c,
                                                             const QString&              rootDir,
                                                             int                         baseZLayers)
{
	MeshIndependenceResult out;
	out.caseName   = c.caseName;
	out.globalSize = c.globalSize;
	out.rootSize   = c.rootSize;
	out.layers     = baseZLayers;

	const QString workDir = QDir(rootDir).filePath(c.caseName);
	QDir().mkpath(workDir);
	out.workDir = QDir(workDir).absolutePath();

	emit log(QStringLiteral("[MeshIndep] === Case %1: global=%2 mm root=%3 mm layers=%4 ===")
	             .arg(c.caseName)
	             .arg(c.globalSize, 0, 'g', 4)
	             .arg(c.rootSize, 0, 'g', 4)
	             .arg(baseZLayers));

	const GearOptConfig caseCfg = makeCaseConfig(c);

	GearDesignPoint dp = baseDesignPoint();
	dp.generation = 0;
	dp.id         = 0;
	dp.runDir     = workDir;
	dp.applyRunSimDefaults(caseCfg, c.globalSize, false);

	GearOptCaseRunner runner;
	runner.setConfig(caseCfg);
	runner.setWorkDir(workDir);
	runner.setThreads(caseCfg.solver.threads);

	GearMeshParams meshParams;
	meshParams.globalSize = c.globalSize;
	meshParams.rootSize   = c.rootSize;
	meshParams.zLayers    = baseZLayers;
	runner.setMeshParams(meshParams);
	runner.setLogLevel(caseCfg.solver.debugMode ? GearLogLevel::Debug : GearLogLevel::Normal);

	connect(&runner, &GearOptCaseRunner::log, this, &MeshIndependenceRunner::log);

	runner.runOne(dp);
	runner.cleanup();

	out.nodeCount    = dp.nodeCount;
	out.elementCount = dp.elementCount;
	out.solveTimeSec = dp.solverTime;
	out.converged    = (dp.status == PointStatus::Done);

	if (out.converged && dp.sigmaMax >= 0.0 && dp.uMax >= 0.0) {
		out.sigmaMax = dp.sigmaMax;
		out.uMax     = dp.uMax;
	} else {
		out.sigmaMax = -1.0;
		out.uMax     = -1.0;
		out.converged = false;
		if (!dp.errorMsg.isEmpty())
			emit log(QStringLiteral("[MeshIndep] Case %1 failed: %2").arg(c.caseName, dp.errorMsg));
	}

	emit log(QStringLiteral("[MeshIndep] Case %1 done: converged=%2 σ=%3 MPa u=%4 mm nodes=%5 elems=%6 t=%7 s")
	             .arg(c.caseName)
	             .arg(out.converged ? QStringLiteral("yes") : QStringLiteral("no"))
	             .arg(out.sigmaMax, 0, 'g', 6)
	             .arg(out.uMax, 0, 'g', 6)
	             .arg(out.nodeCount)
	             .arg(out.elementCount)
	             .arg(out.solveTimeSec, 0, 'f', 1));

	return out;
}

void MeshIndependenceRunner::computeErrors(QVector<MeshIndependenceResult>& results) const
{
	for (int i = 1; i < results.size(); ++i) {
		const MeshIndependenceResult& prev = results[i - 1];
		MeshIndependenceResult&       cur  = results[i];
		if (!prev.converged || !cur.converged || prev.sigmaMax < 0.0 || cur.sigmaMax < 0.0
		    || prev.uMax < 0.0 || cur.uMax < 0.0) {
			cur.stressErrorPercent = -1.0;
			cur.dispErrorPercent   = -1.0;
			continue;
		}
		cur.stressErrorPercent = errorPercent(cur.sigmaMax, prev.sigmaMax);
		cur.dispErrorPercent   = errorPercent(cur.uMax, prev.uMax);
	}
}

double MeshIndependenceRunner::recommendedGlobalSize(
    const QVector<MeshIndependenceResult>& results) const
{
	if (results.isEmpty())
		return -1.0;

	// 取最后一对相邻收敛工况：误差 < 5% 推荐较粗网格，否则推荐较细网格。
	for (int i = results.size() - 1; i >= 1; --i) {
		const MeshIndependenceResult& prev = results[i - 1];
		const MeshIndependenceResult& cur  = results[i];
		if (!prev.converged || !cur.converged)
			continue;
		if (cur.stressErrorPercent < 0.0)
			continue;
		return (cur.stressErrorPercent < 5.0) ? prev.globalSize : cur.globalSize;
	}

	const MeshIndependenceResult& last = results.last();
	return last.converged ? last.globalSize : -1.0;
}

bool MeshIndependenceRunner::writeCsv(const QVector<MeshIndependenceResult>& results,
                                      const QString&                         csvPath,
                                      double                                 recommendedGlobal) const
{
	QFile f(csvPath);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
		return false;

	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	ts << QStringLiteral(
	          "case_name,global_size_mm,root_size_mm,layers,node_count,element_count,"
	          "sigma_max_mpa,u_max_mm,solve_time_s,converged,stress_error_percent,disp_error_percent,work_dir\n");

	for (const MeshIndependenceResult& r : results) {
		ts << r.caseName << ','
		   << r.globalSize << ','
		   << r.rootSize << ','
		   << r.layers << ','
		   << r.nodeCount << ','
		   << r.elementCount << ','
		   << csvField(r.sigmaMax) << ','
		   << csvField(r.uMax) << ','
		   << csvField(r.solveTimeSec) << ','
		   << (r.converged ? QStringLiteral("true") : QStringLiteral("false")) << ','
		   << csvField(r.stressErrorPercent) << ','
		   << csvField(r.dispErrorPercent) << ','
		   << '\"' << r.workDir << "\"\n";
	}

	if (recommendedGlobal > 0.0) {
		ts << QStringLiteral("recommended_mesh_size,")
		   << recommendedGlobal << ",,,,,,,,,,\n";
	}

	return f.error() == QFile::NoError;
}

void MeshIndependenceRunner::execute()
{
	const QString baseDir = _baseCfg.solver.runBaseDir.trimmed();
	if (baseDir.isEmpty()) {
		emit log(QStringLiteral("[MeshIndep] ERROR: runBaseDir is empty"));
		emit finished(false, QString());
		return;
	}

	const GearDesignPoint baseDp = baseDesignPoint();
	if (baseDp.z1 <= 0 || baseDp.module <= 0.0) {
		emit log(QStringLiteral("[MeshIndep] ERROR: invalid base gear parameters (create gear first)"));
		emit finished(false, QString());
		return;
	}

	const int baseZLayers = GearOptCaseRunner::computeAutoZLayers(baseDp.commonWidth);
	const QString rootDir = QDir(baseDir).filePath(QStringLiteral("mesh_independence"));
	QDir().mkpath(rootDir);

	emit log(QStringLiteral("[MeshIndep] start: base m=%1 z1=%2 z2=%3 width=%4 mm zLayers=%5")
	             .arg(baseDp.module, 0, 'g', 6)
	             .arg(baseDp.z1)
	             .arg(baseDp.z2)
	             .arg(baseDp.commonWidth, 0, 'g', 6)
	             .arg(baseZLayers));
	emit log(QStringLiteral("[MeshIndep] output root: %1").arg(QDir(rootDir).absolutePath()));

	const QVector<MeshIndependenceCase> cases = defaultCases();
	QVector<MeshIndependenceResult>       results;
	results.reserve(cases.size());

	for (int i = 0; i < cases.size(); ++i) {
		MeshIndependenceResult r = runSingleCase(cases[i], rootDir, baseZLayers);
		results.append(r);
		emit caseFinished(i, r);
	}

	computeErrors(results);

	const double recommended = recommendedGlobalSize(results);
	const QString csvPath =
	    QDir(rootDir).filePath(QStringLiteral("mesh_independence_results.csv"));
	const bool csvOk = writeCsv(results, csvPath, recommended);

	if (recommended > 0.0) {
		emit log(QStringLiteral("[MeshIndep] recommended global mesh size = %1 mm")
		             .arg(recommended, 0, 'g', 6));
	}
	if (csvOk) {
		emit log(QStringLiteral("[Save] csv: %1").arg(csvPath));
	} else {
		emit log(QStringLiteral("[MeshIndep] ERROR: failed to write CSV"));
	}

	emit finished(csvOk, csvPath);
}

} // namespace GearAutoOpt
