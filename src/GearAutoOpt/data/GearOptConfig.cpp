#include "GearOptConfig.h"

#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QSaveFile>
#include <QDebug>

namespace GearAutoOpt {

namespace {
QJsonObject boundsToJson(const Bounds& b)
{
	QJsonObject o;
	o["lower"]      = b.lower;
	o["upper"]      = b.upper;
	o["is_integer"] = b.isInteger;
	if(!b.discreteValues.isEmpty()) {
		QJsonArray arr;
		for(double v : b.discreteValues) arr.append(v);
		o["discrete"] = arr;
	}
	return o;
}

Bounds boundsFromJson(const QJsonObject& o)
{
	Bounds b;
	b.lower     = o.value("lower").toDouble(0.0);
	b.upper     = o.value("upper").toDouble(1.0);
	b.isInteger = o.value("is_integer").toBool(false);
	if(o.contains("discrete")) {
		const QJsonArray arr = o.value("discrete").toArray();
		for(const auto& v : arr) b.discreteValues.append(v.toDouble());
	}
	return b;
}
} // anonymous

GearOptConfig::GearOptConfig() = default;

GearOptConfig GearOptConfig::defaultConfig()
{
	GearOptConfig c;

	c.moduleBound.discreteValues = {1.5, 2.0, 2.5, 3.0, 4.0};
	c.moduleBound.lower          = 1.5;
	c.moduleBound.upper          = 4.0;

	c.z1Bound.lower      = 17; c.z1Bound.upper = 40; c.z1Bound.isInteger = true;
	c.z2Bound.lower      = 25; c.z2Bound.upper = 80; c.z2Bound.isInteger = true;

	c.alphaBound.discreteValues = {20.0, 22.5, 25.0};
	c.alphaBound.lower = 20.0; c.alphaBound.upper = 25.0;

	c.x1Bound.lower = -0.3; c.x1Bound.upper = 0.6;
	c.x2Bound.lower = -0.3; c.x2Bound.upper = 0.6;

	c.ca1Bound.lower  = 0.0; c.ca1Bound.upper  = 0.05;
	c.lca1Bound.lower = 0.0; c.lca1Bound.upper = 8.0;   // 后续由 module 缩放
	c.ca2Bound.lower  = 0.0; c.ca2Bound.upper  = 0.05;
	c.lca2Bound.lower = 0.0; c.lca2Bound.upper = 8.0;

	c.widthBound.lower    = 10.0; c.widthBound.upper    = 30.0;
	c.hubRatioBound.lower = 0.3;  c.hubRatioBound.upper = 0.5;

	return c;
}

QJsonObject GearOptConfig::toJson() const
{
	QJsonObject root;
	root["run_name"] = runName;

	QJsonObject vars;
	vars["module"]    = boundsToJson(moduleBound);
	vars["z1"]        = boundsToJson(z1Bound);
	vars["z2"]        = boundsToJson(z2Bound);
	vars["alpha"]     = boundsToJson(alphaBound);
	vars["x1"]        = boundsToJson(x1Bound);
	vars["x2"]        = boundsToJson(x2Bound);
	vars["ca1"]       = boundsToJson(ca1Bound);
	vars["lca1"]      = boundsToJson(lca1Bound);
	vars["ca2"]       = boundsToJson(ca2Bound);
	vars["lca2"]      = boundsToJson(lca2Bound);
	vars["width"]     = boundsToJson(widthBound);
	vars["hub_ratio"] = boundsToJson(hubRatioBound);
	root["variables"] = vars;

	QJsonObject objJson;
	objJson["min_sigma_max"]  = objectives.minSigmaMax;
	objJson["min_mass"]       = objectives.minMass;
	objJson["min_ratio_err"]  = objectives.minRatioErr;
	objJson["target_ratio"]   = objectives.targetRatio;
	root["objectives"]        = objJson;

	QJsonObject conJson;
	conJson["sigma_allow"]              = constraints.sigmaAllow;
	conJson["min_tooth_count"]          = constraints.minToothCount;
	conJson["min_sa_over_m"]            = constraints.minSaOverM;
	conJson["center_distance_tol"]      = constraints.centerDistanceTol;
	conJson["target_center_distance"]   = constraints.targetCenterDistance;
	root["constraints"]                 = conJson;

	QJsonObject nsgaJson;
	nsgaJson["population_size"]    = nsga2.populationSize;
	nsgaJson["max_generations"]    = nsga2.maxGenerations;
	nsgaJson["crossover_rate"]     = nsga2.crossoverRate;
	nsgaJson["mutation_rate"]      = nsga2.mutationRate;
	nsgaJson["sbx_eta"]            = nsga2.sbxEta;
	nsgaJson["mut_eta"]            = nsga2.mutEta;
	nsgaJson["random_seed"]        = nsga2.randomSeed;
	nsgaJson["hypervolume_tol"]    = nsga2.hypervolumeTol;
	nsgaJson["convergence_window"] = nsga2.convergenceWindow;
	root["nsga2"]                  = nsgaJson;

	QJsonObject solverJson;
	solverJson["solver"]           = solver.solver;
	solverJson["torque"]           = solver.torque;
	solverJson["young_modulus"]    = solver.youngModulus;
	solverJson["poisson_ratio"]    = solver.poissonRatio;
	solverJson["density"]          = solver.density;
	solverJson["timeout_seconds"]  = solver.timeoutSeconds;
	solverJson["keep_run_dir"]     = solver.keepRunDir;
	root["solver"]                 = solverJson;

	return root;
}

GearOptConfig GearOptConfig::fromJson(const QJsonObject& root)
{
	GearOptConfig c = defaultConfig();   // 先取默认值，未覆盖字段保留
	c.runName = root.value("run_name").toString("default");

	const QJsonObject vars = root.value("variables").toObject();
	if(vars.contains("module"))    c.moduleBound   = boundsFromJson(vars.value("module").toObject());
	if(vars.contains("z1"))        c.z1Bound       = boundsFromJson(vars.value("z1").toObject());
	if(vars.contains("z2"))        c.z2Bound       = boundsFromJson(vars.value("z2").toObject());
	if(vars.contains("alpha"))     c.alphaBound    = boundsFromJson(vars.value("alpha").toObject());
	if(vars.contains("x1"))        c.x1Bound       = boundsFromJson(vars.value("x1").toObject());
	if(vars.contains("x2"))        c.x2Bound       = boundsFromJson(vars.value("x2").toObject());
	if(vars.contains("ca1"))       c.ca1Bound      = boundsFromJson(vars.value("ca1").toObject());
	if(vars.contains("lca1"))      c.lca1Bound     = boundsFromJson(vars.value("lca1").toObject());
	if(vars.contains("ca2"))       c.ca2Bound      = boundsFromJson(vars.value("ca2").toObject());
	if(vars.contains("lca2"))      c.lca2Bound     = boundsFromJson(vars.value("lca2").toObject());
	if(vars.contains("width"))     c.widthBound    = boundsFromJson(vars.value("width").toObject());
	if(vars.contains("hub_ratio")) c.hubRatioBound = boundsFromJson(vars.value("hub_ratio").toObject());

	const QJsonObject obj = root.value("objectives").toObject();
	c.objectives.minSigmaMax = obj.value("min_sigma_max").toBool(true);
	c.objectives.minMass     = obj.value("min_mass").toBool(true);
	c.objectives.minRatioErr = obj.value("min_ratio_err").toBool(false);
	c.objectives.targetRatio = obj.value("target_ratio").toDouble(1.0);

	const QJsonObject con = root.value("constraints").toObject();
	c.constraints.sigmaAllow            = con.value("sigma_allow").toDouble(600.0);
	c.constraints.minToothCount         = con.value("min_tooth_count").toDouble(17);
	c.constraints.minSaOverM            = con.value("min_sa_over_m").toDouble(0.25);
	c.constraints.centerDistanceTol     = con.value("center_distance_tol").toDouble(0.05);
	c.constraints.targetCenterDistance  = con.value("target_center_distance").toDouble(-1.0);

	const QJsonObject ns = root.value("nsga2").toObject();
	c.nsga2.populationSize    = ns.value("population_size").toInt(30);
	c.nsga2.maxGenerations    = ns.value("max_generations").toInt(20);
	c.nsga2.crossoverRate     = ns.value("crossover_rate").toDouble(0.9);
	c.nsga2.mutationRate      = ns.value("mutation_rate").toDouble(0.1);
	c.nsga2.sbxEta            = ns.value("sbx_eta").toDouble(15.0);
	c.nsga2.mutEta            = ns.value("mut_eta").toDouble(20.0);
	c.nsga2.randomSeed        = ns.value("random_seed").toInt(42);
	c.nsga2.hypervolumeTol    = ns.value("hypervolume_tol").toDouble(1e-3);
	c.nsga2.convergenceWindow = ns.value("convergence_window").toInt(3);

	const QJsonObject sv = root.value("solver").toObject();
	c.solver.solver         = sv.value("solver").toString("ccx");
	c.solver.torque         = sv.value("torque").toDouble(100.0);
	c.solver.youngModulus   = sv.value("young_modulus").toDouble(2.06e5);
	c.solver.poissonRatio   = sv.value("poisson_ratio").toDouble(0.30);
	c.solver.density        = sv.value("density").toDouble(7.85e-9);
	c.solver.timeoutSeconds = sv.value("timeout_seconds").toInt(600);
	c.solver.keepRunDir     = sv.value("keep_run_dir").toBool(true);

	return c;
}

bool GearOptConfig::saveToFile(const QString& path) const
{
	QSaveFile f(path);
	if(!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
		qWarning() << "GearOptConfig::saveToFile open failed:" << f.errorString();
		return false;
	}
	const QJsonDocument doc(toJson());
	f.write(doc.toJson(QJsonDocument::Indented));
	if(!f.commit()) {
		qWarning() << "GearOptConfig::saveToFile commit failed:" << f.errorString();
		return false;
	}
	return true;
}

bool GearOptConfig::loadFromFile(const QString& path)
{
	QFile f(path);
	if(!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
		qWarning() << "GearOptConfig::loadFromFile open failed:" << f.errorString();
		return false;
	}
	QJsonParseError perr;
	const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &perr);
	if(perr.error != QJsonParseError::NoError) {
		qWarning() << "GearOptConfig::loadFromFile parse error:" << perr.errorString();
		return false;
	}
	*this = fromJson(doc.object());
	return true;
}

} // namespace GearAutoOpt
