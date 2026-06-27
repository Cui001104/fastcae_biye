#include "GearOptConfig.h"

#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QSaveFile>
#include <QDebug>
#include <algorithm>
#include <cmath>

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

GearOptConfig::GearOptConfig()
{
	objectives.minCpressMax = true;
	objectives.minEdgeLoadRatio = true;
	objectives.minCpressCV = false;
	objectives.minSigmaMax = false;
	objectives.minMass = false;
}

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

	c.ca1Bound.lower  = 0.0; c.ca1Bound.upper  = 0.35;
	c.lca1Bound.lower = 0.0; c.lca1Bound.upper = 8.0;   // 后续由 module 缩放
	c.ca2Bound.lower  = 0.0; c.ca2Bound.upper  = 0.35;
	c.lca2Bound.lower = 0.0; c.lca2Bound.upper = 8.0;

	c.widthBound.lower    = 10.0; c.widthBound.upper    = 30.0;
	c.hubRatioBound.lower = 0.3;  c.hubRatioBound.upper = 0.5;

	c.solver.torque             = 1.0;
	c.solver.contactPressureP0  = 0.05;
	c.solver.contactStiffness   = 5.0;
	c.solver.enableContact     = true;
	c.solver.meshAutoMinMm     = 2.0;

	return c;
}

static void narrowToGlobal(double base, double halfWidth, double gLo, double gHi, double& lo, double& hi)
{
	lo = base - halfWidth;
	hi = base + halfWidth;
	lo = std::max(lo, gLo);
	hi = std::min(hi, gHi);
	if (lo > hi) {
		const double mid = std::clamp(base, gLo, gHi);
		lo = hi = mid;
	}
}

namespace {

void logBoundsLine(const GearOptConfig& c)
{
	auto fmtB = [](const char* name, const Bounds& b) -> QString {
		if (!b.discreteValues.isEmpty() && b.discreteValues.size() == 1) {
			const double v = b.discreteValues.front();
			return QStringLiteral("%1=[%2,%3]").arg(QLatin1String(name)).arg(v, 0, 'g', 8).arg(v, 0, 'g', 8);
		}
		if (b.isInteger)
			return QStringLiteral("%1=[%2,%3]")
			    .arg(QLatin1String(name))
			    .arg(static_cast<int>(std::lround(b.lower)))
			    .arg(static_cast<int>(std::lround(b.upper)));
		return QStringLiteral("%1=[%2,%3]")
		    .arg(QLatin1String(name))
		    .arg(b.lower, 0, 'g', 8)
		    .arg(b.upper, 0, 'g', 8);
	};

	qDebug().noquote() << QStringLiteral("[GearOpt][Bounds] %1, %2, %3, %4, %5, %6, %7, %8, %9, %10, %11, %12")
	                          .arg(fmtB("module", c.moduleBound))
	                          .arg(fmtB("z1", c.z1Bound))
	                          .arg(fmtB("z2", c.z2Bound))
	                          .arg(fmtB("alpha", c.alphaBound))
	                          .arg(fmtB("x1", c.x1Bound))
	                          .arg(fmtB("x2", c.x2Bound))
	                          .arg(fmtB("ca1", c.ca1Bound))
	                          .arg(fmtB("lca1", c.lca1Bound))
	                          .arg(fmtB("ca2", c.ca2Bound))
	                          .arg(fmtB("lca2", c.lca2Bound))
	                          .arg(fmtB("width", c.widthBound))
	                          .arg(fmtB("hubRatio", c.hubRatioBound));
}

} // namespace

static void setFixedContinuous(Bounds& b, double v)
{
	b.lower = b.upper = v;
	b.isInteger       = false;
	b.discreteValues.clear();
}

static void setFixedDiscreteOne(Bounds& b, double v)
{
	b.discreteValues = {v};
	b.lower = b.upper = v;
	b.isInteger       = false;
}

static void setFixedInteger(Bounds& b, int z)
{
	b.lower = b.upper = static_cast<double>(z);
	b.isInteger       = true;
	b.discreteValues.clear();
}

static void narrowModuleAroundBase(Bounds& out, const GearOptConfig& def, double m)
{
	QVector<double> opts;
	for (double v : def.moduleBound.discreteValues) {
		if (std::abs(v - m) <= 0.75 + 1e-9) opts.append(v);
	}
	if (opts.isEmpty()) {
		double best = def.moduleBound.discreteValues.first();
		for (double v : def.moduleBound.discreteValues) {
			if (std::abs(v - m) < std::abs(best - m)) best = v;
		}
		opts.append(best);
	}
	std::sort(opts.begin(), opts.end());
	if (opts.size() == 1) {
		const QVector<double>& full = def.moduleBound.discreteValues;
		int idx = -1;
		for (int i = 0; i < full.size(); ++i) {
			if (std::abs(full[i] - opts[0]) < 1e-9) {
				idx = i;
				break;
			}
		}
		if (idx > 0) opts.prepend(full[idx - 1]);
		if (idx >= 0 && idx + 1 < full.size()) opts.append(full[idx + 1]);
		std::sort(opts.begin(), opts.end());
	}
	out.discreteValues = opts;
	out.lower          = *std::min_element(opts.begin(), opts.end());
	out.upper          = *std::max_element(opts.begin(), opts.end());
	out.isInteger      = false;
}

static void narrowZAroundBase(Bounds& out, int z, int zMin, int zMax, int halfSpan)
{
	int lo = std::max(zMin, z - halfSpan);
	int hi = std::min(zMax, z + halfSpan);
	if (lo > hi) lo = hi = z;
	out.lower      = static_cast<double>(lo);
	out.upper      = static_cast<double>(hi);
	out.isInteger  = true;
	out.discreteValues.clear();
}

static void narrowAlphaAroundBase(Bounds& out, const GearOptConfig& def, double a)
{
	QVector<double> opts;
	for (double v : def.alphaBound.discreteValues) {
		if (std::abs(v - a) <= 3.0 + 1e-9) opts.append(v);
	}
	if (opts.isEmpty()) {
		double best = def.alphaBound.discreteValues.first();
		for (double v : def.alphaBound.discreteValues) {
			if (std::abs(v - a) < std::abs(best - a)) best = v;
		}
		opts.append(best);
	}
	std::sort(opts.begin(), opts.end());
	out.discreteValues = opts;
	out.lower          = opts.first();
	out.upper          = opts.last();
	out.isInteger      = false;
}

GearOptConfig GearOptConfig::fromBasePoint(const GearDesignPoint& basePoint, const DesignVariableFlags& dv)
{
	const GearOptConfig def = defaultConfig();
	GearOptConfig       c   = def;
	c.designVars            = dv;
	c.optimizationBase      = basePoint;
	c.useOptimizationBase   = true;
	const GearDesignPoint& b = c.optimizationBase;

	const double gXLo = -0.3, gXHi = 0.6;
	const double gCaLo = def.ca1Bound.lower;
	const double gCaHi = def.ca1Bound.upper;
	const double gLcaLo = 0.0, gLcaHi = 8.0;
	const double gWLo = 10.0, gWHi = 30.0;
	const double gHLo = 0.3, gHHi = 0.5;

	double lo = 0.0, hi = 0.0;

	if (!dv.module)
		setFixedDiscreteOne(c.moduleBound, b.module);
	else
		narrowModuleAroundBase(c.moduleBound, def, b.module);

	if (!dv.z1)
		setFixedInteger(c.z1Bound, b.z1);
	else
		narrowZAroundBase(c.z1Bound, b.z1, 17, 40, 2);

	if (!dv.z2)
		setFixedInteger(c.z2Bound, b.z2);
	else
		narrowZAroundBase(c.z2Bound, b.z2, 25, 80, 2);

	if (!dv.alpha) {
		setFixedDiscreteOne(c.alphaBound, b.alpha);
	} else {
		narrowAlphaAroundBase(c.alphaBound, def, b.alpha);
	}

	if (!dv.x1) {
		setFixedContinuous(c.x1Bound, b.x1);
	} else {
		narrowToGlobal(b.x1, 0.15, gXLo, gXHi, lo, hi);
		c.x1Bound.lower = lo;
		c.x1Bound.upper = hi;
	}
	if (!dv.x2) {
		setFixedContinuous(c.x2Bound, b.x2);
	} else {
		narrowToGlobal(b.x2, 0.15, gXLo, gXHi, lo, hi);
		c.x2Bound.lower = lo;
		c.x2Bound.upper = hi;
	}
	if (!dv.ca1) {
		setFixedContinuous(c.ca1Bound, b.ca1);
	} else {
		narrowToGlobal(b.ca1, 0.05, gCaLo, gCaHi, lo, hi);
		c.ca1Bound.lower = lo;
		c.ca1Bound.upper = hi;
	}
	if (!dv.ca2) {
		setFixedContinuous(c.ca2Bound, b.ca2);
	} else {
		narrowToGlobal(b.ca2, 0.05, gCaLo, gCaHi, lo, hi);
		c.ca2Bound.lower = lo;
		c.ca2Bound.upper = hi;
	}

	const double m = std::max(b.module, 1e-6);
	if (!dv.lca1) {
		setFixedContinuous(c.lca1Bound, b.lca1);
	} else {
		narrowToGlobal(b.lca1, 0.5 * m, gLcaLo, gLcaHi, lo, hi);
		c.lca1Bound.lower = lo;
		c.lca1Bound.upper = hi;
	}
	if (!dv.lca2) {
		setFixedContinuous(c.lca2Bound, b.lca2);
	} else {
		narrowToGlobal(b.lca2, 0.5 * m, gLcaLo, gLcaHi, lo, hi);
		c.lca2Bound.lower = lo;
		c.lca2Bound.upper = hi;
	}

	if (!dv.width) {
		setFixedContinuous(c.widthBound, b.commonWidth);
	} else {
		narrowToGlobal(b.commonWidth, 3.0, gWLo, gWHi, lo, hi);
		c.widthBound.lower = lo;
		c.widthBound.upper = hi;
	}

	if (!dv.hubRatio) {
		setFixedContinuous(c.hubRatioBound, b.hubRatio);
	} else {
		narrowToGlobal(b.hubRatio, 0.05, gHLo, gHHi, lo, hi);
		c.hubRatioBound.lower = lo;
		c.hubRatioBound.upper = hi;
	}

	logBoundsLine(c);
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
	objJson["min_cpress_max"] = objectives.minCpressMax;
	objJson["min_edge_load_ratio"] = objectives.minEdgeLoadRatio;
	objJson["min_cpress_cv"] = objectives.minCpressCV;
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
	solverJson["keep_run_dir"]       = solver.keepRunDir;
	solverJson["contact_pressure_p0"] = solver.contactPressureP0;
	solverJson["contact_stiffness"]   = solver.contactStiffness;
	solverJson["contact_adjust_mm"]  = solver.contactAdjustMm;
	solverJson["enable_contact"]     = solver.enableContact;
	solverJson["use_rigid_body"]     = solver.useRigidBody;
	solverJson["use_coupling"]       = solver.useCoupling;
	solverJson["use_kinematic_coupling"] = solver.useKinematicCoupling;
	solverJson["mesh_auto_min_mm"]   = solver.meshAutoMinMm;
	solverJson["mesh_size"]          = solver.meshSize;
	solverJson["mesh_root_size_mm"]  = solver.meshRootSizeMm;
	solverJson["mesh_z_layers"]      = solver.meshZLayers;
	solverJson["run_base_dir"]       = solver.runBaseDir;
	solverJson["ccx_drive_displacement_mm"] = solver.ccxDriveDisplacementMm;
	solverJson["ccx_drive_rotation_rad"]  = solver.ccxDriveRotationRad;
	solverJson["ccx_use_rotation_drive"] = solver.ccxUseRotationDrive;
	solverJson["ccx_rigid_hub_samples"] = solver.ccxRigidHubSamples;
	solverJson["surrogate_assisted"]    = solver.surrogateAssisted;
	solverJson["debug_mode"]            = solver.debugMode;
	solverJson["threads"]               = solver.threads;
	solverJson["parallel_ccx_enabled"]  = solver.parallelCcxEnabled;
	solverJson["parallel_ccx_jobs"]     = solver.parallelCcxJobs;
	solverJson["ccx_threads_per_job"]   = solver.ccxThreadsPerJob;
	root["solver"]                   = solverJson;

	QJsonObject dvJson;
	dvJson["module"]    = designVars.module;
	dvJson["z1"]        = designVars.z1;
	dvJson["z2"]        = designVars.z2;
	dvJson["alpha"]     = designVars.alpha;
	dvJson["x1"]        = designVars.x1;
	dvJson["x2"]        = designVars.x2;
	dvJson["ca1"]       = designVars.ca1;
	dvJson["lca1"]      = designVars.lca1;
	dvJson["ca2"]       = designVars.ca2;
	dvJson["lca2"]      = designVars.lca2;
	dvJson["width"]     = designVars.width;
	dvJson["hub_ratio"] = designVars.hubRatio;
	root["design_vars"] = dvJson;

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
	c.objectives.minCpressMax = obj.value("min_cpress_max").toBool(true);
	c.objectives.minEdgeLoadRatio = obj.value("min_edge_load_ratio").toBool(true);
	c.objectives.minCpressCV = obj.value("min_cpress_cv").toBool(false);
	c.objectives.minSigmaMax  = obj.value("min_sigma_max").toBool(false);
	c.objectives.minMass      = obj.value("min_mass").toBool(false);
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
	c.solver.torque         = sv.value("torque").toDouble(1.0);
	c.solver.youngModulus   = sv.value("young_modulus").toDouble(2.06e5);
	c.solver.poissonRatio   = sv.value("poisson_ratio").toDouble(0.30);
	c.solver.density        = sv.value("density").toDouble(7.85e-9);
	c.solver.timeoutSeconds = sv.value("timeout_seconds").toInt(1200);
	c.solver.keepRunDir       = sv.value("keep_run_dir").toBool(true);
	c.solver.contactPressureP0 = sv.value("contact_pressure_p0").toDouble(0.05);
	c.solver.contactStiffness  = sv.value("contact_stiffness").toDouble(5.0);
	c.solver.contactAdjustMm  = sv.value("contact_adjust_mm").toDouble(0.0);
	c.solver.enableContact    = sv.value("enable_contact").toBool(true);
	c.solver.useRigidBody     = sv.value("use_rigid_body").toBool(true);
	c.solver.useCoupling      = sv.value("use_coupling").toBool(false);
	c.solver.useKinematicCoupling = sv.value("use_kinematic_coupling").toBool(true);
	c.solver.meshAutoMinMm    = sv.value("mesh_auto_min_mm").toDouble(2.0);
	if (sv.contains("mesh_size"))
		c.solver.meshSize = sv.value("mesh_size").toDouble(1.50);
	if (sv.contains("mesh_root_size_mm"))
		c.solver.meshRootSizeMm = sv.value("mesh_root_size_mm").toDouble(0.40);
	if (sv.contains("mesh_z_layers"))
		c.solver.meshZLayers = sv.value("mesh_z_layers").toInt(11);
	c.solver.runBaseDir       = sv.value("run_base_dir").toString();
	if (sv.contains("ccx_drive_displacement_mm"))
		c.solver.ccxDriveDisplacementMm = sv.value("ccx_drive_displacement_mm").toDouble(0.001);
	if (sv.contains("ccx_drive_rotation_rad"))
		c.solver.ccxDriveRotationRad = sv.value("ccx_drive_rotation_rad").toDouble(0.0001);
	else if (sv.value("ccx_coupling_torque").toBool(false))
		c.solver.ccxDriveRotationRad = 0.0001;
	if (sv.contains("ccx_rigid_hub_samples"))
		c.solver.ccxRigidHubSamples = sv.value("ccx_rigid_hub_samples").toInt(24);
	c.solver.surrogateAssisted = sv.value("surrogate_assisted").toBool(false);
	c.solver.debugMode         = sv.value("debug_mode").toBool(false);
	c.solver.ccxUseRotationDrive = sv.value("ccx_use_rotation_drive").toBool(false);
	if (sv.contains("threads"))
		c.solver.threads = sv.value("threads").toInt(0);
	c.solver.parallelCcxEnabled = sv.value("parallel_ccx_enabled").toBool(false);
	c.solver.parallelCcxJobs    = sv.value("parallel_ccx_jobs").toInt(2);
	c.solver.ccxThreadsPerJob   = sv.value("ccx_threads_per_job").toInt(4);

	if (root.contains("design_vars") && root.value("design_vars").isObject()) {
		const QJsonObject dvo = root.value("design_vars").toObject();
		if (dvo.contains("module")) c.designVars.module = dvo.value("module").toBool(c.designVars.module);
		if (dvo.contains("z1")) c.designVars.z1 = dvo.value("z1").toBool(c.designVars.z1);
		if (dvo.contains("z2")) c.designVars.z2 = dvo.value("z2").toBool(c.designVars.z2);
		if (dvo.contains("alpha")) c.designVars.alpha = dvo.value("alpha").toBool(c.designVars.alpha);
		if (dvo.contains("x1")) c.designVars.x1 = dvo.value("x1").toBool(c.designVars.x1);
		if (dvo.contains("x2")) c.designVars.x2 = dvo.value("x2").toBool(c.designVars.x2);
		if (dvo.contains("ca1")) c.designVars.ca1 = dvo.value("ca1").toBool(c.designVars.ca1);
		if (dvo.contains("lca1")) c.designVars.lca1 = dvo.value("lca1").toBool(c.designVars.lca1);
		if (dvo.contains("ca2")) c.designVars.ca2 = dvo.value("ca2").toBool(c.designVars.ca2);
		if (dvo.contains("lca2")) c.designVars.lca2 = dvo.value("lca2").toBool(c.designVars.lca2);
		if (dvo.contains("width")) c.designVars.width = dvo.value("width").toBool(c.designVars.width);
		if (dvo.contains("hub_ratio")) c.designVars.hubRatio = dvo.value("hub_ratio").toBool(c.designVars.hubRatio);
	}

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
