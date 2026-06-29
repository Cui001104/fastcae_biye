#include "GearOptResultDatabase.h"

#include "GearAutoOpt/data/GearMetricDefinition.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSet>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QStringList>
#include <QTextStream>
#include <QThread>
#include <QVariant>
#include <cmath>

namespace GearAutoOpt {

namespace {

constexpr const char kGlobalConnection[]   = "GearOptGlobal";
constexpr const char kRunConnection[]      = "GearOptRun";
constexpr const char kDatabaseFileName[]   = "GearOptResults.db";
constexpr const char kMainTable[]          = "gear_opt_results";
constexpr const char kLegacyTable[]        = "simulation_result";
constexpr const char kMetricDefTable[]     = "metric_definitions";
constexpr const char kObjectiveMapTable[]  = "objective_mapping";
constexpr const char kDesignVarDefTable[]  = "design_variable_definitions";

void logSqlQuery(const QSqlQuery& query, const char* step)
{
	const QSqlError err = query.lastError();
	if (err.isValid())
		qDebug().noquote() << QStringLiteral("[GearOpt][DB] %1 lastError: %2 (%3)")
		                              .arg(QString::fromLatin1(step))
		                              .arg(err.text())
		                              .arg(err.databaseText());
}

double variantToDouble(const QVariant& v, double fallback)
{
	if (!v.isValid() || v.isNull())
		return fallback;
	bool ok = false;
	const double d = v.toDouble(&ok);
	return ok ? d : fallback;
}

bool ensureColumn(QSqlQuery& query, const char* table, const char* columnSql)
{
	const QString sql = QStringLiteral("ALTER TABLE %1 ADD COLUMN %2")
	                        .arg(QString::fromLatin1(table))
	                        .arg(QString::fromLatin1(columnSql));
	if (!query.exec(sql)) {
		const QString err = query.lastError().text();
		if (err.contains(QStringLiteral("duplicate column"), Qt::CaseInsensitive))
			return true;
		logSqlQuery(query, "ensureColumn");
		return false;
	}
	return true;
}

QString validatedSurrogateSampleWhereSql()
{
	return QStringLiteral(
	    " status = 'done'"
	    " AND converged = 1"
	    " AND is_valid = 1"
	    " AND verified_by_ccx = 1"
	    " AND cpressMax_MPa > 0"
	    " AND sigmaMax_MPa > 0"
	    " AND uMax_mm >= 0"
	    " AND mass_kg > 0"
	    " AND edgeLoadRatio IS NOT NULL"
	    " AND edgeLoadRatio > 0"
	    " AND cpressCV IS NOT NULL"
	    " AND cpressCV > 0");
}

bool computeConvergedFromMetrics(const GearDesignPoint& dp)
{
	return dp.status == PointStatus::Done
	       && dp.cpressMax_MPa > 0.0 && std::isfinite(dp.cpressMax_MPa)
	       && dp.sigmaMax > 0.0 && std::isfinite(dp.sigmaMax)
	       && dp.uMax >= 0.0 && std::isfinite(dp.uMax)
	       && dp.mass > 0.0 && std::isfinite(dp.mass)
	       && dp.edgeLoadRatio > 0.0 && std::isfinite(dp.edgeLoadRatio)
	       && dp.cpressCV > 0.0 && std::isfinite(dp.cpressCV);
}

int computeIsValidFlag(const GearDesignPoint& dp)
{
	if (dp.status == PointStatus::Failed || dp.status == PointStatus::Invalid)
		return 0;
	if (dp.cpressMax_MPa < 0.0 || dp.sigmaMax < 0.0 || dp.uMax < 0.0 || dp.mass < 0.0)
		return 0;
	if (!computeConvergedFromMetrics(dp))
		return 0;
	return 1;
}

bool backfillValidityAndConvergedColumns(QSqlQuery& query)
{
	if (!query.exec(QStringLiteral(
	        "UPDATE gear_opt_results SET is_valid = 0"
	        " WHERE status = 'failed'"
	        " OR cpressMax_MPa < 0 OR sigmaMax_MPa < 0"
	        " OR uMax_mm < 0 OR mass_kg < 0"))) {
		logSqlQuery(query, "backfillIsValidFailed");
	}
	if (!query.exec(QStringLiteral(
	        "UPDATE gear_opt_results SET is_valid = 1, converged = 1"
	        " WHERE status = 'done'"
	        " AND cpressMax_MPa > 0 AND sigmaMax_MPa > 0"
	        " AND uMax_mm >= 0 AND mass_kg > 0"
	        " AND edgeLoadRatio > 0 AND cpressCV > 0"))) {
		logSqlQuery(query, "backfillIsValidDone");
	}
	return true;
}

int countSamplesForBaseCase(QSqlQuery& query,
                            const QString& baseCaseH,
                            double fixedWidthMm)
{
	QString sql = QStringLiteral(
	    "SELECT COUNT(*) FROM gear_opt_results WHERE base_case_hash = ?");
	if (fixedWidthMm >= 0.0)
		sql += QStringLiteral(" AND ABS(width - ?) < 1e-6");
	query.prepare(sql);
	query.addBindValue(baseCaseH);
	if (fixedWidthMm >= 0.0)
		query.addBindValue(fixedWidthMm);
	if (!query.exec() || !query.next())
		return 0;
	return query.value(0).toInt();
}

} // namespace

GearOptResultDatabase::GearOptResultDatabase(const char* connectionName)
    : _connectionName(QString::fromLatin1(connectionName))
{}

GearOptResultDatabase& GearOptResultDatabase::global()
{
	static GearOptResultDatabase inst(kGlobalConnection);
	return inst;
}

GearOptResultDatabase& GearOptResultDatabase::runSession()
{
	static GearOptResultDatabase inst(kRunConnection);
	return inst;
}

GearOptResultDatabase& GearOptResultDatabase::instance()
{
	return global();
}

QString GearOptResultDatabase::connectionName() const
{
	const quintptr tid = reinterpret_cast<quintptr>(QThread::currentThreadId());
	return QStringLiteral("%1_%2").arg(_connectionName).arg(tid, 0, 16);
}

QString GearOptResultDatabase::defaultGlobalDatabasePath()
{
	QString base = QCoreApplication::applicationDirPath();
	if (base.isEmpty())
		base = QDir::currentPath();
	return QDir(base).filePath(QString::fromLatin1(kDatabaseFileName));
}

QString GearOptResultDatabase::databasePathInRunDir(const QString& runBaseDir)
{
	return QDir(runBaseDir).filePath(QString::fromLatin1(kDatabaseFileName));
}

QString GearOptResultDatabase::generateRunId(int randomSeed)
{
	return QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))
	       + QLatin1Char('_') + QString::number(randomSeed);
}

QString GearOptResultDatabase::createTableSql()
{
	return QStringLiteral(
	    "CREATE TABLE IF NOT EXISTS gear_opt_results ("
	    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
	    "  run_id TEXT,"
	    "  gen INTEGER,"
	    "  individual_id INTEGER,"
	    "  created_at TEXT,"
	    "  z1 INTEGER, z2 INTEGER, module REAL, alpha REAL,"
	    "  x1 REAL, x2 REAL, ca1 REAL, lca1 REAL, ca2 REAL, lca2 REAL,"
	    "  width REAL, hubRatio REAL,"
	    "  addendumCoeff REAL, dedendumCoeff REAL, rootFilletCoeff REAL,"
	    "  meshSize_mm REAL, meshAuto INTEGER, meshMethod TEXT,"
	    "  elementOrder INTEGER, nodeCount INTEGER, elementCount INTEGER,"
	    "  materialName TEXT, youngModulus_MPa REAL, poissonRatio REAL, density REAL,"
	    "  torque_Nm REAL, enableContact INTEGER, contactType TEXT,"
	    "  contactStiffness REAL, frictionCoeff REAL,"
	    "  solverName TEXT, solverPath TEXT, solverVersion TEXT, staticStep TEXT,"
	    "  sigmaMax_MPa REAL, sigmaMax_gear1_MPa REAL, sigmaMax_gear2_MPa REAL,"
	    "  uMax_mm REAL, uMax_gear1_mm REAL, uMax_gear2_mm REAL,"
	    "  mass_kg REAL, mass_gear1_kg REAL, mass_gear2_kg REAL,"
	    "  cpressMax_MPa REAL,"
	    "  cpressMean_MPa REAL, cpressStd_MPa REAL, cpressCV REAL,"
	    "  contactWidth_mm REAL, edgeLoadRatio REAL,"
	    "  cpressEdgeMean_MPa REAL, cpressCenterMean_MPa REAL,"
	    "  cpressActiveNodes INTEGER, cpressBinCount INTEGER,"
	    "  status TEXT, errorMsg TEXT, runDir TEXT,"
	    "  meshInpPath TEXT, jobInpPath TEXT, datPath TEXT, frdPath TEXT,"
	    "  isPareto INTEGER DEFAULT 0, rank INTEGER, crowdingDistance REAL,"
	    "  param_hash TEXT,"
	    "  case_hash TEXT,"
	    "  sample_source TEXT DEFAULT 'ccx',"
	    "  surrogate_used INTEGER DEFAULT 0,"
	    "  verified_by_ccx INTEGER DEFAULT 1,"
	    "  base_case_hash TEXT,"
	    "  design_hash TEXT"
	    ")");
}

bool GearOptResultDatabase::openDatabase(const QString& dbPath)
{
	_dbPath = dbPath.isEmpty() ? defaultGlobalDatabasePath() : dbPath;

	const QString conn = connectionName();
	if (QSqlDatabase::contains(conn)) {
		QSqlDatabase existing = QSqlDatabase::database(conn);
		if (existing.isOpen() && existing.databaseName() == _dbPath) {
			_open = true;
			return createTablesIfNeeded();
		}
		existing.close();
		QSqlDatabase::removeDatabase(conn);
	}

	{
		QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), conn);
		db.setDatabaseName(_dbPath);
		if (!db.open()) {
			qDebug() << "[GearOpt][DB] database open failed:" << db.lastError().text()
			         << "conn=" << conn << "path=" << _dbPath;
			_open = false;
			return false;
		}
	}

	qDebug().noquote() << QStringLiteral("[GearOpt][DB] opened conn=%1 path=%2")
	                      .arg(conn, _dbPath);
	_open = createTablesIfNeeded();
	return _open;
}

void GearOptResultDatabase::closeDatabase()
{
	const QString conn = connectionName();
	if (QSqlDatabase::contains(conn)) {
		QSqlDatabase db = QSqlDatabase::database(conn, false);
		if (db.isOpen())
			db.close();
		QSqlDatabase::removeDatabase(conn);
	}
	_open = false;
}

bool GearOptResultDatabase::isOpen() const
{
	if (!_open)
		return false;
	const QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	return db.isValid() && db.isOpen();
}

bool GearOptResultDatabase::createTablesIfNeeded()
{
	return ensureGearOptResultsTable()
	       && ensureLegacySimulationTable()
	       && ensureOptimizationConfigTables();
}

bool GearOptResultDatabase::ensureOptimizationConfigTables()
{
	return ensureMetricDefinitionsTable()
	       && migrateMetricDefinitionsColumns()
	       && ensureObjectiveMappingTable()
	       && ensureDesignVariableDefinitionsTable()
	       && insertDefaultOptimizationConfigIfEmpty();
}

bool GearOptResultDatabase::ensureGearOptResultsTable() const
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;

	QSqlQuery query(db);
	if (!query.exec(createTableSql())) {
		logSqlQuery(query, "createGearOptResults");
		return false;
	}
	qDebug().noquote() << QStringLiteral("[GearOpt][DB] create/update table gear_opt_results");

	static const char* kExtraColumns[] = {
	    "run_id TEXT",
	    "individual_id INTEGER",
	    "addendumCoeff REAL",
	    "dedendumCoeff REAL",
	    "rootFilletCoeff REAL",
	    "meshSize_mm REAL",
	    "meshAuto INTEGER",
	    "meshMethod TEXT",
	    "elementOrder INTEGER",
	    "nodeCount INTEGER",
	    "elementCount INTEGER",
	    "materialName TEXT",
	    "youngModulus_MPa REAL",
	    "poissonRatio REAL",
	    "density REAL",
	    "torque_Nm REAL",
	    "enableContact INTEGER",
	    "contactType TEXT",
	    "contactStiffness REAL",
	    "frictionCoeff REAL",
	    "solverName TEXT",
	    "solverPath TEXT",
	    "solverVersion TEXT",
	    "staticStep TEXT",
	    "sigmaMax_gear1_MPa REAL",
	    "sigmaMax_gear2_MPa REAL",
	    "uMax_gear1_mm REAL",
	    "uMax_gear2_mm REAL",
	    "mass_gear1_kg REAL",
	    "mass_gear2_kg REAL",
	    "cpressMax_MPa REAL",
	    "cpressMean_MPa REAL",
	    "cpressStd_MPa REAL",
	    "cpressCV REAL",
	    "contactWidth_mm REAL",
	    "edgeLoadRatio REAL",
	    "cpressEdgeMean_MPa REAL",
	    "cpressCenterMean_MPa REAL",
	    "cpressActiveNodes INTEGER",
	    "cpressBinCount INTEGER",
	    "meshInpPath TEXT",
	    "jobInpPath TEXT",
	    "datPath TEXT",
	    "frdPath TEXT",
	    "isPareto INTEGER DEFAULT 0",
	    "rank INTEGER",
	    "crowdingDistance REAL",
	    "param_hash TEXT",
	    "case_hash TEXT",
	    "sample_source TEXT DEFAULT 'ccx'",
	    "surrogate_used INTEGER DEFAULT 0",
	    "verified_by_ccx INTEGER DEFAULT 1",
	    "base_case_hash TEXT",
	    "design_hash TEXT",
	    "converged INTEGER DEFAULT 0",
	    "solve_time_s REAL",
	    "retry_count INTEGER DEFAULT 0",
	    "last_error_message TEXT",
	    "last_retry_time TEXT",
	    "is_valid INTEGER DEFAULT 1",
	};
	for (const char* col : kExtraColumns)
		ensureColumn(query, kMainTable, col);

	backfillValidityAndConvergedColumns(query);

	return true;
}

bool GearOptResultDatabase::ensureLegacySimulationTable() const
{
	QSqlQuery query(QSqlDatabase::database(connectionName(), false));
	const QString legacyCreate = QStringLiteral(
	    "CREATE TABLE IF NOT EXISTS simulation_result ("
	    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
	    "  created_at TEXT, gen INTEGER, point_id INTEGER,"
	    "  z1 INTEGER, z2 INTEGER, module REAL, alpha REAL,"
	    "  x1 REAL, x2 REAL, ca1 REAL, lca1 REAL, ca2 REAL, lca2 REAL,"
	    "  width REAL, hubRatio REAL,"
	    "  sigmaMax_MPa REAL, uMax_mm REAL, mass_kg REAL,"
	    "  status TEXT, errorMsg TEXT, param_hash TEXT UNIQUE"
	    ")");
	if (!query.exec(legacyCreate))
		logSqlQuery(query, "createLegacySimulation");
	return true;
}

QString GearOptResultDatabase::paramHash(const GearDesignPoint& dp)
{
	const QString key = QStringLiteral("%1_%2_%3_%4_%5_%6_%7_%8_%9_%10_%11_%12")
	                        .arg(dp.z1)
	                        .arg(dp.z2)
	                        .arg(dp.module, 0, 'g', 12)
	                        .arg(dp.alpha, 0, 'g', 12)
	                        .arg(dp.x1, 0, 'g', 12)
	                        .arg(dp.x2, 0, 'g', 12)
	                        .arg(dp.ca1, 0, 'g', 12)
	                        .arg(dp.lca1, 0, 'g', 12)
	                        .arg(dp.ca2, 0, 'g', 12)
	                        .arg(dp.lca2, 0, 'g', 12)
	                        .arg(dp.commonWidth, 0, 'g', 12)
	                        .arg(dp.hubRatio, 0, 'g', 12);
	const QByteArray digest =
	    QCryptographicHash::hash(key.toUtf8(), QCryptographicHash::Md5);
	return QString::fromLatin1(digest.toHex());
}

QString GearOptResultDatabase::caseHash(const GearDesignPoint& dp)
{
	const QString solverName =
	    dp.solver.isEmpty() ? QStringLiteral("CalculiX") : dp.solver;
	const QString key = QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8|%9|%10")
	                        .arg(paramHash(dp))
	                        .arg(dp.torque_Nm, 0, 'g', 12)
	                        .arg(dp.meshSize_mm, 0, 'g', 12)
	                        .arg(dp.meshAuto ? 1 : 0)
	                        .arg(dp.contactStiffness, 0, 'g', 12)
	                        .arg(dp.enableContact ? 1 : 0)
	                        .arg(dp.youngModulus_MPa, 0, 'g', 12)
	                        .arg(dp.poissonRatio, 0, 'g', 12)
	                        .arg(dp.density, 0, 'g', 12)
	                        .arg(solverName);
	const QByteArray digest =
	    QCryptographicHash::hash(key.toUtf8(), QCryptographicHash::Md5);
	return QString::fromLatin1(digest.toHex());
}

QString GearOptResultDatabase::baseCaseHash(const GearDesignPoint& dp)
{
	const QString solverName =
	    dp.solver.isEmpty() ? QStringLiteral("CalculiX") : dp.solver;
	// 固定工况：m,z,alpha,扭矩,网格,接触,材料,求解器；不含修形/变位等设计变量（齿宽由 SQL 过滤）。
	const QString key = QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8|%9|%10|%11|%12")
	                        .arg(dp.z1)
	                        .arg(dp.z2)
	                        .arg(dp.module, 0, 'g', 12)
	                        .arg(dp.alpha, 0, 'g', 12)
	                        .arg(dp.torque_Nm, 0, 'g', 12)
	                        .arg(dp.meshSize_mm, 0, 'g', 12)
	                        .arg(dp.meshAuto ? 1 : 0)
	                        .arg(dp.contactStiffness, 0, 'g', 12)
	                        .arg(dp.enableContact ? 1 : 0)
	                        .arg(dp.youngModulus_MPa, 0, 'g', 12)
	                        .arg(dp.poissonRatio, 0, 'g', 12)
	                        .arg(dp.density, 0, 'g', 12)
	                        .arg(solverName);
	const QByteArray digest =
	    QCryptographicHash::hash(key.toUtf8(), QCryptographicHash::Md5);
	return QString::fromLatin1(digest.toHex());
}

QString GearOptResultDatabase::designHash(const GearDesignPoint& dp)
{
	// 7 维代理设计空间（齿宽由 baseCase 固定，不含在 design_hash 中）。
	const QString key = QStringLiteral("%1|%2|%3|%4|%5|%6|%7")
	                        .arg(dp.x1, 0, 'g', 12)
	                        .arg(dp.x2, 0, 'g', 12)
	                        .arg(dp.ca1, 0, 'g', 12)
	                        .arg(dp.lca1, 0, 'g', 12)
	                        .arg(dp.ca2, 0, 'g', 12)
	                        .arg(dp.lca2, 0, 'g', 12)
	                        .arg(dp.hubRatio, 0, 'g', 12);
	const QByteArray digest =
	    QCryptographicHash::hash(key.toUtf8(), QCryptographicHash::Md5);
	return QString::fromLatin1(digest.toHex());
}

bool GearOptResultDatabase::findIdByCaseHash(const QString& hash, int* outId) const
{
	if (!isOpen() || hash.isEmpty())
		return false;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);

	query.prepare(QStringLiteral(
	    "SELECT id FROM gear_opt_results"
	    " WHERE case_hash = ? AND status = 'done'"
	    " AND converged = 1 AND is_valid = 1"
	    " AND cpressMax_MPa > 0"
	    " AND case_hash IS NOT NULL AND TRIM(case_hash) != ''"
	    " ORDER BY id DESC LIMIT 1"));
	query.addBindValue(hash);
	if (!query.exec() || !query.next())
		return false;
	if (outId)
		*outId = query.value(0).toInt();
	return true;
}

bool GearOptResultDatabase::loadResultByCaseHash(const QString& hash, GearDesignPoint& dp) const
{
	if (!isOpen() || hash.isEmpty())
		return false;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	query.prepare(QStringLiteral(
	    "SELECT id, run_id, gen, individual_id, created_at,"
	    " z1, z2, module, alpha, x1, x2, ca1, lca1, ca2, lca2, width, hubRatio,"
	    " addendumCoeff, dedendumCoeff, rootFilletCoeff,"
	    " meshSize_mm, meshAuto, meshMethod, elementOrder, nodeCount, elementCount,"
	    " materialName, youngModulus_MPa, poissonRatio, density,"
	    " torque_Nm, enableContact, contactType, contactStiffness, frictionCoeff,"
	    " solverName, solverPath, solverVersion, staticStep,"
	    " sigmaMax_MPa, sigmaMax_gear1_MPa, sigmaMax_gear2_MPa,"
	    " uMax_mm, uMax_gear1_mm, uMax_gear2_mm,"
	    " mass_kg, mass_gear1_kg, mass_gear2_kg, cpressMax_MPa,"
	    " cpressMean_MPa, cpressStd_MPa, cpressCV, contactWidth_mm, edgeLoadRatio,"
	    " cpressEdgeMean_MPa, cpressCenterMean_MPa, cpressActiveNodes, cpressBinCount,"
	    " status, errorMsg, runDir, meshInpPath, jobInpPath, datPath, frdPath,"
	    " isPareto, rank, crowdingDistance"
	    " FROM gear_opt_results"
	    " WHERE case_hash = ? AND status = 'done'"
	    " AND converged = 1 AND is_valid = 1"
	    " AND cpressMax_MPa > 0"
	    " AND case_hash IS NOT NULL AND TRIM(case_hash) != ''"
	    " ORDER BY id DESC LIMIT 1"));
	query.addBindValue(hash);
	if (!query.exec() || !query.next())
		return false;

	int col = 0;
	/* col 0 = row pk */ query.value(col++).toInt();
	dp.runId           = query.value(col++).toString();
	dp.generation      = query.value(col++).toInt();
	dp.id              = query.value(col++).toInt();
	const QString created = query.value(col++).toString();
	dp.createdAt       = QDateTime::fromString(created, Qt::ISODate);
	dp.z1              = query.value(col++).toInt();
	dp.z2              = query.value(col++).toInt();
	dp.module          = query.value(col++).toDouble();
	dp.alpha           = query.value(col++).toDouble();
	dp.x1              = query.value(col++).toDouble();
	dp.x2              = query.value(col++).toDouble();
	dp.ca1             = query.value(col++).toDouble();
	dp.lca1            = query.value(col++).toDouble();
	dp.ca2             = query.value(col++).toDouble();
	dp.lca2            = query.value(col++).toDouble();
	dp.commonWidth     = query.value(col++).toDouble();
	dp.hubRatio        = query.value(col++).toDouble();
	dp.addendumCoeff   = variantToDouble(query.value(col++), 1.0);
	dp.dedendumCoeff   = variantToDouble(query.value(col++), 1.25);
	dp.rootFilletCoeff = variantToDouble(query.value(col++), 0.38);
	dp.meshSize_mm     = variantToDouble(query.value(col++), 0.0);
	dp.meshAuto        = query.value(col++).toInt() != 0;
	dp.meshMethod      = query.value(col++).toString();
	dp.elementOrder    = query.value(col++).toInt();
	dp.nodeCount       = query.value(col++).toInt();
	dp.elementCount    = query.value(col++).toInt();
	dp.materialName    = query.value(col++).toString();
	dp.youngModulus_MPa = variantToDouble(query.value(col++), 206000.0);
	dp.poissonRatio    = variantToDouble(query.value(col++), 0.3);
	dp.density         = variantToDouble(query.value(col++), 7.85e-9);
	dp.torque_Nm       = variantToDouble(query.value(col++), 100.0);
	dp.enableContact   = query.value(col++).toInt() != 0;
	dp.contactType     = query.value(col++).toString();
	dp.contactStiffness = variantToDouble(query.value(col++), 500.0);
	dp.frictionCoeff   = variantToDouble(query.value(col++), 0.0);
	dp.solver          = query.value(col++).toString();
	dp.solverPath      = query.value(col++).toString();
	dp.solverVersion   = query.value(col++).toString();
	dp.staticStep      = query.value(col++).toString();
	dp.sigmaMax        = variantToDouble(query.value(col++), -1.0);
	dp.sigmaMax_gear1  = variantToDouble(query.value(col++), -1.0);
	dp.sigmaMax_gear2  = variantToDouble(query.value(col++), -1.0);
	dp.uMax            = variantToDouble(query.value(col++), -1.0);
	dp.uMax_gear1      = variantToDouble(query.value(col++), -1.0);
	dp.uMax_gear2      = variantToDouble(query.value(col++), -1.0);
	dp.mass            = variantToDouble(query.value(col++), -1.0);
	dp.mass_gear1      = variantToDouble(query.value(col++), -1.0);
	dp.mass_gear2      = variantToDouble(query.value(col++), -1.0);
	dp.cpressMax_MPa   = variantToDouble(query.value(col++), -1.0);
	dp.cpressMean_MPa  = variantToDouble(query.value(col++), -1.0);
	dp.cpressStd_MPa   = variantToDouble(query.value(col++), -1.0);
	dp.cpressCV        = variantToDouble(query.value(col++), -1.0);
	dp.contactWidth_mm = variantToDouble(query.value(col++), -1.0);
	dp.edgeLoadRatio   = variantToDouble(query.value(col++), -1.0);
	dp.cpressEdgeMean_MPa = variantToDouble(query.value(col++), -1.0);
	dp.cpressCenterMean_MPa = variantToDouble(query.value(col++), -1.0);
	dp.cpressActiveNodes = query.value(col++).toInt();
	dp.cpressBinCount  = query.value(col++).toInt();
	dp.status          = pointStatusFromString(query.value(col++).toString());
	dp.errorMsg        = query.value(col++).toString();
	dp.runDir          = query.value(col++).toString();
	dp.meshInpPath     = query.value(col++).toString();
	dp.jobInpPath      = query.value(col++).toString();
	dp.datPath         = query.value(col++).toString();
	dp.frdPath         = query.value(col++).toString();
	dp.isPareto        = query.value(col++).toInt();
	dp.rank            = query.value(col++).toInt();
	dp.crowdingDistance = variantToDouble(query.value(col++), 0.0);
	dp.syncLegacyResultFields();
	dp.updatedAt       = QDateTime::currentDateTime();
	return true;
}

bool GearOptResultDatabase::insertDesignPointResult(const QString& runId,
                                                    const GearDesignPoint& dp,
                                                    int generation,
                                                    int individualId,
                                                    int rank,
                                                    double crowdingDistance,
                                                    int isPareto,
                                                    const QString& caseHashValue)
{
	if (!isOpen()) {
		qWarning().noquote() << QStringLiteral("[GearOpt][DB] insertDesignPointResult: database not open");
		return false;
	}

	GearDesignPoint store = dp;
	store.syncLegacyResultFields();
	const QString geomHash = paramHash(store);
	const QString caseH    = caseHashValue.isEmpty() ? caseHash(store) : caseHashValue;

	const QString createdAt = store.createdAt.isValid()
	                              ? store.createdAt.toString(Qt::ISODate)
	                              : QDateTime::currentDateTime().toString(Qt::ISODate);

	// 列顺序与 bind 顺序必须一致；占位符由列数自动生成，避免手写 ? 数量错误。
	// MSVC：不用 brace / QStringList(int,size) 初始化，改用 operator<<。
	QStringList cols;
	cols.reserve(63);
	cols << QStringLiteral("run_id") << QStringLiteral("gen") << QStringLiteral("individual_id")
	     << QStringLiteral("created_at") << QStringLiteral("z1") << QStringLiteral("z2")
	     << QStringLiteral("module") << QStringLiteral("alpha") << QStringLiteral("x1")
	     << QStringLiteral("x2") << QStringLiteral("ca1") << QStringLiteral("lca1")
	     << QStringLiteral("ca2") << QStringLiteral("lca2") << QStringLiteral("width")
	     << QStringLiteral("hubRatio") << QStringLiteral("addendumCoeff")
	     << QStringLiteral("dedendumCoeff") << QStringLiteral("rootFilletCoeff")
	     << QStringLiteral("meshSize_mm") << QStringLiteral("meshAuto") << QStringLiteral("meshMethod")
	     << QStringLiteral("elementOrder") << QStringLiteral("nodeCount") << QStringLiteral("elementCount")
	     << QStringLiteral("materialName") << QStringLiteral("youngModulus_MPa")
	     << QStringLiteral("poissonRatio") << QStringLiteral("density") << QStringLiteral("torque_Nm")
	     << QStringLiteral("enableContact") << QStringLiteral("contactType")
	     << QStringLiteral("contactStiffness") << QStringLiteral("frictionCoeff")
	     << QStringLiteral("solverName") << QStringLiteral("solverPath") << QStringLiteral("solverVersion")
	     << QStringLiteral("staticStep") << QStringLiteral("sigmaMax_MPa")
	     << QStringLiteral("sigmaMax_gear1_MPa") << QStringLiteral("sigmaMax_gear2_MPa")
	     << QStringLiteral("uMax_mm") << QStringLiteral("uMax_gear1_mm") << QStringLiteral("uMax_gear2_mm")
	     << QStringLiteral("mass_kg") << QStringLiteral("mass_gear1_kg") << QStringLiteral("mass_gear2_kg")
	     << QStringLiteral("cpressMax_MPa")
	     << QStringLiteral("cpressMean_MPa") << QStringLiteral("cpressStd_MPa")
	     << QStringLiteral("cpressCV") << QStringLiteral("contactWidth_mm")
	     << QStringLiteral("edgeLoadRatio") << QStringLiteral("cpressEdgeMean_MPa")
	     << QStringLiteral("cpressCenterMean_MPa") << QStringLiteral("cpressActiveNodes")
	     << QStringLiteral("cpressBinCount")
	     << QStringLiteral("status") << QStringLiteral("errorMsg") << QStringLiteral("runDir")
	     << QStringLiteral("meshInpPath") << QStringLiteral("jobInpPath") << QStringLiteral("datPath")
	     << QStringLiteral("frdPath") << QStringLiteral("isPareto") << QStringLiteral("rank")
	     << QStringLiteral("crowdingDistance") << QStringLiteral("param_hash")
	     << QStringLiteral("case_hash")
	     << QStringLiteral("sample_source") << QStringLiteral("surrogate_used")
	     << QStringLiteral("verified_by_ccx") << QStringLiteral("base_case_hash")
	     << QStringLiteral("design_hash") << QStringLiteral("converged")
	     << QStringLiteral("solve_time_s") << QStringLiteral("retry_count")
	     << QStringLiteral("last_error_message") << QStringLiteral("last_retry_time")
	     << QStringLiteral("is_valid");

	const QString tableName = QString::fromLatin1(kMainTable);
	QStringList placeholderList;
	placeholderList.reserve(cols.size());
	for (int k = 0; k < cols.size(); ++k)
		placeholderList.append(QStringLiteral("?"));
	const QString placeholders = placeholderList.join(QLatin1Char(','));
	const QString sql = QStringLiteral("INSERT INTO %1 (%2) VALUES (%3)")
	                        .arg(tableName, cols.join(QLatin1Char(',')), placeholders);

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	if (!query.prepare(sql)) {
		logSqlQuery(query, "insertDesignPointResult.prepare");
		qWarning().noquote()
		    << QStringLiteral("[GearOpt][DB] insertDesignPointResult: prepare failed table=")
		    << tableName << QStringLiteral("sql=") << sql;
		return false;
	}

	const int convergedFlag = computeConvergedFromMetrics(store) ? 1 : 0;
	const int isValidFlag   = computeIsValidFlag(store);

	int bindCount = 0;
	const auto bind = [&](const QVariant& v) { query.bindValue(bindCount++, v); };

	bind(runId);
	bind(generation);
	bind(individualId);
	bind(createdAt);
	bind(store.z1);
	bind(store.z2);
	bind(store.module);
	bind(store.alpha);
	bind(store.x1);
	bind(store.x2);
	bind(store.ca1);
	bind(store.lca1);
	bind(store.ca2);
	bind(store.lca2);
	bind(store.commonWidth);
	bind(store.hubRatio);
	bind(store.addendumCoeff);
	bind(store.dedendumCoeff);
	bind(store.rootFilletCoeff);
	bind(store.meshSize_mm);
	bind(store.meshAuto ? 1 : 0);
	bind(store.meshMethod);
	bind(store.elementOrder);
	bind(store.nodeCount);
	bind(store.elementCount);
	bind(store.materialName);
	bind(store.youngModulus_MPa);
	bind(store.poissonRatio);
	bind(store.density);
	bind(store.torque_Nm);
	bind(store.enableContact ? 1 : 0);
	bind(store.contactType);
	bind(store.contactStiffness);
	bind(store.frictionCoeff);
	bind(store.solver);
	bind(store.solverPath);
	bind(store.solverVersion);
	bind(store.staticStep);
	bind(store.sigmaMax);
	bind(store.sigmaMax_gear1);
	bind(store.sigmaMax_gear2);
	bind(store.uMax);
	bind(store.uMax_gear1);
	bind(store.uMax_gear2);
	bind(store.mass);
	bind(store.mass_gear1);
	bind(store.mass_gear2);
	bind(store.cpressMax_MPa);
	bind(store.cpressMean_MPa);
	bind(store.cpressStd_MPa);
	bind(store.cpressCV);
	bind(store.contactWidth_mm);
	bind(store.edgeLoadRatio);
	bind(store.cpressEdgeMean_MPa);
	bind(store.cpressCenterMean_MPa);
	bind(store.cpressActiveNodes);
	bind(store.cpressBinCount);
	bind(pointStatusToString(store.status));
	bind(store.errorMsg);
	bind(store.runDir);
	bind(store.meshInpPath);
	bind(store.jobInpPath);
	bind(store.datPath);
	bind(store.frdPath);
	bind(isPareto);
	bind(rank);
	bind(crowdingDistance);
	bind(geomHash);
	bind(caseH);
	bind(QStringLiteral("ccx"));
	bind(0);
	bind(1);
	bind(baseCaseHash(store));
	bind(designHash(store));
	bind(convergedFlag);
	bind(store.solverTime);
	bind(0);
	bind(QString());
	bind(QString());
	bind(isValidFlag);

	if (bindCount != cols.size()) {
		qWarning().noquote()
		    << QStringLiteral("[GearOpt][DB] insertDesignPointResult: bind count mismatch")
		    << QStringLiteral("bindCount=") << bindCount << QStringLiteral("cols=") << cols.size()
		    << QStringLiteral("table=") << tableName;
		return false;
	}

	if (!query.exec()) {
		logSqlQuery(query, "insertDesignPointResult.exec");
		qWarning().noquote()
		    << QStringLiteral("[GearOpt][DB] insert failed table=") << tableName
		    << QStringLiteral("gen=") << generation << QStringLiteral("individual_id=")
		    << individualId << QStringLiteral("run_id=") << runId
		    << QStringLiteral("status=") << pointStatusToString(store.status);
		return false;
	}

	const int rowId = query.lastInsertId().toInt();
	qDebug().noquote() << QStringLiteral("[GearOpt][DB] insert ok table=%1 gen=%2 individual_id=%3 "
	                                     "status=%4 row_id=%5 run_id=%6 converged=%7 is_valid=%8")
	                              .arg(tableName)
	                              .arg(generation)
	                              .arg(individualId)
	                              .arg(pointStatusToString(store.status))
	                              .arg(rowId)
	                              .arg(runId)
	                              .arg(convergedFlag)
	                              .arg(isValidFlag);
	return true;
}

bool GearOptResultDatabase::insertSimulationResult(const GearDesignPoint& dp,
                                                   int generation,
                                                   int pointId)
{
	const QString runId =
	    dp.runId.isEmpty() ? generateRunId(0) : dp.runId;
	return insertDesignPointResult(runId, dp, generation, pointId,
	                               dp.rank, dp.crowdingDistance, dp.isPareto);
}

int GearOptResultDatabase::countValidatedSamplesForBaseCase(const QString& baseCaseH,
                                                           double fixedWidthMm) const
{
	if (!isOpen() || baseCaseH.isEmpty())
		return 0;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	QString sql = QStringLiteral(
	    "SELECT COUNT(*) FROM gear_opt_results"
	    " WHERE base_case_hash = ?"
	    " AND") + validatedSurrogateSampleWhereSql();
	if (fixedWidthMm >= 0.0)
		sql += QStringLiteral(" AND ABS(width - ?) < 1e-6");
	query.prepare(sql);
	query.addBindValue(baseCaseH);
	if (fixedWidthMm >= 0.0)
		query.addBindValue(fixedWidthMm);
	if (!query.exec() || !query.next()) {
		logSqlQuery(query, "countValidatedSamplesForBaseCase");
		return 0;
	}
	return query.value(0).toInt();
}

bool GearOptResultDatabase::designHashExists(const QString& baseCaseH,
                                           const QString& designH) const
{
	if (!isOpen() || baseCaseH.isEmpty() || designH.isEmpty())
		return false;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	query.prepare(QStringLiteral(
	    "SELECT 1 FROM gear_opt_results"
	    " WHERE base_case_hash = ? AND design_hash = ?"
	    " AND status = 'done' AND verified_by_ccx = 1"
	    " AND converged = 1 AND is_valid = 1"
	    " LIMIT 1"));
	query.addBindValue(baseCaseH);
	query.addBindValue(designH);
	if (!query.exec())
		return false;
	return query.next();
}

QSet<QString> GearOptResultDatabase::knownCaseHashesForBaseCase(const QString& baseCaseH,
                                                                double fixedWidthMm) const
{
	QSet<QString> out;
	if (!isOpen() || baseCaseH.isEmpty())
		return out;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	QString sql = QStringLiteral(
	    "SELECT DISTINCT case_hash FROM gear_opt_results"
	    " WHERE base_case_hash = ?"
	    " AND status = 'done'"
	    " AND case_hash IS NOT NULL AND TRIM(case_hash) != ''");
	if (fixedWidthMm >= 0.0)
		sql += QStringLiteral(" AND ABS(width - ?) < 1e-6");
	query.prepare(sql);
	query.addBindValue(baseCaseH);
	if (fixedWidthMm >= 0.0)
		query.addBindValue(fixedWidthMm);
	if (!query.exec()) {
		logSqlQuery(query, "knownCaseHashesForBaseCase");
		return out;
	}
	while (query.next()) {
		const QString h = query.value(0).toString().trimmed();
		if (!h.isEmpty())
			out.insert(h);
	}
	return out;
}

QSet<QString> GearOptResultDatabase::knownDesignHashesForBaseCase(const QString& baseCaseH,
                                                                  double fixedWidthMm) const
{
	QSet<QString> out;
	if (!isOpen() || baseCaseH.isEmpty())
		return out;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	QString sql = QStringLiteral(
	    "SELECT DISTINCT design_hash FROM gear_opt_results"
	    " WHERE base_case_hash = ?"
	    " AND status = 'done'"
	    " AND design_hash IS NOT NULL AND TRIM(design_hash) != ''");
	if (fixedWidthMm >= 0.0)
		sql += QStringLiteral(" AND ABS(width - ?) < 1e-6");
	query.prepare(sql);
	query.addBindValue(baseCaseH);
	if (fixedWidthMm >= 0.0)
		query.addBindValue(fixedWidthMm);
	if (!query.exec()) {
		logSqlQuery(query, "knownDesignHashesForBaseCase");
		return out;
	}
	while (query.next()) {
		const QString h = query.value(0).toString().trimmed();
		if (!h.isEmpty())
			out.insert(h);
	}
	return out;
}

QVector<SurrogateSample> GearOptResultDatabase::loadValidatedSamples(const QString& baseCaseH,
                                                                     double fixedWidthMm,
                                                                     int maxCount,
                                                                     SurrogateSampleLoadStats* stats) const
{
	QVector<SurrogateSample> out;
	if (!isOpen() || baseCaseH.isEmpty())
		return out;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);

	if (stats) {
		stats->totalSamples = countSamplesForBaseCase(query, baseCaseH, fixedWidthMm);
		stats->validSamples = 0;
		stats->failedSkipped = 0;
	}

	QString sql = QStringLiteral(
	    "SELECT x1, x2, ca1, lca1, ca2, lca2, width, hubRatio, design_hash, case_hash,"
	    " cpressMax_MPa, edgeLoadRatio, cpressCV, sigmaMax_MPa, uMax_mm, mass_kg"
	    " FROM gear_opt_results"
	    " WHERE base_case_hash = ?"
	    " AND") + validatedSurrogateSampleWhereSql();
	if (fixedWidthMm >= 0.0)
		sql += QStringLiteral(" AND ABS(width - ?) < 1e-6");
	sql += QStringLiteral(" ORDER BY id ASC");
	if (maxCount > 0)
		sql += QStringLiteral(" LIMIT ?");
	query.prepare(sql);
	query.addBindValue(baseCaseH);
	if (fixedWidthMm >= 0.0)
		query.addBindValue(fixedWidthMm);
	if (maxCount > 0)
		query.addBindValue(maxCount);
	if (!query.exec()) {
		logSqlQuery(query, "loadValidatedSamples");
		return out;
	}

	while (query.next()) {
		int col = 0;
		SurrogateSample s;
		const double x1  = query.value(col++).toDouble();
		const double x2  = query.value(col++).toDouble();
		const double ca1 = query.value(col++).toDouble();
		const double lca1 = query.value(col++).toDouble();
		const double ca2 = query.value(col++).toDouble();
		const double lca2 = query.value(col++).toDouble();
		Q_UNUSED(query.value(col++).toDouble()); // width：baseCase 固定，不入 RBF 输入
		const double hubRatio = query.value(col++).toDouble();
		GearDesignPoint reliefCheck;
		reliefCheck.ca1  = ca1;
		reliefCheck.lca1 = lca1;
		reliefCheck.ca2  = ca2;
		reliefCheck.lca2 = lca2;
		if (!validateReliefDesign(reliefCheck))
			continue;
		s.x = { x1, x2, reliefCheck.ca1, reliefCheck.lca1, reliefCheck.ca2, reliefCheck.lca2, hubRatio };
		s.designHash = query.value(col++).toString();
		s.caseHash   = query.value(col++).toString().trimmed();
		s.cpressMax  = query.value(col++).toDouble();
		s.edgeLoadRatio = variantToDouble(query.value(col++), -1.0);
		s.cpressCV   = variantToDouble(query.value(col++), -1.0);
		s.sigmaMax   = query.value(col++).toDouble();
		s.uMax       = query.value(col++).toDouble();
		s.mass       = query.value(col++).toDouble();
		const bool metricsOk = s.x.size() == kSurrogateInputDim
		                       && s.cpressMax > 0.0 && std::isfinite(s.cpressMax)
		                       && s.sigmaMax > 0.0 && std::isfinite(s.sigmaMax)
		                       && s.uMax >= 0.0 && std::isfinite(s.uMax)
		                       && s.mass > 0.0 && std::isfinite(s.mass)
		                       && s.edgeLoadRatio > 0.0 && std::isfinite(s.edgeLoadRatio)
		                       && s.cpressCV > 0.0 && std::isfinite(s.cpressCV);
		if (metricsOk)
			out.append(s);
		else if (s.cpressMax <= 0.0 || !std::isfinite(s.cpressMax))
			qDebug().noquote() << QStringLiteral("[GearOpt][DB] Skip sample because cpressMax_MPa is missing.");
		else if (s.sigmaMax <= 0.0 || !std::isfinite(s.sigmaMax))
			qDebug().noquote() << QStringLiteral("[GearOpt][DB] Skip sample because sigmaMax_MPa is missing.");
		else if (s.uMax < 0.0 || !std::isfinite(s.uMax))
			qDebug().noquote() << QStringLiteral("[GearOpt][DB] Skip sample because uMax_mm is missing.");
		else if (s.mass <= 0.0 || !std::isfinite(s.mass))
			qDebug().noquote() << QStringLiteral("[GearOpt][DB] Skip sample because mass_kg is missing.");
		else if (s.edgeLoadRatio <= 0.0)
			qDebug().noquote() << QStringLiteral("[GearOpt][DB] Skip sample because edgeLoadRatio is missing.");
		else if (s.cpressCV <= 0.0)
			qDebug().noquote() << QStringLiteral("[GearOpt][DB] Skip sample because cpressCV is missing.");
	}

	if (stats) {
		stats->validSamples  = out.size();
		stats->failedSkipped = std::max(0, stats->totalSamples - stats->validSamples);
		qDebug().noquote() << QStringLiteral("[GearOpt][DB] loadValidatedSamples path=%1 total=%2 valid=%3 failed_skipped=%4")
		                      .arg(_dbPath)
		                      .arg(stats->totalSamples)
		                      .arg(stats->validSamples)
		                      .arg(stats->failedSkipped);
	}
	return out;
}

QSet<QString> GearOptResultDatabase::failedCaseHashesForBaseCase(const QString& baseCaseH,
                                                                 double fixedWidthMm) const
{
	QSet<QString> out;
	if (!isOpen() || baseCaseH.isEmpty())
		return out;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	QString sql = QStringLiteral(
	    "SELECT DISTINCT case_hash FROM gear_opt_results"
	    " WHERE base_case_hash = ?"
	    " AND case_hash IS NOT NULL AND TRIM(case_hash) != ''"
	    " AND (status = 'failed' OR status = 'invalid' OR cpressMax_MPa < 0 OR is_valid = 0)");
	if (fixedWidthMm >= 0.0)
		sql += QStringLiteral(" AND ABS(width - ?) < 1e-6");
	query.prepare(sql);
	query.addBindValue(baseCaseH);
	if (fixedWidthMm >= 0.0)
		query.addBindValue(fixedWidthMm);
	if (!query.exec()) {
		logSqlQuery(query, "failedCaseHashesForBaseCase");
		return out;
	}
	while (query.next()) {
		const QString h = query.value(0).toString().trimmed();
		if (!h.isEmpty())
			out.insert(h);
	}
	return out;
}

QVector<int> GearOptResultDatabase::findRowIdsByCaseHash(const QString& caseHash) const
{
	QVector<int> out;
	if (!isOpen() || caseHash.isEmpty())
		return out;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	query.prepare(QStringLiteral(
	    "SELECT id FROM gear_opt_results"
	    " WHERE case_hash = ? AND case_hash IS NOT NULL AND TRIM(case_hash) != ''"
	    " ORDER BY id ASC"));
	query.addBindValue(caseHash);
	if (!query.exec()) {
		logSqlQuery(query, "findRowIdsByCaseHash");
		return out;
	}
	while (query.next())
		out.append(query.value(0).toInt());
	return out;
}

QVector<FailedCaseRecord> GearOptResultDatabase::loadFailedCasesForRetry(const QString& baseCaseH,
                                                                         double fixedWidthMm) const
{
	QVector<FailedCaseRecord> out;
	if (!isOpen())
		return out;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	QString sql = QStringLiteral(
	    "SELECT id, case_hash, run_id, gen, individual_id, created_at,"
	    " z1, z2, module, alpha, x1, x2, ca1, lca1, ca2, lca2, width, hubRatio,"
	    " addendumCoeff, dedendumCoeff, rootFilletCoeff,"
	    " meshSize_mm, meshAuto, meshMethod, elementOrder,"
	    " materialName, youngModulus_MPa, poissonRatio, density,"
	    " torque_Nm, enableContact, contactType, contactStiffness, frictionCoeff,"
	    " solverName, solverPath, staticStep, runDir, retry_count, errorMsg"
	    " FROM gear_opt_results"
	    " WHERE (status = 'failed' OR cpressMax_MPa < 0 OR sigmaMax_MPa < 0)"
	    " AND status != 'invalid'"
	    " AND status != 'infeasible'"
	    " AND case_hash IS NOT NULL AND TRIM(case_hash) != ''");
	if (!baseCaseH.isEmpty())
		sql += QStringLiteral(" AND base_case_hash = ?");
	if (fixedWidthMm >= 0.0)
		sql += QStringLiteral(" AND ABS(width - ?) < 1e-6");
	sql += QStringLiteral(" ORDER BY id ASC");

	query.prepare(sql);
	if (!baseCaseH.isEmpty())
		query.addBindValue(baseCaseH);
	if (fixedWidthMm >= 0.0)
		query.addBindValue(fixedWidthMm);
	if (!query.exec()) {
		logSqlQuery(query, "loadFailedCasesForRetry");
		return out;
	}

	QHash<QString, int> indexByHash;
	while (query.next()) {
		int col = 0;
		const int rowId = query.value(col++).toInt();
		const QString caseH = query.value(col++).toString().trimmed();
		if (caseH.isEmpty())
			continue;

		const QString runId = query.value(col++).toString();
		const int gen = query.value(col++).toInt();
		const int individualId = query.value(col++).toInt();
		const QString created = query.value(col++).toString();
		const int z1 = query.value(col++).toInt();
		const int z2 = query.value(col++).toInt();
		const double module = query.value(col++).toDouble();
		const double alpha = query.value(col++).toDouble();
		const double x1 = query.value(col++).toDouble();
		const double x2 = query.value(col++).toDouble();
		const double ca1 = query.value(col++).toDouble();
		const double lca1 = query.value(col++).toDouble();
		const double ca2 = query.value(col++).toDouble();
		const double lca2 = query.value(col++).toDouble();
		const double width = query.value(col++).toDouble();
		const double hubRatio = query.value(col++).toDouble();
		const double addendumCoeff = query.value(col++).toDouble();
		const double dedendumCoeff = query.value(col++).toDouble();
		const double rootFilletCoeff = query.value(col++).toDouble();
		const double meshSize_mm = query.value(col++).toDouble();
		const bool meshAuto = query.value(col++).toInt() != 0;
		const QString meshMethod = query.value(col++).toString();
		const int elementOrder = query.value(col++).toInt();
		const QString materialName = query.value(col++).toString();
		const double youngModulus = query.value(col++).toDouble();
		const double poissonRatio = query.value(col++).toDouble();
		const double density = query.value(col++).toDouble();
		const double torque_Nm = query.value(col++).toDouble();
		const bool enableContact = query.value(col++).toInt() != 0;
		const QString contactType = query.value(col++).toString();
		const double contactStiffness = query.value(col++).toDouble();
		const double frictionCoeff = query.value(col++).toDouble();
		const QString solverName = query.value(col++).toString();
		const QString solverPath = query.value(col++).toString();
		const QString staticStep = query.value(col++).toString();
		const QString runDir = query.value(col++).toString();
		const int retryCount = query.value(col++).toInt();
		const QString errMsg = query.value(col++).toString();

		FailedCaseDbRow row;
		row.rowId        = rowId;
		row.databasePath = _dbPath;
		row.retryCount   = retryCount;

		if (!indexByHash.contains(caseH)) {
			FailedCaseRecord fresh;
			fresh.caseHash = caseH;
			GearDesignPoint& dp = fresh.dp;
			dp.runId       = runId;
			dp.generation  = gen;
			dp.id          = individualId;
			dp.createdAt   = QDateTime::fromString(created, Qt::ISODate);
			dp.z1          = z1;
			dp.z2          = z2;
			dp.module      = module;
			dp.alpha       = alpha;
			dp.x1          = x1;
			dp.x2          = x2;
			dp.ca1         = ca1;
			dp.lca1        = lca1;
			dp.ca2         = ca2;
			dp.lca2        = lca2;
			dp.commonWidth = width;
			dp.hubRatio    = hubRatio;
			dp.addendumCoeff   = addendumCoeff;
			dp.dedendumCoeff   = dedendumCoeff;
			dp.rootFilletCoeff = rootFilletCoeff;
			dp.meshSize_mm     = meshSize_mm;
			dp.meshAuto        = meshAuto;
			dp.meshMethod      = meshMethod;
			dp.elementOrder    = elementOrder;
			dp.materialName    = materialName;
			dp.youngModulus_MPa = youngModulus;
			dp.poissonRatio    = poissonRatio;
			dp.density         = density;
			dp.torque_Nm       = torque_Nm;
			dp.enableContact   = enableContact;
			dp.contactType     = contactType;
			dp.contactStiffness = contactStiffness;
			dp.frictionCoeff   = frictionCoeff;
			dp.solver          = solverName;
			dp.solverPath      = solverPath;
			dp.staticStep      = staticStep;
			dp.runDir          = runDir;
			dp.errorMsg        = errMsg;
			dp.status          = PointStatus::Failed;
			out.append(fresh);
			indexByHash.insert(caseH, out.size() - 1);
		}

		out[indexByHash.value(caseH)].dbRows.append(row);
	}
	return out;
}

bool GearOptResultDatabase::updateDesignPointResultByRowId(int rowId, const GearDesignPoint& dp)
{
	if (!isOpen() || rowId <= 0)
		return false;

	GearDesignPoint store = dp;
	store.syncLegacyResultFields();
	const int convergedFlag = computeConvergedFromMetrics(store) ? 1 : 0;
	const int isValidFlag   = computeIsValidFlag(store);

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	query.prepare(QStringLiteral(
	    "UPDATE gear_opt_results SET"
	    " sigmaMax_MPa=?, sigmaMax_gear1_MPa=?, sigmaMax_gear2_MPa=?,"
	    " uMax_mm=?, uMax_gear1_mm=?, uMax_gear2_mm=?,"
	    " mass_kg=?, mass_gear1_kg=?, mass_gear2_kg=?,"
	    " cpressMax_MPa=?, cpressMean_MPa=?, cpressStd_MPa=?, cpressCV=?,"
	    " contactWidth_mm=?, edgeLoadRatio=?, cpressEdgeMean_MPa=?, cpressCenterMean_MPa=?,"
	    " cpressActiveNodes=?, cpressBinCount=?,"
	    " nodeCount=?, elementCount=?,"
	    " status=?, errorMsg=?, runDir=?, meshInpPath=?, jobInpPath=?, datPath=?, frdPath=?,"
	    " converged=?, solve_time_s=?, is_valid=?, last_error_message=?, last_retry_time=?"
	    " WHERE id=?"));

	int b = 0;
	query.bindValue(b++, store.sigmaMax);
	query.bindValue(b++, store.sigmaMax_gear1);
	query.bindValue(b++, store.sigmaMax_gear2);
	query.bindValue(b++, store.uMax);
	query.bindValue(b++, store.uMax_gear1);
	query.bindValue(b++, store.uMax_gear2);
	query.bindValue(b++, store.mass);
	query.bindValue(b++, store.mass_gear1);
	query.bindValue(b++, store.mass_gear2);
	query.bindValue(b++, store.cpressMax_MPa);
	query.bindValue(b++, store.cpressMean_MPa);
	query.bindValue(b++, store.cpressStd_MPa);
	query.bindValue(b++, store.cpressCV);
	query.bindValue(b++, store.contactWidth_mm);
	query.bindValue(b++, store.edgeLoadRatio);
	query.bindValue(b++, store.cpressEdgeMean_MPa);
	query.bindValue(b++, store.cpressCenterMean_MPa);
	query.bindValue(b++, store.cpressActiveNodes);
	query.bindValue(b++, store.cpressBinCount);
	query.bindValue(b++, store.nodeCount);
	query.bindValue(b++, store.elementCount);
	query.bindValue(b++, pointStatusToString(store.status));
	query.bindValue(b++, store.errorMsg);
	query.bindValue(b++, store.runDir);
	query.bindValue(b++, store.meshInpPath);
	query.bindValue(b++, store.jobInpPath);
	query.bindValue(b++, store.datPath);
	query.bindValue(b++, store.frdPath);
	query.bindValue(b++, convergedFlag);
	query.bindValue(b++, store.solverTime);
	query.bindValue(b++, isValidFlag);
	query.bindValue(b++, QString());
	query.bindValue(b++, QDateTime::currentDateTime().toString(Qt::ISODate));
	query.bindValue(b++, rowId);

	if (!query.exec()) {
		logSqlQuery(query, "updateDesignPointResultByRowId");
		return false;
	}
	qDebug().noquote() << QStringLiteral("[GearOpt][DB] update ok row_id=%1 case_hash=%2 status=%3 converged=%4 is_valid=%5")
	                      .arg(rowId)
	                      .arg(caseHash(store))
	                      .arg(pointStatusToString(store.status))
	                      .arg(convergedFlag)
	                      .arg(isValidFlag);
	return true;
}

bool GearOptResultDatabase::recordRetryFailureByRowId(int rowId, const QString& errorMsg)
{
	if (!isOpen() || rowId <= 0)
		return false;

	const QString now = QDateTime::currentDateTime().toString(Qt::ISODate);
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	query.prepare(QStringLiteral(
	    "UPDATE gear_opt_results SET"
	    " status='failed', converged=0, is_valid=0,"
	    " retry_count=COALESCE(retry_count, 0) + 1,"
	    " last_error_message=?, last_retry_time=?, errorMsg=?"
	    " WHERE id=?"));
	query.addBindValue(errorMsg.left(2000));
	query.addBindValue(now);
	query.addBindValue(errorMsg.left(2000));
	query.addBindValue(rowId);
	if (!query.exec()) {
		logSqlQuery(query, "recordRetryFailureByRowId");
		return false;
	}
	qDebug().noquote() << QStringLiteral("[GearOpt][DB] retry failed row_id=%1 error=%2")
	                      .arg(rowId)
	                      .arg(errorMsg.left(120));
	return true;
}

bool GearOptResultDatabase::markDesignInvalidByRowId(int rowId, const QString& lastErrorMessage)
{
	if (!isOpen() || rowId <= 0)
		return false;

	const QString now   = QDateTime::currentDateTime().toString(Qt::ISODate);
	const QString msg   = lastErrorMessage.left(2000);
	QSqlDatabase db     = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	query.prepare(QStringLiteral(
	    "UPDATE gear_opt_results SET"
	    " status='invalid', converged=0, is_valid=0,"
	    " errorMsg=?, last_error_message=?, last_retry_time=?"
	    " WHERE id=?"));
	query.addBindValue(msg);
	query.addBindValue(msg);
	query.addBindValue(now);
	query.addBindValue(rowId);
	if (!query.exec()) {
		logSqlQuery(query, "markDesignInvalidByRowId");
		return false;
	}
	qDebug().noquote() << QStringLiteral("[GearOpt][DB] mark invalid row_id=%1 error=%2")
	                      .arg(rowId)
	                      .arg(msg.left(120));
	return true;
}

int GearOptResultDatabase::countValidResults() const
{
	if (!isOpen())
		return 0;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	if (!query.exec(QStringLiteral(
	        "SELECT COUNT(*) FROM gear_opt_results"
	        " WHERE status = 'done' AND is_valid = 1 AND converged = 1"
	        " AND cpressMax_MPa > 0 AND sigmaMax_MPa > 0"))) {
		logSqlQuery(query, "countValidResults");
		qWarning().noquote() << QStringLiteral("[GearOpt][DB] countValidResults failed");
		return 0;
	}
	if (!query.next())
		return 0;
	return query.value(0).toInt();
}

int GearOptResultDatabase::backfillMissingCpressDistributionMetrics(int binCount, int maxRows) const
{
	return backfillMissingCpressDistributionMetricsDetailed(binCount, maxRows).updatedCount;
}

CpressDistributionBackfillStats GearOptResultDatabase::backfillMissingCpressDistributionMetricsDetailed(
    int binCount,
    int maxRows) const
{
	CpressDistributionBackfillStats stats;
	if (!isOpen())
		return stats;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	const QString missingWhere = QStringLiteral(
	    " status = 'done'"
	    " AND cpressMax_MPa > 0"
	    " AND (edgeLoadRatio IS NULL OR edgeLoadRatio <= 0"
	    "      OR cpressCV IS NULL OR cpressCV <= 0)");

	QSqlQuery countQuery(db);
	if (!countQuery.exec(QStringLiteral("SELECT COUNT(*) FROM gear_opt_results WHERE %1")
	                         .arg(missingWhere))) {
		logSqlQuery(countQuery, "backfillMissingCpressDistributionMetrics.count");
		return stats;
	}
	if (countQuery.next())
		stats.totalMissing = countQuery.value(0).toInt();

	QSqlQuery select(db);
	QString sql = QStringLiteral(
	    "SELECT id, frdPath FROM gear_opt_results"
	    " WHERE %1"
	    " ORDER BY id ASC");
	sql = sql.arg(missingWhere);
	if (maxRows > 0)
		sql += QStringLiteral(" LIMIT ?");
	select.prepare(sql);
	if (maxRows > 0)
		select.addBindValue(maxRows);
	if (!select.exec()) {
		logSqlQuery(select, "backfillMissingCpressDistributionMetrics.select");
		return stats;
	}

	while (select.next()) {
		const int rowId = select.value(0).toInt();
		const QString frdPath = select.value(1).toString();
		if (frdPath.trimmed().isEmpty() || !QFileInfo::exists(frdPath)) {
			++stats.skipMissingFrd;
			qInfo().noquote() << QStringLiteral("[GearOpt][OfflineBackfill] skip id=%1 missing frdPath=%2")
			                     .arg(rowId)
			                     .arg(frdPath);
			continue;
		}
		const CpressDistributionMetrics m =
		    parseCpressDistributionMetrics(frdPath, binCount > 0 ? binCount : 11);
		if (!m.valid()) {
			++stats.failedParseCount;
			qWarning().noquote() << QStringLiteral("[GearOpt][OfflineBackfill] failed parse id=%1 frdPath=%2")
			                        .arg(rowId)
			                        .arg(frdPath);
			continue;
		}

		QSqlQuery update(db);
		update.prepare(QStringLiteral(
		    "UPDATE gear_opt_results SET"
		    " cpressMean_MPa = ?, cpressStd_MPa = ?, cpressCV = ?,"
		    " contactWidth_mm = ?, edgeLoadRatio = ?,"
		    " cpressEdgeMean_MPa = ?, cpressCenterMean_MPa = ?,"
		    " cpressActiveNodes = ?, cpressBinCount = ?"
		    " WHERE id = ?"));
		update.addBindValue(m.cpressMean_MPa);
		update.addBindValue(m.cpressStd_MPa);
		update.addBindValue(m.cpressCV);
		update.addBindValue(m.contactWidth_mm);
		update.addBindValue(m.edgeLoadRatio);
		update.addBindValue(m.cpressEdgeMean_MPa);
		update.addBindValue(m.cpressCenterMean_MPa);
		update.addBindValue(m.cpressActiveNodes);
		update.addBindValue(m.cpressBinCount);
		update.addBindValue(rowId);
		if (!update.exec()) {
			logSqlQuery(update, "backfillMissingCpressDistributionMetrics.update");
			++stats.failedParseCount;
			continue;
		}
		++stats.updatedCount;
	}

	qInfo().noquote() << QStringLiteral(
	                         "[GearOpt][OfflineBackfill] total_missing=%1 updated_count=%2 "
	                         "skip_missing_frd=%3 failed_parse_count=%4")
	                         .arg(stats.totalMissing)
	                         .arg(stats.updatedCount)
	                         .arg(stats.skipMissingFrd)
	                         .arg(stats.failedParseCount);
	return stats;
}

bool GearOptResultDatabase::updateParetoFlag(const QString& runId,
                                             int generation,
                                             int individualId,
                                             int isPareto,
                                             int rank,
                                             double crowdingDistance)
{
	if (!isOpen())
		return false;

	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	QSqlQuery      query(db);
	query.prepare(QStringLiteral(
	    "UPDATE gear_opt_results SET isPareto = ?, rank = ?, crowdingDistance = ?"
	    " WHERE run_id = ? AND gen = ? AND individual_id = ?"));
	query.addBindValue(isPareto);
	query.addBindValue(rank);
	query.addBindValue(crowdingDistance);
	query.addBindValue(runId);
	query.addBindValue(generation);
	query.addBindValue(individualId);
	if (!query.exec()) {
		logSqlQuery(query, "updateParetoFlag");
		return false;
	}
	return query.numRowsAffected() > 0;
}

QString GearOptResultDatabase::csvHeaderLine()
{
	return QStringLiteral(
	    "run_id,gen,individual_id,created_at,"
	    "z1,z2,module,alpha,x1,x2,ca1,lca1,ca2,lca2,width,hubRatio,"
	    "addendumCoeff,dedendumCoeff,rootFilletCoeff,"
	    "meshSize_mm,meshAuto,meshMethod,elementOrder,nodeCount,elementCount,"
	    "materialName,youngModulus_MPa,poissonRatio,density,"
	    "torque_Nm,enableContact,contactType,contactStiffness,frictionCoeff,"
	    "solverName,solverPath,solverVersion,staticStep,"
	    "sigmaMax_MPa,sigmaMax_gear1_MPa,sigmaMax_gear2_MPa,"
	    "uMax_mm,uMax_gear1_mm,uMax_gear2_mm,"
	    "mass_kg,mass_gear1_kg,mass_gear2_kg,cpressMax_MPa,"
	    "cpressMean_MPa,cpressStd_MPa,cpressCV,contactWidth_mm,edgeLoadRatio,"
	    "cpressEdgeMean_MPa,cpressCenterMean_MPa,cpressActiveNodes,cpressBinCount,"
	    "status,errorMsg,runDir,meshInpPath,jobInpPath,datPath,frdPath,"
	    "isPareto,rank,crowdingDistance\n");
}

static QString csvField(const QString& s)
{
	return QStringLiteral("\"") + QString(s).replace(QLatin1Char('"'), QLatin1Char('\'')) + QLatin1Char('"');
}

void GearOptResultDatabase::writeCsvRow(QTextStream& ts, const GearDesignPoint& dp)
{
	const GearDesignPoint& s = dp;
	const QString created = s.createdAt.isValid()
	                            ? s.createdAt.toString(Qt::ISODate)
	                            : QString();
	ts << csvField(s.runId) << ','
	   << s.generation << ','
	   << s.id << ','
	   << csvField(created) << ','
	   << s.z1 << ',' << s.z2 << ','
	   << s.module << ',' << s.alpha << ','
	   << s.x1 << ',' << s.x2 << ','
	   << s.ca1 << ',' << s.lca1 << ','
	   << s.ca2 << ',' << s.lca2 << ','
	   << s.commonWidth << ',' << s.hubRatio << ','
	   << s.addendumCoeff << ',' << s.dedendumCoeff << ',' << s.rootFilletCoeff << ','
	   << s.meshSize_mm << ','
	   << (s.meshAuto ? 1 : 0) << ','
	   << csvField(s.meshMethod) << ','
	   << s.elementOrder << ',' << s.nodeCount << ',' << s.elementCount << ','
	   << csvField(s.materialName) << ','
	   << s.youngModulus_MPa << ',' << s.poissonRatio << ',' << s.density << ','
	   << s.torque_Nm << ','
	   << (s.enableContact ? 1 : 0) << ','
	   << csvField(s.contactType) << ','
	   << s.contactStiffness << ',' << s.frictionCoeff << ','
	   << csvField(s.solver) << ','
	   << csvField(s.solverPath) << ','
	   << csvField(s.solverVersion) << ','
	   << csvField(s.staticStep) << ','
	   << s.sigmaMax << ',' << s.sigmaMax_gear1 << ',' << s.sigmaMax_gear2 << ','
	   << s.uMax << ',' << s.uMax_gear1 << ',' << s.uMax_gear2 << ','
	   << s.mass << ',' << s.mass_gear1 << ',' << s.mass_gear2 << ','
	   << s.cpressMax_MPa << ','
	   << s.cpressMean_MPa << ',' << s.cpressStd_MPa << ',' << s.cpressCV << ','
	   << s.contactWidth_mm << ',' << s.edgeLoadRatio << ','
	   << s.cpressEdgeMean_MPa << ',' << s.cpressCenterMean_MPa << ','
	   << s.cpressActiveNodes << ',' << s.cpressBinCount << ','
	   << pointStatusToString(s.status) << ','
	   << csvField(s.errorMsg) << ','
	   << csvField(s.runDir) << ','
	   << csvField(s.meshInpPath) << ','
	   << csvField(s.jobInpPath) << ','
	   << csvField(s.datPath) << ','
	   << csvField(s.frdPath) << ','
	   << s.isPareto << ',' << s.rank << ',' << s.crowdingDistance << '\n';
}

bool GearOptResultDatabase::appendCpressWidthBinsCsv(const QString& runDir,
                                                     const QString& runId,
                                                     int generation,
                                                     int individualId,
                                                     const QString& caseHash,
                                                     const CpressDistributionMetrics& metrics)
{
	const QString dir = runDir.trimmed();
	if (dir.isEmpty() || metrics.bins.isEmpty())
		return false;
	QDir().mkpath(dir);
	const QString path = QDir(dir).filePath(QStringLiteral("cpress_width_bins.csv"));
	const bool writeHeader = !QFileInfo::exists(path) || QFileInfo(path).size() == 0;
	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
		return false;
	QTextStream ts(&file);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
	ts.setEncoding(QStringConverter::Utf8);
#else
	ts.setCodec("UTF-8");
#endif
	if (writeHeader) {
		ts << "run_id,generation,individual_id,case_hash,binIndex,zStart,zEnd,nodeCount,meanCPRESS,maxCPRESS\n";
	}
	for (const CpressWidthBin& b : metrics.bins) {
		ts << csvField(runId) << ','
		   << generation << ','
		   << individualId << ','
		   << csvField(caseHash) << ','
		   << b.binIndex << ','
		   << b.zStart << ','
		   << b.zEnd << ','
		   << b.nodeCount << ','
		   << b.meanCPRESS << ','
		   << b.maxCPRESS << '\n';
	}
	ts.flush();
	file.flush();
	return true;
}

bool GearOptResultDatabase::ensureMetricDefinitionsTable() const
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;

	QSqlQuery query(db);
	const QString sql = QStringLiteral(
	    "CREATE TABLE IF NOT EXISTS %1 ("
	    "  metric_name TEXT PRIMARY KEY,"
	    "  display_name TEXT,"
	    "  unit TEXT,"
	    "  source TEXT,"
	    "  enabled INTEGER,"
	    "  implemented INTEGER,"
	    "  direction TEXT,"
	    "  note TEXT"
	    ")").arg(QString::fromLatin1(kMetricDefTable));

	if (!query.exec(sql)) {
		logSqlQuery(query, "ensureMetricDefinitionsTable");
		return false;
	}
	return true;
}

bool GearOptResultDatabase::migrateMetricDefinitionsColumns() const
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;

	QSqlQuery query(db);
	if (!ensureColumn(query, kMetricDefTable, "enabled INTEGER"))
		return false;

	bool hasDefaultEnabled = false;
	QSqlQuery pragma(db);
	if (pragma.exec(QStringLiteral("PRAGMA table_info(%1)")
	                    .arg(QString::fromLatin1(kMetricDefTable)))) {
		while (pragma.next()) {
			if (pragma.value(1).toString() == QStringLiteral("default_enabled"))
				hasDefaultEnabled = true;
		}
	}

	if (hasDefaultEnabled) {
		query.exec(QStringLiteral(
		    "UPDATE %1 SET enabled = default_enabled WHERE enabled IS NULL")
		             .arg(QString::fromLatin1(kMetricDefTable)));
	}
	return true;
}

bool GearOptResultDatabase::ensureObjectiveMappingTable() const
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;

	QSqlQuery query(db);
	const QString sql = QStringLiteral(
	    "CREATE TABLE IF NOT EXISTS %1 ("
	    "  obj_key TEXT PRIMARY KEY,"
	    "  metric_name TEXT,"
	    "  display_name TEXT,"
	    "  direction TEXT,"
	    "  obj_order INTEGER,"
	    "  enabled INTEGER,"
	    "  note TEXT"
	    ")").arg(QString::fromLatin1(kObjectiveMapTable));

	if (!query.exec(sql)) {
		logSqlQuery(query, "ensureObjectiveMappingTable");
		return false;
	}
	return true;
}

bool GearOptResultDatabase::ensureDesignVariableDefinitionsTable() const
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;

	QSqlQuery query(db);
	const QString sql = QStringLiteral(
	    "CREATE TABLE IF NOT EXISTS %1 ("
	    "  var_name TEXT PRIMARY KEY,"
	    "  display_name TEXT,"
	    "  unit TEXT,"
	    "  lower_bound REAL,"
	    "  upper_bound REAL,"
	    "  enabled INTEGER,"
	    "  part_type TEXT,"
	    "  note TEXT"
	    ")").arg(QString::fromLatin1(kDesignVarDefTable));

	if (!query.exec(sql)) {
		logSqlQuery(query, "ensureDesignVariableDefinitionsTable");
		return false;
	}
	return true;
}

bool GearOptResultDatabase::insertDefaultOptimizationConfigIfEmpty() const
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;

	auto seedIfEmpty = [&](const char* table, auto seedFn, auto insertFn) -> bool {
		QSqlQuery countQuery(db);
		if (!countQuery.exec(QStringLiteral("SELECT COUNT(*) FROM %1")
		                         .arg(QString::fromLatin1(table)))) {
			logSqlQuery(countQuery, "seedCount");
			return false;
		}
		if (!countQuery.next() || countQuery.value(0).toInt() > 0)
			return true;

		const auto items = seedFn();
		if (!db.transaction()) {
			logSqlQuery(countQuery, "seedBegin");
			return false;
		}
		QSqlQuery insQuery(db);
		for (const auto& item : items) {
			if (!insertFn(insQuery, item)) {
				db.rollback();
				return false;
			}
		}
		if (!db.commit()) {
			db.rollback();
			return false;
		}
		qDebug().noquote() << QStringLiteral("[GearOpt][DB] seeded %1 rows into %2")
		                      .arg(items.size())
		                      .arg(QString::fromLatin1(table));
		return true;
	};

	const bool ok1 = seedIfEmpty(
	    kMetricDefTable, defaultMetricDefinitions,
	    [](QSqlQuery& q, const MetricDefinition& d) {
		    q.prepare(QStringLiteral(
		        "INSERT INTO metric_definitions"
		        " (metric_name, display_name, unit, source, enabled, implemented, direction, note)"
		        " VALUES (?, ?, ?, ?, ?, ?, ?, ?)"));
		    q.addBindValue(d.metricName);
		    q.addBindValue(d.displayName);
		    q.addBindValue(d.unit);
		    q.addBindValue(d.source);
		    q.addBindValue(d.enabled ? 1 : 0);
		    q.addBindValue(d.implemented ? 1 : 0);
		    q.addBindValue(d.direction);
		    q.addBindValue(d.note);
		    if (!q.exec()) {
			    logSqlQuery(q, "seedMetricDefinitions");
			    return false;
		    }
		    return true;
	    });

	const bool ok2 = seedIfEmpty(
	    kObjectiveMapTable, defaultObjectiveMappings,
	    [](QSqlQuery& q, const ObjectiveMapping& m) {
		    q.prepare(QStringLiteral(
		        "INSERT INTO objective_mapping"
		        " (obj_key, metric_name, display_name, direction, obj_order, enabled, note)"
		        " VALUES (?, ?, ?, ?, ?, ?, ?)"));
		    q.addBindValue(m.objKey);
		    q.addBindValue(m.metricName);
		    q.addBindValue(m.displayName);
		    q.addBindValue(m.direction);
		    q.addBindValue(m.objOrder);
		    q.addBindValue(m.enabled ? 1 : 0);
		    q.addBindValue(m.note);
		    if (!q.exec()) {
			    logSqlQuery(q, "seedObjectiveMapping");
			    return false;
		    }
		    return true;
	    });

	const bool ok3 = seedIfEmpty(
	    kDesignVarDefTable, defaultDesignVariableDefinitions,
	    [](QSqlQuery& q, const DesignVariableDefinition& v) {
		    q.prepare(QStringLiteral(
		        "INSERT INTO design_variable_definitions"
		        " (var_name, display_name, unit, lower_bound, upper_bound, enabled, part_type, note)"
		        " VALUES (?, ?, ?, ?, ?, ?, ?, ?)"));
		    q.addBindValue(v.varName);
		    q.addBindValue(v.displayName);
		    q.addBindValue(v.unit);
		    q.addBindValue(v.lowerBound);
		    q.addBindValue(v.upperBound);
		    q.addBindValue(v.enabled ? 1 : 0);
		    q.addBindValue(v.partType);
		    q.addBindValue(v.note);
		    if (!q.exec()) {
			    logSqlQuery(q, "seedDesignVariableDefinitions");
			    return false;
		    }
		    return true;
	    });

	return ok1 && ok2 && ok3;
}

QVector<MetricDefinition> GearOptResultDatabase::loadMetricDefinitions() const
{
	QVector<MetricDefinition> out;
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return out;

	QSqlQuery query(db);
	if (!query.exec(QStringLiteral(
	        "SELECT metric_name, display_name, unit, source,"
	        " enabled, implemented, direction, note"
	        " FROM %1 ORDER BY rowid").arg(QString::fromLatin1(kMetricDefTable)))) {
		logSqlQuery(query, "loadMetricDefinitions");
		return out;
	}

	while (query.next()) {
		MetricDefinition d;
		d.metricName  = query.value(0).toString();
		d.displayName = query.value(1).toString();
		d.unit        = query.value(2).toString();
		d.source      = query.value(3).toString();
		d.enabled     = query.value(4).toInt() != 0;
		d.implemented = query.value(5).toInt() != 0;
		d.direction   = query.value(6).toString();
		d.note        = query.value(7).toString();
		out.append(d);
	}
	return out;
}

QVector<ObjectiveMapping> GearOptResultDatabase::loadObjectiveMappings() const
{
	QVector<ObjectiveMapping> out;
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return out;

	QSqlQuery query(db);
	if (!query.exec(QStringLiteral(
	        "SELECT obj_key, metric_name, display_name, direction, obj_order, enabled, note"
	        " FROM %1 ORDER BY obj_order").arg(QString::fromLatin1(kObjectiveMapTable)))) {
		logSqlQuery(query, "loadObjectiveMappings");
		return out;
	}

	while (query.next()) {
		ObjectiveMapping m;
		m.objKey      = query.value(0).toString();
		m.metricName  = query.value(1).toString();
		m.displayName = query.value(2).toString();
		m.direction   = query.value(3).toString();
		m.objOrder    = query.value(4).toInt();
		m.enabled     = query.value(5).toInt() != 0;
		m.note        = query.value(6).toString();
		out.append(m);
	}
	return out;
}

QVector<DesignVariableDefinition> GearOptResultDatabase::loadDesignVariableDefinitions() const
{
	QVector<DesignVariableDefinition> out;
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return out;

	QSqlQuery query(db);
	if (!query.exec(QStringLiteral(
	        "SELECT var_name, display_name, unit, lower_bound, upper_bound, enabled, part_type, note"
	        " FROM %1 ORDER BY rowid").arg(QString::fromLatin1(kDesignVarDefTable)))) {
		logSqlQuery(query, "loadDesignVariableDefinitions");
		return out;
	}

	while (query.next()) {
		DesignVariableDefinition v;
		v.varName     = query.value(0).toString();
		v.displayName = query.value(1).toString();
		v.unit        = query.value(2).toString();
		v.lowerBound  = query.value(3).toDouble();
		v.upperBound  = query.value(4).toDouble();
		v.enabled     = query.value(5).toInt() != 0;
		v.partType    = query.value(6).toString();
		v.note        = query.value(7).toString();
		out.append(v);
	}
	return out;
}

bool GearOptResultDatabase::saveMetricDefinitions(const QVector<MetricDefinition>& defs)
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;
	if (!db.transaction())
		return false;

	QSqlQuery delQuery(db);
	if (!delQuery.exec(QStringLiteral("DELETE FROM %1").arg(QString::fromLatin1(kMetricDefTable)))) {
		logSqlQuery(delQuery, "saveMetricDefinitionsDelete");
		db.rollback();
		return false;
	}

	QSqlQuery insQuery(db);
	for (const MetricDefinition& d : defs) {
		insQuery.prepare(QStringLiteral(
		    "INSERT INTO %1 (metric_name, display_name, unit, source,"
		    " enabled, implemented, direction, note) VALUES (?, ?, ?, ?, ?, ?, ?, ?)")
		                 .arg(QString::fromLatin1(kMetricDefTable)));
		insQuery.addBindValue(d.metricName);
		insQuery.addBindValue(d.displayName);
		insQuery.addBindValue(d.unit);
		insQuery.addBindValue(d.source);
		insQuery.addBindValue(d.enabled ? 1 : 0);
		insQuery.addBindValue(d.implemented ? 1 : 0);
		insQuery.addBindValue(d.direction);
		insQuery.addBindValue(d.note);
		if (!insQuery.exec()) {
			logSqlQuery(insQuery, "saveMetricDefinitionsInsert");
			db.rollback();
			return false;
		}
	}

	if (!db.commit()) {
		db.rollback();
		return false;
	}
	return true;
}

bool GearOptResultDatabase::saveObjectiveMappings(const QVector<ObjectiveMapping>& mappings)
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;
	if (!db.transaction())
		return false;

	QSqlQuery delQuery(db);
	if (!delQuery.exec(QStringLiteral("DELETE FROM %1").arg(QString::fromLatin1(kObjectiveMapTable)))) {
		logSqlQuery(delQuery, "saveObjectiveMappingsDelete");
		db.rollback();
		return false;
	}

	QSqlQuery insQuery(db);
	for (const ObjectiveMapping& m : mappings) {
		insQuery.prepare(QStringLiteral(
		    "INSERT INTO %1 (obj_key, metric_name, display_name, direction,"
		    " obj_order, enabled, note) VALUES (?, ?, ?, ?, ?, ?, ?)")
		                 .arg(QString::fromLatin1(kObjectiveMapTable)));
		insQuery.addBindValue(m.objKey);
		insQuery.addBindValue(m.metricName);
		insQuery.addBindValue(m.displayName);
		insQuery.addBindValue(m.direction);
		insQuery.addBindValue(m.objOrder);
		insQuery.addBindValue(m.enabled ? 1 : 0);
		insQuery.addBindValue(m.note);
		if (!insQuery.exec()) {
			logSqlQuery(insQuery, "saveObjectiveMappingsInsert");
			db.rollback();
			return false;
		}
	}

	if (!db.commit()) {
		db.rollback();
		return false;
	}
	return true;
}

bool GearOptResultDatabase::saveDesignVariableDefinitions(
    const QVector<DesignVariableDefinition>& vars)
{
	QSqlDatabase db = QSqlDatabase::database(connectionName(), false);
	if (!db.isValid() || !db.isOpen())
		return false;
	if (!db.transaction())
		return false;

	QSqlQuery delQuery(db);
	if (!delQuery.exec(QStringLiteral("DELETE FROM %1").arg(QString::fromLatin1(kDesignVarDefTable)))) {
		logSqlQuery(delQuery, "saveDesignVariableDefinitionsDelete");
		db.rollback();
		return false;
	}

	QSqlQuery insQuery(db);
	for (const DesignVariableDefinition& v : vars) {
		insQuery.prepare(QStringLiteral(
		    "INSERT INTO %1 (var_name, display_name, unit, lower_bound, upper_bound,"
		    " enabled, part_type, note) VALUES (?, ?, ?, ?, ?, ?, ?, ?)")
		                 .arg(QString::fromLatin1(kDesignVarDefTable)));
		insQuery.addBindValue(v.varName);
		insQuery.addBindValue(v.displayName);
		insQuery.addBindValue(v.unit);
		insQuery.addBindValue(v.lowerBound);
		insQuery.addBindValue(v.upperBound);
		insQuery.addBindValue(v.enabled ? 1 : 0);
		insQuery.addBindValue(v.partType);
		insQuery.addBindValue(v.note);
		if (!insQuery.exec()) {
			logSqlQuery(insQuery, "saveDesignVariableDefinitionsInsert");
			db.rollback();
			return false;
		}
	}

	if (!db.commit()) {
		db.rollback();
		return false;
	}
	return true;
}

} // namespace GearAutoOpt
