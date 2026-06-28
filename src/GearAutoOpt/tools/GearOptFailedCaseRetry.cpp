#include "GearOptFailedCaseRetry.h"

#include "GearAutoOpt/data/GearLogLevel.h"
#include "GearAutoOpt/runner/GearOptCaseRunner.h"
#include "GearAutoOpt/runner/GearOptMainThreadRunner.h"
#include "GearAutoOpt/solver/CCXSolverController.h"
#include "GearAutoOpt/surrogate/GearInfillSelector.h"

#include <QApplication>
#include <QDir>
#include <QHash>
#include <QMetaObject>
#include <algorithm>

namespace GearAutoOpt {

namespace {

void mergeFailedRecords(GearOptResultDatabase& db,
                        const QString& baseCaseH,
                        double fixedWidthMm,
                        QHash<QString, FailedCaseRecord>& merged)
{
	for (const FailedCaseRecord& rec : db.loadFailedCasesForRetry(baseCaseH, fixedWidthMm)) {
		if (!merged.contains(rec.caseHash))
			merged.insert(rec.caseHash, rec);
		else
			merged[rec.caseHash].dbRows += rec.dbRows;
	}
}

void updateRowsInDatabase(GearOptResultDatabase& db,
                          const FailedCaseRecord& rec,
                          const GearDesignPoint& dp,
                          bool success,
                          const QString& errorMsg,
                          const std::function<void(const QString&)>& logFn)
{
	if (!db.isOpen())
		return;

	for (const FailedCaseDbRow& row : rec.dbRows) {
		if (row.databasePath != db.databasePath())
			continue;
		if (success) {
			const bool ok = db.updateDesignPointResultByRowId(row.rowId, dp);
			if (logFn) {
				logFn(QStringLiteral("[Retry] case_hash=%1 row_id=%2 dbUpdateStatus=%3")
				          .arg(rec.caseHash)
				          .arg(row.rowId)
				          .arg(ok ? QStringLiteral("update_ok") : QStringLiteral("update_failed")));
			}
		} else {
			db.recordRetryFailureByRowId(row.rowId, errorMsg);
			if (logFn) {
				logFn(QStringLiteral("[Retry] case_hash=%1 row_id=%2 still_failed retry_count=%3")
				          .arg(rec.caseHash)
				          .arg(row.rowId)
				          .arg(row.retryCount + 1));
			}
		}
	}
}

/// 仅补全表中缺失的仿真字段，不覆盖 DB 已存的设计变量与工况参数。
void fillMissingRunSimFields(GearDesignPoint& dp, const GearOptConfig& cfg)
{
	if (dp.meshSize_mm <= 0.0 && !dp.meshAuto && cfg.solver.meshSize > 0.0)
		dp.meshSize_mm = cfg.solver.meshSize;
	if (dp.torque_Nm <= 0.0)
		dp.torque_Nm = cfg.solver.torque;
	if (dp.contactStiffness <= 0.0)
		dp.contactStiffness = cfg.solver.contactStiffness;
	if (dp.youngModulus_MPa <= 0.0)
		dp.youngModulus_MPa = cfg.solver.youngModulus > 0.0 ? cfg.solver.youngModulus : 206000.0;
	if (dp.poissonRatio <= 0.0)
		dp.poissonRatio = cfg.solver.poissonRatio > 0.0 ? cfg.solver.poissonRatio : 0.30;
	if (dp.density <= 0.0)
		dp.density = cfg.solver.density > 0.0 ? cfg.solver.density : 7.85e-9;
	if (dp.materialName.isEmpty())
		dp.materialName = QStringLiteral("STEEL");
	if (dp.meshMethod.isEmpty())
		dp.meshMethod = QStringLiteral("gmsh");
	if (dp.elementOrder <= 0)
		dp.elementOrder = 1;
	if (dp.contactType.isEmpty())
		dp.contactType = QStringLiteral("surface_to_surface_penalty");
	if (dp.staticStep.isEmpty())
		dp.staticStep = QStringLiteral("STATIC");
	if (dp.solver.isEmpty())
		dp.solver = QStringLiteral("CalculiX");
	if (dp.solverPath.isEmpty())
		dp.solverPath = CCXSolverController::detectCcxPath();
	dp.enableContact = cfg.solver.enableContact;
}

int maxRetryCountForRecord(const FailedCaseRecord& rec)
{
	int maxRetry = 0;
	for (const FailedCaseDbRow& row : rec.dbRows)
		maxRetry = std::max(maxRetry, row.retryCount);
	return maxRetry;
}

} // namespace

FailedCaseRetryStats GearOptFailedCaseRetry::run(const FailedCaseRetryOptions& opt,
                                                 const std::function<void(const QString&)>& logFn)
{
	FailedCaseRetryStats stats;

	const QString runDbPath    = GearOptResultDatabase::databasePathInRunDir(opt.runDir);
	const QString globalDbPath = GearOptResultDatabase::defaultGlobalDatabasePath();

	auto& globalDb = GearOptResultDatabase::global();
	auto& runDb    = GearOptResultDatabase::runSession();
	if (!globalDb.openDatabase(globalDbPath)) {
		if (logFn)
			logFn(QStringLiteral("[Retry] failed to open global database: %1").arg(globalDbPath));
		return stats;
	}
	if (!opt.runDir.isEmpty())
		runDb.openDatabase(runDbPath);

	GearOptMainThreadRunner::ensureInstance(QApplication::instance());

	QHash<QString, FailedCaseRecord> merged;
	if (runDb.isOpen())
		mergeFailedRecords(runDb, opt.baseCaseHash, opt.fixedCommonWidthMm, merged);
	mergeFailedRecords(globalDb, opt.baseCaseHash, opt.fixedCommonWidthMm, merged);

	stats.totalFailed = merged.size();
	if (logFn) {
		logFn(QStringLiteral("[Retry] found %1 unique failed case_hash (filter baseCase=%2 width=%3)")
		          .arg(stats.totalFailed)
		          .arg(opt.baseCaseHash.isEmpty() ? QStringLiteral("ALL") : opt.baseCaseHash)
		          .arg(opt.fixedCommonWidthMm >= 0.0
		                   ? QString::number(opt.fixedCommonWidthMm, 'g', 6)
		                   : QStringLiteral("ALL")));
	}

	GearOptConfig runCfg = opt.cfg;
	runCfg.useOptimizationBase = false;

	for (auto it = merged.constBegin(); it != merged.constEnd(); ++it) {
		const FailedCaseRecord& rec = it.value();
		++stats.retried;

		GearDesignPoint dp = rec.dp;
		dp.status   = PointStatus::Running;
		dp.errorMsg.clear();
		fillMissingRunSimFields(dp, runCfg);

		QString reliefReason;
		if (!validateReliefDesign(dp, &reliefReason)) {
			++stats.stillFailed;
			const QString err = QStringLiteral("invalid relief: %1").arg(reliefReason);
			logInvalidReliefDesign(dp, reliefReason);
			if (logFn)
				logFn(QStringLiteral("[Retry] skip case_hash=%1 | %2").arg(rec.caseHash, err));
			updateRowsInDatabase(runDb, rec, dp, false, err, logFn);
			updateRowsInDatabase(globalDb, rec, dp, false, err, logFn);
			if (QApplication::instance())
				QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
			continue;
		}

		const int attempt  = maxRetryCountForRecord(rec) + 1;
		const QString workDir =
		    gearOptCaseRetryWorkDir(opt.runDir, dp.generation, dp.id, attempt);
		{
			QDir stale(workDir);
			if (stale.exists()) {
				if (logFn)
					logFn(QStringLiteral("[Retry] clean stale workDir=%1").arg(workDir));
				stale.removeRecursively();
			}
		}
		QDir().mkpath(workDir);
		dp.runDir = workDir;

		const double meshSize =
		    dp.meshSize_mm > 0.0 ? dp.meshSize_mm
		                         : (opt.fixedMeshSizeMm > 0.0 ? opt.fixedMeshSizeMm : 0.0);
		const bool meshAuto = dp.meshAuto || meshSize <= 0.0;

		if (logFn) {
			logFn(QStringLiteral("[Retry] case_id=%1 case_hash=%2 workDir=%3 attempt=%4")
			          .arg(dp.id)
			          .arg(rec.caseHash)
			          .arg(workDir)
			          .arg(attempt));
			logFn(QStringLiteral("[Retry] DB design -> %1")
			          .arg(GearInfillSelector::formatDesignVarsForLog(dp)));
			logFn(QStringLiteral("[Retry] DB sim -> m=%1 z1=%2 z2=%3 alpha=%4 width=%5 torque=%6 meshAuto=%7 meshSize=%8")
			          .arg(dp.module, 0, 'g', 8)
			          .arg(dp.z1)
			          .arg(dp.z2)
			          .arg(dp.alpha, 0, 'g', 8)
			          .arg(dp.commonWidth, 0, 'g', 8)
			          .arg(dp.torque_Nm, 0, 'g', 8)
			          .arg(meshAuto ? QStringLiteral("true") : QStringLiteral("false"))
			          .arg(meshSize, 0, 'g', 8));
			logFn(QStringLiteral("[Retry] pipeline: geometry -> mesh -> inp -> ccx -> parse"));
		}

		GearOptCasePrepRequest prepReq;
		prepReq.dp         = dp;
		prepReq.workDir    = workDir;
		prepReq.cfg        = runCfg;
		prepReq.meshParams = { meshSize, opt.fixedRootMeshSizeMm, opt.fixedZLayers };
		prepReq.logLevel   = runCfg.solver.debugMode ? GearLogLevel::Debug : GearLogLevel::Normal;
		prepReq.threads    = runCfg.solver.threads;

		QMetaObject::Connection mtLogConn;
		if (GearOptMainThreadRunner* mt = GearOptMainThreadRunner::instance()) {
			if (logFn) {
				mtLogConn = QObject::connect(mt, &GearOptMainThreadRunner::logMessage,
				                             [logFn](const QString& msg) { logFn(msg); });
			}
		}

		bool success = false;
		if (GearOptMainThreadRunner::runPreCcxBlocking(&prepReq)) {
			dp = prepReq.dp;
			GearOptCaseRunner ccxRunner;
			ccxRunner.setConfig(runCfg);
			ccxRunner.setWorkDir(workDir);
			ccxRunner.setMeshParams(prepReq.meshParams);
			ccxRunner.setLogLevel(prepReq.logLevel);
			ccxRunner.setThreads(runCfg.solver.threads);
			QMetaObject::Connection ccxLogConn;
			if (logFn) {
				ccxLogConn = QObject::connect(&ccxRunner, &GearOptCaseRunner::log,
				                              [logFn](const QString& msg) { logFn(msg); });
			}
			if (ccxRunner.runCcxOnlySteps(dp)) {
				dp.status = PointStatus::Done;
				success   = true;
			}
			QObject::disconnect(ccxLogConn);
		} else {
			dp = prepReq.dp;
			if (logFn) {
				logFn(QStringLiteral("[Retry] pre-CCX failed case_hash=%1 | %2")
				          .arg(rec.caseHash, dp.errorMsg.left(200)));
			}
		}
		QObject::disconnect(mtLogConn);

		dp.runDir = workDir;
		dp.fillResultArtifactPaths();
		const bool converged = dp.status == PointStatus::Done
		                       && dp.cpressMax_MPa > 0.0 && dp.sigmaMax > 0.0
		                       && dp.uMax >= 0.0 && dp.mass > 0.0
		                       && dp.edgeLoadRatio > 0.0 && dp.cpressCV > 0.0;

		if (success && converged) {
			++stats.fixed;
			updateRowsInDatabase(runDb, rec, dp, true, QString(), logFn);
			updateRowsInDatabase(globalDb, rec, dp, true, QString(), logFn);
			if (logFn) {
				logFn(QStringLiteral("[Retry] fixed case_hash=%1 cpress=%2 MPa sigma=%3 MPa")
				          .arg(rec.caseHash)
				          .arg(dp.cpressMax_MPa, 0, 'g', 6)
				          .arg(dp.sigmaMax, 0, 'g', 6));
			}
		} else {
			++stats.stillFailed;
			const QString err = dp.errorMsg.isEmpty()
			                        ? QStringLiteral("retry failed without CCX convergence")
			                        : dp.errorMsg;
			updateRowsInDatabase(runDb, rec, dp, false, err, logFn);
			updateRowsInDatabase(globalDb, rec, dp, false, err, logFn);
		}

		if (QApplication::instance())
			QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
	}

	if (logFn) {
		logFn(QStringLiteral("[Retry] done: total=%1 retried=%2 fixed=%3 still_failed=%4")
		          .arg(stats.totalFailed)
		          .arg(stats.retried)
		          .arg(stats.fixed)
		          .arg(stats.stillFailed));
	}
	return stats;
}

} // namespace GearAutoOpt
