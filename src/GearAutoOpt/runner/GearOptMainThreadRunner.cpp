#include "GearOptMainThreadRunner.h"

#include <QCoreApplication>
#include <QDir>
#include <QMetaObject>
#include <QThread>

namespace GearAutoOpt {

GearOptMainThreadRunner* GearOptMainThreadRunner::s_instance = nullptr;

QString gearOptCaseWorkDir(const QString& runDir, int generation, int caseId)
{
	return QDir(runDir).filePath(QStringLiteral("gen%1/case_%2").arg(generation).arg(caseId));
}

QString gearOptCaseRetryWorkDir(const QString& runDir, int generation, int caseId, int attempt)
{
	return QDir(runDir).filePath(
	    QStringLiteral("retry/gen%1/case_%2/attempt_%3").arg(generation).arg(caseId).arg(attempt));
}

GearOptMainThreadRunner::GearOptMainThreadRunner(QObject* parent)
	: QObject(parent)
{
}

GearOptMainThreadRunner* GearOptMainThreadRunner::instance()
{
	return s_instance;
}

void GearOptMainThreadRunner::ensureInstance(QObject* mainThreadParent)
{
	if (s_instance)
		return;
	QObject* parent = mainThreadParent ? mainThreadParent : QCoreApplication::instance();
	s_instance      = new GearOptMainThreadRunner(parent);
	qRegisterMetaType<GearOptCasePrepRequest*>("GearOptCasePrepRequest*");
}

void GearOptMainThreadRunner::executePreCcx(GearOptCasePrepRequest* req)
{
	if (!req)
		return;

	req->success = false;
	req->dp.runDir = req->workDir;
	QDir().mkpath(req->workDir);

	GearOptCaseRunner runner;
	runner.setConfig(req->cfg);
	runner.setWorkDir(req->workDir);
	runner.setMeshParams(req->meshParams);
	runner.setLogLevel(req->logLevel);
	runner.setThreads(req->threads);
	QObject::connect(&runner, &GearOptCaseRunner::log, this, &GearOptMainThreadRunner::logMessage);

	req->success = runner.runPreCcxSteps(req->dp);
	runner.cleanup();
}

bool GearOptMainThreadRunner::runPreCcxBlocking(GearOptCasePrepRequest* req)
{
	if (!req)
		return false;

	ensureInstance(QCoreApplication::instance());
	if (!s_instance) {
		req->success = false;
		req->dp.errorMsg = QStringLiteral("GearOptMainThreadRunner not initialized");
		return false;
	}

	if (QThread::currentThread() == s_instance->thread()) {
		s_instance->executePreCcx(req);
		return req->success;
	}

	const bool invoked = QMetaObject::invokeMethod(
	    s_instance,
	    "executePreCcx",
	    Qt::BlockingQueuedConnection,
	    Q_ARG(GearOptCasePrepRequest*, req));
	return invoked && req->success;
}

} // namespace GearAutoOpt
