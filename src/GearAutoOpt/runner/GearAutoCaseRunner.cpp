#include "GearAutoCaseRunner.h"
#include "GearAutoOpt/data/GearOptLog.h"

namespace GearAutoOpt {
QString caseRunnerStateToString(CaseRunnerState s) {
	switch (s) {
	case CaseRunnerState::Idle:     return QStringLiteral("idle");
	case CaseRunnerState::Geom:     return QStringLiteral("geom");
	case CaseRunnerState::Mesh:     return QStringLiteral("mesh");
	case CaseRunnerState::InpWrite: return QStringLiteral("inp_write");
	case CaseRunnerState::Solve:    return QStringLiteral("solve");
	case CaseRunnerState::Parse:    return QStringLiteral("parse");
	case CaseRunnerState::Done:     return QStringLiteral("done");
	case CaseRunnerState::Failed:   return QStringLiteral("failed");
	}
	return QStringLiteral("unknown");
}

GearAutoCaseRunner::GearAutoCaseRunner(QObject* parent)
	: QObject(parent)
{}

void GearAutoCaseRunner::setWorkDir(const QString& dir) {
	_workDir = dir;
}

void GearAutoCaseRunner::setLogLevel(GearLogLevel level) {
	_logLevel = level;
	GearOptLog::setLevel(level);
}

void GearAutoCaseRunner::transition(CaseRunnerState s) {	if (_state == s) return;
	_state = s;
	emit stateChanged(s);
}

void GearAutoCaseRunner::emitLogNormal(const QString& msg) {
	emit log(msg);
}

void GearAutoCaseRunner::emitLogDebug(const QString& msg) {
	if (_logLevel == GearLogLevel::Debug)
		emit log(msg);
}

void GearAutoCaseRunner::emitLog(const QString& msg) {
	emitLogDebug(msg);
}

namespace {

struct CaseRunnerStep {
	CaseRunnerState target;
	bool (GearAutoCaseRunner::*fn)(GearDesignPoint&);
};

} // namespace

bool GearAutoCaseRunner::runPreCcxSteps(GearDesignPoint& dp)
{
	transition(CaseRunnerState::Idle);
	dp.status     = PointStatus::Running;
	dp.errorMsg.clear();

	static const CaseRunnerStep kPreSteps[] = {
	    { CaseRunnerState::Geom,     &GearAutoCaseRunner::runGeometryStep },
	    { CaseRunnerState::Mesh,     &GearAutoCaseRunner::runMeshStep     },
	    { CaseRunnerState::InpWrite, &GearAutoCaseRunner::runInpWriteStep },
	};
	for (const auto& step : kPreSteps) {
		transition(step.target);
		emitLogDebug(QString::fromUtf8("→ ") + caseRunnerStateToString(step.target));
		if (!(this->*step.fn)(dp)) {
			if (dp.errorMsg.isEmpty())
				dp.errorMsg = QString("step '%1' failed").arg(caseRunnerStateToString(step.target));
			dp.status = PointStatus::Failed;
			transition(CaseRunnerState::Failed);
			emitLogNormal(QStringLiteral("FAIL ") + caseRunnerStateToString(step.target)
			              + QStringLiteral(": ") + dp.errorMsg);
			return false;
		}
	}
	return true;
}

bool GearAutoCaseRunner::runCcxOnlySteps(GearDesignPoint& dp)
{
	static const CaseRunnerStep kCcxSteps[] = {
	    { CaseRunnerState::Solve, &GearAutoCaseRunner::runSolveStep },
	    { CaseRunnerState::Parse, &GearAutoCaseRunner::runParseStep },
	};
	for (const auto& step : kCcxSteps) {
		transition(step.target);
		emitLogDebug(QString::fromUtf8("→ ") + caseRunnerStateToString(step.target));
		if (!(this->*step.fn)(dp)) {
			if (dp.errorMsg.isEmpty())
				dp.errorMsg = QString("step '%1' failed").arg(caseRunnerStateToString(step.target));
			dp.status = PointStatus::Failed;
			transition(CaseRunnerState::Failed);
			emitLogNormal(QStringLiteral("FAIL ") + caseRunnerStateToString(step.target)
			              + QStringLiteral(": ") + dp.errorMsg);
			return false;
		}
	}
	return true;
}

void GearAutoCaseRunner::runOne(GearDesignPoint& dp)
{
	transition(CaseRunnerState::Idle);
	dp.status     = PointStatus::Running;
	dp.errorMsg.clear();

	if (!runPreCcxSteps(dp)) {
		emit finished(false);
		return;
	}
	if (!runCcxOnlySteps(dp)) {
		emit finished(false);
		return;
	}
	dp.status = PointStatus::Done;
	transition(CaseRunnerState::Done);
	emit finished(true);
}

// 默认 stub（在子类或直接 override 中填实）
bool GearAutoCaseRunner::runGeometryStep(GearDesignPoint&)  { return true; }
bool GearAutoCaseRunner::runMeshStep(GearDesignPoint&)      { return true; }
bool GearAutoCaseRunner::runInpWriteStep(GearDesignPoint&)  { return true; }
bool GearAutoCaseRunner::runSolveStep(GearDesignPoint&)     { return true; }
bool GearAutoCaseRunner::runParseStep(GearDesignPoint&)     { return true; }

} // namespace GearAutoOpt
