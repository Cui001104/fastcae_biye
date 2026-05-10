#include "GearAutoCaseRunner.h"

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

void GearAutoCaseRunner::transition(CaseRunnerState s) {
	if (_state == s) return;
	_state = s;
	emit stateChanged(s);
}

void GearAutoCaseRunner::emitLog(const QString& msg) {
	emit log(msg);
}

void GearAutoCaseRunner::runOne(GearDesignPoint& dp) {
	transition(CaseRunnerState::Idle);
	dp.status = PointStatus::Running;
	dp.errorMsg.clear();

	struct Step {
		CaseRunnerState target;
		bool (GearAutoCaseRunner::*fn)(GearDesignPoint&);
	};
	const Step steps[] = {
		{ CaseRunnerState::Geom,     &GearAutoCaseRunner::runGeometryStep },
		{ CaseRunnerState::Mesh,     &GearAutoCaseRunner::runMeshStep     },
		{ CaseRunnerState::InpWrite, &GearAutoCaseRunner::runInpWriteStep },
		{ CaseRunnerState::Solve,    &GearAutoCaseRunner::runSolveStep    },
		{ CaseRunnerState::Parse,    &GearAutoCaseRunner::runParseStep    },
	};

	for (const auto& step : steps) {
		transition(step.target);
		emitLog(QStringLiteral("→ ") + caseRunnerStateToString(step.target));
		const bool ok = (this->*step.fn)(dp);
		if (!ok) {
			if (dp.errorMsg.isEmpty()) {
				dp.errorMsg = QString("step '%1' failed")
					.arg(caseRunnerStateToString(step.target));
			}
			dp.status = PointStatus::Failed;
			transition(CaseRunnerState::Failed);
			emit finished(false);
			return;
		}
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
