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

void GearAutoCaseRunner::runOne(GearDesignPoint& dp) {	transition(CaseRunnerState::Idle);
	dp.status = PointStatus::Running; //当前设计点正在运行
	dp.errorMsg.clear(); //清空错误信息

	//定义一个结构体，用于存储每个步骤的名称和对应的函数指针
	struct Step {
		CaseRunnerState target;
		bool (GearAutoCaseRunner::*fn)(GearDesignPoint&);
	};
	const Step steps[] = { //定义一个数组，用于存储每个步骤的名称和对应的函数指针
		{ CaseRunnerState::Geom,     &GearAutoCaseRunner::runGeometryStep },
		{ CaseRunnerState::Mesh,     &GearAutoCaseRunner::runMeshStep     },
		{ CaseRunnerState::InpWrite, &GearAutoCaseRunner::runInpWriteStep },
		{ CaseRunnerState::Solve,    &GearAutoCaseRunner::runSolveStep    },
		{ CaseRunnerState::Parse,    &GearAutoCaseRunner::runParseStep    },
	};

	for (const auto& step : steps) { //遍历每个步骤
		transition(step.target);
		emitLogDebug(QString::fromUtf8("→ ") + caseRunnerStateToString(step.target));		const bool ok = (this->*step.fn)(dp); //调用对应的函数
		if (!ok) {
			if (dp.errorMsg.isEmpty()) {
				dp.errorMsg = QString("step '%1' failed")
				              .arg(caseRunnerStateToString(step.target));
			}
			dp.status = PointStatus::Failed; //当前设计点运行失败	
			transition(CaseRunnerState::Failed); //切换到失败状态
			emitLogNormal(QStringLiteral("FAIL ") + caseRunnerStateToString(step.target)
			        + QStringLiteral(": ") + dp.errorMsg);			emit finished(false); //发出运行结束信号
			return;
		}
	}
	dp.status = PointStatus::Done; //当前设计点运行成功
	transition(CaseRunnerState::Done);
	emit finished(true); //发出运行结束信号
}

// 默认 stub（在子类或直接 override 中填实）
bool GearAutoCaseRunner::runGeometryStep(GearDesignPoint&)  { return true; }
bool GearAutoCaseRunner::runMeshStep(GearDesignPoint&)      { return true; }
bool GearAutoCaseRunner::runInpWriteStep(GearDesignPoint&)  { return true; }
bool GearAutoCaseRunner::runSolveStep(GearDesignPoint&)     { return true; }
bool GearAutoCaseRunner::runParseStep(GearDesignPoint&)     { return true; }

} // namespace GearAutoOpt
