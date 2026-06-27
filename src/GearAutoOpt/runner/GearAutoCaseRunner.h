#ifndef _GEARAUTOOPT_GEAR_AUTO_CASE_RUNNER_H_
#define _GEARAUTOOPT_GEAR_AUTO_CASE_RUNNER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearLogLevel.h"

#include <QObject>
#include <QString>

namespace GearAutoOpt {

/// 单个设计点 single-point 求解流水线状态。
/// 状态机：`Idle → Geom → Mesh → InpWrite → Solve → Parse → Done | Failed`
enum class CaseRunnerState {
	Idle = 0,
	Geom,        ///< 调 GeoCommandCreateGear
	Mesh,        ///< 调 GmshThread + MeshConverter
	InpWrite,    ///< 调 CCXInpWriter，BC/载荷算好
	Solve,       ///< 调 CCXSolverController，等 processFinish
	Parse,       ///< 调 CCXResultParser，回填 dp.sigmaMax 等
	Done,        ///< 全部成功
	Failed,      ///< 任一步骤失败
};

GEARAUTOOPTAPI QString caseRunnerStateToString(CaseRunnerState s);

/// 单点工况编排器。
///
/// 设计目标：把 P2 求解器组件（CCXInpWriter / Controller / Parser）+ 后续 P4
/// 几何/网格步串成一条同步流水线。每个设计点对应一次 `runOne(dp)` 调用。
///
/// 由于 ccx 求解和 gmsh 网格是异步的，子类在 override 时通常通过 `QEventLoop`
/// 把异步包成同步——参考 `test/integration/run_single_point.cpp` 的写法。
class GEARAUTOOPTAPI GearAutoCaseRunner : public QObject {
	Q_OBJECT
public:
	explicit GearAutoCaseRunner(QObject* parent = nullptr);
	~GearAutoCaseRunner() override = default;

	/// 设置该次运行的工作目录（一般是 `runs/{gen}_{id}/`）。
	void    setWorkDir(const QString& dir);
	QString workDir() const { return _workDir; }

	void setLogLevel(GearLogLevel level);
	GearLogLevel logLevel() const { return _logLevel; }

	CaseRunnerState state() const { return _state; }

	/// 同步跑完一个设计点。dp 按引用更新：
	/// - 成功 → dp.status = Done，sigmaMax / uMax / mass 由 ParseStep 写回
	/// - 失败 → dp.status = Failed，errorMsg 写回首条错误
	/// 跑完会 emit `finished(success)`，期间 emit 多次 `stateChanged`。
	virtual void runOne(GearDesignPoint& dp);

signals:
	/// 进入新状态（包括 Done/Failed 终态）
	void stateChanged(GearAutoOpt::CaseRunnerState s);
	/// 整次 runOne 结束（state 已切到 Done 或 Failed）
	void finished(bool success);
	/// 给 GUI 喂日志 / 进度文字
	void log(const QString& msg);

protected:
	// 默认实现 = stub（返回 true）。
	// 失败时返回 false 并把错误写到 dp.errorMsg。
	virtual bool runGeometryStep(GearDesignPoint& dp);
	virtual bool runMeshStep(GearDesignPoint& dp);
	virtual bool runInpWriteStep(GearDesignPoint& dp);
	virtual bool runSolveStep(GearDesignPoint& dp);
	virtual bool runParseStep(GearDesignPoint& dp);

	void transition(CaseRunnerState s);
	void emitLogNormal(const QString& msg);
	void emitLogDebug(const QString& msg);
	void emitLog(const QString& msg);

private:
	QString         _workDir;
	CaseRunnerState _state{CaseRunnerState::Idle};
	GearLogLevel    _logLevel{GearLogLevel::Normal};
};

} // namespace GearAutoOpt

#endif
