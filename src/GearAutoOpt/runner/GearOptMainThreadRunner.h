#ifndef _GEARAUTOOPT_GEAR_OPT_MAIN_THREAD_RUNNER_H_
#define _GEARAUTOOPT_GEAR_OPT_MAIN_THREAD_RUNNER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/runner/GearOptCaseRunner.h"

#include <QObject>
#include <QString>

namespace GearAutoOpt {

/// 主线程串行执行：建模 + 网格 + 写 INP（GeometryData / OCC 非线程安全）。
struct GEARAUTOOPTAPI GearOptCasePrepRequest {
	GearDesignPoint dp;
	QString         workDir;
	GearOptConfig   cfg;
	GearMeshParams  meshParams;
	GearLogLevel    logLevel = GearLogLevel::Normal;
	int             threads  = 0;
	bool            success  = false;
};

class GEARAUTOOPTAPI GearOptMainThreadRunner : public QObject {
	Q_OBJECT
public:
	static GearOptMainThreadRunner* instance();
	/// 在 GUI 主线程创建单例（GearOptDialog 构造时调用）。
	static void ensureInstance(QObject* mainThreadParent);

	/// 从任意线程阻塞调用：在主线程执行 runPreCcxSteps + cleanup。
	static bool runPreCcxBlocking(GearOptCasePrepRequest* req);

public slots:
	void executePreCcx(GearOptCasePrepRequest* req);

signals:
	void logMessage(const QString& msg);

private:
	explicit GearOptMainThreadRunner(QObject* parent = nullptr);
	static GearOptMainThreadRunner* s_instance;
};

GEARAUTOOPTAPI QString gearOptCaseWorkDir(const QString& runDir, int generation, int caseId);
/// 失败重算专用目录（每次 attempt 独立，避免复用旧 mesh/inp）。
GEARAUTOOPTAPI QString gearOptCaseRetryWorkDir(const QString& runDir,
                                               int generation,
                                               int caseId,
                                               int attempt);

} // namespace GearAutoOpt

Q_DECLARE_METATYPE(GearAutoOpt::GearOptCasePrepRequest*)

#endif
