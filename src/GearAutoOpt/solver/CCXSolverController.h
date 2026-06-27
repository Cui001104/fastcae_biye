#ifndef _GEARAUTOOPT_CCX_SOLVER_CONTROLLER_H_
#define _GEARAUTOOPT_CCX_SOLVER_CONTROLLER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearLogLevel.h"

#include <QObject>
#include <QProcess>
#include <QString>

class QTimer;

namespace GearAutoOpt {

/// ccx.exe 单次运行的失败分类，数值越大越严重，用于 worse() 合并时的覆盖判断。
enum class CCXFailureReason {
	None              = 0,  ///< 求解成功（exitCode==0 且无错误关键字）
	ExitNonZero       = 1,  ///< 退出码 != 0 但未识别具体错误关键字
	NoConvergence     = 2,  ///< 最终未收敛（非接触迭代中间行的 no convergence）
	ErrorMessage      = 3,  ///< stdout 含 " *ERROR" 但未匹配下面更具体的项
	SingularMatrix    = 4,  ///< stdout 含 "singular"（刚体未约束 / 矩阵奇异）
	NegativeJacobian  = 5,  ///< stdout 含 "Negative Jacobian"（网格畸变）
	Crash             = 6,  ///< QProcess::CrashExit（ccx 崩溃）
	TimedOut          = 7,  ///< 超时被 controller kill
};

/// 失败原因转字符串，用于日志输出和 GUI 显示。
GEARAUTOOPTAPI QString failureReasonToString(CCXFailureReason r);

/// ccx stdout 摘要（求解结束后由 controller 解析填充）。
struct GEARAUTOOPTAPI CCXSolveSummary {
	int nodes           = -1;
	int elements        = -1;
	int equations       = -1;
	int incrementCount  = 0;
	int usedThreads     = -1;
};

/// ccx.exe 进程的轻量启停控制器，不依赖 FastCAE GUI 上下文（SolverControlBase
/// 强依赖 MainWindow / SolverInfo，不适合 headless 批量优化）。
/// 信号接口（processFinish / sendMessage）与 SolverControlBase 保持一致，
/// 方便日后集成到主界面时包一层适配器。
class GEARAUTOOPTAPI CCXSolverController : public QObject {
	Q_OBJECT
public:
	explicit CCXSolverController(QObject* parent = nullptr);
	~CCXSolverController() override;

	void setExePath(const QString& path);
	void setWorkDir(const QString& dir);
	void setJobName(const QString& job);   ///< 不带后缀，对应 {workDir}/{job}.inp
	void setTimeoutSeconds(int sec);       ///< <= 0 表示不超时（仅调试用）
	void setThreads(int n)         { _threads = (n > 0) ? n : 0; } ///< ccx -t / OMP；0 = defaultThreadCount()
	void setLogLevel(GearLogLevel level) { _logLevel = level; }
	/// CCX stdout/stderr 完整写入该路径（界面不再刷原始输出）。
	void setOutputLogPath(const QString& path) { _outputLogPath = path; }

	/// min(QThread::idealThreadCount(), 8)，至少 1。
	static int defaultThreadCount();

	QString exePath() const        { return _exePath; }
	QString workDir() const        { return _workDir; }
	QString jobName() const        { return _jobName; }
	int     timeoutSeconds() const { return _timeoutSec; }
	int     threads() const        { return _threads; }
	CCXSolveSummary solveSummary() const { return _summary; }

	/// 进程 pid（未启动或已结束时返回 0）
	qint64  pid() const;
	/// 进程是否还在运行
	bool    isRunning() const;
	/// 是否因为超时被本类 kill
	bool    killedByTimeout() const { return _killedByTimeout; }

	/// 启动 ccx.exe；返回 false = 前置检查失败（路径不存在等），不会 emit 信号。
	bool start();

	/// 探测 ccx_MT.exe 路径：先读 CCX_PATH 环境变量，再按多级回溯找 tools/calculix/ccx_MT.exe。
	/// 找不到返回空 QString。
	static QString detectCcxPath();

	/// 同上，但返回多行诊断字符串，末尾附带选中路径或 "<not found>"。
	static QString detectCcxPathDiagnostic();
	/// 主动终止；wait 表示是否阻塞等待退出。
	void stop(bool wait = true);

signals:
	/// ccx 进程已 spawn，pid 已分配
	void started(qint64 pid);
	/// 转发 ccx stdout / stderr 的逐行内容（保留接口；Normal 模式下不再刷界面）
	void sendMessage(const QString& msg);
	/// ccx 已退出（无论成功/失败/超时 kill 都会 emit 一次）
	/// @param exitCode    ccx 退出码（kill 情况下为 -1）
	/// @param reason      失败分类：None = 成功；其它详见 CCXFailureReason
	/// @param firstError  扫到的第一行 *ERROR 内容（无错误时为空），用于写入 db.error_msg
	void processFinish(int exitCode, GearAutoOpt::CCXFailureReason reason, const QString& firstError);

private slots:
	void onStdoutReady();
	void onStderrReady();
	void onProcessFinished(int code, QProcess::ExitStatus status);
	void onTimeout();

private:
	void drainAndEmit(QByteArray& buffer, bool isErr);
	void appendOutputLogLine(const QString& line);

	QProcess* _proc{nullptr};
	QTimer*   _timer{nullptr};
	QString   _exePath;
	QString   _workDir;
	QString   _jobName{"job"};
	int       _timeoutSec{1200};
	int       _threads{0};
	QByteArray _stdoutBuf;
	QByteArray _stderrBuf;
	bool       _killedByTimeout{false};
	CCXFailureReason _detectedReason{CCXFailureReason::None};
	QString    _firstErrorLine;
	/// CalculiX 接触/非线性步：每轮 NR 会交替打印 no convergence / convergence，以最后一次为准。
	bool       _newtonConverged{false};
	bool       _jobFinished{false};
	CCXSolveSummary _summary;
	GearLogLevel    _logLevel{GearLogLevel::Normal};
	QString         _outputLogPath;
};

} // namespace GearAutoOpt

#endif
