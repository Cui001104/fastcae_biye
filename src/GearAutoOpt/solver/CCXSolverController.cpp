#include "CCXSolverController.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QString>
#include <QStringList>
#include <QTimer>

namespace GearAutoOpt {

QString failureReasonToString(CCXFailureReason r) {
	switch (r) {
	case CCXFailureReason::None:             return QStringLiteral("none");
	case CCXFailureReason::ExitNonZero:      return QStringLiteral("exit_nonzero");
	case CCXFailureReason::NoConvergence:    return QStringLiteral("no_convergence");
	case CCXFailureReason::ErrorMessage:     return QStringLiteral("error_message");
	case CCXFailureReason::SingularMatrix:   return QStringLiteral("singular_matrix");
	case CCXFailureReason::NegativeJacobian: return QStringLiteral("negative_jacobian");
	case CCXFailureReason::Crash:            return QStringLiteral("crash");
	case CCXFailureReason::TimedOut:         return QStringLiteral("timed_out");
	}
	return QStringLiteral("unknown");
}

namespace {

// 逐行扫 ccx 输出，返回该行匹配到的最严重失败原因；toLower 后比较，兼容大小写。
CCXFailureReason classifyLine(const QString& line) {
	const QString lower = line.toLower();
	// 雅可比相关：CCX 实际打印 "nonpositive jacobian"（e_c3d.f）；
	// 部分版本 / 分支也可能出现 "negative jacobian"，两者都覆盖。
	if (lower.contains(QLatin1String("nonpositive jacobian"))
	    || lower.contains(QLatin1String("negative jacobian")))
		return CCXFailureReason::NegativeJacobian;
	if (lower.contains(QLatin1String("singular")))
		return CCXFailureReason::SingularMatrix;
	if (lower.contains(QLatin1String("no convergence"))
	    || lower.contains(QLatin1String("did not converge"))
	    || lower.contains(QLatin1String("diverged")))
		return CCXFailureReason::NoConvergence;
	if (lower.contains(QLatin1String("*error")) || lower.contains(QLatin1String(" error in")))
		return CCXFailureReason::ErrorMessage;
	return CCXFailureReason::None;
}

inline CCXFailureReason worse(CCXFailureReason a, CCXFailureReason b) {
	return static_cast<int>(a) >= static_cast<int>(b) ? a : b;
}

} // anonymous namespace

CCXSolverController::CCXSolverController(QObject* parent)
	: QObject(parent)
	, _proc(new QProcess(this))
	, _timer(new QTimer(this))
{
	_timer->setSingleShot(true);
	connect(_proc, &QProcess::readyReadStandardOutput,
	        this, &CCXSolverController::onStdoutReady);
	connect(_proc, &QProcess::readyReadStandardError,
	        this, &CCXSolverController::onStderrReady);
	// QProcess::finished 有重载，static_cast 显式选择带 ExitStatus 的版本
	connect(_proc,
	        static_cast<void(QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
	        this, &CCXSolverController::onProcessFinished);
	connect(_timer, &QTimer::timeout,
	        this, &CCXSolverController::onTimeout);
}

CCXSolverController::~CCXSolverController() {
	if (isRunning()) {
		_proc->kill();
		_proc->waitForFinished(2000);
	}
}

void CCXSolverController::setExePath(const QString& path)  { _exePath = path; }
void CCXSolverController::setWorkDir(const QString& dir)   { _workDir = dir; }
void CCXSolverController::setJobName(const QString& job)   { _jobName = job; }
void CCXSolverController::setTimeoutSeconds(int sec)       { _timeoutSec = sec; }

qint64 CCXSolverController::pid() const {
	return _proc ? _proc->processId() : 0;
}

bool CCXSolverController::isRunning() const {
	return _proc && _proc->state() != QProcess::NotRunning;
}

bool CCXSolverController::start() {
	if (isRunning()) return false;

	const QFileInfo exe(_exePath);
	if (!exe.exists() || !exe.isFile()) return false;

	const QFileInfo dir(_workDir);
	if (!dir.exists() || !dir.isDir()) return false;

	const QString inp = QDir(_workDir).filePath(_jobName + ".inp");
	if (!QFileInfo::exists(inp)) return false;

	_killedByTimeout = false;
	_stdoutBuf.clear();
	_stderrBuf.clear();
	_detectedReason = CCXFailureReason::None;
	_firstErrorLine.clear();

	_proc->setWorkingDirectory(_workDir);
	// 注入 OMP_NUM_THREADS（_threads==0 表示不覆盖，跟随系统默认）
	if (_threads > 0) {
		QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
		env.insert(QStringLiteral("OMP_NUM_THREADS"), QString::number(_threads));
		_proc->setProcessEnvironment(env);
	}
	_proc->start(_exePath, QStringList{ _jobName });
	if (!_proc->waitForStarted(5000)) return false;

	if (_timeoutSec > 0) {
		_timer->start(_timeoutSec * 1000);
	}
	emit started(_proc->processId());
	emit sendMessage(QString("ccx started: pid=%1, job=%2, dir=%3")
	                 .arg(_proc->processId())
	                 .arg(_jobName)
	                 .arg(_workDir));
	return true;
}

void CCXSolverController::stop(bool wait) {
	if (!isRunning()) return;
	_proc->kill();
	if (wait) _proc->waitForFinished(3000);
}

void CCXSolverController::drainAndEmit(QByteArray& buffer, bool isErr) {
	int idx = 0;
	while ((idx = buffer.indexOf('\n')) >= 0) {
		QByteArray lineBytes = buffer.left(idx);
		// 去掉可能的 \r（Windows）
		if (!lineBytes.isEmpty() && lineBytes.endsWith('\r')) {
			lineBytes.chop(1);
		}
		buffer.remove(0, idx + 1);
		const QString line = QString::fromLocal8Bit(lineBytes);

		// 记录最严重的失败原因；第一行 *ERROR 留底供上层显示。
		const CCXFailureReason hit = classifyLine(line);
		if (hit != CCXFailureReason::None) {
			_detectedReason = worse(_detectedReason, hit);
			if (_firstErrorLine.isEmpty()) {
				_firstErrorLine = line.trimmed();
			}
		}

		emit sendMessage(QString(isErr ? "[stderr] " : "[stdout] ") + line);
	}
}

void CCXSolverController::onStdoutReady() {
	_stdoutBuf.append(_proc->readAllStandardOutput());
	drainAndEmit(_stdoutBuf, false);
}

void CCXSolverController::onStderrReady() {
	_stderrBuf.append(_proc->readAllStandardError());
	drainAndEmit(_stderrBuf, true);
}

void CCXSolverController::onProcessFinished(int code, QProcess::ExitStatus status) {
	_timer->stop();

	// 收尾未带 \n 的最后一行（同样需要扫关键字）
	auto flushTail = [&](QByteArray& buf, bool isErr) {
		if (buf.isEmpty()) return;
		const QString line = QString::fromLocal8Bit(buf);
		const CCXFailureReason hit = classifyLine(line);
		if (hit != CCXFailureReason::None) {
			_detectedReason = worse(_detectedReason, hit);
			if (_firstErrorLine.isEmpty()) _firstErrorLine = line.trimmed();
		}
		emit sendMessage(QString(isErr ? "[stderr] " : "[stdout] ") + line);
		buf.clear();
	};
	flushTail(_stdoutBuf, false);
	flushTail(_stderrBuf, true);

	const int finalCode = (status == QProcess::CrashExit) ? -1 : code;

	// 综合判定最终原因（按优先级递增覆盖）
	CCXFailureReason finalReason = _detectedReason;
	if (status == QProcess::CrashExit) {
		finalReason = worse(finalReason, CCXFailureReason::Crash);
	}
	if (_killedByTimeout) {
		finalReason = worse(finalReason, CCXFailureReason::TimedOut);
	}
	if (finalReason == CCXFailureReason::None && code != 0) {
		// exit != 0 但未检测到具体关键字 — 兜底标记
		finalReason = CCXFailureReason::ExitNonZero;
	}

	emit processFinish(finalCode, finalReason, _firstErrorLine);
}

void CCXSolverController::onTimeout() {
	_killedByTimeout = true;
	emit sendMessage(QString("ccx timeout after %1s, killing pid=%2")
	                 .arg(_timeoutSec).arg(pid()));
	if (isRunning()) _proc->kill();
}

namespace {

QString firstExisting(const QStringList& candidates) {
	for (const QString& p : candidates) {
		QFileInfo fi(p);
		if (fi.exists() && fi.isFile()) {
			return QDir::cleanPath(fi.absoluteFilePath());
		}
	}
	return QString();
}

QStringList projectRelativeCandidates() {
	QStringList list;
	// QCoreApplication::applicationDirPath() 在没创建 app 实例时为空，回退到 cwd
	const QString appDir = QCoreApplication::applicationDirPath();
	const QString cwd    = QDir::currentPath();

	auto addLayout = [&](const QString& base) {
		if (base.isEmpty()) return;
		// 向上回溯几级，兼容 install 包、build/Release 输出目录、工程根三种路径布局
		for (int up = 0; up <= 4; ++up) {
			QString prefix = base;
			for (int i = 0; i < up; ++i) prefix += "/..";
			list << prefix + "/tools/calculix/ccx.exe";
		}
	};
	addLayout(appDir);
	addLayout(cwd);
	return list;
}

} // anonymous namespace

QString CCXSolverController::detectCcxPath() {
	// 1. 环境变量优先；设置了无效路径不 fallback
	const QByteArray env = qgetenv("CCX_PATH");
	if (!env.isEmpty()) {
		const QString envPath = QString::fromLocal8Bit(env);
		QFileInfo fi(envPath);
		if (fi.exists() && fi.isFile()) {
			return QDir::cleanPath(fi.absoluteFilePath());
		}
		return QString();
	}

	// 2. 项目相对路径
	return firstExisting(projectRelativeCandidates());
}

QString CCXSolverController::detectCcxPathDiagnostic() {
	QString out;
	const QByteArray env = qgetenv("CCX_PATH");

	if (!env.isEmpty()) {
		const QString envPath = QString::fromLocal8Bit(env);
		out += QString("CCX_PATH=%1\n").arg(envPath);
		QFileInfo fi(envPath);
		if (fi.exists() && fi.isFile()) {
			out += QString("  hit: %1").arg(QDir::cleanPath(fi.absoluteFilePath()));
			return out;
		}
		out += "  invalid (not a file) — refusing fallback\n";
		out += "  result: <not found>";
		return out;
	}

	out += "CCX_PATH=<unset>, scanning relative to applicationDirPath / cwd:\n";
	const QStringList candidates = projectRelativeCandidates();
	for (const QString& p : candidates) {
		QFileInfo fi(p);
		const bool ok = fi.exists() && fi.isFile();
		out += QString("  %1  %2\n").arg(ok ? "[OK]" : "[--]", p);
		if (ok) {
			out += QString("  hit: %1").arg(QDir::cleanPath(fi.absoluteFilePath()));
			return out;
		}
	}
	out += "  result: <not found>";
	return out;
}

} // namespace GearAutoOpt
