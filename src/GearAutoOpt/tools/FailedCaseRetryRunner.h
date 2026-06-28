#ifndef _GEARAUTOOPT_FAILED_CASE_RETRY_RUNNER_H_
#define _GEARAUTOOPT_FAILED_CASE_RETRY_RUNNER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/tools/GearOptFailedCaseRetry.h"

#include <QObject>

namespace GearAutoOpt {

class GEARAUTOOPTAPI FailedCaseRetryRunner : public QObject {
	Q_OBJECT
public:
	explicit FailedCaseRetryRunner(const FailedCaseRetryOptions& opt, QObject* parent = nullptr);

public slots:
	void execute();

signals:
	void log(const QString& msg);
	void finished(const FailedCaseRetryStats& stats);

private:
	FailedCaseRetryOptions _opt;
};

} // namespace GearAutoOpt

#endif
