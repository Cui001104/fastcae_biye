#include "FailedCaseRetryRunner.h"

namespace GearAutoOpt {

FailedCaseRetryRunner::FailedCaseRetryRunner(const FailedCaseRetryOptions& opt, QObject* parent)
    : QObject(parent)
    , _opt(opt)
{}

void FailedCaseRetryRunner::execute()
{
	const FailedCaseRetryStats stats = GearOptFailedCaseRetry::run(
	    _opt, [this](const QString& msg) { emit log(msg); });
	emit finished(stats);
}

} // namespace GearAutoOpt
