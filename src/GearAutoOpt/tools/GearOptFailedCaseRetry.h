#ifndef _GEARAUTOOPT_GEAR_OPT_FAILED_CASE_RETRY_H_
#define _GEARAUTOOPT_GEAR_OPT_FAILED_CASE_RETRY_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"

#include <functional>
#include <QString>

namespace GearAutoOpt {

struct FailedCaseRetryOptions {
	GearOptConfig cfg;
	QString       runDir;
	double        fixedMeshSizeMm{0.0};
	double        fixedRootMeshSizeMm{0.0};
	int           fixedZLayers{0};
	bool          runMeshAuto{true};
	QString       baseCaseHash;
	double        fixedCommonWidthMm{-1.0};
};

class GEARAUTOOPTAPI GearOptFailedCaseRetry {
public:
	static FailedCaseRetryStats run(const FailedCaseRetryOptions& opt,
	                                const std::function<void(const QString&)>& logFn);
};

} // namespace GearAutoOpt

#endif
