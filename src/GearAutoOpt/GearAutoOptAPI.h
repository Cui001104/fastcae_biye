#ifndef _GEARAUTOOPTAPI_H_
#define _GEARAUTOOPTAPI_H_

#include <QtCore/QtGlobal>

#if defined(GEARAUTOOPT_API)
#define GEARAUTOOPTAPI Q_DECL_EXPORT
#else
#define GEARAUTOOPTAPI Q_DECL_IMPORT
#endif

namespace GearAutoOpt {

GEARAUTOOPTAPI const char* moduleName();
GEARAUTOOPTAPI int offlineBackfillCpressDistributionMetrics(const char* dbPath, int binCount = 11);

} // namespace GearAutoOpt

#endif
