// GearAutoOpt module entry point.
// Currently a placeholder. Real classes added incrementally in subsequent tasks

#include "GearAutoOptAPI.h"

#include "GearAutoOpt/db/GearOptResultDatabase.h"

#include <QDebug>
#include <QString>

namespace GearAutoOpt {

GEARAUTOOPTAPI const char* moduleName() {
	return "GearAutoOpt";
}

GEARAUTOOPTAPI int offlineBackfillCpressDistributionMetrics(const char* dbPath, int binCount)
{
	const QString path = QString::fromLocal8Bit(dbPath ? dbPath : "");
	if (path.trimmed().isEmpty()) {
		qWarning().noquote() << QStringLiteral("[GearOpt][OfflineBackfill] empty database path");
		return 0;
	}

	auto& db = GearOptResultDatabase::global();
	if (!db.openDatabase(path)) {
		qWarning().noquote() << QStringLiteral("[GearOpt][OfflineBackfill] open database failed: %1")
		                        .arg(path);
		return 0;
	}

	const CpressDistributionBackfillStats stats =
	    db.backfillMissingCpressDistributionMetricsDetailed(binCount > 0 ? binCount : 11);
	db.closeDatabase();
	return stats.updatedCount;
}

} // namespace GearAutoOpt
