#include "GearAutoOpt/db/GearOptResultDatabase.h"

#include <QCoreApplication>
#include <QTextStream>

int main(int argc, char* argv[])
{
	QCoreApplication app(argc, argv);
	QTextStream out(stdout);
	QTextStream err(stderr);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
	out.setCodec("UTF-8");
	err.setCodec("UTF-8");
#endif

	const QStringList args = QCoreApplication::arguments();
	if (args.size() < 2) {
		err << "Usage: GearOptBackfill.exe <GearOptResults.db> [binCount]\n";
		return 2;
	}

	bool ok = false;
	int binCount = args.size() >= 3 ? args.at(2).toInt(&ok) : 11;
	if (args.size() < 3 || !ok || binCount <= 0)
		binCount = 11;

	const QString dbPath = args.at(1);
	auto& db = GearAutoOpt::GearOptResultDatabase::global();
	if (!db.openDatabase(dbPath)) {
		err << "open database failed: " << dbPath << '\n';
		return 1;
	}

	const GearAutoOpt::CpressDistributionBackfillStats stats =
	    db.backfillMissingCpressDistributionMetricsDetailed(binCount);
	db.closeDatabase();

	out << "total_missing=" << stats.totalMissing << '\n';
	out << "updated_count=" << stats.updatedCount << '\n';
	out << "skip_missing_frd=" << stats.skipMissingFrd << '\n';
	out << "failed_parse_count=" << stats.failedParseCount << '\n';
	out.flush();

	return stats.failedParseCount > 0 ? 3 : 0;
}
