#include "GearSurrogateMetrics.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <algorithm>
#include <cmath>

namespace GearAutoOpt {

namespace {

QString csvNumber(double v)
{
	if (v < 0.0 || !std::isfinite(v))
		return QString();
	return QString::number(v, 'g', 12);
}

QString csvField(const QString& s)
{
	if (!s.contains(QLatin1Char(',')) && !s.contains(QLatin1Char('"')))
		return s;
	QString escaped = s;
	escaped.replace(QLatin1Char('"'), QStringLiteral("\"\""));
	return QStringLiteral("\"%1\"").arg(escaped);
}

bool appendCsvRow(const QString& runDir,
                  const QString& fileName,
                  const QString& header,
                  const QString& line)
{
	const QString dir = runDir.trimmed();
	if (dir.isEmpty())
		return false;

	QDir().mkpath(dir);
	const QString path = QDir(dir).filePath(fileName);
	const bool writeHeader = !QFileInfo::exists(path) || QFileInfo(path).size() == 0;

	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
		return false;

	QTextStream out(&file);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
	out.setEncoding(QStringConverter::Utf8);
#else
	out.setCodec("UTF-8");
#endif

	if (writeHeader)
		out << header << '\n';
	out << line << '\n';
	out.flush();
	file.flush();
	return true;
}

} // namespace

double computeRmae(const QVector<double>& predValues, const QVector<double>& trueValues)
{
	const int n = std::min(predValues.size(), trueValues.size());
	if (n <= 0)
		return -1.0;

	constexpr double eps = 1e-12;
	double sum = 0.0;
	for (int i = 0; i < n; ++i)
		sum += std::abs(predValues[i] - trueValues[i]) / std::max(std::abs(trueValues[i]), eps);
	return sum / static_cast<double>(n);
}

bool appendSurrogateHvCsv(const QString& runDir, const SurrogateHvRow& row)
{
	const QString line = QStringLiteral("%1,%2,%3")
	                         .arg(row.round)
	                         .arg(csvNumber(row.predHv))
	                         .arg(row.predPareto);
	return appendCsvRow(runDir,
	                    QStringLiteral("surrogate_metrics.csv"),
	                    QStringLiteral("Round,PredHV,PredPareto"),
	                    line);
}

bool appendCcxHvCsv(const QString& runDir, const CcxHvRow& row)
{
	const QString line = QStringLiteral("%1,%2,%3")
	                         .arg(row.round)
	                         .arg(row.ccxSamples)
	                         .arg(csvNumber(row.ccxHv));
	return appendCsvRow(runDir,
	                    QStringLiteral("ccx_metrics.csv"),
	                    QStringLiteral("Round,CCXSamples,CCXHV"),
	                    line);
}

bool appendSurrogatePredictionsCsv(const QString& runDir, const SurrogatePredictionRow& row)
{
	const QString line = QStringLiteral(
	    "%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11")
	                         .arg(row.generation)
	                         .arg(row.individualId)
	                         .arg(csvNumber(row.x1))
	                         .arg(csvNumber(row.x2))
	                         .arg(csvNumber(row.ca1))
	                         .arg(csvNumber(row.lca1))
	                         .arg(csvNumber(row.ca2))
	                         .arg(csvNumber(row.lca2))
	                         .arg(csvNumber(row.hubRatio))
	                         .arg(csvNumber(row.predCpressMax_MPa))
	                         .arg(csvNumber(row.predEdgeLoadRatio));
	return appendCsvRow(runDir,
	                    QStringLiteral("surrogate_predictions.csv"),
	                    QStringLiteral(
	                        "generation,individual_id,x1,x2,ca1,lca1,ca2,lca2,hubRatio,"
	                        "pred_cpressMax_MPa,pred_edgeLoadRatio"),
	                    line);
}

bool appendSurrogateRmaeCsv(const QString& runDir, const SurrogateRmaeRow& row)
{
	const QDateTime ts =
	    row.createdAt.isValid() ? row.createdAt : QDateTime::currentDateTime();
	const QString line = QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11")
	                         .arg(row.round)
	                         .arg(row.sampleCount)
	                         .arg(row.infillSelected)
	                         .arg(row.newCcxCount)
	                         .arg(row.reusedCacheCount)
	                         .arg(csvNumber(row.rmaeCpressAll))
	                         .arg(csvNumber(row.rmaeCpressNew))
	                         .arg(csvNumber(row.rmaeSigma))
	                         .arg(csvNumber(row.rmaeUmax))
	                         .arg(csvField(row.stopReason))
	                         .arg(csvField(ts.toString(Qt::ISODate)));
	return appendCsvRow(runDir,
	                    QStringLiteral("surrogate_rmae.csv"),
	                    QStringLiteral(
	                        "round,sample_count,infill_selected,new_ccx,reused_cache,"
	                        "rmae_cpress_all,rmae_cpress_new,rmae_sigma,rmae_umax,"
	                        "stop_reason,created_at"),
	                    line);
}

bool appendSurrogateInfillValidationCsv(const QString& runDir,
                                        const QVector<SurrogateInfillValidationRow>& rows)
{
	if (rows.isEmpty())
		return true;

	const QString header = QStringLiteral(
	    "round,id,case_hash,"
	    "pred_cpressMax_MPa,true_cpressMax_MPa,cpress_error_percent,"
	    "pred_edgeLoadRatio,true_edgeLoadRatio,edgeLoadRatio_error_percent,"
	    "true_sigmaMax_MPa,true_uMax_mm,true_mass_kg,true_cpressCV");
	bool ok = true;
	for (const SurrogateInfillValidationRow& row : rows) {
		const QString line = QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13")
		                         .arg(row.round)
		                         .arg(row.id)
		                         .arg(csvField(row.caseHash))
		                         .arg(csvNumber(row.predCpressMax_MPa))
		                         .arg(csvNumber(row.trueCpressMax_MPa))
		                         .arg(csvNumber(row.cpressErrorPercent))
		                         .arg(csvNumber(row.predEdgeLoadRatio))
		                         .arg(csvNumber(row.trueEdgeLoadRatio))
		                         .arg(csvNumber(row.edgeLoadRatioErrorPercent))
		                         .arg(csvNumber(row.trueSigmaMax_MPa))
		                         .arg(csvNumber(row.trueUMax_mm))
		                         .arg(csvNumber(row.trueMass_kg))
		                         .arg(csvNumber(row.trueCpressCV));
		ok = appendCsvRow(runDir,
		                  QStringLiteral("pred_vs_true.csv"),
		                  header,
		                  line) && ok;
	}
	return ok;
}

} // namespace GearAutoOpt
