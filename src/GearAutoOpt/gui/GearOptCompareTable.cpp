#include "GearOptCompareTable.h"

#include <QDebug>
#include <QDir>
#include <QHeaderView>
#include <QSaveFile>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>

#include <algorithm>

namespace GearAutoOpt {

namespace {

const GearDesignPoint* findGen0Id0(const QVector<QList<GearDesignPoint>>& allPoints)
{
	if (allPoints.isEmpty()) return nullptr;
	for (const GearDesignPoint& dp : allPoints[0]) {
		if (dp.generation == 0 && dp.id == 0) return &dp;
	}
	return nullptr;
}

// 双目标非支配前沿（与 GearAutoOptManager::paretoFront 一致）。无 paretoRank 字段，等价于 NSGA-II 第一层前沿。
static bool dominatesForCompare(const GearDesignPoint& a,
                                const GearDesignPoint& b,
                                const Objectives& obj)
{
	const bool useDefault = !obj.minCpressMax && !obj.minSigmaMax && !obj.minMass;
	bool       allLe      = true;
	bool       anyStrict  = false;
	auto       consider   = [&](bool enabled, double av, double bv) {
		if (!enabled) return;
		if (bv > av) allLe = false;
		else if (bv < av) anyStrict = true;
	};
	consider(useDefault || obj.minCpressMax, a.cpressMax_MPa, b.cpressMax_MPa);
	consider(useDefault || obj.minSigmaMax, a.sigmaMax, b.sigmaMax);
	consider(!useDefault && obj.minMass, a.mass, b.mass);
	return allLe && anyStrict;
}

QList<GearDesignPoint> paretoFrontDone(const QVector<QList<GearDesignPoint>>& allPoints,
                                       const Objectives& obj)
{
	QList<GearDesignPoint> all;
	for (const auto& gen : allPoints) {
		for (const GearDesignPoint& dp : gen) {
			if (dp.status != PointStatus::Done || dp.cpressMax_MPa <= 0.0)
				continue;
			if (obj.minSigmaMax && dp.sigmaMax <= 0.0)
				continue;
			all.append(dp);
		}
	}
	QList<GearDesignPoint> pareto;
	for (const GearDesignPoint& a : all) {
		bool dominated = false;
		for (const GearDesignPoint& b : all) {
			if (dominatesForCompare(a, b, obj)) {
				dominated = true;
				break;
			}
		}
		if (!dominated) pareto.append(a);
	}
	return pareto;
}

bool sameGenId(const GearDesignPoint& a, const GearDesignPoint& b)
{
	return a.generation == b.generation && a.id == b.id;
}

static QString csvEsc(const QString& s)
{
	QString t = s;
	t.replace(QLatin1Char('"'), QStringLiteral("\"\""));
	return QStringLiteral("\"%1\"").arg(t);
}

static void setRow(QTableWidget* table, int row, const QStringList& cells)
{
	for (int c = 0; c < cells.size(); ++c) {
		auto* it = new QTableWidgetItem(cells[c]);
		it->setFlags(it->flags() & ~Qt::ItemIsEditable);
		table->setItem(row, c, it);
	}
}

} // namespace

QString exportOptimizationCompareTable(const GearOptConfig& cfg,
                                       const QVector<QList<GearDesignPoint>>& allPoints,
                                       const QString& runDirRoot,
                                       QTableWidget* table)
{
	const bool baseFound = cfg.useOptimizationBase;
	const GearDesignPoint* ev0 = findGen0Id0(allPoints);

	QList<GearDesignPoint> pareto = paretoFrontDone(allPoints, cfg.objectives);
	const int paretoFullCount = pareto.size();

	// 去掉与 gen0/id0 同一点，避免与 BASE 行重复；再按 cpress、σ 排序取前 5
	if (ev0) {
		QList<GearDesignPoint> filtered;
		for (const GearDesignPoint& p : pareto) {
			if (!sameGenId(p, *ev0)) filtered.append(p);
		}
		pareto = filtered;
	}
	std::sort(pareto.begin(), pareto.end(), [](const GearDesignPoint& a, const GearDesignPoint& b) {
		if (a.cpressMax_MPa != b.cpressMax_MPa) return a.cpressMax_MPa < b.cpressMax_MPa;
		return a.sigmaMax < b.sigmaMax;
	});
	while (pareto.size() > 5)
		pareto.removeLast();

	qDebug().noquote() << QStringLiteral("[GearOpt][CompareTable] baseFound=%1")
	                          .arg(baseFound ? 1 : 0);
	qDebug().noquote() << QStringLiteral("[GearOpt][CompareTable] paretoCount=%1").arg(paretoFullCount);

	QStringList headers = {
	    QString::fromUtf8("方案"), // 方案
	    QStringLiteral("module"),
	    QStringLiteral("z1"),
	    QStringLiteral("z2"),
	    QStringLiteral("alpha"),
	    QStringLiteral("x1"),
	    QStringLiteral("x2"),
	    QStringLiteral("ca1"),
	    QStringLiteral("lca1"),
	    QStringLiteral("ca2"),
	    QStringLiteral("lca2"),
	    QStringLiteral("width"),
	    QStringLiteral("hubRatio"),
	    QStringLiteral("cpressMax_MPa"),
	    QStringLiteral("sigmaMax_MPa"),
	    QStringLiteral("mass_kg"),
	    QStringLiteral("uMax_mm"),
	    QStringLiteral("status"),
	};

	QVector<QStringList> rows;
	// BASE：设计变量来自 optimizationBase（有局部优化基准时）；指标来自 gen0/id0 求解结果
	GearDesignPoint baseParams;
	if (cfg.useOptimizationBase)
		baseParams = cfg.optimizationBase;
	else if (ev0)
		baseParams = *ev0;
	{
		QStringList line;
		line << QStringLiteral("BASE");
		line << QString::number(baseParams.module, 'g', 12);
		line << QString::number(baseParams.z1);
		line << QString::number(baseParams.z2);
		line << QString::number(baseParams.alpha, 'g', 12);
		line << QString::number(baseParams.x1, 'g', 12);
		line << QString::number(baseParams.x2, 'g', 12);
		line << QString::number(baseParams.ca1, 'g', 12);
		line << QString::number(baseParams.lca1, 'g', 12);
		line << QString::number(baseParams.ca2, 'g', 12);
		line << QString::number(baseParams.lca2, 'g', 12);
		line << QString::number(baseParams.commonWidth, 'g', 12);
		line << QString::number(baseParams.hubRatio, 'g', 12);
		if (ev0) {
			line << (ev0->cpressMax_MPa > 0 ? QString::number(ev0->cpressMax_MPa, 'g', 12) : QString());
			line << (ev0->sigmaMax >= 0 ? QString::number(ev0->sigmaMax, 'g', 12) : QString());
			line << (ev0->mass >= 0 ? QString::number(ev0->mass, 'g', 12) : QString());
			line << (ev0->uMax >= 0 ? QString::number(ev0->uMax, 'g', 12) : QString());
			line << pointStatusToString(ev0->status);
		} else {
			line << QString() << QString() << QString() << QString() << QStringLiteral("N/A");
		}
		rows.append(line);
	}
	for (int i = 0; i < pareto.size(); ++i) {
		const GearDesignPoint& p = pareto[i];
		QStringList line;
		line << QStringLiteral("Pareto-%1").arg(i + 1);
		line << QString::number(p.module, 'g', 12);
		line << QString::number(p.z1);
		line << QString::number(p.z2);
		line << QString::number(p.alpha, 'g', 12);
		line << QString::number(p.x1, 'g', 12);
		line << QString::number(p.x2, 'g', 12);
		line << QString::number(p.ca1, 'g', 12);
		line << QString::number(p.lca1, 'g', 12);
		line << QString::number(p.ca2, 'g', 12);
		line << QString::number(p.lca2, 'g', 12);
		line << QString::number(p.commonWidth, 'g', 12);
		line << QString::number(p.hubRatio, 'g', 12);
		line << (p.cpressMax_MPa > 0 ? QString::number(p.cpressMax_MPa, 'g', 12) : QString());
		line << (p.sigmaMax >= 0 ? QString::number(p.sigmaMax, 'g', 12) : QString());
		line << (p.mass >= 0 ? QString::number(p.mass, 'g', 12) : QString());
		line << (p.uMax >= 0 ? QString::number(p.uMax, 'g', 12) : QString());
		line << pointStatusToString(p.status);
		rows.append(line);
	}

	if (table) {
		table->clear();
		table->setColumnCount(headers.size());
		table->setHorizontalHeaderLabels(headers);
		table->setRowCount(rows.size());
		table->verticalHeader()->setVisible(false);
		table->horizontalHeader()->setStretchLastSection(true);
		for (int r = 0; r < rows.size(); ++r)
			setRow(table, r, rows[r]);
		table->resizeColumnsToContents();
	}

	QString savedPath;
	if (!runDirRoot.isEmpty()) {
		QDir root(runDirRoot);
		if (root.mkpath(QStringLiteral("result"))) {
			const QString path = QDir::cleanPath(root.filePath(QStringLiteral("result/optimization_compare.csv")));
			QSaveFile f(path);
			if (f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
				QTextStream ts(&f);
				ts.setCodec("UTF-8");
				ts << headers.join(QLatin1Char(',')) << QLatin1Char('\n');
				for (const QStringList& line : rows) {
					QStringList out;
					for (int i = 0; i < line.size(); ++i) {
						const QString& cell = line[i];
						if (i == 0 || i == line.size() - 1)
							out << csvEsc(cell);
						else
							out << cell;
					}
					ts << out.join(QLatin1Char(',')) << QLatin1Char('\n');
				}
				if (f.commit()) savedPath = path;
			}
		}
	}

	qDebug().noquote() << QStringLiteral("[GearOpt][CompareTable] saved=%1")
	                          .arg(savedPath.isEmpty() ? QStringLiteral("0") : savedPath);

	return savedPath;
}

} // namespace GearAutoOpt
