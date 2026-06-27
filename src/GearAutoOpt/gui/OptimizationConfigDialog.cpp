// UTF-8 BOM
#include "OptimizationConfigDialog.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"

#include <QAbstractItemView>
#include <QColor>
#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace GearAutoOpt {

namespace {

constexpr int kMetColEnabled     = 0;
constexpr int kMetColName        = 1;
constexpr int kMetColDisplay     = 2;
constexpr int kMetColUnit        = 3;
constexpr int kMetColSource      = 4;
constexpr int kMetColImplemented = 5;
constexpr int kMetColDirection   = 6;
constexpr int kMetColNote        = 7;

constexpr int kObjColEnabled  = 0;
constexpr int kObjColKey      = 1;
constexpr int kObjColMetric   = 2;
constexpr int kObjColDisplay  = 3;
constexpr int kObjColDirection = 4;
constexpr int kObjColOrder    = 5;
constexpr int kObjColNote     = 6;

constexpr int kDvColEnabled  = 0;
constexpr int kDvColName     = 1;
constexpr int kDvColDisplay  = 2;
constexpr int kDvColUnit     = 3;
constexpr int kDvColLower    = 4;
constexpr int kDvColUpper    = 5;
constexpr int kDvColPart     = 6;
constexpr int kDvColNote     = 7;

QTableWidgetItem* roItem(const QString& text, bool gray = false)
{
	auto* item = new QTableWidgetItem(text);
	item->setFlags(item->flags() & ~Qt::ItemIsEditable);
	if (gray)
		item->setForeground(QColor(140, 140, 140));
	return item;
}

QTableWidgetItem* checkItem(bool checked, bool gray = false)
{
	auto* item = new QTableWidgetItem;
	item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	item->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
	if (gray)
		item->setForeground(QColor(140, 140, 140));
	return item;
}

void setupTable(QTableWidget* t)
{
	t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
	t->horizontalHeader()->setStretchLastSection(true);
	t->setSelectionBehavior(QAbstractItemView::SelectRows);
	t->setAlternatingRowColors(true);
}

} // namespace

OptimizationConfigDialog::OptimizationConfigDialog(QWidget* parent)
    : QDialog(parent)
{
	setWindowTitle(QString::fromUtf8("优化配置"));
	setMinimumSize(1020, 520);
	buildUi();
	loadFromDatabase();
}

void OptimizationConfigDialog::buildUi()
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(12, 12, 12, 12);
	layout->setSpacing(10);

	_tabs = new QTabWidget(this);

	// ---- 页签一：性能指标库 ----
	{
		auto* page = new QWidget(this);
		auto* pageLayout = new QVBoxLayout(page);
		_metricTable = new QTableWidget(0, 8, page);
		_metricTable->setHorizontalHeaderLabels({
		    QString::fromUtf8("是否启用"),
		    QStringLiteral("metric_name"),
		    QString::fromUtf8("显示名称"),
		    QString::fromUtf8("单位"),
		    QString::fromUtf8("来源"),
		    QString::fromUtf8("是否已实现"),
		    QString::fromUtf8("默认方向"),
		    QString::fromUtf8("备注"),
		});
		_metricTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
		setupTable(_metricTable);
		pageLayout->addWidget(_metricTable);
		_tabs->addTab(page, QString::fromUtf8("性能指标库"));
	}

	// ---- 页签二：优化目标映射 ----
	{
		auto* page = new QWidget(this);
		auto* pageLayout = new QVBoxLayout(page);
		_objectiveTable = new QTableWidget(0, 7, page);
		_objectiveTable->setHorizontalHeaderLabels({
		    QString::fromUtf8("是否启用"),
		    QStringLiteral("obj_key"),
		    QStringLiteral("metric_name"),
		    QString::fromUtf8("显示名称"),
		    QString::fromUtf8("优化方向"),
		    QString::fromUtf8("目标顺序"),
		    QString::fromUtf8("备注"),
		});
		_objectiveTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
		setupTable(_objectiveTable);
		pageLayout->addWidget(_objectiveTable, 1);
		auto* hint = new QLabel(
		    QString::fromUtf8("当前版本优化流程仍采用已验证的齿轮默认目标；"
		                     "本页用于目标编号与物理性能指标的映射关系预留。"),
		    page);
		hint->setWordWrap(true);
		hint->setStyleSheet(QStringLiteral("color: #555;"));
		pageLayout->addWidget(hint);
		_tabs->addTab(page, QString::fromUtf8("优化目标映射"));
	}

	// ---- 页签三：设计变量配置 ----
	{
		auto* page = new QWidget(this);
		auto* pageLayout = new QVBoxLayout(page);
		_designVarTable = new QTableWidget(0, 8, page);
		_designVarTable->setHorizontalHeaderLabels({
		    QString::fromUtf8("是否启用"),
		    QStringLiteral("var_name"),
		    QString::fromUtf8("display_name"),
		    QString::fromUtf8("unit"),
		    QStringLiteral("lower_bound"),
		    QStringLiteral("upper_bound"),
		    QStringLiteral("part_type"),
		    QString::fromUtf8("备注"),
		});
		setupTable(_designVarTable);
		pageLayout->addWidget(_designVarTable, 1);
		auto* hint = new QLabel(
		    QString::fromUtf8("设计变量用于描述优化算法可调整的输入参数及其上下限。"
		                     "当前版本仍采用原有齿轮优化参数范围，本页作为后续多零部件扩展接口。"),
		    page);
		hint->setWordWrap(true);
		hint->setStyleSheet(QStringLiteral("color: #555;"));
		pageLayout->addWidget(hint);
		_tabs->addTab(page, QString::fromUtf8("设计变量配置"));
	}

	layout->addWidget(_tabs, 1);

	auto* btnRow = new QHBoxLayout;
	auto* btnSave  = new QPushButton(QString::fromUtf8("保存"), this);
	auto* btnClose = new QPushButton(QString::fromUtf8("关闭"), this);
	btnRow->addStretch();
	btnRow->addWidget(btnSave);
	btnRow->addWidget(btnClose);
	layout->addLayout(btnRow);

	connect(_metricTable, &QTableWidget::cellChanged,
	        this, &OptimizationConfigDialog::onMetricCellChanged);
	connect(_objectiveTable, &QTableWidget::cellChanged,
	        this, &OptimizationConfigDialog::onObjectiveCellChanged);
	connect(btnSave, &QPushButton::clicked, this, &OptimizationConfigDialog::onSave);
	connect(btnClose, &QPushButton::clicked, this, &QDialog::reject);
}

void OptimizationConfigDialog::loadFromDatabase()
{
	auto& db = GearOptResultDatabase::global();
	if (!db.isOpen())
		db.openDatabase();

	if (!db.ensureOptimizationConfigTables())
		QMessageBox::warning(this, QString::fromUtf8("数据库"),
		                     QString::fromUtf8("无法初始化优化配置表。"));

	_metrics    = db.loadMetricDefinitions();
	_objectives = db.loadObjectiveMappings();
	_designVars = db.loadDesignVariableDefinitions();

	populateMetricTab();
	populateObjectiveTab();
	populateDesignVarTab();
}

QStringList OptimizationConfigDialog::metricNameList() const
{
	QStringList names;
	for (const auto& m : _metrics)
		names.append(m.metricName);
	return names;
}

void OptimizationConfigDialog::populateMetricTab()
{
	_metricTable->blockSignals(true);
	_metricTable->setRowCount(_metrics.size());

	for (int row = 0; row < _metrics.size(); ++row) {
		const MetricDefinition& d = _metrics.at(row);
		const bool gray = !d.implemented;

		_metricTable->setItem(row, kMetColEnabled,
		                      checkItem(d.enabled && d.implemented, gray));
		_metricTable->setItem(row, kMetColName, roItem(d.metricName, gray));
		_metricTable->setItem(row, kMetColDisplay, roItem(d.displayName, gray));
		_metricTable->setItem(row, kMetColUnit, roItem(d.unit, gray));
		_metricTable->setItem(row, kMetColSource, roItem(d.source, gray));
		_metricTable->setItem(row, kMetColImplemented,
		                      roItem(d.implemented ? QString::fromUtf8("是")
		                                           : QString::fromUtf8("否（预留）"),
		                             gray));
		_metricTable->setItem(row, kMetColDirection,
		                      roItem(directionDisplayText(d.direction), gray));
		_metricTable->setItem(row, kMetColNote, roItem(d.note, gray));
	}
	_metricTable->blockSignals(false);
}

void OptimizationConfigDialog::populateObjectiveTab()
{
	_objectiveTable->blockSignals(true);
	_objectiveTable->setRowCount(_objectives.size());
	const QStringList names = metricNameList();

	for (int row = 0; row < _objectives.size(); ++row) {
		const ObjectiveMapping& m = _objectives.at(row);

		_objectiveTable->setItem(row, kObjColEnabled, checkItem(m.enabled));
		_objectiveTable->setItem(row, kObjColKey, roItem(m.objKey));
		_objectiveTable->setItem(row, kObjColDisplay, roItem(m.displayName));
		_objectiveTable->setItem(row, kObjColDirection,
		                         roItem(directionDisplayText(m.direction)));
		_objectiveTable->setItem(row, kObjColOrder,
		                         roItem(QString::number(m.objOrder)));
		_objectiveTable->setItem(row, kObjColNote, roItem(m.note));

		auto* combo = new QComboBox(_objectiveTable);
		combo->addItems(names);
		const int idx = combo->findText(m.metricName);
		if (idx >= 0)
			combo->setCurrentIndex(idx);
		_objectiveTable->setCellWidget(row, kObjColMetric, combo);
	}
	_objectiveTable->blockSignals(false);
}

void OptimizationConfigDialog::populateDesignVarTab()
{
	_designVarTable->blockSignals(true);
	_designVarTable->setRowCount(_designVars.size());

	for (int row = 0; row < _designVars.size(); ++row) {
		const DesignVariableDefinition& v = _designVars.at(row);

		_designVarTable->setItem(row, kDvColEnabled, checkItem(v.enabled));
		_designVarTable->setItem(row, kDvColName, roItem(v.varName));
		_designVarTable->setItem(row, kDvColDisplay, roItem(v.displayName));
		_designVarTable->setItem(row, kDvColUnit, roItem(v.unit));

		auto* lo = new QTableWidgetItem(QString::number(v.lowerBound, 'f', 2));
		auto* hi = new QTableWidgetItem(QString::number(v.upperBound, 'f', 2));
		lo->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
		hi->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
		_designVarTable->setItem(row, kDvColLower, lo);
		_designVarTable->setItem(row, kDvColUpper, hi);

		_designVarTable->setItem(row, kDvColPart, roItem(v.partType));
		_designVarTable->setItem(row, kDvColNote, roItem(v.note));
	}
	_designVarTable->blockSignals(false);
}

void OptimizationConfigDialog::onMetricCellChanged(int row, int column)
{
	if (column != kMetColEnabled || row < 0 || row >= _metrics.size())
		return;

	const MetricDefinition& d = _metrics.at(row);
	if (d.implemented)
		return;

	auto* item = _metricTable->item(row, kMetColEnabled);
	if (!item || item->checkState() != Qt::Checked)
		return;

	_metricTable->blockSignals(true);
	item->setCheckState(Qt::Unchecked);
	_metricTable->blockSignals(false);
	QMessageBox::information(this, QString::fromUtf8("提示"),
	                         QString::fromUtf8("该指标为预留指标，当前版本暂不参与优化。"));
}

void OptimizationConfigDialog::onObjectiveCellChanged(int /*row*/, int /*column*/)
{
	// 预留：目标映射页签当前仅展示与保存，无额外校验
}

QVector<MetricDefinition> OptimizationConfigDialog::collectMetrics() const
{
	QVector<MetricDefinition> out;
	out.reserve(_metrics.size());

	for (int row = 0; row < _metrics.size(); ++row) {
		MetricDefinition d = _metrics.at(row);
		if (auto* chk = _metricTable->item(row, kMetColEnabled))
			d.enabled = d.implemented && chk->checkState() == Qt::Checked;
		out.append(d);
	}
	return out;
}

QVector<ObjectiveMapping> OptimizationConfigDialog::collectObjectiveMappings() const
{
	QVector<ObjectiveMapping> out;
	out.reserve(_objectives.size());

	for (int row = 0; row < _objectives.size(); ++row) {
		ObjectiveMapping m = _objectives.at(row);
		if (auto* chk = _objectiveTable->item(row, kObjColEnabled))
			m.enabled = chk->checkState() == Qt::Checked;
		if (auto* combo = qobject_cast<QComboBox*>(
		        _objectiveTable->cellWidget(row, kObjColMetric)))
			m.metricName = combo->currentText();
		out.append(m);
	}
	return out;
}

QVector<DesignVariableDefinition> OptimizationConfigDialog::collectDesignVariables() const
{
	QVector<DesignVariableDefinition> out;
	out.reserve(_designVars.size());

	for (int row = 0; row < _designVars.size(); ++row) {
		DesignVariableDefinition v = _designVars.at(row);
		if (auto* chk = _designVarTable->item(row, kDvColEnabled))
			v.enabled = chk->checkState() == Qt::Checked;
		if (auto* lo = _designVarTable->item(row, kDvColLower))
			v.lowerBound = lo->text().toDouble();
		if (auto* hi = _designVarTable->item(row, kDvColUpper))
			v.upperBound = hi->text().toDouble();
		out.append(v);
	}
	return out;
}

void OptimizationConfigDialog::onSave()
{
	auto& db = GearOptResultDatabase::global();

	const QVector<MetricDefinition> metrics = collectMetrics();
	const QVector<ObjectiveMapping> objs    = collectObjectiveMappings();
	const QVector<DesignVariableDefinition> dvs = collectDesignVariables();

	if (!db.saveMetricDefinitions(metrics)
	    || !db.saveObjectiveMappings(objs)
	    || !db.saveDesignVariableDefinitions(dvs)) {
		QMessageBox::warning(this, QString::fromUtf8("保存失败"),
		                     QString::fromUtf8("无法写入优化配置表。"));
		return;
	}

	_metrics    = metrics;
	_objectives = objs;
	_designVars = dvs;

	QMessageBox::information(this, QString::fromUtf8("已保存"),
	                         QString::fromUtf8("优化配置已保存，当前优化流程不受影响。"));
	accept();
}

} // namespace GearAutoOpt
