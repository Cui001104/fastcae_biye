// UTF-8 BOM
#ifndef _GEARAUTOOPT_OPTIMIZATION_CONFIG_DIALOG_H_
#define _GEARAUTOOPT_OPTIMIZATION_CONFIG_DIALOG_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearMetricDefinition.h"

#include <QDialog>
#include <QVector>

class QTabWidget;
class QTableWidget;
class QLabel;

namespace GearAutoOpt {

/// 平台化优化配置（三页签：指标库 / 目标映射 / 设计变量），不接入 NSGA-II。
class GEARAUTOOPTAPI OptimizationConfigDialog : public QDialog {
	Q_OBJECT
public:
	explicit OptimizationConfigDialog(QWidget* parent = nullptr);

private slots:
	void onMetricCellChanged(int row, int column);
	void onObjectiveCellChanged(int row, int column);
	void onSave();

private:
	void buildUi();
	void loadFromDatabase();
	void populateMetricTab();
	void populateObjectiveTab();
	void populateDesignVarTab();

	QVector<MetricDefinition>         collectMetrics() const;
	QVector<ObjectiveMapping>         collectObjectiveMappings() const;
	QVector<DesignVariableDefinition> collectDesignVariables() const;

	QStringList metricNameList() const;

	QTabWidget*   _tabs{};
	QTableWidget* _metricTable{};
	QTableWidget* _objectiveTable{};
	QTableWidget* _designVarTable{};

	QVector<MetricDefinition>         _metrics;
	QVector<ObjectiveMapping>         _objectives;
	QVector<DesignVariableDefinition> _designVars;
};

} // namespace GearAutoOpt

#endif
