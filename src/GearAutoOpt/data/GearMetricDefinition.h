#ifndef _GEARAUTOOPT_GEAR_METRIC_DEFINITION_H_
#define _GEARAUTOOPT_GEAR_METRIC_DEFINITION_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QVector>
#include <QString>

namespace GearAutoOpt {

/// 平台通用性能指标定义（metric_definitions 表）。
struct GEARAUTOOPTAPI MetricDefinition {
	QString metricName;
	QString displayName;
	QString unit;
	QString source;
	bool    enabled{false};
	bool    implemented{false};
	QString direction; ///< minimize | maximize
	QString note;
};

/// NSGA-II 通用目标编号与物理指标的映射（objective_mapping 表）。
struct GEARAUTOOPTAPI ObjectiveMapping {
	QString objKey;
	QString metricName;
	QString displayName;
	QString direction; ///< minimize | maximize
	int     objOrder{0};
	bool    enabled{false};
	QString note;
};

/// 设计变量定义（design_variable_definitions 表）。
struct GEARAUTOOPTAPI DesignVariableDefinition {
	QString varName;
	QString displayName;
	QString unit;
	double  lowerBound{0.0};
	double  upperBound{1.0};
	bool    enabled{false};
	QString partType;
	QString note;
};

QVector<MetricDefinition>           defaultMetricDefinitions();
QVector<ObjectiveMapping>           defaultObjectiveMappings();
QVector<DesignVariableDefinition>   defaultDesignVariableDefinitions();

QString directionDisplayText(const QString& direction);

} // namespace GearAutoOpt

#endif
