#include "GearMetricDefinition.h"

namespace GearAutoOpt {

QString directionDisplayText(const QString& direction)
{
	if (direction == QStringLiteral("maximize"))
		return QString::fromUtf8("最大化");
	return QString::fromUtf8("最小化");
}

QVector<MetricDefinition> defaultMetricDefinitions()
{
	QVector<MetricDefinition> defs;
	auto add = [&](const char* name, const char* display, const char* unit,
	               const char* source, bool enabled, bool implemented,
	               const char* direction, const char* note) {
		MetricDefinition d;
		d.metricName  = QString::fromLatin1(name);
		d.displayName = QString::fromUtf8(display);
		d.unit        = QString::fromLatin1(unit);
		d.source      = QString::fromLatin1(source);
		d.enabled     = enabled;
		d.implemented = implemented;
		d.direction   = QString::fromLatin1(direction);
		d.note        = QString::fromUtf8(note);
		defs.append(d);
	};

	add("sigmaMax_MPa", "最大等效应力", "MPa", "FRD/STRESS",
	    true, true, "minimize", "当前齿轮优化已支持");
	add("uMax_mm", "最大位移", "mm", "FRD/DISP",
	    false, true, "minimize", "当前结果解析已支持");
	add("mass_kg", "质量", "kg", "Geometry/PostProcess",
	    false, true, "minimize", "当前结果记录已支持");
	add("cpressMax_MPa", "最大接触压力", "MPa", "FRD/CONTACT",
	    true, true, "minimize", "当前齿轮接触分析已支持");
	add("mode1Freq_Hz", "一阶固有频率", "Hz", "DAT/FREQUENCY",
	    false, false, "maximize", "预留指标，当前版本暂不参与优化");
	add("tempMax_C", "最高温度", "C", "FRD/TEMP",
	    false, false, "minimize", "预留指标，当前版本暂不参与优化");
	add("fatigueLife", "疲劳寿命", "cycle", "PostProcess",
	    false, false, "maximize", "预留指标，当前版本暂不参与优化");

	return defs;
}

QVector<ObjectiveMapping> defaultObjectiveMappings()
{
	QVector<ObjectiveMapping> maps;
	auto add = [&](const char* key, const char* metric, const char* display,
	               const char* direction, int order, bool enabled, const char* note) {
		ObjectiveMapping m;
		m.objKey      = QString::fromLatin1(key);
		m.metricName  = QString::fromLatin1(metric);
		m.displayName = QString::fromUtf8(display);
		m.direction   = QString::fromLatin1(direction);
		m.objOrder    = order;
		m.enabled     = enabled;
		m.note        = QString::fromUtf8(note);
		maps.append(m);
	};

	add("obj1", "sigmaMax_MPa", "最大等效应力", "minimize", 1, true, "当前默认目标");
	add("obj2", "cpressMax_MPa", "最大接触压力", "minimize", 2, true, "当前默认目标");
	add("obj3", "mass_kg", "质量", "minimize", 3, false, "预留目标");
	add("obj4", "uMax_mm", "最大位移", "minimize", 4, false, "预留目标");

	return maps;
}

QVector<DesignVariableDefinition> defaultDesignVariableDefinitions()
{
	QVector<DesignVariableDefinition> vars;
	auto add = [&](const char* name, const char* display, const char* unit,
	               double lo, double hi, bool enabled, const char* part,
	               const char* note = "") {
		DesignVariableDefinition v;
		v.varName     = QString::fromLatin1(name);
		v.displayName = QString::fromUtf8(display);
		v.unit        = QString::fromLatin1(unit);
		v.lowerBound  = lo;
		v.upperBound  = hi;
		v.enabled     = enabled;
		v.partType    = QString::fromLatin1(part);
		v.note        = QString::fromUtf8(note);
		vars.append(v);
	};

	add("x1", "主动轮变位系数", "-", -0.5, 0.5, true, "gear", "齿轮几何参数");
	add("x2", "从动轮变位系数", "-", -0.5, 0.5, true, "gear");
	add("ca1", "主动轮齿廓修形量", "mm", 0.00, 0.05, true, "gear");
	add("lca1", "主动轮修形长度", "mm", 0.10, 0.80, true, "gear");
	add("ca2", "从动轮齿廓修形量", "mm", 0.00, 0.05, true, "gear");
	add("lca2", "从动轮修形长度", "mm", 0.10, 0.80, true, "gear");
	add("commonWidth", "齿宽", "mm", 8.00, 14.00, true, "gear");
	add("hubRatio", "轮毂比", "-", 0.30, 0.60, true, "gear");

	return vars;
}

} // namespace GearAutoOpt
