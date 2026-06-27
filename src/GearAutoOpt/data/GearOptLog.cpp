#include "GearOptLog.h"

#include <QFile>
#include <QTextStream>

namespace GearAutoOpt {

GearLogLevel GearOptLog::s_level = GearLogLevel::Normal;

void GearOptLog::setLevel(GearLogLevel level) {
	s_level = level;
}

GearLogLevel GearOptLog::level() {
	return s_level;
}

bool GearOptLog::isDebug() {
	return isDebugLogLevel(s_level);
}

bool GearOptLog::appendLine(const QString& filePath, const QString& line) {
	if (filePath.isEmpty())
		return false;
	QFile f(filePath);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts << line << QLatin1Char('\n');
	return true;
}

bool GearOptLog::writeText(const QString& filePath, const QString& text) {
	if (filePath.isEmpty())
		return false;
	QFile f(filePath);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts << text;
	return true;
}

} // namespace GearAutoOpt
