#ifndef _GEARAUTOOPT_GEAR_OPT_LOG_H_
#define _GEARAUTOOPT_GEAR_OPT_LOG_H_

#include "GearAutoOpt/data/GearLogLevel.h"

#include <QDebug>
#include <QString>

namespace GearAutoOpt {

/// 进程内全局日志等级（单线程优化流水线使用）。
class GEARAUTOOPTAPI GearOptLog {
public:
	static void  setLevel(GearLogLevel level);
	static GearLogLevel level();
	static bool  isDebug();

	/// 追加一行到指定日志文件（CCX/Gmsh 原始输出）。
	static bool appendLine(const QString& filePath, const QString& line);
	/// 覆盖写入文本块（Gmsh 结束后一次性写入）。
	static bool writeText(const QString& filePath, const QString& text);

private:
	static GearLogLevel s_level;
};

#define GEAR_OPT_DEBUG if (::GearAutoOpt::GearOptLog::isDebug()) qDebug()
#define GEAR_OPT_DEBUG_NOQUOTE if (::GearAutoOpt::GearOptLog::isDebug()) qDebug().noquote()

} // namespace GearAutoOpt

#endif
