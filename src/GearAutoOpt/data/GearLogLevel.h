#ifndef _GEARAUTOOPT_GEAR_LOG_LEVEL_H_
#define _GEARAUTOOPT_GEAR_LOG_LEVEL_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

namespace GearAutoOpt {

/// 齿轮优化/CCX/Gmsh 日志等级：Normal 仅关键流程；Debug 额外输出网格识别细节。
enum class GEARAUTOOPTAPI GearLogLevel {
	Normal,
	Debug
};

inline bool isDebugLogLevel(GearLogLevel level) {
	return level == GearLogLevel::Debug;
}

} // namespace GearAutoOpt

#endif
