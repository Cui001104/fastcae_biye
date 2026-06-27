#ifndef _GEARAUTOOPT_GEAROPTGEOMETRYBRIDGE_H_
#define _GEARAUTOOPT_GEAROPTGEOMETRYBRIDGE_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"

namespace Geometry {
class GeometryParaGear;
}

namespace GearAutoOpt {

struct GearOptConfig;

/// 从当前工程几何列表中查找带齿轮建模参数的几何集（自后向前，取最近一个）。
GEARAUTOOPTAPI Geometry::GeometryParaGear* findCurrentGeometryParaGear();

/// 将界面齿轮参数转为优化设计点（hubRatio 无对应字段时用默认值 0.4）。
/// 注：GeometryParaGear 的 getter 未声明为 const，故参数为非 const 引用。
GEARAUTOOPTAPI GearDesignPoint gearDesignPointFromGeometryParaGear(Geometry::GeometryParaGear& g);

GEARAUTOOPTAPI void logGearOptBasePointLine(const GearDesignPoint& dp);

/// 设计变量越界时写 qWarning（仅 GearAutoOpt 模块内使用，不导出 DLL）。
void checkGearOptDesignPointBounds(const GearDesignPoint& dp, const GearOptConfig& cfg);

} // namespace GearAutoOpt

#endif
