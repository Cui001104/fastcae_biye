#ifndef _GEARAUTOOPT_CALCULIX_RESULT_VTK_EXPORT_H_
#define _GEARAUTOOPT_CALCULIX_RESULT_VTK_EXPORT_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QString>

namespace GearAutoOpt {

/// 将 mesh.inp 拓扑 + job.frd（位移/节点应力）+ job.dat（单元应力备选）导出 result.vtu。
/// ParaView：Surface With Edges、SXX/SYY/vonMises、Warp By Vector(displacement)。
GEARAUTOOPTAPI bool exportCalculixResultToVTK(const QString& meshInpPath,
                                              const QString& frdPath,
                                              const QString& datPath,
                                              const QString& outVtuPath,
                                              QString*       errorMsg = nullptr);

} // namespace GearAutoOpt

#endif
