#ifndef _GEARAUTOOPT_GEAR_OPT_COMPARE_TABLE_H_
#define _GEARAUTOOPT_GEAR_OPT_COMPARE_TABLE_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearOptConfig.h"

#include <QList>
#include <QString>
#include <QVector>

class QTableWidget;

namespace GearAutoOpt {

/// 生成优化前后对比表：写 CSV 到 runDir/result/optimization_compare.csv，并可选填充 QTableWidget。
/// 返回已保存文件的绝对路径；失败返回空字符串。
GEARAUTOOPTAPI QString exportOptimizationCompareTable(const GearOptConfig& cfg,
                                                      const QVector<QList<GearDesignPoint>>& allPoints,
                                                      const QString& runDirRoot,
                                                      QTableWidget* table);

} // namespace GearAutoOpt

#endif
