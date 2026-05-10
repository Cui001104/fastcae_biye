// UTF-8 BOM
#ifndef _GEARAUTOOPT_GEAR_OPT_RESULT_VIEWER_H_
#define _GEARAUTOOPT_GEAR_OPT_RESULT_VIEWER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"

#include <QDialog>
#include <QList>

class QTableWidget;
class QPushButton;
class QLabel;

namespace GearAutoOpt {

/// 展示：
///   - 上方 QTableWidget：全设计点列表（gen / id / z1 / m / σ / mass / status）
///   - 下方 散点图（σ vs mass），Pareto 前沿高亮（若 Qt Charts 可用）
///   - "加载该点几何"按钮：双击行后可恢复该设计点的几何
class GEARAUTOOPTAPI GearOptResultViewer : public QDialog {
    Q_OBJECT
public:
    explicit GearOptResultViewer(QWidget* parent = nullptr);

    /// 填入全部已评估设计点（可在优化结束后调用，也可实时追加）
    void setPoints(const QList<GearDesignPoint>& points);

    /// 高亮标记 Pareto 最优行
    void setPareto(const QList<GearDesignPoint>& pareto);

signals:
    /// 用户双击某行后触发（传递该设计点）
    void loadGeometryRequested(GearAutoOpt::GearDesignPoint dp);

private slots:
    void onRowDoubleClicked(int row, int col);
    void onLoadGeometry();

private:
    void buildUi();
    void refreshTable();

    QTableWidget*   _table{};
    QPushButton*    _btnLoad{};
    QLabel*         _labelInfo{};

    QList<GearDesignPoint> _points;
    QList<GearDesignPoint> _pareto;
    int                    _selectedRow{-1};
};

} // namespace GearAutoOpt
#endif
