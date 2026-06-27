// UTF-8 BOM
#include "GearOptResultViewer.h"

#include <algorithm>

#include <QAbstractItemView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace GearAutoOpt {

GearOptResultViewer::GearOptResultViewer(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QString::fromUtf8("优化结果浏览"));
    setMinimumSize(1100, 520);
    buildUi();
}

void GearOptResultViewer::buildUi() {
    auto* layout = new QVBoxLayout(this);

    // ---- 表格区 ----
    auto* tableGroup = new QGroupBox(QString::fromUtf8("设计点列表"), this);
    auto* tableLayout = new QVBoxLayout(tableGroup);

    _table = new QTableWidget(0, 19, this);
    _table->setHorizontalHeaderLabels({
        QString::fromUtf8("代"),
        QString::fromUtf8("ID"),
        QString::fromUtf8("z1"),
        QString::fromUtf8("z2"),
        QString::fromUtf8("m"),
        QString::fromUtf8("α [°]"),
        QString::fromUtf8("x1"),
        QString::fromUtf8("x2"),
        QString::fromUtf8("width"),
        QString::fromUtf8("轮毂比"),
        QString::fromUtf8("ca1"),
        QString::fromUtf8("lca1"),
        QString::fromUtf8("ca2"),
        QString::fromUtf8("lca2"),
        QString::fromUtf8("cpress_max [MPa]"),
        QString::fromUtf8("mass [kg]"),
        QString::fromUtf8("σ_max [MPa] (辅助)"),
        QString::fromUtf8("u_max [mm]"),
        QString::fromUtf8("状态"),
    });
    _table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    _table->setSelectionBehavior(QAbstractItemView::SelectRows);
    _table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    _table->setAlternatingRowColors(true);
    connect(_table, &QTableWidget::cellDoubleClicked,
            this, &GearOptResultViewer::onRowDoubleClicked);
    tableLayout->addWidget(_table);
    layout->addWidget(tableGroup, 2);

    // ---- 状态标签 ----
    _labelInfo = new QLabel(QString::fromUtf8("双击行可加载该点几何"), this);
    layout->addWidget(_labelInfo);

    // ---- 按钮行 ----
    auto* btnRow = new QHBoxLayout;
    _btnLoad = new QPushButton(QString::fromUtf8("加载选中点几何"), this);
    _btnLoad->setEnabled(false);
    auto* btnClose = new QPushButton(QString::fromUtf8("关闭"), this);
    btnRow->addWidget(_btnLoad);
    btnRow->addStretch();
    btnRow->addWidget(btnClose);
    layout->addLayout(btnRow);

    connect(_btnLoad, &QPushButton::clicked, this, &GearOptResultViewer::onLoadGeometry);
    connect(btnClose, &QPushButton::clicked, this, &QDialog::accept);
}

void GearOptResultViewer::setPoints(const QList<GearDesignPoint>& points) {
    _points = points;
    refreshTable();
}

void GearOptResultViewer::setPareto(const QList<GearDesignPoint>& pareto) {
    _pareto = pareto;
    refreshTable();
}

void GearOptResultViewer::refreshTable() {
    _table->setRowCount(0);
    for (const GearDesignPoint& dp : _points) {
        const int row = _table->rowCount();
        _table->insertRow(row);

        auto cell = [&](int col, const QString& text) {
            auto* item = new QTableWidgetItem(text);
            item->setTextAlignment(Qt::AlignCenter);
            _table->setItem(row, col, item);
        };

        cell(0, QString::number(dp.generation));
        cell(1, QString::number(dp.id));
        cell(2, QString::number(dp.z1));
        cell(3, QString::number(dp.z2));
        cell(4, QString::number(dp.module, 'g', 4));
        cell(5, QString::number(dp.alpha, 'f', 2));
        cell(6, QString::number(dp.x1, 'f', 3));
        cell(7, QString::number(dp.x2, 'f', 3));
        cell(8, QString::number(dp.commonWidth, 'f', 2));
        cell(9, QString::number(dp.hubRatio, 'f', 3));
        cell(10, QString::number(dp.ca1, 'g', 5));
        cell(11, QString::number(dp.lca1, 'g', 5));
        cell(12, QString::number(dp.ca2, 'g', 5));
        cell(13, QString::number(dp.lca2, 'g', 5));
        cell(14, dp.cpressMax_MPa > 0
                ? QString::number(dp.cpressMax_MPa, 'f', 1)
                : QString::fromUtf8("—"));
        cell(15, dp.mass >= 0
                ? QString::number(dp.mass, 'f', 4)
                : QString::fromUtf8("—"));
        cell(16, dp.sigmaMax >= 0
                ? QString::number(dp.sigmaMax, 'f', 1)
                : QString::fromUtf8("—"));
        cell(17, dp.uMax >= 0
                ? QString::number(dp.uMax, 'g', 5)
                : QString::fromUtf8("—"));
        cell(18, pointStatusToString(dp.status));

        // Pareto 高亮（淡绿色）
        const bool isPareto = std::any_of(_pareto.constBegin(), _pareto.constEnd(),
            [&](const GearDesignPoint& p) {
                return p.generation == dp.generation && p.id == dp.id;
            });
        if (isPareto) {
            for (int c = 0; c < _table->columnCount(); ++c) {
                auto* it = _table->item(row, c);
                if (it) it->setBackground(QColor(180, 255, 180));
            }
        }
    }

    _labelInfo->setText(QString("共 %1 个设计点，%2 个 Pareto 最优（绿色高亮）")
                        .arg(_points.size()).arg(_pareto.size()));
}

void GearOptResultViewer::onRowDoubleClicked(int row, int /*col*/) {
    if (row < 0 || row >= _points.size()) return;
    _selectedRow = row;
    _btnLoad->setEnabled(true);
    const GearDesignPoint& dp = _points[row];
    _labelInfo->setText(QString("选中: gen=%1 id=%2  z1=%3 z2=%4 m=%5  ca1=%6 lca1=%7  cpress=%8 MPa  σ=%9 MPa  u_max=%10 mm  mass=%11 kg")
                        .arg(dp.generation).arg(dp.id)
                        .arg(dp.z1).arg(dp.z2).arg(dp.module, 0, 'g', 5)
                        .arg(dp.ca1, 0, 'g', 5).arg(dp.lca1, 0, 'g', 5)
                        .arg(dp.cpressMax_MPa > 0 ? QString::number(dp.cpressMax_MPa, 'f', 1) : QString::fromUtf8("—"))
                        .arg(dp.sigmaMax >= 0 ? QString::number(dp.sigmaMax, 'f', 1) : QString::fromUtf8("—"))
                        .arg(dp.uMax >= 0 ? QString::number(dp.uMax, 'g', 5) : QString::fromUtf8("—"))
                        .arg(dp.mass, 0, 'f', 4));
}

void GearOptResultViewer::onLoadGeometry() {
    if (_selectedRow < 0 || _selectedRow >= _points.size()) return;
    QMessageBox::information(this, QString::fromUtf8("提示"),
    QString::fromUtf8("几何加载功能需在 FastCAE 主窗口中调用 GeoCommandCreateGear。\n"
                       "设计参数已选中，可手动在齿轮创建对话框中输入相同参数。"));
    emit loadGeometryRequested(_points[_selectedRow]);
}

} // namespace GearAutoOpt
