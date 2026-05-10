// UTF-8 BOM
#include "GearOptResultViewer.h"

#include <QDialogButtonBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace GearAutoOpt {

GearOptResultViewer::GearOptResultViewer(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("优化结果浏览"));
    setMinimumSize(800, 500);
    buildUi();
}

void GearOptResultViewer::buildUi() {
    auto* layout = new QVBoxLayout(this);

    // ---- 表格区 ----
    auto* tableGroup = new QGroupBox(QStringLiteral("设计点列表"), this);
    auto* tableLayout = new QVBoxLayout(tableGroup);

    _table = new QTableWidget(0, 9, this);
    _table->setHorizontalHeaderLabels({
        QStringLiteral("代"),
        QStringLiteral("ID"),
        QStringLiteral("z1"),
        QStringLiteral("m"),
        QStringLiteral("x1"),
        QStringLiteral("width"),
        QStringLiteral("σ_max [MPa]"),
        QStringLiteral("mass [kg]"),
        QStringLiteral("状态")
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
    _labelInfo = new QLabel(QStringLiteral("双击行可加载该点几何"), this);
    layout->addWidget(_labelInfo);

    // ---- 按钮行 ----
    auto* btnRow = new QHBoxLayout;
    _btnLoad = new QPushButton(QStringLiteral("加载选中点几何"), this);
    _btnLoad->setEnabled(false);
    auto* btnClose = new QPushButton(QStringLiteral("关闭"), this);
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
        cell(3, QString::number(dp.module, 'g', 3));
        cell(4, QString::number(dp.x1, 'f', 2));
        cell(5, QString::number(dp.width, 'f', 1));
        cell(6, dp.sigmaMax >= 0
                ? QString::number(dp.sigmaMax, 'f', 1)
                : QStringLiteral("—"));
        cell(7, dp.mass >= 0
                ? QString::number(dp.mass, 'f', 4)
                : QStringLiteral("—"));
        cell(8, pointStatusToString(dp.status));

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
    _labelInfo->setText(QString("选中: gen=%1 id=%2  z1=%3 m=%4  σ=%5 MPa  mass=%6 kg")
                        .arg(dp.generation).arg(dp.id)
                        .arg(dp.z1).arg(dp.module)
                        .arg(dp.sigmaMax, 0, 'f', 1)
                        .arg(dp.mass, 0, 'f', 4));
}

void GearOptResultViewer::onLoadGeometry() {
    if (_selectedRow < 0 || _selectedRow >= _points.size()) return;
    QMessageBox::information(this, QStringLiteral("提示"),
        QStringLiteral("几何加载功能需在 FastCAE 主窗口中调用 GeoCommandCreateGear。\n"
                       "设计参数已选中，可手动在齿轮创建对话框中输入相同参数。"));
    emit loadGeometryRequested(_points[_selectedRow]);
}

} // namespace GearAutoOpt
