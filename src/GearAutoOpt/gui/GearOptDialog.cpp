// UTF-8 BOM
#include "GearOptDialog.h"
#include "GearAutoOpt/runner/GearAutoOptManager.h"
#include "GearAutoOpt/runner/GearOptMainThreadRunner.h"
#include "GearAutoOpt/runner/MeshIndependenceRunner.h"
#include "GearAutoOpt/tools/FailedCaseRetryRunner.h"
#include "GearAutoOpt/tools/GearOptFailedCaseRetry.h"
#include "GearOptResultViewer.h"
#include "GearAutoOpt/data/GearOptGeometryBridge.h"
#include "GearAutoOpt/gui/GearOptCompareTable.h"
#include "GearAutoOpt/db/GearOptResultDatabase.h"
#include "GearAutoOpt/surrogate/GearSurrogateModel.h"
#include "Geometry/geometryParaGear.h"

#include <algorithm>
#include <QAbstractItemView>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaType>
#include <QMessageBox>
#include <QDebug>
#include <QHeaderView>
#include <QDialog>
#include <QProgressBar>
#include <QPushButton>
#include <QSqlDatabase>
#include <QSpinBox>
#include <QSizePolicy>
#include <QTableWidget>
#include <QTextEdit>
#include <QTextStream>
#include <QThread>
#include <QVBoxLayout>

namespace GearAutoOpt {

// 设为 0 可关闭仅打开对话框时的 SQL 驱动探测（不跑优化）
#ifndef GEAR_OPT_DEBUG_SQL_DRIVERS
#define GEAR_OPT_DEBUG_SQL_DRIVERS 1
#endif

GearOptDialog::GearOptDialog(QWidget* parent)
    : QDialog(parent)
{
    qRegisterMetaType<GearAutoOpt::FailedCaseRetryStats>("GearAutoOpt::FailedCaseRetryStats");
    setWindowTitle(QString::fromUtf8("齿轮多目标优化"));
    setMinimumSize(620, 520);
    resize(620, 520);
    buildUi();

    GearOptMainThreadRunner::ensureInstance(this);

#if GEAR_OPT_DEBUG_SQL_DRIVERS
	// 仅打开对话框即打印，无需点「开始优化」；看 VS「输出」或 DebugView
	qDebug() << "[GearOpt][SQL] QSqlDatabase::drivers() =" << QSqlDatabase::drivers();
	qDebug() << "[GearOpt][SQL] applicationDirPath =" << QCoreApplication::applicationDirPath();
#endif
	auto& globalDb = GearOptResultDatabase::global();
	const QString globalPath = GearOptResultDatabase::defaultGlobalDatabasePath();
	if (globalDb.openDatabase(globalPath)) {
		qDebug().noquote() << QStringLiteral("[GearOpt][DB] global ready path=%1 samples=%2")
		                      .arg(globalPath)
		                      .arg(globalDb.countValidResults());
	} else {
		qWarning().noquote() << QStringLiteral("[GearOpt][DB] global open failed path=%1")
		                        .arg(globalPath);
	}
}

GearOptDialog::~GearOptDialog() {
	_ignoreManagerSlots = true;
	if (_manager)
		QObject::disconnect(_manager, nullptr, this, nullptr);
	onStop();
	if (_meshIndepThread && _meshIndepThread->isRunning()) {
		_meshIndepThread->quit();
		_meshIndepThread->wait(300000);
	}
	if (_retryThread && _retryThread->isRunning()) {
		_retryThread->quit();
		_retryThread->wait(300000);
	}
}

void GearOptDialog::closeEvent(QCloseEvent* event) {
	if (_thread && _thread->isRunning()) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("优化仍在运行，请先点击「停止」并等待结束后再关闭窗口。"));
		event->ignore();
		return;
	}
	if (_meshIndepThread && _meshIndepThread->isRunning()) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("网格无关性验证仍在运行，请等待结束后再关闭窗口。"));
		event->ignore();
		return;
	}
	if (_retryThread && _retryThread->isRunning()) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("失败样本重算仍在运行，请等待结束后再关闭窗口。"));
		event->ignore();
		return;
	}
	QDialog::closeEvent(event);
}

void GearOptDialog::buildUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 10, 12, 10);
    mainLayout->setSpacing(8);

    auto polishNumericInput = [](QWidget* w) {
        if (!w) return;
        constexpr int h = 28;
        w->setMinimumHeight(h);
        w->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    };

    auto* topRow = new QHBoxLayout;
    topRow->setSpacing(10);

    // ---- 优化参数 ----
    auto* cfgGroup = new QGroupBox(QString::fromUtf8("优化参数"), this);
    auto* cfgLayout = new QVBoxLayout(cfgGroup);
    cfgLayout->setContentsMargins(10, 10, 10, 8);
    cfgLayout->setSpacing(6);

    auto addSpinRow = [&](const QString& label, QWidget* spin) {
        auto* row = new QHBoxLayout;
        row->setSpacing(6);
        auto* lab = new QLabel(label, this);
        lab->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        lab->setMinimumWidth(72);
        row->addWidget(lab, 0);
        row->addWidget(spin, 1);
        polishNumericInput(spin);
        cfgLayout->addLayout(row);
    };

    _spinTorque = new QDoubleSpinBox(this);
    _spinTorque->setRange(1.0, 10000.0);
    _spinTorque->setValue(100.0);
    _spinTorque->setSuffix(QString::fromUtf8(" N·m"));
    _spinTorque->setDecimals(1);
    addSpinRow(QString::fromUtf8("输入扭矩:"), _spinTorque);

    _spinPopulation = new QSpinBox(this);
    _spinPopulation->setRange(10, 200);
    _spinPopulation->setValue(30);
    addSpinRow(QString::fromUtf8("种群大小:"), _spinPopulation);

    _spinGenerations = new QSpinBox(this);
    _spinGenerations->setRange(1, 100);
    _spinGenerations->setValue(20);
    addSpinRow(QString::fromUtf8("最大代数:"), _spinGenerations);
    topRow->addWidget(cfgGroup, 2);

    // ---- 优化目标（统一区域，2×2）----
    auto* objGroup = new QGroupBox(QString::fromUtf8("优化目标"), this);
    auto* objGrid = new QGridLayout(objGroup);
    objGrid->setContentsMargins(10, 10, 10, 8);
    objGrid->setHorizontalSpacing(12);
    objGrid->setVerticalSpacing(4);

    _chkMinCpress = new QCheckBox(
        QString::fromUtf8("最大接触压力（cpressMax_MPa）"), this);
    _chkMinCpress->setChecked(true);
    _chkMinEdgeLoadRatio = new QCheckBox(
        QString::fromUtf8("齿宽边缘偏载系数（edgeLoadRatio）"), this);
    _chkMinEdgeLoadRatio->setChecked(true);
    _chkMinSigmaMax = new QCheckBox(
        QString::fromUtf8("最大等效应力（sigmaMax_MPa）"), this);
    _chkMinSigmaMax->setChecked(false);
    _chkMinMass = new QCheckBox(
        QString::fromUtf8("质量（mass_kg）"), this);
    _chkMinMass->setChecked(false);

    objGrid->addWidget(_chkMinCpress, 0, 0);
    objGrid->addWidget(_chkMinEdgeLoadRatio, 0, 1);
    objGrid->addWidget(_chkMinSigmaMax, 1, 0);
    objGrid->addWidget(_chkMinMass, 1, 1);
    topRow->addWidget(objGroup, 3);
    mainLayout->addLayout(topRow);

    // ---- 设计变量（紧凑两行布局）----
    auto* dvGroup = new QGroupBox(QString::fromUtf8("设计变量"), this);
    dvGroup->setStyleSheet(QStringLiteral(
        "QGroupBox QCheckBox { font-size: 9pt; spacing: 3px; padding: 0 2px; margin: 0; }"));
    auto* dvGrid = new QGridLayout(dvGroup);
    dvGrid->setContentsMargins(8, 6, 8, 6);
    dvGrid->setHorizontalSpacing(8);
    dvGrid->setVerticalSpacing(2);

    auto addDvCb = [&](int row, int col, const QString& text, QCheckBox*& ptr, bool checked) {
        ptr = new QCheckBox(text, dvGroup);
        ptr->setChecked(checked);
        dvGrid->addWidget(ptr, row, col);
    };
    addDvCb(0, 0, QString::fromUtf8("模数m"), _cbDvModule, false);
    addDvCb(0, 1, QString::fromUtf8("齿数z1"), _cbDvZ1, false);
    addDvCb(0, 2, QString::fromUtf8("齿数z2"), _cbDvZ2, false);
    addDvCb(0, 3, QString::fromUtf8("压力角α"), _cbDvAlpha, false);
    addDvCb(0, 4, QString::fromUtf8("x1"), _cbDvX1, true);
    addDvCb(0, 5, QString::fromUtf8("x2"), _cbDvX2, true);
    addDvCb(1, 0, QString::fromUtf8("ca1"), _cbDvCa1, true);
    addDvCb(1, 1, QString::fromUtf8("lca1"), _cbDvLca1, true);
    addDvCb(1, 2, QString::fromUtf8("ca2"), _cbDvCa2, true);
    addDvCb(1, 3, QString::fromUtf8("lca2"), _cbDvLca2, true);
    addDvCb(1, 4, QString::fromUtf8("齿宽"), _cbDvWidth, false);
    addDvCb(1, 5, QString::fromUtf8("hubRatio"), _cbDvHubRatio, false);
    if (_cbDvWidth) {
        _cbDvWidth->setEnabled(false);
        _cbDvWidth->setToolTip(
            QString::fromUtf8("齿宽固定为当前建模基准值，不参与 RBF/代理优化（仍写入数据库）"));
    }
    mainLayout->addWidget(dvGroup);

    // ---- 进度 ----
    auto* progressRow = new QHBoxLayout;
    _progress = new QProgressBar(this);
    _progress->setRange(0, 100);
    _progress->setValue(0);
    _labelStatus = new QLabel(QString::fromUtf8("就绪"), this);
    _labelStatus->setMinimumWidth(200);
    progressRow->addWidget(_progress, 3);
    progressRow->addWidget(_labelStatus, 2);
    mainLayout->addLayout(progressRow);

    // ---- 日志 ----
    _log = new QTextEdit(this);
    _log->setReadOnly(true);
    _log->setMinimumHeight(180);
    mainLayout->addWidget(_log, 1);

    // ---- 按钮 ----
    auto* btnRow = new QHBoxLayout;
    _btnStart      = new QPushButton(QString::fromUtf8("开始优化"), this);
    _btnMeshIndep  = new QPushButton(QString::fromUtf8("网格无关性验证"), this);
    _btnStop       = new QPushButton(QString::fromUtf8("停止"), this);
    _btnCompare    = new QPushButton(QString::fromUtf8("优化对比"), this);
    _btnViewResult = new QPushButton(QString::fromUtf8("查看结果..."), this);
    _btnMetricConfig = new QPushButton(QString::fromUtf8("优化配置"), this);
    _btnBackfillCpress = new QPushButton(QString::fromUtf8("离线补算接触分布指标"), this);
    _btnRetryFailed    = new QPushButton(QString::fromUtf8("Retry Failed Cases"), this);
    _btnExport     = new QPushButton(QString::fromUtf8("导出 Excel..."), this);
    _btnStop->setEnabled(false);
    _btnCompare->setEnabled(false);
    _btnViewResult->setEnabled(false);
    _btnExport->setEnabled(false);
    btnRow->addStretch();
    btnRow->addWidget(_btnStart);
    btnRow->addWidget(_btnMeshIndep);
    btnRow->addWidget(_btnStop);
    btnRow->addWidget(_btnCompare);
    btnRow->addWidget(_btnViewResult);
    btnRow->addWidget(_btnMetricConfig);
    btnRow->addWidget(_btnBackfillCpress);
    btnRow->addWidget(_btnRetryFailed);
    btnRow->addWidget(_btnExport);
    mainLayout->addLayout(btnRow);

    // ---- 求解设置 ----
    auto* solverGroup = new QGroupBox(QString::fromUtf8("求解设置"), this);
    auto* solverLayout = new QHBoxLayout(solverGroup);
    solverLayout->setContentsMargins(10, 12, 10, 10);
    solverLayout->setSpacing(8);

    // 结果路径
    solverLayout->addWidget(new QLabel(QString::fromUtf8("结果目录:"), this));
    _editRunDir = new QLineEdit(this);
    _editRunDir->setPlaceholderText(QString::fromUtf8("必选：本次优化结果目录（其下生成 GearOptResults.db）"));
    _editRunDir->setMinimumWidth(200);
    solverLayout->addWidget(_editRunDir, 2);
    auto* btnBrowse = new QPushButton(QString::fromUtf8("浏览..."), this);
    solverLayout->addWidget(btnBrowse);

    solverLayout->addSpacing(16);

    // 求解线程数
    solverLayout->addWidget(new QLabel(QString::fromUtf8("求解线程:"), this));
    _spinThreads = new QSpinBox(this);
    _spinThreads->setRange(0, 256);
    _spinThreads->setValue(0);
    _spinThreads->setSpecialValueText(QString::fromUtf8("自动"));
    _spinThreads->setToolTip(QString::fromUtf8("ccx -t 线程数；0=自动 min(逻辑核,8)"));
    solverLayout->addWidget(_spinThreads);
    polishNumericInput(_spinThreads);

    solverLayout->addSpacing(16);

    // 网格尺寸
    solverLayout->addWidget(new QLabel(QString::fromUtf8("网格尺寸:"), this));
    _chkMeshAuto = new QCheckBox(QString::fromUtf8("自动"), this);
    _chkMeshAuto->setChecked(false);
    _spinMeshSize = new QDoubleSpinBox(this);
    _spinMeshSize->setRange(0.1, 20.0);
    _spinMeshSize->setValue(1.50);
    _spinMeshSize->setSingleStep(0.1);
    _spinMeshSize->setDecimals(2);
    _spinMeshSize->setSuffix(QString::fromUtf8(" mm"));
    _spinMeshSize->setEnabled(true);
    solverLayout->addWidget(_chkMeshAuto);
    solverLayout->addWidget(_spinMeshSize);
    polishNumericInput(_spinMeshSize);
    solverLayout->addSpacing(16);

    _chkSurrogateAssisted = new QCheckBox(QString::fromUtf8("启用代理辅助优化"), this);
    _chkSurrogateAssisted->setToolTip(
        QString::fromUtf8("一键式代理优化：自动检查同 baseCaseHash 的 CCX 样本，"
                         "不足时自动 LHS 补样并并行 CCX，随后 RBF 训练、代理 NSGA-II 与 Infill 验证。"));
    solverLayout->addWidget(_chkSurrogateAssisted);
    solverLayout->addStretch();
    mainLayout->addWidget(solverGroup);

    auto* parallelGroup = new QGroupBox(QString::fromUtf8("并行 CCX（初始 LHS + Infill）"), this);
    auto* parallelLayout = new QHBoxLayout(parallelGroup);
    parallelLayout->setContentsMargins(10, 12, 10, 10);
    parallelLayout->setSpacing(8);

    _chkParallelCcx = new QCheckBox(QString::fromUtf8("启用并行 CCX 验证"), this);
    _chkParallelCcx->setChecked(false);
    _chkParallelCcx->setToolTip(
        QString::fromUtf8("作用于代理辅助优化中的 CCX 求解："
                         "初始 LHS 补样与 Infill 验证均先完成 CAD/网格，再并行提交 CCX。"));
    parallelLayout->addWidget(_chkParallelCcx);

    parallelLayout->addSpacing(12);
    parallelLayout->addWidget(new QLabel(QString::fromUtf8("并行任务数:"), this));
    _spinParallelCcxJobs = new QSpinBox(this);
    _spinParallelCcxJobs->setRange(1, 8);
    _spinParallelCcxJobs->setValue(2);
    _spinParallelCcxJobs->setEnabled(false);
    parallelLayout->addWidget(_spinParallelCcxJobs);
    polishNumericInput(_spinParallelCcxJobs);

    parallelLayout->addSpacing(12);
    parallelLayout->addWidget(new QLabel(QString::fromUtf8("单任务线程数:"), this));
    _spinCcxThreadsPerJob = new QSpinBox(this);
    _spinCcxThreadsPerJob->setRange(1, 16);
    _spinCcxThreadsPerJob->setValue(4);
    _spinCcxThreadsPerJob->setEnabled(false);
    parallelLayout->addWidget(_spinCcxThreadsPerJob);
    polishNumericInput(_spinCcxThreadsPerJob);

    auto* parallelHint = new QLabel(
        QString::fromUtf8("总线程数 ≈ 并行任务数 × 单任务线程数，建议不要超过 CPU 线程数。"), this);
    parallelHint->setStyleSheet(QStringLiteral("color: gray;"));
    parallelLayout->addWidget(parallelHint);
    parallelLayout->addStretch();
    mainLayout->addWidget(parallelGroup);

    connect(_chkParallelCcx, &QCheckBox::toggled, this, [this](bool checked) {
        if (_spinParallelCcxJobs)
            _spinParallelCcxJobs->setEnabled(checked);
        if (_spinCcxThreadsPerJob)
            _spinCcxThreadsPerJob->setEnabled(checked);
    });

    connect(btnBrowse, &QPushButton::clicked, this, [this]() {
        const QString dir = QFileDialog::getExistingDirectory(
            this, QString::fromUtf8("选择结果保存目录"),
            _editRunDir->text().isEmpty() ? QDir::currentPath() : _editRunDir->text());
        if (!dir.isEmpty()) _editRunDir->setText(dir);
    });
    connect(_chkMeshAuto, &QCheckBox::toggled, this, [this](bool checked) {
        _spinMeshSize->setEnabled(!checked);
    });

    connect(_btnStart,      &QPushButton::clicked, this, &GearOptDialog::onStart);
    connect(_btnMeshIndep,  &QPushButton::clicked, this, &GearOptDialog::onMeshIndependence);
    connect(_btnStop,       &QPushButton::clicked, this, &GearOptDialog::onStop);
    connect(_btnCompare, &QPushButton::clicked, this, [this]() {
        if (_cachedAllPoints.isEmpty()) {
            QMessageBox::information(this, QString::fromUtf8("提示"),
                                     QString::fromUtf8("暂无优化结果，请先运行优化。"));
            return;
        }
        QDialog dlg(this);
        dlg.setWindowTitle(QString::fromUtf8("优化前后对比"));
        dlg.setMinimumSize(880, 360);
        auto* lay = new QVBoxLayout(&dlg);
        auto* grp = new QGroupBox(QString::fromUtf8("BASE 与 Pareto 对比"), &dlg);
        auto* gl = new QVBoxLayout(grp);
        auto* tbl = new QTableWidget(&dlg);
        tbl->setEditTriggers(QAbstractItemView::NoEditTriggers);
        tbl->setSelectionBehavior(QAbstractItemView::SelectRows);
        tbl->setAlternatingRowColors(true);
        gl->addWidget(tbl);
        lay->addWidget(grp, 1);
        auto* btnRowDlg = new QHBoxLayout;
        auto* btnClose = new QPushButton(QString::fromUtf8("关闭"), &dlg);
        QObject::connect(btnClose, &QPushButton::clicked, &dlg, &QDialog::accept);
        btnRowDlg->addStretch();
        btnRowDlg->addWidget(btnClose);
        lay->addLayout(btnRowDlg);
        exportOptimizationCompareTable(_lastRunCfg, _cachedAllPoints, QString(), tbl);
        dlg.exec();
    });
    connect(_btnViewResult, &QPushButton::clicked, this, [this]() {
        GearOptResultViewer viewer(this);
        QList<GearDesignPoint> all;
        for (const auto& gen : _cachedAllPoints)
            all += gen;
        viewer.setPoints(all);
        viewer.setPareto(_cachedPareto);
        viewer.exec();
    });
    connect(_btnExport, &QPushButton::clicked, this, &GearOptDialog::onExport);
    connect(_btnMetricConfig, &QPushButton::clicked, this, &GearOptDialog::onMetricConfig);
    connect(_btnBackfillCpress, &QPushButton::clicked, this, &GearOptDialog::onOfflineBackfillCpress);
    connect(_btnRetryFailed, &QPushButton::clicked, this, &GearOptDialog::onRetryFailedCases);
}

GearOptConfig GearOptDialog::currentConfig() const {
	DesignVariableFlags dv;
	dv.module   = _cbDvModule->isChecked();
	dv.z1       = _cbDvZ1->isChecked();
	dv.z2       = _cbDvZ2->isChecked();
	dv.alpha    = _cbDvAlpha->isChecked();
	dv.x1       = _cbDvX1->isChecked();
	dv.x2       = _cbDvX2->isChecked();
	dv.ca1      = _cbDvCa1->isChecked();
	dv.lca1     = _cbDvLca1->isChecked();
	dv.ca2      = _cbDvCa2->isChecked();
	dv.lca2     = _cbDvLca2->isChecked();
	dv.width    = false; // 齿宽固定为基准值，不参与 RBF/代理优化
	dv.hubRatio = _cbDvHubRatio->isChecked();
	qDebug().noquote() << QStringLiteral("[GearOpt][DesignVars] module=%1, z1=%2, z2=%3, alpha=%4, x1=%5, x2=%6, ca1=%7, lca1=%8, ca2=%9, lca2=%10, width=%11, hubRatio=%12")
	                          .arg(dv.module ? 1 : 0)
	                          .arg(dv.z1 ? 1 : 0)
	                          .arg(dv.z2 ? 1 : 0)
	                          .arg(dv.alpha ? 1 : 0)
	                          .arg(dv.x1 ? 1 : 0)
	                          .arg(dv.x2 ? 1 : 0)
	                          .arg(dv.ca1 ? 1 : 0)
	                          .arg(dv.lca1 ? 1 : 0)
	                          .arg(dv.ca2 ? 1 : 0)
	                          .arg(dv.lca2 ? 1 : 0)
	                          .arg(dv.width ? 1 : 0)
	                          .arg(dv.hubRatio ? 1 : 0);

	GearOptConfig cfg;
	Geometry::GeometryParaGear* g = findCurrentGeometryParaGear();
	if (g) {
		const GearDesignPoint base = gearDesignPointFromGeometryParaGear(*g);
		logGearOptBasePointLine(base);
		cfg = GearOptConfig::fromBasePoint(base, dv);
	} else {
		qWarning() << "[GearOpt][WARN] No current GeometryParaGear found, fallback to defaultConfig().";
		cfg = GearOptConfig::defaultConfig();
		cfg.designVars = dv;
	}

	cfg.solver.torque          = _spinTorque->value();
	cfg.nsga2.populationSize   = _spinPopulation->value();
	cfg.nsga2.maxGenerations   = _spinGenerations->value();
	cfg.objectives.minCpressMax = _chkMinCpress->isChecked();
	cfg.objectives.minEdgeLoadRatio = _chkMinEdgeLoadRatio->isChecked();
	cfg.objectives.minSigmaMax  = _chkMinSigmaMax->isChecked();
	cfg.objectives.minMass      = _chkMinMass->isChecked();
	cfg.solver.meshSize        = _chkMeshAuto->isChecked() ? 0.0 : _spinMeshSize->value();
	cfg.solver.runBaseDir      = _editRunDir->text().trimmed();
	cfg.solver.threads         = _spinThreads->value();
	cfg.solver.surrogateAssisted = _chkSurrogateAssisted && _chkSurrogateAssisted->isChecked();
	cfg.solver.parallelCcxEnabled = _chkParallelCcx && _chkParallelCcx->isChecked();
	cfg.solver.parallelCcxJobs    = _spinParallelCcxJobs ? _spinParallelCcxJobs->value() : 2;
	cfg.solver.ccxThreadsPerJob   = _spinCcxThreadsPerJob ? _spinCcxThreadsPerJob->value() : 4;
	return cfg;
}

void GearOptDialog::setRunning(bool running) {
    _btnStart->setEnabled(!running);
    if (_btnMeshIndep) _btnMeshIndep->setEnabled(!running);
    _btnStop->setEnabled(running);
    if (!running) {
        const bool hasData = _manager != nullptr || _donePoints > 0;
        if (_btnCompare) _btnCompare->setEnabled(hasData);
        if (_btnViewResult) _btnViewResult->setEnabled(hasData);
    }
    _spinTorque->setEnabled(!running);
    _spinPopulation->setEnabled(!running);
    _spinGenerations->setEnabled(!running);
    _chkMinCpress->setEnabled(!running);
    _chkMinEdgeLoadRatio->setEnabled(!running);
    _chkMinSigmaMax->setEnabled(!running);
    _chkMinMass->setEnabled(!running);
    if (_cbDvModule) _cbDvModule->setEnabled(!running);
    if (_cbDvZ1) _cbDvZ1->setEnabled(!running);
    if (_cbDvZ2) _cbDvZ2->setEnabled(!running);
    if (_cbDvAlpha) _cbDvAlpha->setEnabled(!running);
    if (_cbDvX1) _cbDvX1->setEnabled(!running);
    if (_cbDvX2) _cbDvX2->setEnabled(!running);
    if (_cbDvCa1) _cbDvCa1->setEnabled(!running);
    if (_cbDvLca1) _cbDvLca1->setEnabled(!running);
    if (_cbDvCa2) _cbDvCa2->setEnabled(!running);
    if (_cbDvLca2) _cbDvLca2->setEnabled(!running);
    if (_cbDvWidth) {
        _cbDvWidth->setEnabled(false);
        _cbDvWidth->setChecked(false);
    }
    if (_cbDvHubRatio) _cbDvHubRatio->setEnabled(!running);
    if (_editRunDir) _editRunDir->setEnabled(!running);
    if (_spinThreads) _spinThreads->setEnabled(!running);
    if (_chkMeshAuto) _chkMeshAuto->setEnabled(!running);
    if (_spinMeshSize) _spinMeshSize->setEnabled(!running && _chkMeshAuto && !_chkMeshAuto->isChecked());
    if (_chkSurrogateAssisted) _chkSurrogateAssisted->setEnabled(!running);
    if (_chkParallelCcx) _chkParallelCcx->setEnabled(!running);
    if (_spinParallelCcxJobs)
        _spinParallelCcxJobs->setEnabled(!running && _chkParallelCcx && _chkParallelCcx->isChecked());
    if (_spinCcxThreadsPerJob)
        _spinCcxThreadsPerJob->setEnabled(!running && _chkParallelCcx && _chkParallelCcx->isChecked());
    if (_btnBackfillCpress) _btnBackfillCpress->setEnabled(!running);
    if (_btnRetryFailed) _btnRetryFailed->setEnabled(!running);
}

void GearOptDialog::setMeshIndependenceRunning(bool running) {
    if (_btnStart) _btnStart->setEnabled(!running);
    if (_btnMeshIndep) _btnMeshIndep->setEnabled(!running);
    if (_btnStop) _btnStop->setEnabled(false);
    _spinTorque->setEnabled(!running);
    if (_editRunDir) _editRunDir->setEnabled(!running);
    if (_spinThreads) _spinThreads->setEnabled(!running);
    if (_chkMeshAuto) _chkMeshAuto->setEnabled(!running);
    if (_spinMeshSize) _spinMeshSize->setEnabled(!running && _chkMeshAuto && !_chkMeshAuto->isChecked());
}

// ---- Slots ----

void GearOptDialog::onMeshIndependence() {
	if (_thread && _thread->isRunning()) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("优化正在运行，请先停止优化。"));
		return;
	}
	if (_meshIndepThread && _meshIndepThread->isRunning()) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("网格无关性验证已在运行。"));
		return;
	}

	const GearOptConfig cfg = currentConfig();
	if (cfg.solver.runBaseDir.trimmed().isEmpty()) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("请先选择「结果目录」。\n"
		                                       "验证结果将保存在该目录下的 mesh_independence/ 子目录。"));
		return;
	}

	_log->append(QStringLiteral("[MeshIndep] 启动网格无关性验证（A/B/C/D 四组工况）..."));
	setMeshIndependenceRunning(true);
	if (_labelStatus)
		_labelStatus->setText(QString::fromUtf8("网格无关性验证运行中..."));

	_meshIndepRunner = new MeshIndependenceRunner(cfg);
	_meshIndepThread = new QThread(this);
	_meshIndepRunner->moveToThread(_meshIndepThread);

	connect(_meshIndepThread, &QThread::started,
	        _meshIndepRunner, &MeshIndependenceRunner::execute);
	connect(_meshIndepRunner, &MeshIndependenceRunner::log,
	        this, &GearOptDialog::onLog, Qt::QueuedConnection);
	connect(_meshIndepRunner, &MeshIndependenceRunner::finished,
	        this, &GearOptDialog::onMeshIndependenceFinished, Qt::QueuedConnection);
	connect(_meshIndepRunner, &MeshIndependenceRunner::finished,
	        _meshIndepThread, &QThread::quit, Qt::QueuedConnection);
	connect(_meshIndepThread, &QThread::finished,
	        _meshIndepRunner, &QObject::deleteLater);
	connect(_meshIndepThread, &QThread::finished,
	        _meshIndepThread, &QObject::deleteLater);

	_meshIndepThread->start();
}

void GearOptDialog::onMeshIndependenceFinished(bool success, const QString& csvPath) {
	setMeshIndependenceRunning(false);
	if (_labelStatus) {
		_labelStatus->setText(success ? QString::fromUtf8("网格无关性验证完成")
		                              : QString::fromUtf8("网格无关性验证失败"));
	}
	if (success && !csvPath.isEmpty()) {
		QMessageBox::information(this, QString::fromUtf8("验证完成"),
		                         QString::fromUtf8("网格无关性验证已完成。\nCSV：%1").arg(csvPath));
	} else if (!success) {
		QMessageBox::warning(this, QString::fromUtf8("验证失败"),
		                     QString::fromUtf8("网格无关性验证未成功完成，请查看日志。"));
	}
}

void GearOptDialog::onStart() {
	if (_meshIndepThread && _meshIndepThread->isRunning()) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("网格无关性验证正在运行，请等待结束。"));
		return;
	}
	_ignoreManagerSlots = false;

    const GearOptConfig cfg = currentConfig();
    if (cfg.solver.runBaseDir.trimmed().isEmpty()) {
        QMessageBox::warning(this, QString::fromUtf8("提示"),
                             QString::fromUtf8("请先选择「结果目录」。\n"
                                               "本次运行的 GearOptResults.db 将保存在该目录；\n"
                                               "跨次累积的总库位于程序目录（Debug 输出目录）。"));
        return;
    }

    auto& globalDb = GearOptResultDatabase::global();
    if (!globalDb.isOpen()) {
        globalDb.openDatabase(GearOptResultDatabase::defaultGlobalDatabasePath());
    }

    _maxGen      = cfg.nsga2.maxGenerations;
    if (cfg.solver.surrogateAssisted) {
        const int minSamples = minSurrogateSampleCount(cfg.nsga2.populationSize);
        const int infillPerRound = std::min(5, std::max(3, std::max(1, cfg.nsga2.populationSize / 10)));
        int initialNeeded = minSamples;
        if (globalDb.isOpen()) {
            GearDesignPoint baseDp;
            if (Geometry::GeometryParaGear* g = findCurrentGeometryParaGear()) {
                baseDp = gearDesignPointFromGeometryParaGear(*g);
                baseDp.applyRunSimDefaults(cfg, cfg.solver.meshSize, cfg.solver.meshSize <= 0.0);
            } else {
                baseDp = cfg.useOptimizationBase ? cfg.optimizationBase : GearDesignPoint();
                baseDp.applyRunSimDefaults(cfg, cfg.solver.meshSize, cfg.solver.meshSize <= 0.0);
            }
            const QString baseCaseH = GearOptResultDatabase::baseCaseHash(baseDp);
            const int existing =
                globalDb.countValidatedSamplesForBaseCase(baseCaseH, baseDp.commonWidth);
            initialNeeded = std::max(0, minSamples - existing);
        }
        _totalPoints = initialNeeded + _maxGen * infillPerRound;
    } else {
        _totalPoints = cfg.nsga2.populationSize * (_maxGen + 1);
    }
    _donePoints  = 0;
    _progress->setRange(0, _totalPoints);
    _progress->setValue(0);
    _log->clear();
    _btnExport->setEnabled(false);

    const QString runDir = QDir(cfg.solver.runBaseDir.trimmed()).absolutePath();
    QDir().mkpath(runDir);

    _lastRunCfg = cfg;
    _lastRunDir = runDir;

    _log->append(QStringLiteral("[GearOpt][DB] 总库: %1（样本 %2）")
                     .arg(globalDb.isOpen() ? globalDb.databasePath()
                                            : GearOptResultDatabase::defaultGlobalDatabasePath())
                     .arg(globalDb.isOpen() ? globalDb.countValidResults() : 0));
    _log->append(QStringLiteral("[GearOpt][DB] 本次运行库: %1")
                     .arg(GearOptResultDatabase::databasePathInRunDir(runDir)));
    if (cfg.solver.surrogateAssisted) {
        _log->append(QStringLiteral("[GearOpt] 模式: 一键式代理辅助优化（自动 LHS 补样 → RBF → Infill）"));
    } else {
        _log->append(QStringLiteral("[GearOpt] 模式: 全 CCX 优化"));
    }

    _manager = new GearAutoOptManager;
    _manager->setConfig(cfg);
    _manager->setRunDir(runDir);

    _thread = new QThread(this);
    _manager->moveToThread(_thread);

    if (cfg.solver.surrogateAssisted) {
        connect(_thread, &QThread::started,
                _manager, &GearAutoOptManager::startSurrogateAssisted);
    } else {
        connect(_thread, &QThread::started,
                _manager, &GearAutoOptManager::start);
    }
    connect(_manager, &GearAutoOptManager::generationFinished,
            this, &GearOptDialog::onGenerationFinished, Qt::QueuedConnection);
    connect(_manager, &GearAutoOptManager::pointFinished,
            this, &GearOptDialog::onPointFinished, Qt::QueuedConnection);
    connect(_manager, &GearAutoOptManager::log,
            this, &GearOptDialog::onLog, Qt::QueuedConnection);
    connect(_manager, &GearAutoOptManager::finished,
            this, &GearOptDialog::onOptFinished, Qt::QueuedConnection);
    connect(_manager, &GearAutoOptManager::finished,
            _thread, &QThread::quit, Qt::QueuedConnection);
    connect(_thread, &QThread::finished,
            _manager, &QObject::deleteLater);
    connect(_thread, &QThread::finished,
            _thread, &QObject::deleteLater);

    setRunning(true);
    _labelStatus->setText(QString::fromUtf8("优化运行中..."));
    _thread->start();
}

void GearOptDialog::onStop() {
	_ignoreManagerSlots = true;

	if (_manager)
		QObject::disconnect(_manager, nullptr, this, nullptr);

	if (_manager)
		_manager->requestStop();

	// start() 在工作线程内同步执行，runOne 期间不处理事件循环，quit 往往不能立刻生效
	if (_thread && _thread->isRunning()) {
		_thread->quit();
		if (!_thread->wait(300000)) {
			_thread->terminate();
			_thread->wait(5000);
		}
	}

	setRunning(false);
	if (_labelStatus)
		_labelStatus->setText(QString::fromUtf8("已停止"));
}

void GearOptDialog::onMetricConfig()
{
	QMessageBox::information(
	    this,
	    QString::fromUtf8("优化配置"),
	    QString::fromUtf8("优化配置功能暂未启用，当前代理优化固定采用 cpressMax_MPa + edgeLoadRatio。"));
}

void GearOptDialog::onOfflineBackfillCpress()
{
	if ((_thread && _thread->isRunning()) || (_meshIndepThread && _meshIndepThread->isRunning())
	    || (_retryThread && _retryThread->isRunning())) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("当前有计算任务正在运行，请结束后再补算历史库。"));
		return;
	}

	const QString defaultPath = _editRunDir && !_editRunDir->text().trimmed().isEmpty()
	                                ? GearOptResultDatabase::databasePathInRunDir(_editRunDir->text().trimmed())
	                                : GearOptResultDatabase::defaultGlobalDatabasePath();
	const QString dbPath = QFileDialog::getOpenFileName(
	    this,
	    QString::fromUtf8("选择 GearOptResults.db"),
	    defaultPath,
	    QString::fromUtf8("SQLite 数据库 (*.db);;所有文件 (*.*)"));
	if (dbPath.isEmpty())
		return;

	auto& db = GearOptResultDatabase::global();
	if (!db.openDatabase(dbPath)) {
		QMessageBox::critical(this, QString::fromUtf8("补算失败"),
		                      QString::fromUtf8("无法打开数据库：\n") + dbPath);
		return;
	}

	const CpressDistributionBackfillStats stats =
	    db.backfillMissingCpressDistributionMetricsDetailed(11);
	db.closeDatabase();

	const QString msg = QString::fromUtf8(
	                        "total_missing = %1\n"
	                        "updated_count = %2\n"
	                        "skip_missing_frd = %3\n"
	                        "failed_parse_count = %4")
	                        .arg(stats.totalMissing)
	                        .arg(stats.updatedCount)
	                        .arg(stats.skipMissingFrd)
	                        .arg(stats.failedParseCount);
	if (_log)
		_log->append(QString::fromUtf8("[离线补算接触分布指标]\n") + msg);
	QMessageBox::information(this, QString::fromUtf8("补算完成"), msg);
}

void GearOptDialog::onRetryFailedCases()
{
	if ((_thread && _thread->isRunning()) || (_meshIndepThread && _meshIndepThread->isRunning())) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("当前有计算任务正在运行，请结束后再重算失败样本。"));
		return;
	}

	const GearOptConfig cfg = currentConfig();
	const QString runDir = cfg.solver.runBaseDir.trimmed();
	if (runDir.isEmpty()) {
		QMessageBox::warning(this, QString::fromUtf8("提示"),
		                     QString::fromUtf8("请先设置结果目录。"));
		return;
	}

	FailedCaseRetryOptions opt;
	opt.cfg                   = cfg;
	opt.runDir                = runDir;
	opt.fixedMeshSizeMm       = cfg.solver.meshSize;
	opt.fixedRootMeshSizeMm   = cfg.solver.meshRootSizeMm;
	opt.fixedZLayers          = cfg.solver.meshZLayers;
	opt.runMeshAuto           = cfg.solver.meshSize <= 0.0;
	opt.baseCaseHash          = QString();
	opt.fixedCommonWidthMm    = -1.0;

	if (_log)
		_log->append(QString::fromUtf8("[Retry] 开始重算 failed 样本（按数据库设计变量重新建模+CCX）…"));
	if (_labelStatus)
		_labelStatus->setText(QString::fromUtf8("失败样本重算中…"));
	setRunning(true);
	if (_btnRetryFailed) _btnRetryFailed->setEnabled(false);
	QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

	const FailedCaseRetryStats stats = GearOptFailedCaseRetry::run(
	    opt, [this](const QString& msg) {
		    if (_log)
			    _log->append(msg);
		    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
	    });

	onRetryFailedCasesFinished(stats);
}

void GearOptDialog::onRetryFailedCasesFinished(const FailedCaseRetryStats& stats)
{
	setRunning(false);
	if (_btnRetryFailed) _btnRetryFailed->setEnabled(true);
	if (_labelStatus)
		_labelStatus->setText(QString::fromUtf8("失败样本重算完成"));

	const QString msg = QString::fromUtf8(
	                        "total_failed = %1\n"
	                        "retried = %2\n"
	                        "fixed = %3\n"
	                        "still_failed = %4\n"
	                        "invalid_skipped = %5\n"
	                        "mesh_timeout_failed = %6")
	                        .arg(stats.totalFailed)
	                        .arg(stats.retried)
	                        .arg(stats.fixed)
	                        .arg(stats.stillFailed)
	                        .arg(stats.invalidSkipped)
	                        .arg(stats.meshTimeoutFailed);
	if (_log)
		_log->append(QString::fromUtf8("[Retry] 完成\n") + msg);
	QMessageBox::information(this, QString::fromUtf8("重算完成"), msg);
}

void GearOptDialog::onExport() {
    const QString path = QFileDialog::getSaveFileName(
        this, QString::fromUtf8("保存结果"),
        QDir::currentPath() + "/gear_opt_results.csv",
        QString::fromUtf8("CSV 文件 (*.csv);;所有文件 (*.*)"));
    if (path.isEmpty()) return;

    if (_cachedAllPoints.isEmpty()) {
        QMessageBox::warning(this, QString::fromUtf8("无数据"),
                             QString::fromUtf8("请先运行优化后再导出。"));
        return;
    }

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QMessageBox::critical(this, QString::fromUtf8("错误"),
                              QString::fromUtf8("无法写入文件：") + path);
        return;
    }
    QTextStream ts(&f);
    ts.setCodec("UTF-8");
    ts << GearOptResultDatabase::csvHeaderLine();

    const QList<GearDesignPoint>& pareto = _cachedPareto;
    for (const auto& gen : _cachedAllPoints) {
        for (GearDesignPoint dp : gen) {
            const bool isPareto = std::any_of(pareto.constBegin(), pareto.constEnd(),
                [&](const GearDesignPoint& p) {
                    return p.generation == dp.generation && p.id == dp.id;
                });
            if (isPareto)
                dp.isPareto = 1;
            GearOptResultDatabase::writeCsvRow(ts, dp);
        }
    }
    _log->append(QString("已导出 %1").arg(path));
    QMessageBox::information(this, QString::fromUtf8("导出完成"),
                             QString("CSV 文件已保存：%1\n"
                                     "（QXlsx submodule 集成后可直接导出 .xlsx）").arg(path));
}

void GearOptDialog::onGenerationFinished(int gen, int paretoSize, double hv) {
	if (_ignoreManagerSlots)
		return;
	if (_labelStatus)
		_labelStatus->setText(QString("Gen %1/%2 | CCX Pareto: %3 | CCX HV: %4")
		                      .arg(gen).arg(_maxGen).arg(paretoSize).arg(hv, 0, 'g', 4));
	if (_log)
		_log->append(QString("[gen%1] CCX Pareto=%2 CCX HV=%3 (authoritative)")
		             .arg(gen).arg(paretoSize).arg(hv, 0, 'g', 5));
}

void GearOptDialog::onPointFinished(int /*gen*/, int /*idx*/, GearDesignPoint /*dp*/) {
	if (_ignoreManagerSlots)
		return;
	_donePoints++;
	if (_progress)
		_progress->setValue(_donePoints);
}

void GearOptDialog::onLog(const QString& msg) {
	if (_ignoreManagerSlots || !_log)
		return;
	_log->append(msg);
}

void GearOptDialog::onOptFinished(bool success) {
	if (_ignoreManagerSlots) {
		// 析构/停止过程中晚到的 finished，避免访问已销毁子控件
		return;
	}
	if (_manager) {
		_cachedAllPoints = _manager->allPoints();
		_cachedPareto    = _manager->paretoFront();
	}
	const QString cmpPath = exportOptimizationCompareTable(_lastRunCfg, _cachedAllPoints, _lastRunDir, nullptr);
	if (_log && !cmpPath.isEmpty())
		_log->append(QString::fromUtf8("对比表 CSV：") + cmpPath);
	setRunning(false);
	_btnExport->setEnabled(!_cachedAllPoints.isEmpty());
	if (_btnCompare) _btnCompare->setEnabled(!_cachedAllPoints.isEmpty());
	if (_btnViewResult) _btnViewResult->setEnabled(!_cachedAllPoints.isEmpty());
	if (_labelStatus) {
		_labelStatus->setText(success ? QString::fromUtf8("优化完成")
		                              : QString::fromUtf8("已停止/出错"));
	}
	if (_log) {
		_log->append(success ? QString::fromUtf8("=== 优化完成 ===")
		                     : QString::fromUtf8("=== 优化中断 ==="));
	}
	// QThread::finished 上会 deleteLater；QPointer 随后自动置空
}

} // namespace GearAutoOpt
