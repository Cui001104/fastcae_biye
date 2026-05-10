// UTF-8 BOM
#include "GearOptDialog.h"
#include "GearAutoOpt/runner/GearAutoOptManager.h"
#include "GearOptResultViewer.h"

#include <algorithm>
#include <QCheckBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QTextEdit>
#include <QTextStream>
#include <QThread>
#include <QVBoxLayout>

namespace GearAutoOpt {

GearOptDialog::GearOptDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("齿轮多目标优化"));
    setMinimumSize(600, 500);
    buildUi();
}

GearOptDialog::~GearOptDialog() {
    onStop();
}

void GearOptDialog::buildUi() {
    auto* mainLayout = new QVBoxLayout(this);

    // ---- 参数面板 ----
    auto* cfgGroup = new QGroupBox(QStringLiteral("优化参数"), this);
    auto* cfgLayout = new QHBoxLayout(cfgGroup);

    // 左列：数值参数
    auto* leftLayout = new QVBoxLayout;
    auto addSpinRow = [&](const QString& label, QWidget* spin) {
        auto* row = new QHBoxLayout;
        row->addWidget(new QLabel(label));
        row->addWidget(spin);
        leftLayout->addLayout(row);
    };

    _spinTorque = new QDoubleSpinBox(this);
    _spinTorque->setRange(1.0, 10000.0);
    _spinTorque->setValue(100.0);
    _spinTorque->setSuffix(QStringLiteral(" N·m"));
    _spinTorque->setDecimals(1);
    addSpinRow(QStringLiteral("输入扭矩:"), _spinTorque);

    _spinPopulation = new QSpinBox(this);
    _spinPopulation->setRange(10, 200);
    _spinPopulation->setValue(30);
    addSpinRow(QStringLiteral("种群大小:"), _spinPopulation);

    _spinGenerations = new QSpinBox(this);
    _spinGenerations->setRange(1, 100);
    _spinGenerations->setValue(20);
    addSpinRow(QStringLiteral("最大代数:"), _spinGenerations);
    leftLayout->addStretch();

    // 右列：目标选择
    auto* rightLayout = new QVBoxLayout;
    rightLayout->addWidget(new QLabel(QStringLiteral("优化目标:")));
    _chkMinSigma = new QCheckBox(QStringLiteral("最小化最大应力"), this);
    _chkMinSigma->setChecked(true);
    _chkMinMass  = new QCheckBox(QStringLiteral("最小化质量"), this);
    _chkMinMass->setChecked(true);
    rightLayout->addWidget(_chkMinSigma);
    rightLayout->addWidget(_chkMinMass);
    rightLayout->addStretch();

    cfgLayout->addLayout(leftLayout, 2);
    cfgLayout->addLayout(rightLayout, 1);
    mainLayout->addWidget(cfgGroup);

    // ---- 进度 ----
    auto* progressRow = new QHBoxLayout;
    _progress = new QProgressBar(this);
    _progress->setRange(0, 100);
    _progress->setValue(0);
    _labelStatus = new QLabel(QStringLiteral("就绪"), this);
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
    _btnStart      = new QPushButton(QStringLiteral("开始优化"), this);
    _btnStop       = new QPushButton(QStringLiteral("停止"), this);
    _btnViewResult = new QPushButton(QStringLiteral("查看结果..."), this);
    _btnExport     = new QPushButton(QStringLiteral("导出 Excel..."), this);
    _btnStop->setEnabled(false);
    _btnViewResult->setEnabled(false);
    _btnExport->setEnabled(false);
    btnRow->addStretch();
    btnRow->addWidget(_btnStart);
    btnRow->addWidget(_btnStop);
    btnRow->addWidget(_btnViewResult);
    btnRow->addWidget(_btnExport);
    mainLayout->addLayout(btnRow);

    // ---- 求解设置 ----
    auto* solverGroup = new QGroupBox(QStringLiteral("求解设置"), this);
    auto* solverLayout = new QHBoxLayout(solverGroup);

    // 结果路径
    solverLayout->addWidget(new QLabel(QStringLiteral("结果目录:"), this));
    _editRunDir = new QLineEdit(this);
    _editRunDir->setPlaceholderText(QStringLiteral("默认：runs/opt_<时间戳>"));
    _editRunDir->setMinimumWidth(200);
    solverLayout->addWidget(_editRunDir, 2);
    auto* btnBrowse = new QPushButton(QStringLiteral("浏览..."), this);
    solverLayout->addWidget(btnBrowse);

    solverLayout->addSpacing(16);

    // 求解线程数
    solverLayout->addWidget(new QLabel(QStringLiteral("求解线程:"), this));
    _spinThreads = new QSpinBox(this);
    _spinThreads->setRange(0, 256);
    _spinThreads->setValue(0);
    _spinThreads->setSpecialValueText(QStringLiteral("自动"));
    _spinThreads->setToolTip(QStringLiteral("OMP_NUM_THREADS，0=跟随系统默认"));
    solverLayout->addWidget(_spinThreads);

    solverLayout->addSpacing(16);

    // 网格尺寸
    solverLayout->addWidget(new QLabel(QStringLiteral("网格尺寸:"), this));
    _chkMeshAuto = new QCheckBox(QStringLiteral("自动"), this);
    _chkMeshAuto->setChecked(true);
    _spinMeshSize = new QDoubleSpinBox(this);
    _spinMeshSize->setRange(0.1, 20.0);
    _spinMeshSize->setValue(1.0);
    _spinMeshSize->setSingleStep(0.1);
    _spinMeshSize->setDecimals(2);
    _spinMeshSize->setSuffix(QStringLiteral(" mm"));
    _spinMeshSize->setEnabled(false);
    solverLayout->addWidget(_chkMeshAuto);
    solverLayout->addWidget(_spinMeshSize);
    solverLayout->addStretch();
    mainLayout->addWidget(solverGroup);

    connect(btnBrowse, &QPushButton::clicked, this, [this]() {
        const QString dir = QFileDialog::getExistingDirectory(
            this, QStringLiteral("选择结果保存目录"),
            _editRunDir->text().isEmpty() ? QDir::currentPath() : _editRunDir->text());
        if (!dir.isEmpty()) _editRunDir->setText(dir);
    });
    connect(_chkMeshAuto, &QCheckBox::toggled, this, [this](bool checked) {
        _spinMeshSize->setEnabled(!checked);
    });

    connect(_btnStart,      &QPushButton::clicked, this, &GearOptDialog::onStart);
    connect(_btnStop,       &QPushButton::clicked, this, &GearOptDialog::onStop);
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
}

GearOptConfig GearOptDialog::currentConfig() const {
    GearOptConfig cfg = GearOptConfig::defaultConfig();
    cfg.solver.torque          = _spinTorque->value();
    cfg.nsga2.populationSize   = _spinPopulation->value();
    cfg.nsga2.maxGenerations   = _spinGenerations->value();
    cfg.objectives.minSigmaMax = _chkMinSigma->isChecked();
    cfg.objectives.minMass     = _chkMinMass->isChecked();
    cfg.solver.meshSize        = _chkMeshAuto->isChecked() ? 0.0 : _spinMeshSize->value();
    cfg.solver.runBaseDir      = _editRunDir->text().trimmed();
    cfg.solver.threads         = _spinThreads->value();
    return cfg;
}

void GearOptDialog::setRunning(bool running) {
    _btnStart->setEnabled(!running);
    _btnStop->setEnabled(running);
    if (!running) _btnViewResult->setEnabled(_manager != nullptr || _donePoints > 0);
    _spinTorque->setEnabled(!running);
    _spinPopulation->setEnabled(!running);
    _spinGenerations->setEnabled(!running);
    _chkMinSigma->setEnabled(!running);
    _chkMinMass->setEnabled(!running);
}

// ---- Slots ----

void GearOptDialog::onStart() {
    const GearOptConfig cfg = currentConfig();
    _maxGen      = cfg.nsga2.maxGenerations;
    _totalPoints = cfg.nsga2.populationSize * (_maxGen + 1);
    _donePoints  = 0;
    _progress->setRange(0, _totalPoints);
    _progress->setValue(0);
    _log->clear();
    _btnExport->setEnabled(false);

    // 工作目录：优先用用户填写的路径，否则用时间戳目录
    const QString runDir = cfg.solver.runBaseDir.isEmpty()
        ? QDir::currentPath() + "/runs/opt_"
          + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")
        : cfg.solver.runBaseDir;

    _manager = new GearAutoOptManager;
    _manager->setConfig(cfg);
    _manager->setRunDir(runDir);

    _thread = new QThread(this);
    _manager->moveToThread(_thread);

    connect(_thread, &QThread::started,
            _manager, &GearAutoOptManager::start);
    connect(_manager, &GearAutoOptManager::generationFinished,
            this, &GearOptDialog::onGenerationFinished);
    connect(_manager, &GearAutoOptManager::pointFinished,
            this, &GearOptDialog::onPointFinished);
    connect(_manager, &GearAutoOptManager::log,
            this, &GearOptDialog::onLog);
    connect(_manager, &GearAutoOptManager::finished,
            this, &GearOptDialog::onOptFinished);
    connect(_manager, &GearAutoOptManager::finished,
            _thread, &QThread::quit);
    connect(_thread, &QThread::finished,
            _manager, &QObject::deleteLater);
    connect(_thread, &QThread::finished,
            _thread, &QObject::deleteLater);

    setRunning(true);
    _labelStatus->setText(QStringLiteral("优化运行中..."));
    _thread->start();
}

void GearOptDialog::onStop() {
    if (_manager) _manager->requestStop();
    if (_thread && _thread->isRunning()) {
        _thread->quit();
        _thread->wait(3000);
    }
    _manager = nullptr;
    _thread  = nullptr;
    setRunning(false);
    _labelStatus->setText(QStringLiteral("已停止"));
}

void GearOptDialog::onExport() {
    const QString path = QFileDialog::getSaveFileName(
        this, QStringLiteral("保存结果"),
        QDir::currentPath() + "/gear_opt_results.csv",
        QStringLiteral("CSV 文件 (*.csv);;所有文件 (*.*)"));
    if (path.isEmpty()) return;

    if (_cachedAllPoints.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("无数据"),
                             QStringLiteral("请先运行优化后再导出。"));
        return;
    }

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QMessageBox::critical(this, QStringLiteral("错误"),
                              QStringLiteral("无法写入文件：") + path);
        return;
    }
    QTextStream ts(&f);
    ts.setCodec("UTF-8");
    // 表头
    ts << "gen,id,z1,z2,module,alpha,x1,x2,ca1,lca1,ca2,lca2,width,hubRatio,"
          "sigmaMax_MPa,uMax_mm,mass_kg,status,errorMsg\n";

    const QList<GearDesignPoint>& pareto = _cachedPareto;
    for (const auto& gen : _cachedAllPoints) {
        for (const GearDesignPoint& dp : gen) {
            const bool isPareto = std::any_of(pareto.constBegin(), pareto.constEnd(),
                [&](const GearDesignPoint& p) {
                    return p.generation == dp.generation && p.id == dp.id;
                });
            ts << dp.generation << ',' << dp.id << ','
               << dp.z1 << ',' << dp.z2 << ','
               << dp.module << ',' << dp.alpha << ','
               << dp.x1 << ',' << dp.x2 << ','
               << dp.ca1 << ',' << dp.lca1 << ','
               << dp.ca2 << ',' << dp.lca2 << ','
               << dp.width << ',' << dp.hubRatio << ','
               << dp.sigmaMax << ',' << dp.uMax << ',' << dp.mass << ','
               << pointStatusToString(dp.status) << ','
               << '"' << QString(dp.errorMsg).replace('"', '\'') << '"'
               << (isPareto ? ",PARETO" : "") << '\n';
        }
    }
    _log->append(QString("已导出 %1").arg(path));
    QMessageBox::information(this, QStringLiteral("导出完成"),
                             QString("CSV 文件已保存：%1\n"
                                     "（QXlsx submodule 集成后可直接导出 .xlsx）").arg(path));
}

void GearOptDialog::onGenerationFinished(int gen, int paretoSize, double hv) {
    _labelStatus->setText(QString("Gen %1/%2 | Pareto: %3 | HV: %4")
                          .arg(gen).arg(_maxGen).arg(paretoSize).arg(hv, 0, 'g', 4));
    _log->append(QString("[gen%1] Pareto=%2 HV=%3")
                 .arg(gen).arg(paretoSize).arg(hv, 0, 'g', 5));
}

void GearOptDialog::onPointFinished(int /*gen*/, int /*idx*/, GearDesignPoint /*dp*/) {
    _donePoints++;
    _progress->setValue(_donePoints);
}

void GearOptDialog::onLog(const QString& msg) {
    _log->append(msg);
}

void GearOptDialog::onOptFinished(bool success) {
    if (_manager) {
        _cachedAllPoints = _manager->allPoints();
        _cachedPareto    = _manager->paretoFront();
    }
    setRunning(false);
    _btnExport->setEnabled(!_cachedAllPoints.isEmpty());
    _btnViewResult->setEnabled(!_cachedAllPoints.isEmpty());
    _labelStatus->setText(success
        ? QStringLiteral("优化完成")
        : QStringLiteral("已停止/出错"));
    _log->append(success
        ? QStringLiteral("=== 优化完成 ===")
        : QStringLiteral("=== 优化中断 ==="));
    _manager = nullptr;
    _thread  = nullptr;
}

} // namespace GearAutoOpt
