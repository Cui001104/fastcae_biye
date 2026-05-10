// UTF-8 BOM
#ifndef _GEARAUTOOPT_GEAR_OPT_DIALOG_H_
#define _GEARAUTOOPT_GEAR_OPT_DIALOG_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/data/GearDesignPoint.h"

#include <QDialog>
#include <QThread>

class QLabel;
class QLineEdit;
class QCheckBox;
class QPushButton;
class QProgressBar;
class QTextEdit;
class QDoubleSpinBox;
class QSpinBox;
class QGroupBox;

namespace GearAutoOpt {

class GearAutoOptManager;

/// 齿轮多目标优化主对话框。
class GEARAUTOOPTAPI GearOptDialog : public QDialog {
    Q_OBJECT
public:
    explicit GearOptDialog(QWidget* parent = nullptr);
    ~GearOptDialog() override;

    /// 读取 UI 中的参数，构造 GearOptConfig
    GearOptConfig currentConfig() const;

private slots:
    void onStart();
    void onStop();
    void onExport();
    void onGenerationFinished(int gen, int paretoSize, double hv);
    void onPointFinished(int gen, int idx, GearAutoOpt::GearDesignPoint dp);
    void onLog(const QString& msg);
    void onOptFinished(bool success);

private:
    void buildUi();
    void setRunning(bool running);

    // 配置参数
    QDoubleSpinBox* _spinTorque{};     // N·m
    QSpinBox*       _spinPopulation{};
    QSpinBox*       _spinGenerations{};
    QCheckBox*      _chkMinSigma{};
    QCheckBox*      _chkMinMass{};

    // 求解设置
    QLineEdit*      _editRunDir{};     // 结果保存路径
    QSpinBox*       _spinThreads{};   // OMP_NUM_THREADS（0=自动）
    QCheckBox*      _chkMeshAuto{};   // 网格尺寸自动
    QDoubleSpinBox* _spinMeshSize{};  // 网格尺寸（mm）

    // 进度
    QProgressBar*   _progress{};
    QLabel*         _labelStatus{};

    // 日志
    QTextEdit*      _log{};

    // 按钮
    QPushButton*    _btnStart{};
    QPushButton*    _btnStop{};
    QPushButton*    _btnViewResult{};
    QPushButton*    _btnExport{};

    // 运行
    GearAutoOptManager* _manager{};
    QThread*            _thread{};

    // 结果
    int _maxGen{0};
    int _totalPoints{0};
    int _donePoints{0};

    QVector<QList<GearDesignPoint>> _cachedAllPoints;
    QList<GearDesignPoint>          _cachedPareto;
};

} // namespace GearAutoOpt
#endif
