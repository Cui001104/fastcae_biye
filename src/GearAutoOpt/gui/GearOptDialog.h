// UTF-8 BOM
#ifndef _GEARAUTOOPT_GEAR_OPT_DIALOG_H_
#define _GEARAUTOOPT_GEAR_OPT_DIALOG_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/data/GearDesignPoint.h"

#include <QDialog>
#include <QPointer>
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
class QCloseEvent;

namespace GearAutoOpt {

class GearAutoOptManager;
class MeshIndependenceRunner;

/// 齿轮多目标优化主对话框。
class GEARAUTOOPTAPI GearOptDialog : public QDialog {
    Q_OBJECT
public:
    explicit GearOptDialog(QWidget* parent = nullptr);
    ~GearOptDialog() override;

    /// 读取 UI 中的参数，构造 GearOptConfig
    GearOptConfig currentConfig() const;

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void onStart();
    void onStop();
    void onMeshIndependence();
    void onMeshIndependenceFinished(bool success, const QString& csvPath);
    void onOfflineBackfillCpress();
    void onExport();
    void onMetricConfig();
    void onGenerationFinished(int gen, int paretoSize, double hv);
    void onPointFinished(int gen, int idx, GearAutoOpt::GearDesignPoint dp);
    void onLog(const QString& msg);
    void onOptFinished(bool success);

private:
    void buildUi();
    void setRunning(bool running);
    void setMeshIndependenceRunning(bool running);

    // 配置参数
    QDoubleSpinBox* _spinTorque{};     // N·m
    QSpinBox*       _spinPopulation{};
    QSpinBox*       _spinGenerations{};
    QCheckBox*      _chkMinCpress{};
    QCheckBox*      _chkMinEdgeLoadRatio{};
    QCheckBox*      _chkMinSigmaMax{};
    QCheckBox*      _chkMinMass{};

    // 设计变量（参与 NSGA-II；未勾选则固定为当前建模值）
    QCheckBox*      _cbDvModule{};
    QCheckBox*      _cbDvZ1{};
    QCheckBox*      _cbDvZ2{};
    QCheckBox*      _cbDvAlpha{};
    QCheckBox*      _cbDvX1{};
    QCheckBox*      _cbDvX2{};
    QCheckBox*      _cbDvCa1{};
    QCheckBox*      _cbDvLca1{};
    QCheckBox*      _cbDvCa2{};
    QCheckBox*      _cbDvLca2{};
    QCheckBox*      _cbDvWidth{};
    QCheckBox*      _cbDvHubRatio{};

    // 求解设置
    QLineEdit*      _editRunDir{};     // 结果保存路径
    QSpinBox*       _spinThreads{};   // OMP_NUM_THREADS（0=自动）
    QCheckBox*      _chkMeshAuto{};   // 网格尺寸自动
    QDoubleSpinBox* _spinMeshSize{};  // 网格尺寸（mm）
    QCheckBox*      _chkSurrogateAssisted{}; // 代理辅助 NSGA-II
    QCheckBox*      _chkParallelCcx{};      // 并行 CCX infill 验证
    QSpinBox*       _spinParallelCcxJobs{};
    QSpinBox*       _spinCcxThreadsPerJob{};

    // 进度
    QProgressBar*   _progress{};
    QLabel*         _labelStatus{};

    // 日志（QPointer：工作线程晚到的 log 槽不得在已销毁控件上 append）
    QPointer<QTextEdit> _log{};

    // 按钮
    QPushButton*    _btnStart{};
    QPushButton*    _btnMeshIndep{};
    QPushButton*    _btnStop{};
    QPushButton*    _btnCompare{};
    QPushButton*    _btnViewResult{};
    QPushButton*    _btnMetricConfig{};
    QPushButton*    _btnBackfillCpress{};
    QPushButton*    _btnExport{};

    // 运行（QPointer 配合 deleteLater，避免悬空指针）
    QPointer<GearAutoOptManager> _manager{};
    QPointer<QThread>            _thread{};

    QPointer<MeshIndependenceRunner> _meshIndepRunner{};
    QPointer<QThread>                _meshIndepThread{};

    /// 最近一次「开始优化」时的配置与工作目录（用于对比表与 CSV）
    GearOptConfig _lastRunCfg{};
    QString        _lastRunDir;

    /// 已断开 Manager→本窗口信号或正在析构，忽略晚到的槽
    bool _ignoreManagerSlots{false};

    // 结果
    int _maxGen{0};
    int _totalPoints{0};
    int _donePoints{0};

    QVector<QList<GearDesignPoint>> _cachedAllPoints;
    QList<GearDesignPoint>          _cachedPareto;
};

} // namespace GearAutoOpt
#endif
