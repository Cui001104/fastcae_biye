// UTF-8 BOM
#ifndef _GEARAUTOOPT_GEAR_AUTO_OPT_MANAGER_H_
#define _GEARAUTOOPT_GEAR_AUTO_OPT_MANAGER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/opt/NSGA2.h"

#include <QObject>
#include <QThread>
#include <QVector>

namespace GearAutoOpt {

/// NSGA-II 多目标优化总控器。
///
/// 运行于独立的 QThread 中（通过 moveToThread + 信号触发）：
///   1. 用 initLatin 生成初始种群（N 个设计点）
///   2. 调 GearOptCaseRunner::runOne() 依次评估每个个体
///   3. 调 evolve() 生成下一代子代
///   4. 合并 selectNextGen() 精英选择
///   5. 循环直至超体积收敛或达到 maxGenerations
///
/// 典型用法：
/// @code
///   auto* mgr = new GearAutoOptManager;
///   mgr->setConfig(cfg);
///   mgr->setRunDir("runs/opt_001");
///   auto* thread = new QThread;
///   mgr->moveToThread(thread);
///   connect(thread, &QThread::started, mgr, &GearAutoOptManager::start);
///   connect(mgr, &GearAutoOptManager::finished, thread, &QThread::quit);
///   thread->start();
/// @endcode
class GEARAUTOOPTAPI GearAutoOptManager : public QObject {
    Q_OBJECT
public:
    explicit GearAutoOptManager(QObject* parent = nullptr);
    ~GearAutoOptManager() override;

    void setConfig(const GearOptConfig& cfg) { _cfg = cfg; }
    const GearOptConfig& config() const      { return _cfg; }

    /// 优化结果目录根（每代每个个体在 {runDir}/{gen}_{id}/ 下运行）
    void setRunDir(const QString& dir) { _runDir = dir; }

    /// 中止正在进行的优化（发信号给运行中的代）。
    void requestStop() { _stopRequested = true; }

    /// 当前所有已评估个体（按代分组）
    const QVector<QList<GearDesignPoint>>& allPoints() const { return _allPoints; }

    /// 提取最终 Pareto 前沿
    QList<GearDesignPoint> paretoFront() const;

public slots:
    /// 启动优化（在 QThread::started 信号后调用）
    void start();

signals:
    /// 一代评估完成（gen = 代号，1-based；paretoSize = 当前 Pareto 集大小）
    void generationFinished(int gen, int paretoSize, double hypervolume);
    /// 单个设计点完成（gen, idx 在本代中的序号，dp 结果）
    void pointFinished(int gen, int idx, GearAutoOpt::GearDesignPoint dp);
    /// 进度日志
    void log(const QString& msg);
    /// 优化完成（success=true 表示正常收敛/达到代数上限；false 表示中途出错）
    void finished(bool success);

private:
    /// 对 population 中未评估的个体逐一调 runOne，填写 objs。
    void evaluatePopulation(Population& pop, int generation);

    GearOptConfig _cfg;
    QString       _runDir;
    bool          _stopRequested{false};

    // in-memory 全量记录，按代分组
    QVector<QList<GearDesignPoint>> _allPoints;

    // NSGA-II 种群
    Population _population;
};

} // namespace GearAutoOpt
#endif
