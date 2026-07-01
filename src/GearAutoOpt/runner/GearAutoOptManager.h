// UTF-8 BOM
#ifndef _GEARAUTOOPT_GEAR_AUTO_OPT_MANAGER_H_
#define _GEARAUTOOPT_GEAR_AUTO_OPT_MANAGER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearOptConfig.h"
#include "GearAutoOpt/opt/NSGA2.h"
#include "GearAutoOpt/surrogate/GearSurrogateModel.h"

#include <QObject>
#include <QThread>
#include <QVector>
#include <functional>

namespace GearAutoOpt {

struct SurrogateCcxLogOptions;

/// CCX 评估统计（infill 验证时区分新算与 cache 复用）。
struct GEARAUTOOPTAPI CcxEvalSummary {
	int           newCcxCount     = 0;
	int           reusedCacheCount = 0;
	QVector<bool> cacheHitByIndex;
};

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

    /// 提取最终 Pareto 前沿（代理模式结束后为 CCX 真实验证样本前沿）
    QList<GearDesignPoint> paretoFront() const;

    /// 代理优化结束时由 CCX 验证样本构成的全量真实设计点（供导出/对比表）
    const QList<GearDesignPoint>& surrogateValidatedPoints() const { return _surrogateValidatedPoints; }

public slots:
    /// 启动优化（在 QThread::started 信号后调用）
    void start();
    void startSurrogateAssisted();

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
    void evaluatePopulationByCcx(
        Population& pop,
        int generation,
        CcxEvalSummary* summary = nullptr,
        const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback = {},
        const SurrogateCcxLogOptions* ccxLog = nullptr);
    /// CCX 批量评估（初始 LHS 补样与 infill 验证共用）：先串行 CAD/网格，再按配置并行 CCX。
    void evaluateInfillByCcx(
        Population& pop,
        int generation,
        CcxEvalSummary* summary = nullptr,
        const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback = {},
        const SurrogateCcxLogOptions* ccxLog = nullptr);
    void evaluateInfillByCcxParallel(
        Population& pop,
        int generation,
        CcxEvalSummary* summary,
        const std::function<void(int, const GearDesignPoint&, bool)>& pointCallback,
        const SurrogateCcxLogOptions* ccxLog = nullptr);
    void evaluatePopulationBySurrogate(Population& pop,
                                       int generation,
                                       const GearSurrogateModel& model);

    void syncGenerationParetoToDatabase(int generation, const Population& pop);
    void syncFinalParetoToDatabase();
    void openResultDatabases();
    GearDesignPoint configuredBasePoint() const;

    GearOptConfig _cfg;
    QString       _runDir;
    bool          _stopRequested{false};

    /// 本次优化 run 固定的网格设置（start() 时从 config 快照，全代一致）
    double _fixedMeshSizeMm{1.50};
    double _fixedRootMeshSizeMm{0.40};
    int    _fixedZLayers{11};
    bool   _runMeshAuto{false};
    QString _runId;
    bool    _surrogateThresholdLogged{false};
    double  _fixedCommonWidthMm{10.0};

    // in-memory 全量记录，按代分组
    QVector<QList<GearDesignPoint>> _allPoints;

    // NSGA-II 种群
    Population _population;

    /// 代理模式：从 DB 重载的全部 CCX 验证样本（真实目标值，非代理预测）
    QList<GearDesignPoint> _surrogateValidatedPoints;
};

} // namespace GearAutoOpt
#endif
