#ifndef _GEARAUTOOPT_GEAR_OPT_RESULT_DATABASE_H_
#define _GEARAUTOOPT_GEAR_OPT_RESULT_DATABASE_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearDesignPoint.h"
#include "GearAutoOpt/data/GearMetricDefinition.h"
#include "GearAutoOpt/solver/CCXResultParser.h"
#include "GearAutoOpt/surrogate/GearSurrogateModel.h"

#include <QMetaType>
#include <QSet>
#include <QString>
#include <QVector>

class QTextStream;

namespace GearAutoOpt {

struct CpressDistributionBackfillStats {
	int totalMissing{0};
	int updatedCount{0};
	int skipMissingFrd{0};
	int failedParseCount{0};
};

/// RBF 训练样本加载统计（total / valid / skipped）。
struct SurrogateSampleLoadStats {
	int totalSamples{0};
	int validSamples{0};
	int failedSkipped{0};
};

struct FailedCaseDbRow {
	int     rowId{-1};
	QString databasePath;
	int     retryCount{0};
};

/// 待重算的 failed 样本（按 case_hash 去重后的一条记录）。
struct FailedCaseRecord {
	QString                  caseHash;
	GearDesignPoint          dp;
	QVector<FailedCaseDbRow> dbRows;
};

struct FailedCaseRetryStats {
	int totalFailed{0};
	int retried{0};
	int fixed{0};
	int stillFailed{0};
	int invalidSkipped{0};
	int meshTimeoutFailed{0};
};

/// SQLite 持久化：gear_opt_results 主表。
/// - global()：程序目录总库，跨次优化累积，case_hash 去重缓存。
/// - runSession()：用户所选「结果目录」下本次运行的库，优化结束关闭。
class GEARAUTOOPTAPI GearOptResultDatabase {
public:
	/// 总库（FastCAE.exe 同目录 GearOptResults.db）。
	static GearOptResultDatabase& global();
	/// 本次优化运行库（结果目录/GearOptResults.db）。
	static GearOptResultDatabase& runSession();
	/// 兼容旧代码，等同 global()。
	static GearOptResultDatabase& instance();

	/// 打开数据库（不存在则创建）。dbPath 为空时用 defaultGlobalDatabasePath()。
	bool openDatabase(const QString& dbPath = QString());
	bool initialize(const QString& dbPath = QString()) { return openDatabase(dbPath); }

	bool createTablesIfNeeded();
	bool isOpen() const;
	void closeDatabase();
	QString databasePath() const { return _dbPath; }

	/// 总库默认路径：applicationDirPath()/GearOptResults.db
	static QString defaultGlobalDatabasePath();
	static QString databasePathInRunDir(const QString& runBaseDir);

	/// run_id = YYYYMMDD_HHMMSS_seed
	static QString generateRunId(int randomSeed);

	static QString paramHash(const GearDesignPoint& dp);
	static QString caseHash(const GearDesignPoint& dp);
	static QString baseCaseHash(const GearDesignPoint& dp);
	static QString designHash(const GearDesignPoint& dp);

	bool findIdByCaseHash(const QString& hash, int* outId = nullptr) const;
	bool loadResultByCaseHash(const QString& hash, GearDesignPoint& dp) const;
	bool designHashExists(const QString& baseCaseHash,
	                      const QString& designHash) const;
	/// 同一 baseCaseHash 且 verified_by_ccx=1、status=done 的 CCX 样本数。
	/// fixedWidthMm>=0 时仅统计齿宽与固定值一致的样本。
	int countValidatedSamplesForBaseCase(const QString& baseCaseHash,
	                                     double fixedWidthMm = -1.0) const;

	/// 加载可用于 RBF 训练的样本；maxCount<=0 表示不限制条数。
	/// fixedWidthMm>=0 时仅加载 ABS(width-fixedWidth)<1e-6 的样本。
	/// stats 非空时填充 total / valid / failedSkipped 计数并写 debug 日志。
	QVector<SurrogateSample> loadValidatedSamples(const QString& baseCaseHash,
	                                              double fixedWidthMm = -1.0,
	                                              int maxCount = 0,
	                                              SurrogateSampleLoadStats* stats = nullptr) const;

	/// status='failed' 或 cpressMax_MPa<0 或 is_valid=0 的 case_hash（infill 黑名单）。
	QSet<QString> failedCaseHashesForBaseCase(const QString& baseCaseHash,
	                                          double fixedWidthMm = -1.0) const;

	/// 从当前库查询可重算的 failed 样本行（不含去重）。
	QVector<FailedCaseRecord> loadFailedCasesForRetry(const QString& baseCaseHash = QString(),
	                                                  double fixedWidthMm = -1.0) const;

	/// 成功重算后 UPDATE 原记录（不 INSERT）。
	bool updateDesignPointResultByRowId(int rowId, const GearDesignPoint& dp);

	/// 重算仍失败：保留 failed，递增 retry_count 并记录 last_error_message。
	bool recordRetryFailureByRowId(int rowId, const QString& errorMsg);

	/// 非法设计（如 invalid relief）：status=invalid, is_valid=0，不递增 retry_count。
	bool markDesignInvalidByRowId(int rowId, const QString& lastErrorMessage);

	QVector<int> findRowIdsByCaseHash(const QString& caseHash) const;

	/// 同一 baseCase 下已有 CCX 结果的 case_hash 集合（infill 去重 / 排除 cache 用）。
	QSet<QString> knownCaseHashesForBaseCase(const QString& baseCaseHash,
	                                         double fixedWidthMm = -1.0) const;
	/// 同一 baseCase 下已有 CCX 结果的 design_hash 集合（7 维设计去重）。
	QSet<QString> knownDesignHashesForBaseCase(const QString& baseCaseHash,
	                                           double fixedWidthMm = -1.0) const;

	bool insertDesignPointResult(const QString& runId,
	                             const GearDesignPoint& dp,
	                             int generation,
	                             int individualId,
	                             int rank = 0,
	                             double crowdingDistance = 0.0,
	                             int isPareto = 0,
	                             const QString& caseHashValue = QString());

	bool insertSimulationResult(const GearDesignPoint& dp, int generation, int pointId);

	bool updateParetoFlag(const QString& runId,
	                      int generation,
	                      int individualId,
	                      int isPareto,
	                      int rank,
	                      double crowdingDistance);

	static bool appendCpressWidthBinsCsv(const QString& runDir,
	                                     const QString& runId,
	                                     int generation,
	                                     int individualId,
	                                     const QString& caseHash,
	                                     const CpressDistributionMetrics& metrics);

	int countValidResults() const;
	int backfillMissingCpressDistributionMetrics(int binCount = 11, int maxRows = 0) const;
	CpressDistributionBackfillStats backfillMissingCpressDistributionMetricsDetailed(
	    int binCount = 11,
	    int maxRows = 0) const;

	/// 优化配置表（metric_definitions / objective_mapping / design_variable_definitions）。
	bool ensureOptimizationConfigTables();
	QVector<MetricDefinition>         loadMetricDefinitions() const;
	QVector<ObjectiveMapping>         loadObjectiveMappings() const;
	QVector<DesignVariableDefinition> loadDesignVariableDefinitions() const;
	bool saveMetricDefinitions(const QVector<MetricDefinition>& defs);
	bool saveObjectiveMappings(const QVector<ObjectiveMapping>& mappings);
	bool saveDesignVariableDefinitions(const QVector<DesignVariableDefinition>& vars);

	static QString createTableSql();
	static QString csvHeaderLine();
	static void writeCsvRow(QTextStream& ts, const GearDesignPoint& dp);

private:
	explicit GearOptResultDatabase(const char* connectionName);

	GearOptResultDatabase(const GearOptResultDatabase&)            = delete;
	GearOptResultDatabase& operator=(const GearOptResultDatabase&) = delete;

	bool ensureGearOptResultsTable() const;
	bool ensureLegacySimulationTable() const;
	bool ensureMetricDefinitionsTable() const;
	bool ensureObjectiveMappingTable() const;
	bool ensureDesignVariableDefinitionsTable() const;
	bool insertDefaultOptimizationConfigIfEmpty() const;
	bool migrateMetricDefinitionsColumns() const;

	QString connectionName() const;

	QString _connectionName;
	QString _dbPath;
	bool    _open{false};
};

} // namespace GearAutoOpt

Q_DECLARE_METATYPE(GearAutoOpt::FailedCaseRetryStats)

#endif
