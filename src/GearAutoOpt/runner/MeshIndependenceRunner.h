#ifndef _GEARAUTOOPT_MESH_INDEPENDENCE_RUNNER_H_
#define _GEARAUTOOPT_MESH_INDEPENDENCE_RUNNER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"
#include "GearAutoOpt/data/GearOptConfig.h"

#include <QObject>
#include <QString>
#include <QVector>

namespace GearAutoOpt {

struct GEARAUTOOPTAPI MeshIndependenceCase {
	QString caseName;
	double globalSize = 0.0;
	double rootSize   = 0.35;
};

struct GEARAUTOOPTAPI MeshIndependenceResult {
	QString caseName;
	double globalSize   = 0.0;
	double rootSize     = 0.35;
	int    layers       = 0;
	int    nodeCount    = 0;
	int    elementCount = 0;
	double sigmaMax     = -1.0;
	double uMax         = -1.0;
	double solveTimeSec = -1.0;
	bool   converged    = false;
	double stressErrorPercent = -1.0;
	double dispErrorPercent   = -1.0;
	QString workDir;
};

/// 网格无关性验证：串行运行多组网格尺寸，输出 CSV 汇总。
class GEARAUTOOPTAPI MeshIndependenceRunner : public QObject {
	Q_OBJECT
public:
	explicit MeshIndependenceRunner(const GearOptConfig& baseCfg, QObject* parent = nullptr);

	/// 是否包含可选 Case D（global=1.00）；默认 true。
	void setIncludeCaseD(bool on) { _includeCaseD = on; }

public slots:
	void execute();

signals:
	void log(const QString& msg);
	void caseFinished(int index, const MeshIndependenceResult& result);
	void finished(bool success, const QString& csvPath);

private:
	QVector<MeshIndependenceCase> defaultCases() const;
	GearOptConfig                 makeCaseConfig(const MeshIndependenceCase& c) const;
	GearDesignPoint               baseDesignPoint() const;
	MeshIndependenceResult        runSingleCase(const MeshIndependenceCase& c,
	                                            const QString&              rootDir,
	                                            int                         baseZLayers);
	void                          computeErrors(QVector<MeshIndependenceResult>& results) const;
	double                        recommendedGlobalSize(const QVector<MeshIndependenceResult>& results) const;
	bool                          writeCsv(const QVector<MeshIndependenceResult>& results,
	                                       const QString&                         csvPath,
	                                       double                                 recommendedGlobal) const;

	GearOptConfig _baseCfg;
	bool          _includeCaseD{true};
};

} // namespace GearAutoOpt

#endif
