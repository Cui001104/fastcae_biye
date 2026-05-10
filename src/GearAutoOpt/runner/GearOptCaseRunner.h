#ifndef _GEARAUTOOPT_GEAR_OPT_CASE_RUNNER_H_
#define _GEARAUTOOPT_GEAR_OPT_CASE_RUNNER_H_

#include "GearAutoCaseRunner.h"
#include "GearAutoOpt/data/GearOptConfig.h"

namespace Command { class GeoCommandCreateGear; }
namespace Geometry { class GeometrySet; }

namespace GearAutoOpt {

/// 真实工况编排器：把 GearAutoCaseRunner 的 stub 步骤逐个填实。
///
/// 构造时 MainWindow / PreWindow 传 nullptr，GeoCommandBase 内部已做空指针保护，
/// 所以可在 headless 批量优化中使用，但必须在主线程（GeometryData 单例非线程安全）。
class GEARAUTOOPTAPI GearOptCaseRunner : public GearAutoCaseRunner {
	Q_OBJECT
public:
	explicit GearOptCaseRunner(QObject* parent = nullptr);
	~GearOptCaseRunner() override;

	/// 当前 run 关联的两 GeometrySet（execute 成功后填入；失败/undo 后清空）。
	Geometry::GeometrySet* gearSet1() const { return _set1; }
	Geometry::GeometrySet* gearSet2() const { return _set2; }

	/// 显式指定 gmsh.exe 路径；空表示走 detectGmshPath() 自动探测。
	void    setGmshPath(const QString& path) { _gmshPathOverride = path; }
	/// 探测 gmsh.exe：先读 GMSH_PATH 环境变量，否则 <appDir 多级回溯>/extlib/Gmsh/gmsh.exe。
	static QString detectGmshPath();

	/// 网格步全局尺寸：默认 0（= 0.5 × module）。
	void setMeshSize(double s) { _meshSize = s; }

	/// CCX 求解线程数（OMP_NUM_THREADS）；0 = 跟随系统默认。
	void setThreads(int n)     { _threads = n; }

	/// 求解配置（材料参数 / 扭矩 / ccx 超时等）。
	void             setConfig(const GearOptConfig& cfg) { _config = cfg; }
	const GearOptConfig& config() const { return _config; }

	/// 把本次 run 创建的两 GeometrySet 从全局 GeometryData 中撤销 + 析构。
	/// 工况编排层在每个设计点求解结束后应调用，避免 in-memory 累积成百上千个临时齿轮。
	void cleanup();

protected:
	bool runGeometryStep(GearDesignPoint& dp) override;
	bool runMeshStep(GearDesignPoint& dp) override;
	bool runInpWriteStep(GearDesignPoint& dp) override;
	bool runSolveStep(GearDesignPoint& dp) override;
	bool runParseStep(GearDesignPoint& dp) override;

private:
	Command::GeoCommandCreateGear* _geomCmd{nullptr};
	Geometry::GeometrySet* _set1{nullptr};
	Geometry::GeometrySet* _set2{nullptr};
	QString      _gmshPathOverride;
	double       _meshSize{0.0};
	int          _threads{0};
	GearOptConfig _config;
};

} // namespace GearAutoOpt

#endif
