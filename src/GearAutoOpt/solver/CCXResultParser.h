#ifndef _GEARAUTOOPT_CCX_RESULT_PARSER_H_
#define _GEARAUTOOPT_CCX_RESULT_PARSER_H_

#include "GearAutoOpt/GearAutoOptAPI.h"

#include <QHash>
#include <QSet>
#include <QString>
#include <QVector>

namespace GearAutoOpt {

struct GEARAUTOOPTAPI StressSample {
	int    elemId   = 0;
	int    integPt  = 0;
	double sxx      = 0.0;
	double syy      = 0.0;
	double szz      = 0.0;
	double sxy      = 0.0;
	double sxz      = 0.0;
	double syz      = 0.0;

	double vonMises() const;
};

/// .dat 中某 elset 在最终 time 步的 von Mises 最大值。
struct GEARAUTOOPTAPI DatStressResult {
	QString elsetName;
	int     sampleCount = 0;
	double  maxVonMises = -1.0; ///< 最终 time 步积分点 von Mises 全局 max [MPa]
	int     maxElemId   = -1;
	int     maxIntegPt  = -1;
	double  finalTime   = -1.0; ///< 采用的 stresses block 的 time

	bool found() const { return sampleCount > 0 && maxVonMises >= 0.0; }
};

/// 解析 .dat：仅取目标 elset 在最大 time 的 stresses 段，von Mises 取 max（非 P99）。
GEARAUTOOPTAPI DatStressResult parseDat(const QString& path, const QString& elsetName);

/// 同 parseDat（应力专用别名，强调只读最终 time 步 stresses block）。
inline DatStressResult parseDatStresses(const QString& path, const QString& elsetName) {
	return parseDat(path, elsetName);
}

/// .dat 中某 nset 在最终 time 步的位移最大值（仅当启用 *NODE PRINT 时使用）。
struct GEARAUTOOPTAPI DatDispResult {
	QString nsetName;
	int     sampleCount  = 0;
	double  maxMagnitude = -1.0; ///< 最终 time 步 |U| max [mm]
	int     maxNodeId    = -1;
	double  finalTime    = -1.0;

	bool found() const { return sampleCount > 0 && maxMagnitude >= 0.0; }
};

GEARAUTOOPTAPI DatDispResult parseDatDisplacements(const QString& path, const QString& nsetName);

GEARAUTOOPTAPI QVector<StressSample> parseDatSamples(const QString& path,
                                                      const QString& elsetName);

struct GEARAUTOOPTAPI NodeXYZ {
	double x = 0.0, y = 0.0, z = 0.0;
};

struct GEARAUTOOPTAPI NodeDisp {
	double ux = 0.0, uy = 0.0, uz = 0.0;
	double magnitude() const;
};

struct GEARAUTOOPTAPI NodeStress {
	double sxx = 0.0, syy = 0.0, szz = 0.0;
	double sxy = 0.0, syz = 0.0, szx = 0.0;
	double vonMises() const;
};

struct GEARAUTOOPTAPI FrdResult {
	QHash<int, NodeXYZ>    nodes;
	QHash<int, NodeDisp>   displacements;
	QHash<int, NodeStress> stresses;
	QHash<int, double>     cpressByNode;

	/// 最终 Step/Increment 的 CONTACT 块 CPRESS 全局最大值 [MPa]；-1 表示未找到。
	double cpressMax_MPa = -1.0;

	int  nodeCount() const   { return nodes.size(); }
	int  displCount() const  { return displacements.size(); }
	int  stressCount() const { return stresses.size(); }
	bool hasDisplacement() const { return !displacements.isEmpty(); }

	double maxVonMises(int* outNodeId = nullptr) const;
	double maxDisplMagnitude(int* outNodeId = nullptr) const;
};

GEARAUTOOPTAPI FrdResult parseFrd(const QString& path);

struct GEARAUTOOPTAPI CpressWidthBin {
	int    binIndex    = 0;
	double zStart      = 0.0;
	double zEnd        = 0.0;
	int    nodeCount   = 0;
	double meanCPRESS  = -1.0;
	double maxCPRESS   = -1.0;
};

struct GEARAUTOOPTAPI CpressDistributionMetrics {
	double cpressMean_MPa       = -1.0;
	double cpressStd_MPa        = -1.0;
	double cpressCV             = -1.0;
	double contactWidth_mm      = -1.0;
	double edgeLoadRatio        = -1.0;
	double cpressEdgeMean_MPa   = -1.0;
	double cpressCenterMean_MPa = -1.0;
	int    cpressActiveNodes    = 0;
	int    cpressBinCount       = 11;
	QVector<CpressWidthBin> bins;

	bool valid() const { return cpressActiveNodes > 0 && cpressMean_MPa > 0.0; }
};

GEARAUTOOPTAPI CpressDistributionMetrics parseCpressDistributionMetrics(
    const QString& frdPath,
    int binCount = 11);

/// 从 FRD 粘连科学计数法串中提取浮点（如 8.50630E-0051.08845E-003-6.35505E-008）。
GEARAUTOOPTAPI QVector<double> parseFrdFloatFields(const QString& input);

GEARAUTOOPTAPI double maxDisplMagnitudeForNodes(const FrdResult& frd,
                                                 const QSet<int>& nodeIds,
                                                 int* outNodeId = nullptr);

/// FRD 节点应力 von Mises 真全局 max；nodeIds 为空则扫描全部应力节点。
GEARAUTOOPTAPI double maxVonMisesForNodes(const FrdResult& frd,
                                            const QSet<int>& nodeIds,
                                            int* outNodeId = nullptr);

/// 扫描 job.frd 中所有 -4 数据集名（调试：确认 DISP/U 等块是否存在）。
GEARAUTOOPTAPI QStringList listFrdDatasetNames(const QString& path);

GEARAUTOOPTAPI double maxVonMisesFromDatElsets(const QString& datPath,
                                               const QVector<QString>& elsetNames,
                                               int* outSampleCount = nullptr);

/// 已弃用：优化目标改用 parseDat 最终步 max。
GEARAUTOOPTAPI double vonMisesPercentilePeak(const QVector<double>& misesValues,
                                             double percentile = 0.99);

GEARAUTOOPTAPI double vonMisesPercentileForNodes(const FrdResult& frd,
                                                 const QSet<int>& nodeIds,
                                                 double percentile = 0.99,
                                                 int* outNodeId = nullptr);

} // namespace GearAutoOpt

#endif
