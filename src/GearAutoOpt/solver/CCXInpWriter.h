#ifndef _GEARAUTOOPT_CCX_INP_WRITER_H_

#define _GEARAUTOOPT_CCX_INP_WRITER_H_



#include "GearAutoOpt/GearAutoOptAPI.h"



#include <QString>

#include <QVector>



namespace GearAutoOpt {



/// 主动轮齿面 CLOAD 分解（仅单齿轮旧路径；双齿轮已禁用齿面等效力）。

struct GEARAUTOOPTAPI GearCcxLoadBreakdown {

	double torqueNm     = 0.0;

	double torqueNmm    = 0.0;

	double d1_mm        = 0.0;

	double Ft_N         = 0.0;

	double Fn_N         = 0.0;

	double FxPerNode_N  = 0.0;

	double FyPerNode_N  = 0.0;

	int    toothNodeCount = 0;

};



GEARAUTOOPTAPI GearCcxLoadBreakdown computeGear1ToothCload(

	double torqueNm, double module, int z1, double alphaDeg, int toothNodeCount);



/// 师兄模板 REF–ROT 在 X 向间距 [mm]；Step-2 用 Fz = T_Nmm / L_rot 等效驱动。
static constexpr double kGearMentorRotOffsetMm = 5.0;



/// 由 UI 扭矩 [N·mm] 换算 Step-2 的 CENTER2_ROT DOF3 载荷 [N]（默认 L_rot = 5 mm）。
GEARAUTOOPTAPI double mentorStep2CloadZFromTorqueNmm(double torqueNmm,

                                                     double rotOffsetMm = kGearMentorRotOffsetMm);



struct GEARAUTOOPTAPI GearHubNodeCoord {

	int    nodeId = 0;

	double x      = 0.0;

	double y      = 0.0;

};



struct GEARAUTOOPTAPI CLoadSpec {

	QString nset;

	int     nodeId = -1;

	int     dof = 2;

	double  value = 0.0;

};



GEARAUTOOPTAPI QVector<CLoadSpec> buildHubTangentialCloads(const QVector<GearHubNodeCoord>& hubNodes,

                                                          double centerX,

                                                          double centerY,

                                                          double torqueNmm,

                                                          double* outSumMz = nullptr);



/// 单齿轮 / 旧路径参考点（*COUPLING，非双齿轮主路径）。

struct GEARAUTOOPTAPI CouplingGearSpec {

	QString hubSurfName = QStringLiteral("GEAR1_HUB_SURF");

	QString refNset     = QStringLiteral("CENTER1_REF");

	QString constraintName;

	int     refNodeId = 0;

	double  cx        = 0.0;

	double  cy        = 0.0;

	double  zMid      = 0.0;

};



struct GEARAUTOOPTAPI CouplingDriveSpec {

	CouplingGearSpec gear1;

	QString fixedHubNset    = QStringLiteral("GEAR2_HUB");

	int     fixedHubDofStart = 1;

	int     fixedHubDofEnd   = 3;

	QString orientationName = QStringLiteral("GEAR_AXIS");

	double  torqueNmm       = 0.0;

};



/// 双齿轮 *RIGID BODY：轮毂节点 NSET + 中心参考点（CalculiX 不支持 SURFACE 作 RIGID BODY 从属集）。

struct GEARAUTOOPTAPI RigidBodyGearSpec {

	QString hubNset     = QStringLiteral("GEAR1_HUB");       ///< *RIGID BODY,NSET=

	QString hubSurfName = QStringLiteral("GEAR1_HUB_SURF"); ///< mesh 校验 / 日志

	QString refNset     = QStringLiteral("CENTER1_REF");

	int     refNodeId   = 0;

	QString rotNset     = QStringLiteral("CENTER1_ROT");

	int     rotNodeId   = 0;

	double  cx = 0.0;

	double  cy = 0.0;

	double  zMid = 0.0;

};



struct GEARAUTOOPTAPI DualGearRigidBodySpec {

	RigidBodyGearSpec gear1;

	RigidBodyGearSpec gear2;

	/// 目标扭矩 [N·mm]（UI N·m × 1000）；用于 Step-2 Fz 映射与 inp 注释。
	double torqueNmm = 0.0;

	/// 双齿轮接触工况：CENTER1_ROT 位移控制（不写 *CLOAD / 扭矩）。
	bool displacementDriven = true;

	/// CENTER1_ROT DOF3 施加位移 [mm]（DOF1–2 固定 0，默认 0.001）。
	double driveDispMm = 0.001;

	/// 齿宽 [mm]，非师兄模板时用于 ROT 节点相对 REF 的 x 向偏移 dx。
	double gearWidthMm = 0.0;

	/// 师兄 Step-2：CENTER2_ROT DOF3 载荷 [N]；>0 时直接写入 *CLOAD。
	/// 通常由 GearOptCaseRunner 按 mentorStep2CloadZFromTorqueNmm(torqueNmm) 赋值。
	double mentorStep2RotCloadZ = 0.0;

};



struct GEARAUTOOPTAPI BoundarySpec {

	QString nset;

	int     dofStart = 1;

	int     dofEnd   = 3;

	bool    hasValue = false;

	double  value    = 0.0;

};



struct GEARAUTOOPTAPI InpContext {

	QString runName     = "default";

	int     generation  = 0;

	int     pointId     = 0;



	QString meshInpFile = "mesh.inp";



	QString materialName = "STEEL";

	double  youngModulus = 2.06e5;

	double  poisson      = 0.30;

	double  density      = 7.85e-9;



	QString volumeElset  = "GEAR1";

	QVector<QString> volumeElsets;

	QString outputNset;

	QString outputElset;

	bool    twoGearJob = false;

	QVector<QString> nodePrintNsets;

	QVector<QString> elPrintElsets;

	QString toothLoadNset = "GEAR1_TOOTH_OUTER";



	QVector<BoundarySpec> boundaries;

	QVector<CLoadSpec>    cloads;



	/// 双齿轮：*RIGID BODY + CENTER1/2_REF（默认 true）。

	bool                 useRigidBody = true;

	DualGearRigidBodySpec rigidBody;



	/// 单齿轮或旧双齿轮 *COUPLING 路径。

	bool               useCoupling = false;

	bool               useKinematicCoupling = true;

	CouplingDriveSpec  couplingDrive;



	bool   enableContact   = true;

	/// EXPONENTIAL 接触 c0（双齿轮验证模板默认 5）。
	double contactStiffness = 5.0;
	/// EXPONENTIAL 接触 p0（双齿轮验证模板默认 0.05）。
	double contactPressureP0 = 0.05;

	double contactAdjustMm = 0.0;



	bool requestNodeFile      = true;

	bool requestElFile        = true;

	bool requestContactOutput = true;

	/// 双齿轮：由 writeJobInp 从 mesh ELSET 生成 *NSET=GEAR1_NODES/GEAR2_NODES（与 ELSET 区分）。
	QString gearVolumeNsetBlock;

	/// legacy | gear1_positive | gear1_negative（18/26 加载方向调试）
	QString gearDriveMode = QStringLiteral("legacy");

};



GEARAUTOOPTAPI bool writeJobInp(const QString& dir, const InpContext& ctx);

GEARAUTOOPTAPI QString renderJobInp(const InpContext& ctx);



GEARAUTOOPTAPI QStringList inpCouplingKeywordLines(const QString& inpPath);

GEARAUTOOPTAPI QStringList inpRigidBodyKeywordLines(const QString& inpPath);

GEARAUTOOPTAPI QString verifyInpCouplingKeywordSpacing(const QString& inpPath);

GEARAUTOOPTAPI void logJobInpAudit(const QString& jobInpPath, const QString& meshInpPath);



} // namespace GearAutoOpt



#endif

