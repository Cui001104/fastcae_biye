#include "CCXInpWriter.h"
#include "MeshConverter.h"
#include "GearAutoOpt/data/GearOptLog.h"

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QTextStream>

#include <cmath>

#include <algorithm>

namespace GearAutoOpt {

namespace {

static const char* kCcxStaticTemplate =
	"**\n"
	"**  GearAutoOpt - CalculiX 静力分析驱动 inp\n"
	"**  Run: {{run_name}}, Gen: {{gen}}, Point: {{point_id}}\n"
	"**  Generated: {{timestamp}}\n"
	"**  注意：mesh.inp 自带 *Heading，本驱动不可重复声明\n"
	"**\n"
	"*INCLUDE, INPUT={{mesh_inp}}\n"
	"{{gear_volume_nset_block}}"
	"**\n"
	"*MATERIAL, NAME={{material_name}}\n"
	"*ELASTIC\n"
	"{{young}}, {{poisson}}\n"
	"{{density_block}}"
	"{{solid_section_block}}"
	"**\n"
	"{{ref_nodes_block}}"
	"{{rigid_body_block}}"
	"{{coupling_block}}"
	"**\n"
	"{{contact_block}}"
	"{{step_and_output_block}}"
	"{{end_step_suffix}}";

static const char* kCcxSingleStepSuffix = "*END STEP\n";

static constexpr double kCcxStaticT0     = 1e-5;
static constexpr double kCcxStaticTTotal = 1.0;
static constexpr double kCcxStaticTMin   = 1e-6;
static constexpr double kCcxStaticTMax   = 0.001;

static constexpr double kCcxDualContactT0     = 1e-4;
static constexpr double kCcxDualContactTTotal = 1.0;
static constexpr double kCcxDualContactTMin   = 1e-7;
static constexpr double kCcxDualContactTMax   = 0.05;

static constexpr double kCcxDualContactAdjustDefaultMm = 1e-5;

static constexpr double kMentorRotOffsetMm      = kGearMentorRotOffsetMm;
static constexpr double kMentorStep1RotDispMm   = 0.001;
static constexpr double kMentorStep2CloadZ      = 200000.0;
static constexpr double kMentorFrictionCoeff      = 0.1;
static constexpr double kMentorFrictionSlip       = 100000.0;
static constexpr double kMentorStep1T0            = 1.0;
static constexpr double kMentorStep1TTotal        = 1.0;
static constexpr double kMentorStep2T0            = 0.001;
static constexpr double kMentorStep2TTotal        = 1.0;
static constexpr double kMentorStaticTMin          = 1e-10;
static constexpr double kMentorStaticTMax          = 0.5;

bool useDualGearContactTemplate(const InpContext& ctx)
{
	return ctx.twoGearJob && ctx.enableContact && ctx.useRigidBody;
}

bool useMentorTwoStepHardContactTemplate(const InpContext& ctx)
{
	return useDualGearContactTemplate(ctx);
}

static double resolveMentorStep2CloadZ(const DualGearRigidBodySpec& r)
{
	if (r.mentorStep2RotCloadZ > 0.0)
		return r.mentorStep2RotCloadZ;
	if (r.torqueNmm > 0.0)
		return mentorStep2CloadZFromTorqueNmm(r.torqueNmm, kMentorRotOffsetMm);
	return kMentorStep2CloadZ;
}

QString fmtNumber(double v) {
	return QString::number(v, 'g', 10);
}

double rigidBodyRotAxisOffsetMm(const InpContext& ctx) {
	if (useMentorTwoStepHardContactTemplate(ctx))
		return kMentorRotOffsetMm;
	const double gearWidthMm = ctx.rigidBody.gearWidthMm;
	constexpr double kMinMm       = 1.0;
	constexpr double kWidthFactor = 0.5;
	if (gearWidthMm <= 0.0)
		return kMinMm;
	return std::max(kMinMm, kWidthFactor * gearWidthMm);
}

QString buildCloadBlock(const InpContext& ctx) {
	if (useMentorTwoStepHardContactTemplate(ctx))
		return QStringLiteral("**  mentor template: *CLOAD inside Step-2\n");

	if (ctx.twoGearJob && ctx.useRigidBody && ctx.rigidBody.displacementDriven)
		return QStringLiteral("**  no *CLOAD (displacement-controlled loading on CENTER1_ROT)\n");

	const QVector<CLoadSpec>& cloads = ctx.cloads;
	if (cloads.isEmpty())
		return QStringLiteral("**  no *CLOAD specified\n");
	QString s = QStringLiteral("*CLOAD\n");
	for (const auto& c : cloads) {
		const QString target =
		    (c.nodeId > 0) ? QString::number(c.nodeId) : c.nset;
		s += QString("%1, %2, %3\n").arg(target).arg(c.dof).arg(fmtNumber(c.value));
	}
	return s;
}

QString buildBoundaryBlock(const InpContext& ctx) {
	if (useMentorTwoStepHardContactTemplate(ctx))
		return QStringLiteral("**  mentor template: *BOUNDARY inside each *STEP\n");

	if (ctx.twoGearJob && ctx.useRigidBody) {
		const DualGearRigidBodySpec& r = ctx.rigidBody;
		const double torqueNm = r.torqueNmm / 1000.0;
		if (r.displacementDriven) {
			return QStringLiteral(			                      "**  Dual gear: CENTER1_ROT DOF1-2=0, DOF3=%1 mm (z-drive)\n"
			                      "**  torque_Nm = %2; Step-2 Fz = T_Nmm/L_rot on CENTER2_ROT\n"
			                      "*BOUNDARY\n"
			                      "%3, 1, 3, 0\n"
			                      "%4, 1, 2, 0\n"
			                      "%4, 3, 3, %1\n"
			                      "%5, 1, 3, 0\n"
			                      "%6, 1, 3, 0\n")
			    .arg(fmtNumber(r.driveDispMm))
			    .arg(fmtNumber(torqueNm))
			    .arg(r.gear1.refNodeId)
			    .arg(r.gear1.rotNodeId)
			    .arg(r.gear2.refNodeId)
			    .arg(r.gear2.rotNodeId);
		}
		return QStringLiteral("**  Dual gear: REF/ROT SPC (no displacement drive)\n"
		                      "**  torque_Nm = %1 (record only)\n"
		                      "*BOUNDARY\n"
		                      "%2, 1, 3, 0\n"
		                      "%3, 1, 3, 0\n"
		                      "%4, 1, 3, 0\n"
		                      "%5, 1, 3, 0\n")
		    .arg(fmtNumber(torqueNm))
		    .arg(r.gear1.refNodeId)
		    .arg(r.gear1.rotNodeId)
		    .arg(r.gear2.refNodeId)
		    .arg(r.gear2.rotNodeId);
	}

	if (ctx.useCoupling) {
		const CouplingDriveSpec& d = ctx.couplingDrive;
		return QStringLiteral("**  Coupling reference-node + passive hub constraints\n"
		                      "*BOUNDARY\n"
		                      "%1, 1, 5, 0.\n"
		                      "%2, %3, %4, 0.\n")
		    .arg(d.gear1.refNodeId)
		    .arg(d.fixedHubNset)
		    .arg(d.fixedHubDofStart)
		    .arg(d.fixedHubDofEnd);
	}

	if (ctx.boundaries.isEmpty()) {
		return QStringLiteral("**  no *BOUNDARY specified\n");
	}
	QString s = QStringLiteral("*BOUNDARY\n");
	for (const auto& b : ctx.boundaries) {
		if (b.hasValue)
			s += QString("%1, %2, %3, %4\n")
			         .arg(b.nset)
			         .arg(b.dofStart)
			         .arg(b.dofEnd)
			         .arg(fmtNumber(b.value));
		else
			s += QString("%1, %2, %3\n").arg(b.nset).arg(b.dofStart).arg(b.dofEnd);
	}
	return s;
}

QString buildRefNodesBlock(const InpContext& ctx) {
	if (ctx.twoGearJob && ctx.useRigidBody) {
		const double dx = rigidBodyRotAxisOffsetMm(ctx);
		const auto writeRefRotNodes = [dx](const RigidBodyGearSpec& g) {
			return QStringLiteral("*NODE, NSET=%1\n%2, %3, %4, %5\n"
			                      "*NODE, NSET=%6\n%7, %8, %9, %10\n")
			    .arg(g.refNset)
			    .arg(g.refNodeId)
			    .arg(fmtNumber(g.cx))
			    .arg(fmtNumber(g.cy))
			    .arg(fmtNumber(g.zMid))
			    .arg(g.rotNset)
			    .arg(g.rotNodeId)
			    .arg(fmtNumber(g.cx + dx))
			    .arg(fmtNumber(g.cy))
			    .arg(fmtNumber(g.zMid));
		};
		return QStringLiteral("**  Dual gear REF/ROT nodes (ROT x = REF x + %1 mm)\n")
		           .arg(fmtNumber(dx))
		       + writeRefRotNodes(ctx.rigidBody.gear1) + writeRefRotNodes(ctx.rigidBody.gear2);
	}

	if (!ctx.useCoupling)
		return QString();

	const CouplingGearSpec& g = ctx.couplingDrive.gear1;
	return QStringLiteral("**  Coupling reference node\n")
	       + QStringLiteral("*NODE, NSET=%1\n%2, %3, %4, %5\n")
	              .arg(g.refNset)
	              .arg(g.refNodeId)
	              .arg(fmtNumber(g.cx))
	              .arg(fmtNumber(g.cy))
	              .arg(fmtNumber(g.zMid));
}

QString buildRigidBodyBlock(const InpContext& ctx) {
	if (!ctx.twoGearJob || !ctx.useRigidBody)
		return QString();

	const DualGearRigidBodySpec& r = ctx.rigidBody;
	auto oneBody = [](const RigidBodyGearSpec& g) {
		return QStringLiteral("**  %1/%6: hub NSET=%2 (SURFACE %3 mesh check only)\n"
		                    "*RIGID BODY, NSET=%2, REF NODE=%4, ROT NODE=%5\n")
		    .arg(g.refNset)
		    .arg(g.hubNset)
		    .arg(g.hubSurfName)
		    .arg(g.refNodeId)
		    .arg(g.rotNodeId)
		    .arg(g.rotNset);
	};

	return QStringLiteral("**  CalculiX *RIGID BODY requires NSET/ELSET, not *SURFACE\n") + oneBody(r.gear1)
	       + oneBody(r.gear2);
}

QString buildCouplingBlock(const InpContext& ctx) {
	if (!ctx.useCoupling)
		return QString();

	const CouplingDriveSpec& d = ctx.couplingDrive;
	const QString c1 =
	    d.gear1.constraintName.isEmpty() ? QStringLiteral("GEAR1_COUPLING") : d.gear1.constraintName;

	const QString couplingTag =
	    ctx.useKinematicCoupling ? QStringLiteral("KINEMATIC") : QStringLiteral("DISTRIBUTING");

	// *KINEMATIC 下方只能写局部 DOF 1/2/3（每行一个或范围），禁止 4~6；参考点 DOF6 扭矩用 *CLOAD。
	QString couplingDofLines;
	if (ctx.useKinematicCoupling) {
		couplingDofLines = QStringLiteral("1\n2\n3\n");
	} else {
		couplingDofLines = QStringLiteral("1, 6\n");
	}

	return QStringLiteral("**  *COUPLING + *%5 (active gear hub -> ref node)\n"
	                      "*ORIENTATION,NAME=%1_ORIENT\n"
	                      "1., 0., 0., 0., 1., 0.\n"
	                      "*COUPLING, REF NODE=%2, SURFACE=%3, CONSTRAINT NAME=%4, ORIENTATION=%1_ORIENT\n"
	                      "*%5\n"
	                      "%6")
	    .arg(d.orientationName)
	    .arg(d.gear1.refNodeId)
	    .arg(d.gear1.hubSurfName)
	    .arg(c1)
	    .arg(couplingTag)
	    .arg(couplingDofLines);
}

QString buildDensityBlock(double density) {
	if (density <= 0.0)
		return QString();
	return QString("*DENSITY\n%1\n").arg(fmtNumber(density));
}

QString buildSolidSectionBlock(const InpContext& ctx) {
	const QVector<QString> elsets =
	    ctx.volumeElsets.isEmpty() ? QVector<QString>{ctx.volumeElset} : ctx.volumeElsets;
	QString s;
	for (const QString& el : elsets) {
		if (el.isEmpty())
			continue;
		s += QStringLiteral("*SOLID SECTION, ELSET=%1, MATERIAL=%2\n").arg(el, ctx.materialName);
	}
	if (s.isEmpty())
		s = QStringLiteral("**  no *SOLID SECTION (empty ELSET list)\n");
	return s;
}

QString buildContactBlock(const InpContext& ctx) {
	if (!ctx.enableContact) {
		return QStringLiteral("**  GearAutoOpt: contact disabled\n");
	}

	const bool   mentorTpl  = useMentorTwoStepHardContactTemplate(ctx);
	const double adjustMm   = ctx.contactAdjustMm > 0.0
	                              ? ctx.contactAdjustMm
	                              : (mentorTpl ? kCcxDualContactAdjustDefaultMm : 0.001);

	QString pairLine = QStringLiteral(
	    "*CONTACT PAIR, INTERACTION=GEAR_CONTACT_PROP, TYPE=SURFACE TO SURFACE, ADJUST=%1\n")
	                       .arg(fmtNumber(adjustMm));
	if (mentorTpl)
		pairLine += QStringLiteral("slave, master\n");
	else
		pairLine += QStringLiteral("GEAR1_TOOTH_SURF, GEAR2_TOOTH_SURF\n");

	if (mentorTpl) {
		return QStringLiteral("*SURFACE INTERACTION, NAME=GEAR_CONTACT_PROP\n"
		                      "*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=HARD\n"
		                      "*FRICTION\n"
		                      "%1, %2\n"
		                      "%3")
		    .arg(fmtNumber(kMentorFrictionCoeff))
		    .arg(fmtNumber(kMentorFrictionSlip))
		    .arg(pairLine);
	}

	return QStringLiteral("*SURFACE INTERACTION, NAME=GEAR_CONTACT_PROP\n"
	                      "*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=EXPONENTIAL\n"
	                      "%1, %2\n"
	                      "%3")
	    .arg(fmtNumber(ctx.contactPressureP0))
	    .arg(fmtNumber(ctx.contactStiffness))
	    .arg(pairLine);
}

QString buildMentorBoundaryStep1(const InpContext& ctx) {
	const DualGearRigidBodySpec& r = ctx.rigidBody;
	return QStringLiteral("*BOUNDARY, OP=NEW\n"
	                      "%1, 1, 1, 0\n"
	                      "%1, 2, 2, 0\n"
	                      "%1, 3, 3, 0\n"
	                      "%2, 1, 1, 0\n"
	                      "%2, 2, 2, 0\n"
	                      "%2, 3, 3, %3\n"
	                      "%4, 1, 1, 0\n"
	                      "%4, 2, 2, 0\n"
	                      "%4, 3, 3, 0\n"
	                      "%5, 1, 1, 0\n"
	                      "%5, 2, 2, 0\n"
	                      "%5, 3, 3, 0\n")
	    .arg(r.gear1.refNodeId)
	    .arg(r.gear1.rotNodeId)
	    .arg(fmtNumber(kMentorStep1RotDispMm))
	    .arg(r.gear2.refNodeId)
	    .arg(r.gear2.rotNodeId);
}

QString buildMentorBoundaryStep2(const InpContext& ctx) {
	const DualGearRigidBodySpec& r = ctx.rigidBody;
	return QStringLiteral("*BOUNDARY, OP=NEW\n"
	                      "%1, 1, 1, 0\n"
	                      "%1, 2, 2, 0\n"
	                      "%1, 3, 3, 0\n"
	                      "%2, 1, 1, 0\n"
	                      "%2, 2, 2, 0\n"
	                      "%2, 3, 3, %3\n"
	                      "%4, 1, 1, 0\n"
	                      "%4, 2, 2, 0\n"
	                      "%4, 3, 3, 0\n"
	                      "%5, 1, 1, 0\n"
	                      "%5, 2, 2, 0\n")
	    .arg(r.gear1.refNodeId)
	    .arg(r.gear1.rotNodeId)
	    .arg(fmtNumber(kMentorStep1RotDispMm))
	    .arg(r.gear2.refNodeId)
	    .arg(r.gear2.rotNodeId);
}

QString buildMentorElPrintBlock() {
	return QStringLiteral("*EL PRINT, ELSET=GEAR1, GLOBAL=YES\n"
	                      "S\n"
	                      "*EL PRINT, ELSET=GEAR2, GLOBAL=YES\n"
	                      "S\n");
}

QString buildMentorStepOutputBlock(bool step2) {
	if (step2) {
		// 仅末步写 FRD；FREQUENCY=999999 避免每增量帧导致 frd 过大
		return QStringLiteral("*NODE FILE, FREQUENCY=999999\n"
		                      "RF, U\n"
		                      "*EL FILE, FREQUENCY=999999\n"
		                      "S\n");
	}
	QString s;
	s += QStringLiteral("*NODE FILE\nU\n");
	s += QStringLiteral("*EL FILE\nS, NOE\n"
	                    "*CONTACT FILE\nCDIS, CSTR\n");
	s += buildMentorElPrintBlock();
	return s;
}

QString buildMentorTwoStepBlock(const InpContext& ctx) {
	const DualGearRigidBodySpec& r = ctx.rigidBody;
	const double step2Cload = resolveMentorStep2CloadZ(r);

	QString s;
	s += QStringLiteral("**\n"
	                    "** Step-1: establish contact (CENTER1_ROT z-drive)\n"
	                    "**\n"
	                    "*STEP, NLGEOM, INC=100000\n"
	                    "*STATIC\n"
	                    "%1, %2, %3, %4\n"
	                    "*CONTROLS, PARAMETERS=FIELD\n"
	                    "0.005, 0.02, , , 0.02, 1e-5, 1e-3, 1e-8\n"
	                    "*CONTROLS, PARAMETERS=CONTACT\n"
	                    "0.05, 0.1, 100, 20\n"
	                    "**\n")
	           .arg(fmtNumber(kMentorStep1T0))
	           .arg(fmtNumber(kMentorStep1TTotal))
	           .arg(fmtNumber(kMentorStaticTMin))
	           .arg(fmtNumber(kMentorStaticTMax));
	s += buildMentorBoundaryStep1(ctx);
	s += QStringLiteral("**\n"
	                    "** Loads (Step-1: no effective *CLOAD)\n"
	                    "**\n"
	                    "*CLOAD, OP=NEW\n");
	s += buildMentorStepOutputBlock(false);
	s += QStringLiteral("*END STEP\n"
	                    "**\n"
	                    "** Step-2: CENTER2_ROT DOF3 Fz = %1 N (T = %2 N·m, L_rot = %3 mm)\n"
	                    "**\n"
	                    "*STEP, NLGEOM, INC=100000\n")
	           .arg(fmtNumber(step2Cload))
	           .arg(fmtNumber(r.torqueNmm / 1000.0))
	           .arg(fmtNumber(kMentorRotOffsetMm));
	s += QStringLiteral(
	                    "*STATIC\n"
	                    "%1, %2, %3, %4\n"
	                    "**\n")
	           .arg(fmtNumber(kMentorStep2T0))
	           .arg(fmtNumber(kMentorStep2TTotal))
	           .arg(fmtNumber(kMentorStaticTMin))
	           .arg(fmtNumber(kMentorStaticTMax));
	s += buildMentorBoundaryStep2(ctx);
	s += QStringLiteral("**\n"
	                    "** Loads\n"
	                    "**\n"
	                    "*CLOAD, OP=NEW\n"
	                    "%1, 3, %2\n")
	           .arg(r.gear2.rotNodeId)
	           .arg(fmtNumber(step2Cload));
	s += buildMentorStepOutputBlock(true);
	s += QStringLiteral("*END STEP\n");
	return s;
}

QString buildStepBlock(const InpContext& ctx)
{
	if (useMentorTwoStepHardContactTemplate(ctx))
		return buildMentorTwoStepBlock(ctx);

	if (useDualGearContactTemplate(ctx)) {
		return QStringLiteral("*STEP, NLGEOM, INC=100000\n"
		                      "*STATIC\n"
		                      "%1, %2, %3, %4\n"
		                      "**\n"
		                      "*CONTROLS, PARAMETERS=FIELD\n"
		                      "0.005, 0.02, , , 0.02, 1e-5, 1e-3, 1e-8\n"
		                      "*CONTROLS, PARAMETERS=CONTACT\n"
		                      "0.05, 0.1, 100, 20\n"
		                      "**\n")
		    .arg(fmtNumber(kCcxDualContactT0))
		    .arg(fmtNumber(kCcxDualContactTTotal))
		    .arg(fmtNumber(kCcxDualContactTMin))
		    .arg(fmtNumber(kCcxDualContactTMax));
	}
	return QStringLiteral("*STEP, NLGEOM, INC=10000\n"
	                      "*STATIC\n"
	                      "%1, %2, %3, %4\n")
	    .arg(fmtNumber(kCcxStaticT0))
	    .arg(fmtNumber(kCcxStaticTTotal))
	    .arg(fmtNumber(kCcxStaticTMin))
	    .arg(fmtNumber(kCcxStaticTMax));
}

QString buildGearVolumeNsetsBlock(const QString& meshInpPath)
{
	QString s;
	for (const QString& elset : {QStringLiteral("GEAR1"), QStringLiteral("GEAR2")}) {
		QSet<int> nodes;
		if (!loadElsetNodeIds(meshInpPath, elset, &nodes) || nodes.isEmpty())
			continue;
		const QString nset = elset + QStringLiteral("_NODES");
		QList<int> ids = nodes.values();
		std::sort(ids.begin(), ids.end());
		s += QStringLiteral("**  volume nodes as NSET (from ELSET %1)\n"
		                    "*NSET, NSET=%2\n")
		         .arg(elset, nset);
		for (int i = 0; i < ids.size(); ++i) {
			s += QString::number(ids[i]);
			if ((i + 1) % 16 == 0 || i + 1 == ids.size())
				s += QLatin1Char('\n');
			else
				s += QStringLiteral(", ");
		}
	}
	return s;
}

QString buildPrintBlock(const InpContext& ctx) {
	if (useMentorTwoStepHardContactTemplate(ctx))
		return QStringLiteral("**  mentor template: *EL PRINT inside each *STEP\n");

	if (useDualGearContactTemplate(ctx))
		return QStringLiteral("**  dual-gear contact: no *EL PRINT (results from job.frd only)\n");

	if (ctx.twoGearJob) {
		QString s = QStringLiteral("**  *EL PRINT → job.dat stresses (parseDat uses max time step)\n");
		const QVector<QString> elPrint = ctx.elPrintElsets.isEmpty()
		                                     ? QVector<QString>{QStringLiteral("GEAR1"),
		                                                        QStringLiteral("GEAR2")}
		                                     : ctx.elPrintElsets;
		for (const QString& el : elPrint)
			s += QStringLiteral("*EL PRINT, ELSET=%1, Global=Yes\nS\n").arg(el);

		// 默认不写 *NODE PRINT（.dat 过大）；调试位移时显式设置 ctx.nodePrintNsets（如 GEAR1_NODES）。
		for (const QString& ns : ctx.nodePrintNsets)
			s += QStringLiteral("*NODE PRINT, NSET=%1\nU\n").arg(ns);
		return s;
	}

	QString s;
	const QString outNset  = ctx.outputNset.isEmpty() ? ctx.volumeElset : ctx.outputNset;
	const QString outElset = ctx.outputElset.isEmpty() ? ctx.volumeElset : ctx.outputElset;
	s += QStringLiteral("*NODE PRINT, NSET=%1\nU\n").arg(outNset);
	s += QStringLiteral("*EL PRINT, ELSET=%1\nS\n").arg(outElset);
	return s;
}

/// 双齿轮转角驱动 / 无接触时不写参考点扭矩 *CLOAD。
void applyReferenceTorquePolicy(InpContext& ctx) {
	if (useMentorTwoStepHardContactTemplate(ctx)) {
		ctx.cloads.clear();
		return;
	}
	if (ctx.twoGearJob && ctx.useRigidBody) {
		if (ctx.rigidBody.displacementDriven)
			ctx.cloads.clear();
		return;
	}
	if (!ctx.useCoupling || ctx.enableContact)
		return;

	const int refId = ctx.couplingDrive.gear1.refNodeId;
	if (refId <= 0)
		return;

	QVector<CLoadSpec> kept;
	kept.reserve(ctx.cloads.size());
	bool stripped = false;
	for (const CLoadSpec& c : ctx.cloads) {
		if (c.nodeId == refId && c.dof == 6) {
			stripped = true;
			continue;
		}
		kept.append(c);
	}
	if (!stripped)
		return;

	ctx.cloads = kept;
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral(
	    "[GearOpt][CCX] contact disabled: reference-node torque is disabled to avoid rigid-body rotation.");
}

void logCcxInpWriteSummary(const InpContext& ctx, const QString& meshInpPath) {
	const int hub1Nodes   = countNsetNodes(meshInpPath, QStringLiteral("GEAR1_HUB"));
	const int hub2Nodes   = countNsetNodes(meshInpPath, QStringLiteral("GEAR2_HUB"));
	const int masterFaces = countSurfaceFaces(meshInpPath, QStringLiteral("master"));
	const int slaveFaces  = countSurfaceFaces(meshInpPath, QStringLiteral("slave"));
	const int t1          = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR1_TOOTH_SURF"));
	const int t2          = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR2_TOOTH_SURF"));
	const int h1          = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR1_HUB_SURF"));
	const int h2          = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR2_HUB_SURF"));

	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] GEAR1_HUB NSET nodes = %1").arg(hub1Nodes);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] GEAR2_HUB NSET nodes = %1").arg(hub2Nodes);
	if (ctx.twoGearJob && ctx.useRigidBody && (hub1Nodes <= 0 || hub2Nodes <= 0)) {
		qWarning().noquote()
		    << QStringLiteral("[GearOpt][CCX] GEAR1_HUB/GEAR2_HUB NSET missing or empty"
		                      " (required for *RIGID BODY; re-run mesh enrich)");
	}
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] master face count = %1").arg(masterFaces);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] slave face count = %1").arg(slaveFaces);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] GEAR1_TOOTH_SURF face count (debug) = %1").arg(t1);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] GEAR2_TOOTH_SURF face count (debug) = %1").arg(t2);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] GEAR1_HUB_SURF face count = %1").arg(h1);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] GEAR2_HUB_SURF face count = %1").arg(h2);

	if (useMentorTwoStepHardContactTemplate(ctx)) {
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] using mentor two-step hard-contact template");
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] contact = hard + friction(%1,%2)")
		                              .arg(kMentorFrictionCoeff, 0, 'g', 8)
		                              .arg(kMentorFrictionSlip, 0, 'g', 8);
	} else {
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] contact %1  exponential(%2,%3)")
		                              .arg(ctx.enableContact ? QStringLiteral("enabled")
		                                                     : QStringLiteral("disabled"))
		                              .arg(ctx.contactPressureP0, 0, 'g', 8)
		                              .arg(ctx.contactStiffness, 0, 'g', 8);
	}
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral(
	    "[GearOpt][CCX] GEAR1_TOOTH_OUTER equivalent CLOAD = disabled (dual-gear contact)");

	if (ctx.twoGearJob && ctx.useRigidBody) {
		const DualGearRigidBodySpec& r = ctx.rigidBody;
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] CENTER1_REF id=%1 coord=(%2,%3,%4)")
		                              .arg(r.gear1.refNodeId)
		                              .arg(r.gear1.cx, 0, 'g', 8)
		                              .arg(r.gear1.cy, 0, 'g', 8)
		                              .arg(r.gear1.zMid, 0, 'g', 8);
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] CENTER2_REF id=%1 coord=(%2,%3,%4)")
		                              .arg(r.gear2.refNodeId)
		                              .arg(r.gear2.cx, 0, 'g', 8)
		                              .arg(r.gear2.cy, 0, 'g', 8)
		                              .arg(r.gear2.zMid, 0, 'g', 8);
		GEAR_OPT_DEBUG_NOQUOTE
		    << QStringLiteral("[GearOpt][CCX] displacement-driven = %1  drive_disp_mm = %2  torque_Nm = %3  step2_Fz_N = %4")
		           .arg(r.displacementDriven ? QStringLiteral("true") : QStringLiteral("false"))
		           .arg(r.driveDispMm, 0, 'g', 10)
		           .arg(r.torqueNmm / 1000.0, 0, 'g', 10)
		           .arg(fmtNumber(resolveMentorStep2CloadZ(r)));
	}
}

void logCcxJobTemplateSummary(const InpContext& ctx) {
	if (!ctx.twoGearJob || !ctx.useRigidBody)
		return;

	const DualGearRigidBodySpec& r = ctx.rigidBody;
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] rigid body mode = ref+rot node");
	if (useMentorTwoStepHardContactTemplate(ctx)) {
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] using mentor two-step hard-contact template");
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] ROT node offset = +%1 mm in X")
		                              .arg(fmtNumber(kMentorRotOffsetMm));
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] Step-1 static = %1, %2, %3, %4")
		                              .arg(fmtNumber(kMentorStep1T0))
		                              .arg(fmtNumber(kMentorStep1TTotal))
		                              .arg(fmtNumber(kMentorStaticTMin))
		                              .arg(fmtNumber(kMentorStaticTMax));
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] Step-2 static = %1, %2, %3, %4")
		                              .arg(fmtNumber(kMentorStep2T0))
		                              .arg(fmtNumber(kMentorStep2TTotal))
		                              .arg(fmtNumber(kMentorStaticTMin))
		                              .arg(fmtNumber(kMentorStaticTMax));
		const double step2Cload = resolveMentorStep2CloadZ(r);
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] Step-2 CLOAD CENTER2_ROT DOF3 = %1 N (T = %2 N·m, Fz = T/L_rot)")
		                              .arg(fmtNumber(step2Cload))
		                              .arg(fmtNumber(r.torqueNmm / 1000.0));
		return;
	}

	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] drive_disp_mm = %1")
	                              .arg(r.displacementDriven ? fmtNumber(r.driveDispMm) : QStringLiteral("0"));
	const bool   dualTpl  = useDualGearContactTemplate(ctx);
	const double adjustMm = ctx.contactAdjustMm > 0.0
	                            ? ctx.contactAdjustMm
	                            : (dualTpl ? kCcxDualContactAdjustDefaultMm : 0.001);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] contact = exponential(%1,%2), adjust=%3")
	                              .arg(fmtNumber(ctx.contactPressureP0))
	                              .arg(fmtNumber(ctx.contactStiffness))
	                              .arg(fmtNumber(adjustMm));
	if (dualTpl) {
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral(
		    "[GearOpt][CCX] static = %1, %2, %3, %4  (+ FIELD/CONTACT CONTROLS)")
		                              .arg(fmtNumber(kCcxDualContactT0))
		                              .arg(fmtNumber(kCcxDualContactTTotal))
		                              .arg(fmtNumber(kCcxDualContactTMin))
		                              .arg(fmtNumber(kCcxDualContactTMax));
	} else {
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral(
		    "[GearOpt][CCX] static = %1, %2, %3, %4")
		                              .arg(fmtNumber(kCcxStaticT0))
		                              .arg(fmtNumber(kCcxStaticTTotal))
		                              .arg(fmtNumber(kCcxStaticTMin))
		                              .arg(fmtNumber(kCcxStaticTMax));
	}
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX] solver = default spooles");
}

QString buildFileOutputBlock(const InpContext& ctx) {
	if (useMentorTwoStepHardContactTemplate(ctx))
		return QStringLiteral("**  mentor template: *NODE/EL/CONTACT FILE inside each *STEP\n");

	if (useDualGearContactTemplate(ctx)) {
		return QStringLiteral("*NODE FILE, FREQUENCY=999999\n"
		                      "RF, U\n"
		                      "*EL FILE, FREQUENCY=999999\n"
		                      "S\n");
	}
	QString s;
	if (ctx.requestContactOutput && ctx.enableContact)
		s += QStringLiteral("*CONTACT FILE, FREQUENCY=999999\nCSTR\n");
	if (ctx.requestNodeFile)
		s += QStringLiteral("*NODE FILE, FREQUENCY=999999\nU\n");
	if (ctx.requestElFile)
		s += QStringLiteral("*EL FILE, FREQUENCY=999999\nS\n");
	return s;
}

} // anonymous namespace

QVector<CLoadSpec> buildHubTangentialCloads(const QVector<GearHubNodeCoord>& hubNodes,
                                            double centerX,
                                            double centerY,
                                            double torqueNmm,
                                            double* outSumMz) {
	QVector<CLoadSpec> loads;
	if (hubNodes.isEmpty())
		return loads;

	double sumR2 = 0.0;
	for (const GearHubNodeCoord& n : hubNodes) {
		const double rx = n.x - centerX;
		const double ry = n.y - centerY;
		sumR2 += rx * rx + ry * ry;
	}
	if (sumR2 < 1e-18)
		return loads;

	const double k = torqueNmm / sumR2;
	double       checkMz = 0.0;

	for (const GearHubNodeCoord& n : hubNodes) {
		const double rx = n.x - centerX;
		const double ry = n.y - centerY;
		const double fx = -k * ry;
		const double fy = k * rx;
		checkMz += rx * fy - ry * fx;

		if (std::fabs(fx) > 1e-30) {
			CLoadSpec cx;
			cx.nodeId = n.nodeId;
			cx.dof    = 1;
			cx.value  = fx;
			loads.append(cx);
		}
		if (std::fabs(fy) > 1e-30) {
			CLoadSpec cy;
			cy.nodeId = n.nodeId;
			cy.dof    = 2;
			cy.value  = fy;
			loads.append(cy);
		}
	}

	if (outSumMz)
		*outSumMz = checkMz;
	return loads;
}

GearCcxLoadBreakdown computeGear1ToothCload(double torqueNm,
                                            double module,
                                            int z1,
                                            double alphaDeg,
                                            int toothNodeCount) {
	GearCcxLoadBreakdown b;
	b.toothNodeCount = toothNodeCount;
	b.torqueNm       = torqueNm;
	b.torqueNmm      = torqueNm * 1000.0;
	b.d1_mm          = module * static_cast<double>(z1);
	const double pitchR = b.d1_mm * 0.5;
	const double alphaRad =
	    alphaDeg * M_PI / 180.0;
	const double cosA = std::cos(alphaRad);
	b.Ft_N = (pitchR > 1e-9) ? (2.0 * b.torqueNmm / b.d1_mm) : 0.0;
	b.Fn_N = (std::fabs(cosA) > 1e-9) ? (b.Ft_N / cosA) : b.Ft_N;

	const double nx = -cosA;
	const double ny = std::sin(alphaRad);
	const double n  = static_cast<double>(std::max(1, toothNodeCount));
	b.FxPerNode_N   = b.Fn_N * nx / n;
	b.FyPerNode_N   = b.Fn_N * ny / n;
	return b;
}

QString renderJobInp(const InpContext& ctx) {
	QString out = QString::fromLatin1(kCcxStaticTemplate);

	const bool mentorTpl = useMentorTwoStepHardContactTemplate(ctx);
	QString stepAndOutput;
	QString endStepSuffix = QString::fromLatin1(kCcxSingleStepSuffix);
	if (mentorTpl) {
		stepAndOutput = buildMentorTwoStepBlock(ctx);
		endStepSuffix.clear();
	} else {
		stepAndOutput = buildStepBlock(ctx) + buildBoundaryBlock(ctx) + buildCloadBlock(ctx)
		                + buildPrintBlock(ctx) + buildFileOutputBlock(ctx);
	}

	struct Pair { const char* key; QString val; };
	const QVector<Pair> subs = {
		{ "{{run_name}}",            ctx.runName },
		{ "{{gen}}",                 QString::number(ctx.generation) },
		{ "{{point_id}}",            QString::number(ctx.pointId) },
		{ "{{timestamp}}",           QDateTime::currentDateTime().toString(Qt::ISODate) },
		{ "{{mesh_inp}}",            ctx.meshInpFile },
		{ "{{gear_volume_nset_block}}", ctx.gearVolumeNsetBlock },
		{ "{{material_name}}",       ctx.materialName },
		{ "{{young}}",               fmtNumber(ctx.youngModulus) },
		{ "{{poisson}}",             fmtNumber(ctx.poisson) },
		{ "{{density_block}}",       buildDensityBlock(ctx.density) },
		{ "{{solid_section_block}}", buildSolidSectionBlock(ctx) },
		{ "{{ref_nodes_block}}",     buildRefNodesBlock(ctx) },
		{ "{{rigid_body_block}}",    buildRigidBodyBlock(ctx) },
		{ "{{coupling_block}}",      buildCouplingBlock(ctx) },
		{ "{{contact_block}}",       buildContactBlock(ctx) },
		{ "{{step_and_output_block}}", stepAndOutput },
		{ "{{end_step_suffix}}",     endStepSuffix },
	};
	for (const auto& p : subs)
		out.replace(QString::fromLatin1(p.key), p.val);
	return out;
}

QStringList inpRigidBodyKeywordLines(const QString& inpPath) {
	QStringList out;
	QFile f(inpPath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return out;

	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	while (!ts.atEnd()) {
		const QString raw = ts.readLine();
		const QString t   = raw.trimmed();
		if (t.isEmpty() || t.startsWith(QStringLiteral("**")))
			continue;
		if (t.startsWith(QStringLiteral("*RIGID BODY"), Qt::CaseInsensitive))
			out.append(raw);
	}
	return out;
}

QStringList inpCouplingKeywordLines(const QString& inpPath) {
	QStringList out;
	QFile f(inpPath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return out;

	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	while (!ts.atEnd()) {
		const QString raw = ts.readLine();
		const QString t   = raw.trimmed();
		if (t.isEmpty() || t.startsWith(QStringLiteral("**")))
			continue;
		if (t.startsWith(QStringLiteral("*COUPLING"), Qt::CaseInsensitive))
			out.append(raw);
	}
	return out;
}

QString verifyInpCouplingKeywordSpacing(const QString& inpPath) {
	const QStringList lines = inpCouplingKeywordLines(inpPath);
	if (lines.isEmpty())
		return QString();

	for (const QString& raw : lines) {
		if (!raw.contains(QLatin1String("REF NODE"))) {
			return QStringLiteral(
			    "job.inp *COUPLING line missing 'REF NODE' (spaces required): %1").arg(raw.trimmed());
		}
		if (!raw.contains(QLatin1String("CONSTRAINT NAME"))) {
			return QStringLiteral(
			    "job.inp *COUPLING line missing 'CONSTRAINT NAME' (spaces required): %1")
			    .arg(raw.trimmed());
		}
		const QString compact = raw.toUpper().remove(QLatin1Char(' '));
		if (compact.contains(QLatin1String("REFNODE="))
		    && !raw.toUpper().contains(QLatin1String("REF NODE"))) {
			return QStringLiteral("job.inp *COUPLING has REFNODE without space: %1").arg(raw.trimmed());
		}
		if (compact.contains(QLatin1String("CONSTRAINTNAME="))
		    && !raw.toUpper().contains(QLatin1String("CONSTRAINT NAME"))) {
			return QStringLiteral("job.inp *COUPLING has CONSTRAINTNAME without space: %1")
			    .arg(raw.trimmed());
		}
	}
	return QString();
}

void logJobInpAudit(const QString& jobInpPath, const QString& meshInpPath) {
	QFile f(jobInpPath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] cannot open %1").arg(jobInpPath);
		return;
	}

	const QStringList allLines = QString::fromUtf8(f.readAll()).split(QLatin1Char('\n'));

	auto emitBlock = [&](const QString& title, const QStringList& lines) {
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] %1").arg(title);
		for (const QString& ln : lines)
			GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("  ") << ln.trimmed();
	};

	QStringList rigidBodyLines;
	QStringList couplingLines;
	QStringList afterCoupling;
	bool        inCouplingFollow = false;
	int         followLeft         = 0;
	QStringList boundaryLines;
	QStringList cloadLines;
	QStringList contactLines;
	int         cloadCount = 0;
	double      cloadAbsSum = 0.0;

	for (int i = 0; i < allLines.size(); ++i) {
		const QString raw = allLines[i];
		const QString t   = raw.trimmed();
		if (t.isEmpty() || t.startsWith(QStringLiteral("**")))
			continue;

		if (t.startsWith(QStringLiteral("*RIGID BODY"), Qt::CaseInsensitive))
			rigidBodyLines.append(raw.trimmed());

		if (t.startsWith(QStringLiteral("*COUPLING"), Qt::CaseInsensitive)) {
			couplingLines.append(raw.trimmed());
			inCouplingFollow = true;
			followLeft       = 5;
			continue;
		}
		if (inCouplingFollow && followLeft > 0) {
			afterCoupling.append(raw.trimmed());
			--followLeft;
			if (followLeft == 0)
				inCouplingFollow = false;
		}

		if (t.startsWith(QStringLiteral("*BOUNDARY"), Qt::CaseInsensitive)) {
			boundaryLines.append(raw.trimmed());
			for (int j = i + 1; j < allLines.size(); ++j) {
				const QString nt = allLines[j].trimmed();
				if (nt.isEmpty() || nt.startsWith(QStringLiteral("**")))
					continue;
				if (nt.startsWith(QLatin1Char('*')))
					break;
				boundaryLines.append(nt);
			}
		}
		if (t.startsWith(QStringLiteral("*CLOAD"), Qt::CaseInsensitive)) {
			cloadLines.append(raw.trimmed());
			for (int j = i + 1; j < allLines.size(); ++j) {
				const QString nt = allLines[j].trimmed();
				if (nt.isEmpty() || nt.startsWith(QStringLiteral("**")))
					continue;
				if (nt.startsWith(QLatin1Char('*')))
					break;
				cloadLines.append(nt);
				++cloadCount;
				const QStringList toks = nt.split(QLatin1Char(','));
				if (toks.size() >= 3) {
					bool ok = false;
					const double v = toks[2].trimmed().toDouble(&ok);
					if (ok)
						cloadAbsSum += std::fabs(v);
				}
			}
		}
		if (t.startsWith(QStringLiteral("*CONTACT PAIR"), Qt::CaseInsensitive)
		    || t.startsWith(QStringLiteral("*SURFACE INTERACTION"), Qt::CaseInsensitive)) {
			contactLines.append(raw.trimmed());
		}
	}

	emitBlock(QStringLiteral("RIGID BODY lines"), rigidBodyLines);
	emitBlock(QStringLiteral("COUPLING lines"), couplingLines);
	emitBlock(QStringLiteral("COUPLING + next 5 lines"), afterCoupling);

	const bool hasKinematic    = afterCoupling.join(QLatin1Char(' ')).contains(QStringLiteral("*KINEMATIC"), Qt::CaseInsensitive);
	const bool hasDistributing = afterCoupling.join(QLatin1Char(' ')).contains(QStringLiteral("*DISTRIBUTING"), Qt::CaseInsensitive);
	bool         kinematicIllegal = false;
	if (hasKinematic) {
		for (const QString& ln : afterCoupling) {
			const QString t = ln.trimmed();
			if (t.isEmpty() || t.startsWith(QLatin1Char('*')))
				continue;
			const QStringList toks = t.split(QLatin1Char(','));
			for (const QString& tok : toks) {
				bool ok = false;
				const int dof = tok.trimmed().toInt(&ok);
				if (ok && (dof < 1 || dof > 3)) {
					kinematicIllegal = true;
					break;
				}
			}
			if (kinematicIllegal)
				break;
		}
	}
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] *KINEMATIC present = %1  *DISTRIBUTING present = %2  KINEMATIC illegal dof(>3) = %3")
	                              .arg(hasKinematic ? QStringLiteral("yes") : QStringLiteral("no"))
	                              .arg(hasDistributing ? QStringLiteral("yes") : QStringLiteral("no"))
	                              .arg(kinematicIllegal ? QStringLiteral("yes") : QStringLiteral("no"));

	emitBlock(QStringLiteral("BOUNDARY"), boundaryLines);
	emitBlock(QStringLiteral("CLOAD"), cloadLines);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] CLOAD data rows = %1  sum|F| = %2")
	                              .arg(cloadCount)
	                              .arg(cloadAbsSum, 0, 'g', 8);
	emitBlock(QStringLiteral("CONTACT"), contactLines);

	const int h1 = countNsetNodes(meshInpPath, QStringLiteral("GEAR1_HUB"));
	const int h2 = countNsetNodes(meshInpPath, QStringLiteral("GEAR2_HUB"));
	const int s1 = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR1_HUB_SURF"));
	const int s2 = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR2_HUB_SURF"));
	const int t1 = countSurfaceFaces(meshInpPath, QStringLiteral("master"));
	const int t2 = countSurfaceFaces(meshInpPath, QStringLiteral("slave"));
	const int d1 = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR1_TOOTH_SURF"));
	const int d2 = countSurfaceFaces(meshInpPath, QStringLiteral("GEAR2_TOOTH_SURF"));

	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] GEAR1_HUB nodes = %1  GEAR2_HUB nodes = %2")
	                              .arg(h1)
	                              .arg(h2);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] GEAR1_HUB_SURF faces = %1  GEAR2_HUB_SURF faces = %2")
	                              .arg(s1)
	                              .arg(s2);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] master faces = %1  slave faces = %2")
	                              .arg(t1)
	                              .arg(t2);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] GEAR1_TOOTH_SURF faces (debug) = %1  GEAR2_TOOTH_SURF faces (debug) = %2")
	                              .arg(d1)
	                              .arg(d2);

	const bool hasContactPair =
	    contactLines.join(QLatin1Char(' ')).contains(QStringLiteral("*CONTACT PAIR"), Qt::CaseInsensitive);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][CCX][Audit] contact pair in job.inp = %1")
	                              .arg(hasContactPair ? QStringLiteral("yes") : QStringLiteral("no"));
}

bool writeJobInp(const QString& dir, const InpContext& ctxIn) {
	QDir d(dir);
	if (!d.exists())
		return false;

	InpContext ctx = ctxIn;
	applyReferenceTorquePolicy(ctx);

	const QString path = d.filePath(QStringLiteral("job.inp"));
	const QString meshPath = d.filePath(ctx.meshInpFile);
	if (ctx.twoGearJob)
		ctx.gearVolumeNsetBlock = buildGearVolumeNsetsBlock(meshPath);
	logCcxInpWriteSummary(ctx, meshPath);

	const QByteArray body = renderJobInp(ctx).toUtf8();

	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
		return false;
	if (f.write(body) != body.size())
		return false;
	f.close();

	if (ctx.twoGearJob && ctx.useRigidBody) {
		for (const QString& raw : inpRigidBodyKeywordLines(path))
			GEAR_OPT_DEBUG_NOQUOTE << "[GearOpt][CCX] job.inp written *RIGID BODY:" << raw.trimmed();
		logCcxJobTemplateSummary(ctx);
	} else if (ctx.useCoupling) {
		const QString err = verifyInpCouplingKeywordSpacing(path);
		if (!err.isEmpty()) {
			GEAR_OPT_DEBUG_NOQUOTE << "[GearOpt][CCX]" << err;
			return false;
		}
		for (const QString& raw : inpCouplingKeywordLines(path))
			GEAR_OPT_DEBUG_NOQUOTE << "[GearOpt][CCX] job.inp written *COUPLING:" << raw;
	}
	logJobInpAudit(path, meshPath);
	return true;
}

double mentorStep2CloadZFromTorqueNmm(double torqueNmm, double rotOffsetMm)
{
	if (rotOffsetMm <= 0.0 || torqueNmm <= 0.0)
		return 0.0;
	return torqueNmm / rotOffsetMm;
}

} // namespace GearAutoOpt
