#include "CCXInpWriter.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>

namespace GearAutoOpt {

namespace {

// CalculiX 主控 inp 模板。占位符 {{key}} 在 renderJobInp 内做字符串替换。
//
// 设计要点：
//   * 不写 *HEADING，让 mesh.inp 提供（gmsh 会自动写一个，重复声明 ccx 会报错）
//   * 单步 *STATIC，单一 *MATERIAL，最常见齿根应力分析配置
//   * BOUNDARY / CLOAD 行由代码动态拼接，模板里只放占位 {{boundary_block}} {{cload_block}}
//   * 输出请求按 ctx.requestNodeFile / requestElFile 决定是否拼入
static const char* kCcxStaticTemplate =
	"**\n"
	"**  GearAutoOpt - CalculiX 静力分析驱动 inp\n"
	"**  Run: {{run_name}}, Gen: {{gen}}, Point: {{point_id}}\n"
	"**  Generated: {{timestamp}}\n"
	"**  注意：mesh.inp 自带 *Heading，本驱动不可重复声明\n"
	"**\n"
	"*INCLUDE, INPUT={{mesh_inp}}\n"
	"**\n"
	"*MATERIAL, NAME={{material_name}}\n"
	"*ELASTIC\n"
	"{{young}}, {{poisson}}\n"
	"{{density_block}}"
	"*SOLID SECTION, ELSET={{volume_elset}}, MATERIAL={{material_name}}\n"
	"**\n"
	"{{boundary_block}}"
	"**\n"
	"*STEP\n"
	"*STATIC\n"
	"{{cload_block}}"
	"*NODE PRINT, NSET={{output_nset}}\n"
	"U\n"
	"*EL PRINT, ELSET={{output_elset}}\n"
	"S\n"
	"{{file_output_block}}"
	"*END STEP\n";

QString fmtNumber(double v) {
	// CalculiX 对 1e-9 这种很小的数支持 E 记号；用 g 保证精度且不冗余
	return QString::number(v, 'g', 10);
}

QString buildBoundaryBlock(const QVector<BoundarySpec>& boundaries) {
	if (boundaries.isEmpty()) {
		return QStringLiteral("**  no *BOUNDARY specified\n");
	}
	QString s = QStringLiteral("*BOUNDARY\n");
	for (const auto& b : boundaries) {
		s += QString("%1, %2, %3\n").arg(b.nset).arg(b.dofStart).arg(b.dofEnd);
	}
	return s;
}

QString buildCloadBlock(const QVector<CLoadSpec>& cloads) {
	if (cloads.isEmpty()) {
		return QStringLiteral("**  no *CLOAD specified\n");
	}
	QString s = QStringLiteral("*CLOAD\n");
	for (const auto& c : cloads) {
		s += QString("%1, %2, %3\n").arg(c.nset).arg(c.dof).arg(fmtNumber(c.value));
	}
	return s;
}

QString buildDensityBlock(double density) {
	if (density <= 0.0) {
		return QString();
	}
	return QString("*DENSITY\n%1\n").arg(fmtNumber(density));
}

QString buildFileOutputBlock(const InpContext& ctx) {
	QString s;
	if (ctx.requestNodeFile) s += QStringLiteral("*NODE FILE\nU\n");
	if (ctx.requestElFile)   s += QStringLiteral("*EL FILE\nS\n");
	return s;
}

} // anonymous namespace

QString renderJobInp(const InpContext& ctx) {
	QString out = QString::fromLatin1(kCcxStaticTemplate);

	const QString outNset  = ctx.outputNset.isEmpty()  ? ctx.volumeElset : ctx.outputNset;
	const QString outElset = ctx.outputElset.isEmpty() ? ctx.volumeElset : ctx.outputElset;

	struct Pair { const char* key; QString val; };
	const QVector<Pair> subs = {
		{ "{{run_name}}",         ctx.runName },
		{ "{{gen}}",              QString::number(ctx.generation) },
		{ "{{point_id}}",         QString::number(ctx.pointId) },
		{ "{{timestamp}}",        QDateTime::currentDateTime().toString(Qt::ISODate) },
		{ "{{mesh_inp}}",         ctx.meshInpFile },
		{ "{{material_name}}",    ctx.materialName },
		{ "{{young}}",            fmtNumber(ctx.youngModulus) },
		{ "{{poisson}}",          fmtNumber(ctx.poisson) },
		{ "{{density_block}}",    buildDensityBlock(ctx.density) },
		{ "{{volume_elset}}",     ctx.volumeElset },
		{ "{{boundary_block}}",   buildBoundaryBlock(ctx.boundaries) },
		{ "{{cload_block}}",      buildCloadBlock(ctx.cloads) },
		{ "{{output_nset}}",      outNset },
		{ "{{output_elset}}",     outElset },
		{ "{{file_output_block}}", buildFileOutputBlock(ctx) },
	};
	for (const auto& p : subs) {
		out.replace(QString::fromLatin1(p.key), p.val);
	}
	return out;
}

bool writeJobInp(const QString& dir, const InpContext& ctx) {
	QDir d(dir);
	if (!d.exists()) {
		return false;
	}
	const QString path = d.filePath(QStringLiteral("job.inp"));
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
		return false;
	}
	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	ts << renderJobInp(ctx);
	return f.error() == QFile::NoError;
}

} // namespace GearAutoOpt
