#include "CCXResultParser.h"
#include "GearAutoOpt/data/GearOptLog.h"

#include <QDebug>
#include <QFile>
#include <QRegExp>
#include <QRegularExpression>
#include <QRegularExpressionMatchIterator>
#include <QString>
#include <QStringList>
#include <QTextStream>

#include <algorithm>
#include <cmath>

namespace GearAutoOpt {

double StressSample::vonMises() const {
	const double a = sxx - syy;
	const double b = syy - szz;
	const double c = szz - sxx;
	const double shearSq = sxy * sxy + sxz * sxz + syz * syz;
	const double vmSq = 0.5 * (a * a + b * b + c * c) + 3.0 * shearSq;
	return std::sqrt(std::max(0.0, vmSq));
}

namespace {

// 段头正则：
//   stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set BEAM and time  ...
//   ^^^^^^^^                                            ^^^^^^^^ ^^^^
//   关键字                                              "for set" + 名字
//
// 我们用关键字判断而非正则全匹配，避免空格变化导致漏匹配。
bool parseDispHeader(const QString& line, const QString& targetSet, double* outTime) {
	const QString lower = line.toLower();
	if (!lower.contains(QLatin1String("displacements")))
		return false;
	if (!lower.contains(QLatin1String("for set")))
		return false;

	const int idx = lower.indexOf(QLatin1String("for set"));
	if (idx < 0)
		return false;
	int begin = idx + 7;
	while (begin < lower.size() && lower[begin].isSpace())
		++begin;
	int end = begin;
	while (end < lower.size() && !lower[end].isSpace())
		++end;
	const QString name = lower.mid(begin, end - begin);
	if (name != targetSet.toLower())
		return false;

	if (outTime) {
		*outTime = -1.0;
		const int tidx = lower.indexOf(QLatin1String("and time"));
		if (tidx >= 0) {
			QString tail = line.mid(tidx + 8).trimmed();
			bool ok = false;
			const double t = tail.section(QLatin1Char(' '), 0, 0).toDouble(&ok);
			if (ok)
				*outTime = t;
		}
	}
	return true;
}

bool parseStressHeader(const QString& line, const QString& targetSet, double* outTime) {
	const QString lower = line.toLower();
	if (!lower.contains(QLatin1String("stresses")))
		return false;
	if (!lower.contains(QLatin1String("for set")))
		return false;

	const int idx = lower.indexOf(QLatin1String("for set"));
	if (idx < 0)
		return false;
	int begin = idx + 7;
	while (begin < lower.size() && lower[begin].isSpace())
		++begin;
	int end = begin;
	while (end < lower.size() && !lower[end].isSpace())
		++end;
	const QString name = lower.mid(begin, end - begin);
	if (name != targetSet.toLower())
		return false;

	if (outTime) {
		*outTime = -1.0;
		const int tidx = lower.indexOf(QLatin1String("and time"));
		if (tidx >= 0) {
			QString tail = line.mid(tidx + 8).trimmed();
			bool ok = false;
			const double t = tail.section(QLatin1Char(' '), 0, 0).toDouble(&ok);
			if (ok)
				*outTime = t;
		}
	}
	return true;
}

QStringList splitTokens(const QString& line) {
	return line.trimmed().split(QRegExp("\\s+"), QString::SkipEmptyParts);
}

bool parseRow(const QString& line, StressSample* out) {
	const QStringList toks = splitTokens(line);
	if (toks.size() < 8) return false;
	bool ok1 = false, ok2 = false;
	const int elemId = toks[0].toInt(&ok1);
	const int ip     = toks[1].toInt(&ok2);
	if (!ok1 || !ok2) return false;
	bool ok = false;
	auto parse = [&](int idx, double* v) -> bool {
		bool localOk = false;
		const double x = toks[idx].toDouble(&localOk);
		if (!localOk) return false;
		*v = x;
		return true;
	};
	out->elemId  = elemId;
	out->integPt = ip;
	if (!parse(2, &out->sxx)) return false;
	if (!parse(3, &out->syy)) return false;
	if (!parse(4, &out->szz)) return false;
	if (!parse(5, &out->sxy)) return false;
	if (!parse(6, &out->sxz)) return false;
	if (!parse(7, &out->syz)) return false;
	(void)ok;
	return true;
}

struct VmRanked {
	double vm   = 0.0;
	int    elem = 0;
	int    ip   = 0;
	int    node = 0;
};

static VmRanked percentileVmFromRanked(QVector<VmRanked> ranked, double percentile)
{
	if (ranked.isEmpty())
		return {};
	std::sort(ranked.begin(), ranked.end(),
	          [](const VmRanked& a, const VmRanked& b) { return a.vm < b.vm; });
	const double p = std::clamp(percentile, 0.0, 1.0);
	const int n      = ranked.size();
	const int idx    = (n <= 1) ? 0 : std::min(n - 1, static_cast<int>(std::floor(p * (n - 1))));
	return ranked[idx];
}

QVector<StressSample> samplesAtTime(const QString& path, const QString& elsetName, double time) {
	QVector<StressSample> samples;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return samples;
	QTextStream ts(&f);
	ts.setCodec("UTF-8");

	bool inSection = false;
	while (!ts.atEnd()) {
		const QString line    = ts.readLine();
		const QString trimmed = line.trimmed();

		double blockTime = -1.0;
		if (parseStressHeader(line, elsetName, &blockTime)) {
			inSection = blockTime < 0.0
			            || std::fabs(blockTime - time) < 1e-9 * std::max(1.0, std::fabs(time));
			continue;
		}
		if (!inSection)
			continue;
		if (trimmed.isEmpty())
			continue;
		if (!trimmed[0].isDigit() && trimmed[0] != QLatin1Char('-')) {
			inSection = false;
			continue;
		}
		StressSample s;
		if (parseRow(line, &s))
			samples.push_back(s);
	}
	return samples;
}

} // anonymous namespace

double vonMisesPercentilePeak(const QVector<double>& misesValues, double percentile)
{
	if (misesValues.isEmpty())
		return -1.0;
	const double p = std::clamp(percentile, 0.0, 1.0);
	QVector<double> sorted = misesValues;
	std::sort(sorted.begin(), sorted.end());
	const int n   = sorted.size();
	const int idx = (n <= 1) ? 0 : std::min(n - 1, static_cast<int>(std::floor(p * (n - 1))));
	return sorted[idx];
}

QVector<StressSample> parseDatSamples(const QString& path, const QString& elsetName) {
	const DatStressResult r = parseDat(path, elsetName);
	if (!r.found())
		return {};
	return samplesAtTime(path, elsetName, r.finalTime);
}

DatStressResult parseDat(const QString& path, const QString& elsetName) {
	DatStressResult r;
	r.elsetName = elsetName;

	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return r;
	QTextStream ts(&f);
	ts.setCodec("UTF-8");

	QHash<double, QVector<StressSample>> byTime;
	bool                             inSection = false;
	double                           curTime   = -1.0;

	while (!ts.atEnd()) {
		const QString line    = ts.readLine();
		const QString trimmed = line.trimmed();

		double blockTime = -1.0;
		if (parseStressHeader(line, elsetName, &blockTime)) {
			inSection = true;
			curTime   = blockTime;
			if (curTime < 0.0)
				curTime = 0.0;
			continue;
		}
		if (!inSection)
			continue;
		if (trimmed.isEmpty())
			continue;
		if (!trimmed[0].isDigit() && trimmed[0] != QLatin1Char('-')) {
			inSection = false;
			continue;
		}
		StressSample s;
		if (!parseRow(line, &s))
			continue;
		byTime[curTime].append(s);
	}

	if (byTime.isEmpty())
		return r;

	double finalTime = -1.0;
	for (auto it = byTime.constBegin(); it != byTime.constEnd(); ++it) {
		if (it.key() > finalTime)
			finalTime = it.key();
	}
	r.finalTime = finalTime;

	const QVector<StressSample> finals = byTime.value(finalTime);
	r.sampleCount = finals.size();
	for (const StressSample& s : finals) {
		const double vm = s.vonMises();
		if (r.maxVonMises < 0.0 || vm > r.maxVonMises) {
			r.maxVonMises = vm;
			r.maxElemId   = s.elemId;
			r.maxIntegPt  = s.integPt;
		}
	}
	return r;
}

DatDispResult parseDatDisplacements(const QString& path, const QString& nsetName) {
	DatDispResult r;
	r.nsetName = nsetName;

	struct DispRow {
		int      nodeId = 0;
		NodeDisp disp;
	};

	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return r;
	QTextStream ts(&f);
	ts.setCodec("UTF-8");

	QHash<double, QVector<DispRow>> byTime;
	bool   inSection = false;
	double curTime   = -1.0;

	while (!ts.atEnd()) {
		const QString line    = ts.readLine();
		const QString trimmed = line.trimmed();

		double blockTime = -1.0;
		if (parseDispHeader(line, nsetName, &blockTime)) {
			inSection = true;
			curTime   = blockTime;
			if (curTime < 0.0)
				curTime = 0.0;
			continue;
		}
		if (!inSection)
			continue;
		if (trimmed.isEmpty())
			continue;
		if (!trimmed[0].isDigit() && trimmed[0] != QLatin1Char('-')) {
			inSection = false;
			continue;
		}

		const QStringList toks = splitTokens(line);
		if (toks.size() < 4)
			continue;
		bool okId = false;
		const int nid = toks[0].toInt(&okId);
		if (!okId)
			continue;
		bool okUx = false, okUy = false, okUz = false;
		const double ux = toks[1].toDouble(&okUx);
		const double uy = toks[2].toDouble(&okUy);
		const double uz = toks[3].toDouble(&okUz);
		if (!okUx || !okUy || !okUz)
			continue;

		DispRow row;
		row.nodeId     = nid;
		row.disp       = NodeDisp{ux, uy, uz};
		byTime[curTime].append(row);
	}

	if (byTime.isEmpty())
		return r;

	double finalTime = -1.0;
	for (auto it = byTime.constBegin(); it != byTime.constEnd(); ++it) {
		if (it.key() > finalTime)
			finalTime = it.key();
	}
	r.finalTime = finalTime;

	const QVector<DispRow> finals = byTime.value(finalTime);
	r.sampleCount = finals.size();
	for (const DispRow& row : finals) {
		const double um = row.disp.magnitude();
		if (r.maxMagnitude < 0.0 || um > r.maxMagnitude) {
			r.maxMagnitude = um;
			r.maxNodeId    = row.nodeId;
		}
	}
	return r;
}

// =====================================================================
// .frd 解析
// =====================================================================

double NodeDisp::magnitude() const {
	return std::sqrt(ux * ux + uy * uy + uz * uz);
}

double NodeStress::vonMises() const {
	const double a = sxx - syy;
	const double b = syy - szz;
	const double c = szz - sxx;
	const double shearSq = sxy * sxy + syz * syz + szx * szx;
	return std::sqrt(std::max(0.0, 0.5 * (a * a + b * b + c * c) + 3.0 * shearSq));
}

double FrdResult::maxVonMises(int* outNodeId) const {
	QVector<VmRanked> ranked;
	ranked.reserve(stresses.size());
	for (auto it = stresses.constBegin(); it != stresses.constEnd(); ++it) {
		VmRanked row;
		row.vm   = it.value().vonMises();
		row.node = it.key();
		ranked.append(row);
	}
	const VmRanked peak = percentileVmFromRanked(ranked, 0.99);
	if (outNodeId)
		*outNodeId = peak.node;
	return ranked.isEmpty() ? -1.0 : peak.vm;
}

double vonMisesPercentileForNodes(const FrdResult& frd,
                                  const QSet<int>& nodeIds,
                                  double percentile,
                                  int* outNodeId)
{
	QVector<VmRanked> ranked;
	for (auto it = frd.stresses.constBegin(); it != frd.stresses.constEnd(); ++it) {
		if (!nodeIds.isEmpty() && !nodeIds.contains(it.key()))
			continue;
		VmRanked row;
		row.vm   = it.value().vonMises();
		row.node = it.key();
		ranked.append(row);
	}
	const VmRanked peak = percentileVmFromRanked(ranked, percentile);
	if (outNodeId)
		*outNodeId = peak.node;
	return ranked.isEmpty() ? -1.0 : peak.vm;
}

double FrdResult::maxDisplMagnitude(int* outNodeId) const {
	double best   = -1.0;
	int    bestId = -1;
	for (auto it = displacements.constBegin(); it != displacements.constEnd(); ++it) {
		const NodeDisp& u = it.value();
		const double  m   = u.magnitude();
		if (!std::isfinite(m) || m > 100.0) {
			qWarning().noquote()
			    << QStringLiteral("[Umax][skip abnormal] node=%1 u=(%2,%3,%4) mag=%5")
			           .arg(it.key())
			           .arg(u.ux, 0, 'g', 10)
			           .arg(u.uy, 0, 'g', 10)
			           .arg(u.uz, 0, 'g', 10)
			           .arg(m, 0, 'g', 10);
			continue;
		}
		if (m > best) {
			best   = m;
			bestId = it.key();
		}
	}
	if (outNodeId)
		*outNodeId = bestId;
	return best;
}

namespace {

enum class FrdBlock {
	None,
	Node,
	Disp,
	Stress,
	Contact,
	OtherNodal,
	Element,
};

enum class FrdDataFormat {
	Short  = 0,
	Long   = 1,
	Binary = 2,
};

int frdValueStartCol(FrdDataFormat fmt)
{
	return 3 + (fmt == FrdDataFormat::Short ? 5 : 10);
}

// CCX 2.x ASCII FRD：节点号后各分量直接拼接，指数固定 3 位（如 2.30343E-003）。
QVector<double> parseFrdGluedSciFields(const QString& input)
{
	QVector<double> vals;
	if (input.isEmpty())
		return vals;

	static const QRegularExpression rx(
	    QStringLiteral("([+-]?\\d+\\.\\d+E[+-]\\d{3})"));
	auto it = rx.globalMatch(input);
	while (it.hasNext()) {
		const QRegularExpressionMatch m = it.next();
		bool                          ok = false;
		const double                  v  = m.captured(1).toDouble(&ok);
		if (ok)
			vals.push_back(v);
	}
	return vals;
}

// 从粘连字符串提取科学计数法浮点（负号可紧贴前一数的指数后）。
QVector<double> parseFrdFloatFieldsImpl(const QString& input)
{
	return parseFrdGluedSciFields(input);
}

struct FrdDispLine {
	int    nodeId = -1;
	double ux     = 0.0;
	double uy     = 0.0;
	double uz     = 0.0;
	bool   ok     = false;
};

// 在 ` -1 ID ...` 数据行里抽 ID。短格式 I5（col 3-7）/ 长格式 I10（col 3-12）。
int extractNodeId(const QString& line, bool longFormat) {
	if (line.size() < 8)
		return -1;
	const int width = longFormat ? 10 : 5;
	if (line.size() < 3 + width)
		return -1;
	bool ok = false;
	const int id = line.mid(3, width).trimmed().toInt(&ok);
	return ok ? id : -1;
}

// 解析 ` -1 <nodeId I5/I10> <ux><uy><uz>` FRD 位移行（定宽节点号 + 粘连 E±ddd）。
FrdDispLine parseFrdDispDataLine(const QString& line, bool longFmt)
{
	FrdDispLine out;
	const FrdDataFormat fmt = longFmt ? FrdDataFormat::Long : FrdDataFormat::Short;
	const int           nid = extractNodeId(line, longFmt);
	if (nid < 0)
		return out;

	const QVector<double> vals =
	    parseFrdGluedSciFields(line.mid(frdValueStartCol(fmt)));
	if (vals.size() < 3)
		return out;

	out.ok     = true;
	out.nodeId = nid;
	out.ux     = vals[0];
	out.uy     = vals[1];
	out.uz     = vals[2];
	return out;
}

void logFrdDispParsed(const QString& rawLine, const FrdDispLine& d, int debugIndex)
{
	if (debugIndex >= 0 && debugIndex < 10) {
		const double mag = std::sqrt(d.ux * d.ux + d.uy * d.uy + d.uz * d.uz);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[FRD-DISP] #%1 node=%2 U=(%3,%4,%5) |U|=%6 raw=%7")
		                              .arg(debugIndex)
		                              .arg(d.nodeId)
		                              .arg(d.ux, 0, 'g', 10)
		                              .arg(d.uy, 0, 'g', 10)
		                              .arg(d.uz, 0, 'g', 10)
		                              .arg(mag, 0, 'g', 10)
		                              .arg(rawLine.trimmed());
	}
}

// 抽出 .frd 数据行 (` -1 <id> <vals...>`) 中的数值。非 DISP 块仍用正则/sci 扫描。
QVector<double> extractFrdValues(const QString& line, int startCol) {
	QVector<double> vals;
	if (startCol >= line.size())
		return vals;
	return parseFrdFloatFieldsImpl(line.mid(startCol));
}

// CCX 2.x FRD：-4 行 NAME 为 8 字符定宽（DISP / DISPL / U 等）。
QString frdBlockNameFromMinus4(const QString& trimmed) {
	if (!trimmed.startsWith(QLatin1String("-4")))
		return {};
	int pos = 2;
	while (pos < trimmed.size() && trimmed[pos].isSpace())
		++pos;
	if (pos >= trimmed.size())
		return {};
	return trimmed.mid(pos, 8).trimmed().toUpper();
}

bool isFrdDisplacementDataset(const QString& name) {
	const QString n = name.trimmed().toUpper();
	return n == QStringLiteral("DISP") || n == QStringLiteral("DISPL") || n == QStringLiteral("U");
}

bool isFrdContactDataset(const QString& name)
{
	return name.trimmed().toUpper() == QStringLiteral("CONTACT");
}

/// `-5  CPRESS ...` 行中的分量名（取 -5 后第一个 token）。
QString frdMinus5VarName(const QString& trimmed)
{
	if (!trimmed.startsWith(QLatin1String("-5")))
		return {};
	const QString rest = trimmed.mid(2).trimmed();
	if (rest.isEmpty())
		return {};
	return rest.section(QRegExp(QStringLiteral("\\s+")), 0, 0).trimmed().toUpper();
}

/// `1PSTEP <step> <inc> ...` 定宽行：取 1PSTEP 后第一、第二个整数字段。
bool parseFrdPlotStepIncrement(const QString& trimmed, int* outStep, int* outInc)
{
	static const QString key = QStringLiteral("1PSTEP");
	const int            pos = trimmed.indexOf(key, 0, Qt::CaseInsensitive);
	if (pos < 0)
		return false;

	const QStringList parts =
	    trimmed.mid(pos + key.size()).trimmed().split(QRegExp(QStringLiteral("\\s+")),
	                                                  QString::SkipEmptyParts);
	if (parts.size() < 2)
		return false;

	bool okStep = false;
	bool okInc  = false;
	const int step = parts[0].toInt(&okStep);
	const int inc  = parts[1].toInt(&okInc);
	if (!okStep || !okInc)
		return false;
	if (outStep)
		*outStep = step;
	if (outInc)
		*outInc = inc;
	return true;
}

void updateFrameCpressMax(double value, double* frameMax)
{
	if (!frameMax || !std::isfinite(value))
		return;
	if (*frameMax < 0.0 || value > *frameMax)
		*frameMax = value;
}

QVector<double> extractFrdValuesAuto(const QString& line, FrdDataFormat fmt)
{
	return parseFrdGluedSciFields(line.mid(frdValueStartCol(fmt)));
}

} // anonymous namespace

QStringList listFrdDatasetNames(const QString& path) {
	QStringList names;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
		return names;
	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	while (!ts.atEnd()) {
		const QString trimmed = ts.readLine().trimmed();
		if (!trimmed.startsWith(QLatin1String("-4")))
			continue;
		const QString name = frdBlockNameFromMinus4(trimmed);
		if (!name.isEmpty() && !names.contains(name))
			names.append(name);
	}
	return names;
}

FrdResult parseFrd(const QString& path) {
	FrdResult result;

	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return result;
	QTextStream ts(&f);
	ts.setCodec("UTF-8");

	FrdBlock      current      = FrdBlock::None;
	FrdDataFormat nodalFormat  = FrdDataFormat::Long;
	int           dispDebugIdx = 0;

	QStringList contactVarNames;
	int           contactCpressIdx     = -1;
	bool          contactAwaitingMinus5  = false;

	int    curStep              = -1;
	int    curInc               = -1;
	double stepCpressMax        = -1.0;
	bool   stepHasContactBlock  = false;
	QHash<int, double> stepCpressByNode;

	double lastContactCpressMax = -1.0;
	QHash<int, double> lastContactCpressByNode;

	auto finalizeStepContact = [&]() {
		if (!stepHasContactBlock || stepCpressMax < 0.0)
			return;
		lastContactCpressMax = stepCpressMax;
		lastContactCpressByNode = stepCpressByNode;
	};

	auto onNewStepIncrement = [&](int step, int inc) {
		finalizeStepContact();
		curStep             = step;
		curInc              = inc;
		stepCpressMax       = -1.0;
		stepHasContactBlock = false;
		stepCpressByNode.clear();
	};

	auto beginContactBlock = [&]() {
		contactVarNames.clear();
		contactCpressIdx    = -1;
		contactAwaitingMinus5 = true;
	};

	while (!ts.atEnd()) {
		const QString line = ts.readLine();
		const QString trimmed = line.trimmed();

		// 文件结束标记
		if (trimmed == QLatin1String("9999")) break;
		if (trimmed.isEmpty()) continue;

		int plotStep = -1;
		int plotInc  = -1;
		if (parseFrdPlotStepIncrement(trimmed, &plotStep, &plotInc))
			onNewStepIncrement(plotStep, plotInc);

		// 块结束标记
		if (trimmed.startsWith(QLatin1String("-3"))) {
			current = FrdBlock::None;
			contactAwaitingMinus5 = false;
			continue;
		}

		// 节点块头："    2C ..."（C 在第 5 列）
		if (line.size() >= 6 && line.at(5) == QChar('C')
		    && line.mid(0, 5).trimmed() == QStringLiteral("2")) {
			current = FrdBlock::Node;
			continue;
		}
		// 单元块头："    3C ..."
		if (line.size() >= 6 && line.at(5) == QChar('C')
		    && line.mid(0, 5).trimmed() == QStringLiteral("3")) {
			current = FrdBlock::Element;
			continue;
		}
		// 节点结果块头：以 "100C" 开头
		if (trimmed.startsWith(QLatin1String("100C"))) {
			current = FrdBlock::OtherNodal;
			// FORMAT 为 100C 记录末尾 I2：0 短 / 1 长 / 2 二进制
			for (int i = trimmed.size() - 1; i >= 0; --i) {
				const QChar c = trimmed[i];
				if (c >= QLatin1Char('0') && c <= QLatin1Char('2')) {
					nodalFormat = static_cast<FrdDataFormat>(c.unicode() - QLatin1Char('0').unicode());
					break;
				}
				if (c.isLetter())
					break;
			}
			continue;
		}
		// "-4 BLOCK_NAME ..." 设置当前块类型（CCX 2.23：DISP / DISPL / U 等）
		if (trimmed.startsWith(QLatin1String("-4"))) {
			const QString name = frdBlockNameFromMinus4(trimmed);
		GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[FRD] -4 block = %1").arg(name);
			if (isFrdDisplacementDataset(name)) {
				result.displacements.clear();
				current      = FrdBlock::Disp;
				dispDebugIdx = 0;
			GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[FRD] displacement block = %1").arg(name);
			} else if (name.startsWith(QLatin1String("STRESS")))
				current = FrdBlock::Stress;
			else if (isFrdContactDataset(name)) {
				current = FrdBlock::Contact;
				beginContactBlock();
				stepHasContactBlock = true;
			GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[FRD] contact block = %1").arg(name);
			} else
				current = FrdBlock::OtherNodal;
			continue;
		}
		// "-5 ..."：CONTACT 块内读取子变量名以定位 CPRESS 列
		if (trimmed.startsWith(QLatin1String("-5"))) {
			if (current == FrdBlock::Contact && contactAwaitingMinus5) {
				const QString varName = frdMinus5VarName(trimmed);
				if (!varName.isEmpty()) {
					contactVarNames.append(varName);
					if (varName == QStringLiteral("CPRESS"))
						contactCpressIdx = contactVarNames.size() - 1;
				}
			}
			continue;
		}

		// 数据行：以 " -1" 开头（二进制块内不会出现该文本行）
		if (!line.startsWith(QStringLiteral(" -1"))) continue;

		const bool longFmt = (nodalFormat != FrdDataFormat::Short);

		if (current == FrdBlock::Contact) {
			contactAwaitingMinus5 = false;
			if (contactCpressIdx < 0)
				continue;
			const int nid = extractNodeId(line, longFmt);
			if (nid < 0)
				continue;

			const QVector<double> vals = extractFrdValuesAuto(line, nodalFormat);
			if (contactCpressIdx >= vals.size())
				continue;

			const double cpress = vals[contactCpressIdx];
			if (!std::isfinite(cpress))
				continue;

			updateFrameCpressMax(cpress, &stepCpressMax);
			stepCpressByNode.insert(nid, cpress);
			continue;
		}

		if (current == FrdBlock::Disp) {
			const FrdDispLine d = parseFrdDispDataLine(line, longFmt);
			if (!d.ok)
				continue;

			const double mag = std::sqrt(d.ux * d.ux + d.uy * d.uy + d.uz * d.uz);
			if (!std::isfinite(mag) || mag > 100.0) {
				qWarning().noquote()
				    << QStringLiteral("[FRD][DISP][SKIP abnormal] node=%1 u=(%2,%3,%4) mag=%5 line=%6")
				           .arg(d.nodeId)
				           .arg(d.ux, 0, 'g', 10)
				           .arg(d.uy, 0, 'g', 10)
				           .arg(d.uz, 0, 'g', 10)
				           .arg(mag, 0, 'g', 10)
				           .arg(line.trimmed());
				continue;
			}

			logFrdDispParsed(line, d, dispDebugIdx);
			++dispDebugIdx;
			NodeDisp disp{d.ux, d.uy, d.uz};
			result.displacements.insert(d.nodeId, disp);
			continue;
		}

		const int  nid     = extractNodeId(line, longFmt);
		if (nid < 0) continue;
		const QVector<double> vals = extractFrdValuesAuto(line, nodalFormat);

		switch (current) {
		case FrdBlock::Node:
			if (vals.size() >= 3) {
				NodeXYZ p{vals[0], vals[1], vals[2]};
				result.nodes.insert(nid, p);
			}
			break;
		case FrdBlock::Stress:
			if (vals.size() >= 6) {
				NodeStress s{vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]};
				result.stresses.insert(nid, s);
			}
			break;
		default:
			break;  // Element/OtherNodal/None 都跳过
		}
	}

	finalizeStepContact();
	result.cpressMax_MPa = lastContactCpressMax;
	result.cpressByNode = lastContactCpressByNode;
	if (result.cpressMax_MPa < 0.0) {
		result.cpressMax_MPa = -1.0;
		qWarning().noquote()
		    << QStringLiteral("[FRD][CONTACT] CPRESS not found in %1").arg(path);
	} else {
		GEAR_OPT_DEBUG_NOQUOTE
		    << QStringLiteral("[FRD][CONTACT] cpressMax_MPa=%1")
		           .arg(result.cpressMax_MPa, 0, 'g', 8);
	}

	return result;
}

CpressDistributionMetrics parseCpressDistributionMetrics(const QString& frdPath, int binCount)
{
	CpressDistributionMetrics m;
	m.cpressBinCount = std::max(1, binCount);
	m.bins.reserve(m.cpressBinCount);
	for (int i = 0; i < m.cpressBinCount; ++i) {
		CpressWidthBin b;
		b.binIndex = i;
		m.bins.append(b);
	}

	const FrdResult frd = parseFrd(frdPath);
	if (frd.cpressMax_MPa <= 0.0 || frd.cpressByNode.isEmpty() || frd.nodes.isEmpty())
		return m;

	double zMinAll = 0.0;
	double zMaxAll = 0.0;
	bool   haveZ   = false;
	for (auto it = frd.nodes.constBegin(); it != frd.nodes.constEnd(); ++it) {
		const double z = it.value().z;
		if (!std::isfinite(z))
			continue;
		if (!haveZ) {
			zMinAll = zMaxAll = z;
			haveZ = true;
		} else {
			zMinAll = std::min(zMinAll, z);
			zMaxAll = std::max(zMaxAll, z);
		}
	}
	if (!haveZ || zMaxAll <= zMinAll)
		return m;

	const double threshold = std::max(10.0, 0.02 * frd.cpressMax_MPa);
	struct ActiveNode {
		double z = 0.0;
		double p = 0.0;
	};
	QVector<ActiveNode> active;
	active.reserve(frd.cpressByNode.size());

	double zMinContact = 0.0;
	double zMaxContact = 0.0;
	double sum = 0.0;
	bool haveContactZ = false;
	for (auto it = frd.cpressByNode.constBegin(); it != frd.cpressByNode.constEnd(); ++it) {
		const double p = it.value();
		if (!std::isfinite(p) || p <= threshold)
			continue;
		const auto nit = frd.nodes.constFind(it.key());
		if (nit == frd.nodes.constEnd())
			continue;
		const double z = nit.value().z;
		if (!std::isfinite(z))
			continue;
		active.append(ActiveNode{z, p});
		sum += p;
		if (!haveContactZ) {
			zMinContact = zMaxContact = z;
			haveContactZ = true;
		} else {
			zMinContact = std::min(zMinContact, z);
			zMaxContact = std::max(zMaxContact, z);
		}
	}

	m.cpressActiveNodes = active.size();
	if (active.isEmpty() || sum <= 0.0)
		return m;

	m.cpressMean_MPa = sum / static_cast<double>(active.size());
	double sq = 0.0;
	for (const ActiveNode& n : active) {
		const double d = n.p - m.cpressMean_MPa;
		sq += d * d;
	}
	m.cpressStd_MPa = std::sqrt(sq / static_cast<double>(active.size()));
	m.cpressCV = m.cpressMean_MPa > 0.0 ? m.cpressStd_MPa / m.cpressMean_MPa : -1.0;
	m.contactWidth_mm = zMaxContact - zMinContact;

	const double widthAll = zMaxAll - zMinAll;
	QVector<double> binSum(m.cpressBinCount, 0.0);
	QVector<int>    binN(m.cpressBinCount, 0);
	QVector<double> binMax(m.cpressBinCount, -1.0);
	for (int i = 0; i < m.cpressBinCount; ++i) {
		const double a = zMinAll + widthAll * static_cast<double>(i) / m.cpressBinCount;
		const double b = zMinAll + widthAll * static_cast<double>(i + 1) / m.cpressBinCount;
		m.bins[i].zStart = a;
		m.bins[i].zEnd = b;
	}

	for (const ActiveNode& n : active) {
		int idx = static_cast<int>(std::floor((n.z - zMinAll) / widthAll * m.cpressBinCount));
		idx = std::max(0, std::min(m.cpressBinCount - 1, idx));
		binSum[idx] += n.p;
		++binN[idx];
		if (binMax[idx] < 0.0 || n.p > binMax[idx])
			binMax[idx] = n.p;
	}

	for (int i = 0; i < m.cpressBinCount; ++i) {
		m.bins[i].nodeCount = binN[i];
		m.bins[i].meanCPRESS = binN[i] > 0 ? binSum[i] / static_cast<double>(binN[i]) : -1.0;
		m.bins[i].maxCPRESS = binMax[i];
	}

	double edgeSum = 0.0;
	int    edgeN = 0;
	double centerSum = 0.0;
	int    centerN = 0;
	if (m.cpressBinCount == 11) {
		for (int i : QVector<int>{0, 1, 9, 10}) {
			edgeSum += binSum[i];
			edgeN += binN[i];
		}
		for (int i : QVector<int>{4, 5, 6}) {
			centerSum += binSum[i];
			centerN += binN[i];
		}
	} else {
		const double edgeLimit = 0.20 * widthAll;
		const double centerHalf = 0.15 * widthAll;
		const double zMid = 0.5 * (zMinAll + zMaxAll);
		for (const ActiveNode& n : active) {
			const bool inEdge = (n.z - zMinAll) <= edgeLimit || (zMaxAll - n.z) <= edgeLimit;
			const bool inCenter = std::abs(n.z - zMid) <= centerHalf;
			if (inEdge) {
				edgeSum += n.p;
				++edgeN;
			}
			if (inCenter) {
				centerSum += n.p;
				++centerN;
			}
		}
	}

	m.cpressEdgeMean_MPa = edgeN > 0 ? edgeSum / static_cast<double>(edgeN) : -1.0;
	m.cpressCenterMean_MPa = centerN > 0 ? centerSum / static_cast<double>(centerN) : -1.0;
	m.edgeLoadRatio = m.cpressCenterMean_MPa > 0.0
	                      ? m.cpressEdgeMean_MPa / m.cpressCenterMean_MPa
	                      : -1.0;
	return m;
}

QVector<double> parseFrdFloatFields(const QString& input)
{
	return parseFrdFloatFieldsImpl(input);
}

double maxVonMisesForNodes(const FrdResult& frd, const QSet<int>& nodeIds, int* outNodeId)
{
	if (frd.stresses.isEmpty()) {
		if (outNodeId)
			*outNodeId = -1;
		return -1.0;
	}
	double bestVm = -1.0;
	int    bestId = -1;
	for (auto it = frd.stresses.constBegin(); it != frd.stresses.constEnd(); ++it) {
		if (!nodeIds.isEmpty() && !nodeIds.contains(it.key()))
			continue;
		const double vm = it.value().vonMises();
		if (!std::isfinite(vm))
			continue;
		if (vm > bestVm) {
			bestVm = vm;
			bestId = it.key();
		}
	}
	if (outNodeId)
		*outNodeId = bestId;
	return bestVm;
}

double maxDisplMagnitudeForNodes(const FrdResult& frd, const QSet<int>& nodeIds, int* outNodeId)
{
	if (nodeIds.isEmpty() || frd.displacements.isEmpty()) {
		if (outNodeId)
			*outNodeId = -1;
		return -1.0;
	}
	double bestU  = -1.0;
	int    bestId = -1;
	for (int nid : nodeIds) {
		const auto it = frd.displacements.constFind(nid);
		if (it == frd.displacements.constEnd())
			continue;
		const NodeDisp& u   = it.value();
		const double    mag = u.magnitude();
		if (!std::isfinite(mag) || mag > 100.0) {
			qWarning().noquote()
			    << QStringLiteral("[Umax][skip abnormal] node=%1 u=(%2,%3,%4) mag=%5")
			           .arg(nid)
			           .arg(u.ux, 0, 'g', 10)
			           .arg(u.uy, 0, 'g', 10)
			           .arg(u.uz, 0, 'g', 10)
			           .arg(mag, 0, 'g', 10);
			continue;
		}
		if (mag > bestU) {
			bestU  = mag;
			bestId = nid;
		}
	}
	if (outNodeId)
		*outNodeId = bestId;
	return bestU;
}

double maxVonMisesFromDatElsets(const QString& datPath,
                              const QVector<QString>& elsetNames,
                              int* outSampleCount)
{
	double maxVm  = -1.0;
	int    samples = 0;
	for (const QString& el : elsetNames) {
		const DatStressResult r = parseDat(datPath, el);
		if (!r.found())
			continue;
		samples += r.sampleCount;
		if (r.maxVonMises > maxVm)
			maxVm = r.maxVonMises;
	}
	if (outSampleCount)
		*outSampleCount = samples;
	return maxVm;
}

} // namespace GearAutoOpt
