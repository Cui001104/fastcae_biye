#include "CCXResultParser.h"

#include <QFile>
#include <QRegExp>
#include <QRegularExpression>
#include <QRegularExpressionMatchIterator>
#include <QString>
#include <QStringList>
#include <QTextStream>

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
bool isStressSectionHeader(const QString& line, const QString& targetSet) {
	const QString lower = line.toLower();
	if (!lower.contains(QLatin1String("stresses"))) return false;
	if (!lower.contains(QLatin1String("for set"))) return false;

	// 抽 "for set <NAME>"
	const int idx = lower.indexOf(QLatin1String("for set"));
	if (idx < 0) return false;
	int begin = idx + 7;  // "for set" 长度 = 7
	while (begin < lower.size() && lower[begin].isSpace()) ++begin;
	int end = begin;
	while (end < lower.size() && !lower[end].isSpace()) ++end;
	const QString name = lower.mid(begin, end - begin);
	return name == targetSet.toLower();
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

} // anonymous namespace

QVector<StressSample> parseDatSamples(const QString& path, const QString& elsetName) {
	QVector<StressSample> samples;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return samples;
	QTextStream ts(&f);
	ts.setCodec("UTF-8");

	bool inSection = false;
	while (!ts.atEnd()) {
		const QString line = ts.readLine();
		const QString trimmed = line.trimmed();

		if (isStressSectionHeader(line, elsetName)) {
			inSection = true;
			continue;
		}

		if (!inSection) continue;

		// 段头与数据之间、段内偶发的空行 — 保持在段内
		if (trimmed.isEmpty()) continue;
		// 新段头（关键字行，非数字开头）→ 段结束
		if (!trimmed[0].isDigit() && trimmed[0] != '-') {
			inSection = false;
			continue;
		}

		StressSample s;
		if (parseRow(line, &s)) samples.push_back(s);
	}
	return samples;
}

DatStressResult parseDat(const QString& path, const QString& elsetName) {
	DatStressResult r;
	r.elsetName = elsetName;

	const QVector<StressSample> samples = parseDatSamples(path, elsetName);
	r.sampleCount = samples.size();
	for (const StressSample& s : samples) {
		const double vm = s.vonMises();
		if (vm > r.maxVonMises) {
			r.maxVonMises = vm;
			r.maxElemId   = s.elemId;
			r.maxIntegPt  = s.integPt;
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
	double best = -1.0;
	int    bestId = -1;
	for (auto it = stresses.constBegin(); it != stresses.constEnd(); ++it) {
		const double vm = it.value().vonMises();
		if (vm > best) { best = vm; bestId = it.key(); }
	}
	if (outNodeId) *outNodeId = bestId;
	return best;
}

double FrdResult::maxDisplMagnitude(int* outNodeId) const {
	double best = -1.0;
	int    bestId = -1;
	for (auto it = displacements.constBegin(); it != displacements.constEnd(); ++it) {
		const double m = it.value().magnitude();
		if (m > best) { best = m; bestId = it.key(); }
	}
	if (outNodeId) *outNodeId = bestId;
	return best;
}

namespace {

// 抽出 .frd 数据行 (` -1 <id> <vals...>`) 中的数值。CCX 输出每个值用 12 或 13
// 字符宽（负号偷一位），相邻值之间无空格。直接 split 不可靠，用正则。
QVector<double> extractFrdValues(const QString& line, int startCol) {
	QVector<double> vals;
	if (startCol >= line.size()) return vals;
	const QString tail = line.mid(startCol);
	static QRegularExpression rx(QStringLiteral(
		"[+-]?\\d+\\.\\d+[Ee][+-]\\d{2,3}"));
	auto it = rx.globalMatch(tail);
	while (it.hasNext()) {
		auto m = it.next();
		bool ok = false;
		const double v = m.captured(0).toDouble(&ok);
		if (ok) vals.push_back(v);
	}
	return vals;
}

// 在 ` -1 ID ...` 数据行里抽 ID（cols 3-12，10 字符宽）。失败返回 -1。
int extractNodeId(const QString& line) {
	if (line.size() < 13) return -1;
	bool ok = false;
	const int id = line.mid(3, 10).trimmed().toInt(&ok);
	return ok ? id : -1;
}

// 一个 frd 数据块的描述：DISP（3 分量）/ STRESS（6 分量）/ NODE（3 分量但来自 2C 块）
enum class FrdBlock {
	None,
	Node,        // 2C 节点坐标块
	Disp,        // -4 DISP 节点位移
	Stress,      // -4 STRESS 节点应力
	OtherNodal,  // 其它节点结果块（如 ERROR/FORC），跳过
	Element,     // 3C 单元块，跳过
};

} // anonymous namespace

FrdResult parseFrd(const QString& path) {
	FrdResult result;

	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return result;
	QTextStream ts(&f);
	ts.setCodec("UTF-8");

	FrdBlock current = FrdBlock::None;

	while (!ts.atEnd()) {
		const QString line = ts.readLine();
		const QString trimmed = line.trimmed();

		// 文件结束标记
		if (trimmed == QLatin1String("9999")) break;
		if (trimmed.isEmpty()) continue;

		// 块结束标记
		if (trimmed.startsWith(QLatin1String("-3"))) {
			current = FrdBlock::None;
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
			// 暂时停在 None；具体 DISP/STRESS 由下面的 "-4" 行决定
			current = FrdBlock::OtherNodal;
			continue;
		}
		// "-4 BLOCK_NAME ..." 设置当前块类型
		if (trimmed.startsWith(QLatin1String("-4"))) {
			const QStringList toks = trimmed.split(QRegExp("\\s+"),
			                                       QString::SkipEmptyParts);
			if (toks.size() >= 2) {
				const QString name = toks[1].toUpper();
				if (name == QLatin1String("DISP"))
					current = FrdBlock::Disp;
				else if (name == QLatin1String("STRESS"))
					current = FrdBlock::Stress;
				else
					current = FrdBlock::OtherNodal;
			}
			continue;
		}
		// "-5 ..." 是分量声明，忽略
		if (trimmed.startsWith(QLatin1String("-5"))) continue;

		// 数据行：以 " -1" 开头
		if (!line.startsWith(QStringLiteral(" -1"))) continue;

		const int nid = extractNodeId(line);
		if (nid < 0) continue;
		const QVector<double> vals = extractFrdValues(line, 13);

		switch (current) {
		case FrdBlock::Node:
			if (vals.size() >= 3) {
				NodeXYZ p{vals[0], vals[1], vals[2]};
				result.nodes.insert(nid, p);
			}
			break;
		case FrdBlock::Disp:
			if (vals.size() >= 3) {
				NodeDisp d{vals[0], vals[1], vals[2]};
				result.displacements.insert(nid, d);
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

	return result;
}

} // namespace GearAutoOpt
