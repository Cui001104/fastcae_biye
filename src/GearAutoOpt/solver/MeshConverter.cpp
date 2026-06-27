#include "MeshConverter.h"
#include "GearAutoOpt/data/GearOptLog.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QList>
#include <QMap>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QVector>

#include <QDebug>

#include <algorithm>
#include <cmath>

namespace GearAutoOpt {

namespace {

// 判断是否为 surface element 卡片（被剥离的目标）：CPS / STRI / S3 / S4 / S6 / S8。
bool isSurfaceElementHeader(const QString& upperLineNoSpace) {
	if (!upperLineNoSpace.startsWith(QLatin1String("*ELEMENT"))) return false;
	return upperLineNoSpace.contains(QLatin1String("TYPE=CPS"))
		|| upperLineNoSpace.contains(QLatin1String("TYPE=STRI"))
		|| upperLineNoSpace.contains(QLatin1String("TYPE=S3"))
		|| upperLineNoSpace.contains(QLatin1String("TYPE=S4"))
		|| upperLineNoSpace.contains(QLatin1String("TYPE=S6"))
		|| upperLineNoSpace.contains(QLatin1String("TYPE=S8"));
}

// 抽取 *ELEMENT,...,ELSET=NAME 中的 NAME；找不到返回空串。
QString extractElsetName(const QString& upperLineNoSpace) {
	const int idx = upperLineNoSpace.indexOf(QLatin1String("ELSET="));
	if (idx < 0) return QString();
	int begin = idx + 6;
	int end = upperLineNoSpace.indexOf(',', begin);
	if (end < 0) end = upperLineNoSpace.length();
	return upperLineNoSpace.mid(begin, end - begin);
}

// *NSET 或 *ELSET 头部抽取 NAME（NSET= / ELSET=）。
QString extractSetName(const QString& upperLineNoSpace, const QString& key) {
	const int idx = upperLineNoSpace.indexOf(key);  // "NSET=" 或 "ELSET="
	if (idx < 0) return QString();
	int begin = idx + key.length();
	int end = upperLineNoSpace.indexOf(',', begin);
	if (end < 0) end = upperLineNoSpace.length();
	return upperLineNoSpace.mid(begin, end - begin);
}

// 安全文件名：用 set 名构造文件名时把非字母数字下划线换成 _。
QString safeFileBase(const QString& name) {
	QString s;
	for (QChar c : name) {
		if (c.isLetterOrNumber() || c == '_') s += c;
		else s += '_';
	}
	return s;
}

// 读全文件为行 list（保留换行符以便原样写回）。
QStringList readLinesKeepEol(const QString& path, bool* okOut = nullptr) {
	QStringList lines;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
		if (okOut) *okOut = false;
		return lines;
	}
	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	while (!ts.atEnd()) {
		// QTextStream::readLine 丢 \n，我们手动加回保持行为一致
		QString line = ts.readLine();
		if (!ts.atEnd() || !line.isEmpty()) line += '\n';
		lines << line;
	}
	if (okOut) *okOut = true;
	return lines;
}

// 解析 inp 中下一段非 * 开头的「数据行」，把每行第一个 token 作为 ID 收集进 ids。
// 返回扫到的下一个卡片行索引（未找到则 lines.size()）。
int collectIdColumn(const QStringList& lines, int startIdx, QSet<int>* ids) {
	int i = startIdx;
	while (i < lines.size()) {
		const QString& raw = lines[i];
		const QString trimmed = raw.trimmed();
		if (trimmed.isEmpty()) { ++i; continue; }
		if (trimmed.startsWith('*')) break;  // 下一卡片
		// 行格式："id, ..."  以逗号或空白拆分
		const QStringList toks = trimmed.split(QRegExp("[,\\s]+"),
		                                       QString::SkipEmptyParts);
		for (const QString& t : toks) {
			bool ok = false;
			int v = t.toInt(&ok);
			if (ok) ids->insert(v);
		}
		++i;
	}
	return i;
}

// 解析 *ELEMENT 块内出现的所有节点 id（每行第二列起）。
int collectElementNodeIds(const QStringList& lines, int startIdx, QSet<int>* nodeIds) {
	int i = startIdx;
	while (i < lines.size()) {
		const QString trimmed = lines[i].trimmed();
		if (trimmed.isEmpty()) { ++i; continue; }
		if (trimmed.startsWith('*')) break;
		const QStringList toks = trimmed.split(QRegExp("[,\\s]+"),
		                                       QString::SkipEmptyParts);
		// 第 0 列是 element id；后面都是节点 id
		for (int k = 1; k < toks.size(); ++k) {
			bool ok = false;
			int v = toks[k].toInt(&ok);
			if (ok) nodeIds->insert(v);
		}
		++i;
	}
	return i;
}

bool writeFile(const QString& path, const QString& content) {
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
		return false;
	}
	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	ts << content;
	return f.error() == QFile::NoError;
}

} // anonymous namespace

QString ConvertReport::toString() const {
	QString s;
	s += QString("nodes=%1, volume_elems=%2, stripped_surface_elems=%3\n")
		.arg(totalNodes).arg(totalVolumeElems).arg(totalSurfaceElems);
	s += QString("  nsets (%1): ").arg(nsetNames.size());
	for (const QString& n : nsetNames)
		s += QString("%1[%2] ").arg(n).arg(nsetNodeCount.value(n, 0));
	s += '\n';
	s += QString("  elsets (%1): ").arg(elsetNames.size());
	for (const QString& n : elsetNames)
		s += QString("%1[%2] ").arg(n).arg(elsetElemCount.value(n, 0));
	return s;
}

int stripSurfaceElements(const QString& inpPath) {
	bool ok = false;
	QStringList lines = readLinesKeepEol(inpPath, &ok);
	if (!ok) return -1;

	QStringList outLines;
	int removed = 0;
	int i = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
		if (isSurfaceElementHeader(upperNoSpace)) {
			++i;  // 跳过卡头
			while (i < lines.size() && !lines[i].trimmed().startsWith('*')) {
				++removed;
				++i;
			}
			continue;
		}
		outLines << lines[i];
		++i;
	}

	QString joined = outLines.join(QString());
	QFile f(inpPath);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
		return -1;
	}
	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	ts << joined;
	return removed;
}

bool loadNsetNodeIds(const QString& inpPath, const QString& nsetName, QSet<int>* outIds)
{
	if (!outIds)
		return false;
	outIds->clear();

	bool ok = false;
	const QStringList lines = readLinesKeepEol(inpPath, &ok);
	if (!ok)
		return false;
	const QString want = nsetName.toUpper();

	int i = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
		if (upperNoSpace.startsWith(QLatin1String("*NSET"))) {
			const QString name = extractSetName(upperNoSpace, QLatin1String("NSET="));
			if (name == want) {
				i = collectIdColumn(lines, i + 1, outIds);
				continue;
			}
		}
		++i;
	}
	return !outIds->isEmpty();
}

int countNsetNodes(const QString& inpPath, const QString& nsetName)
{
	QSet<int> ids;
	if (!loadNsetNodeIds(inpPath, nsetName, &ids))
		return -1;
	return ids.size();
}

int countSurfaceFaces(const QString& inpPath, const QString& surfaceName)
{
	bool ok = false;
	const QStringList lines = readLinesKeepEol(inpPath, &ok);
	if (!ok)
		return -1;
	const QString want = surfaceName.toUpper();

	int i = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
		if (upperNoSpace.startsWith(QLatin1String("*SURFACE"))) {
			const QString name = extractSetName(upperNoSpace, QLatin1String("NAME="));
			if (name == want) {
				int count = 0;
				++i;
				while (i < lines.size()) {
					const QString trimmed = lines[i].trimmed();
					if (trimmed.isEmpty()) {
						++i;
						continue;
					}
					if (trimmed.startsWith(QLatin1Char('*')))
						break;
					const QStringList toks =
					    trimmed.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
					if (toks.size() >= 2)
						++count;
					++i;
				}
				return count;
			}
		}
		++i;
	}
	return -1;
}

bool loadElsetNodeIds(const QString& inpPath, const QString& elsetName, QSet<int>* outIds)
{
	if (!outIds)
		return false;
	outIds->clear();

	bool ok = false;
	const QStringList lines = readLinesKeepEol(inpPath, &ok);
	if (!ok)
		return false;
	const QString want = elsetName.toUpper();

	QSet<int> elemIds;
	int       i = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
		if (upperNoSpace.startsWith(QLatin1String("*ELSET"))) {
			const QString name = extractSetName(upperNoSpace, QLatin1String("ELSET="));
			if (name == want) {
				i = collectIdColumn(lines, i + 1, &elemIds);
				continue;
			}
		}
		if (upperNoSpace.startsWith(QLatin1String("*ELEMENT"))) {
			if (extractElsetName(upperNoSpace) == want) {
				int j = i + 1;
				while (j < lines.size() && !lines[j].trimmed().startsWith('*')) {
					const QString trimmed = lines[j].trimmed();
					if (!trimmed.isEmpty()) {
						const QStringList toks =
						    trimmed.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
						if (!toks.isEmpty()) {
							bool ok2 = false;
							const int v = toks.first().toInt(&ok2);
							if (ok2)
								elemIds.insert(v);
						}
					}
					++j;
				}
				i = j;
				continue;
			}
		}
		++i;
	}
	if (elemIds.isEmpty())
		return false;

	i = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
		if (!upperNoSpace.startsWith(QLatin1String("*ELEMENT"))) {
			++i;
			continue;
		}
		int j = i + 1;
		while (j < lines.size() && !lines[j].trimmed().startsWith('*')) {
			const QString trimmed = lines[j].trimmed();
			if (!trimmed.isEmpty()) {
				const QStringList toks =
				    trimmed.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
				if (!toks.isEmpty()) {
					bool ok2 = false;
					const int eid = toks.first().toInt(&ok2);
					if (ok2 && elemIds.contains(eid)) {
						for (int k = 1; k < toks.size(); ++k) {
							bool ok3 = false;
							const int n = toks[k].toInt(&ok3);
							if (ok3)
								outIds->insert(n);
						}
					}
				}
			}
			++j;
		}
		i = j;
	}
	return !outIds->isEmpty();
}

int countElsetNodes(const QString& inpPath, const QString& elsetName) {
	QSet<int> nodeIds;
	if (!loadElsetNodeIds(inpPath, elsetName, &nodeIds))
		return -1;
	return nodeIds.size();
}

bool convertGmshInpToCcxInp(const QString& srcInp,
                             const QString& dstDir,
                             ConvertReport* report) {
	bool ok = false;
	QStringList lines = readLinesKeepEol(srcInp, &ok);
	if (!ok) return false;

	QDir d(dstDir);
	if (!d.exists()) return false;

	ConvertReport rpt;

	// 1. 写 mesh_clean.inp（剥离 surface element 后的整文件）
	QStringList clean;
	int i = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
		if (isSurfaceElementHeader(upperNoSpace)) {
			++i;
			while (i < lines.size() && !lines[i].trimmed().startsWith('*')) {
				++rpt.totalSurfaceElems;
				++i;
			}
			continue;
		}
		clean << lines[i];
		++i;
	}
	if (!writeFile(d.filePath(QStringLiteral("mesh_clean.inp")), clean.join(QString()))) {
		return false;
	}

	// 2. 单遍扫描收集：node 总数、各 *ELEMENT 块的 elset 名 + 元素数；各 *NSET / *ELSET 块
	QHash<QString, QSet<int>> nsetIds;
	QHash<QString, QSet<int>> elsetIds;

	i = 0;
	while (i < lines.size()) {
		const QString trimmed = lines[i].trimmed();
		const QString upperNoSpace = trimmed.toUpper().remove(' ');

		if (upperNoSpace.startsWith(QLatin1String("*NODE"))) {
			QSet<int> ids;
			i = collectIdColumn(lines, i + 1, &ids);
			rpt.totalNodes += ids.size();
			continue;
		}
		if (upperNoSpace.startsWith(QLatin1String("*ELEMENT"))) {
			const bool isSurface = isSurfaceElementHeader(upperNoSpace);
			const QString elsetName = extractElsetName(upperNoSpace);
			QSet<int> ids;
			i = collectIdColumn(lines, i + 1, &ids);
			if (!isSurface) rpt.totalVolumeElems += ids.size();
			if (!elsetName.isEmpty() && !isSurface) {
				elsetIds[elsetName].unite(ids);
			}
			continue;
		}
		if (upperNoSpace.startsWith(QLatin1String("*NSET"))) {
			const QString name = extractSetName(upperNoSpace, QLatin1String("NSET="));
			QSet<int> ids;
			i = collectIdColumn(lines, i + 1, &ids);
			if (!name.isEmpty()) nsetIds[name].unite(ids);
			continue;
		}
		if (upperNoSpace.startsWith(QLatin1String("*ELSET"))) {
			const QString name = extractSetName(upperNoSpace, QLatin1String("ELSET="));
			QSet<int> ids;
			i = collectIdColumn(lines, i + 1, &ids);
			if (!name.isEmpty()) elsetIds[name].unite(ids);
			continue;
		}
		++i;
	}

	// 3. 逐组写出 nset_<NAME>.inp / elset_<NAME>.inp
	auto writeIdSet = [&](const QString& fileBase, const QString& cardName,
	                      const QString& setName, const QSet<int>& ids) -> bool {
		QString body;
		body += QString("*%1, %1=%2\n").arg(cardName, setName);
		// 每行 8 个 id（CalculiX 习惯）
		QList<int> sorted = QList<int>::fromSet(ids);
		std::sort(sorted.begin(), sorted.end());
		QStringList row;
		for (int id : sorted) {
			row << QString::number(id);
			if (row.size() == 8) {
				body += row.join(", ") + '\n';
				row.clear();
			}
		}
		if (!row.isEmpty()) body += row.join(", ") + '\n';
		const QString path = d.filePath(QString("%1_%2.inp").arg(fileBase, safeFileBase(setName).toLower()));
		return writeFile(path, body);
	};

	for (auto it = nsetIds.constBegin(); it != nsetIds.constEnd(); ++it) {
		if (!writeIdSet("nset", "NSET", it.key(), it.value())) return false;
		rpt.nsetNames << it.key();
		rpt.nsetNodeCount[it.key()] = it.value().size();
	}
	std::sort(rpt.nsetNames.begin(), rpt.nsetNames.end());

	for (auto it = elsetIds.constBegin(); it != elsetIds.constEnd(); ++it) {
		if (!writeIdSet("elset", "ELSET", it.key(), it.value())) return false;
		rpt.elsetNames << it.key();
		rpt.elsetElemCount[it.key()] = it.value().size();
	}
	std::sort(rpt.elsetNames.begin(), rpt.elsetNames.end());

	if (report) *report = rpt;
	return true;
}

static bool mcMeshInpLineIsSurfaceElementHeader(const QString& upperLineNoSpace) {
	if (!upperLineNoSpace.startsWith(QLatin1String("*ELEMENT"))) return false;
	return upperLineNoSpace.contains(QLatin1String("TYPE=CPS"))
	    || upperLineNoSpace.contains(QLatin1String("TYPE=STRI"))
	    || upperLineNoSpace.contains(QLatin1String("TYPE=S3"))
	    || upperLineNoSpace.contains(QLatin1String("TYPE=S4"))
	    || upperLineNoSpace.contains(QLatin1String("TYPE=S6"))
	    || upperLineNoSpace.contains(QLatin1String("TYPE=S8"));
}

static QString mcExtractElementTypeToken(const QString& upperNoSpace) {
	const int idx = upperNoSpace.indexOf(QLatin1String("TYPE="));
	if (idx < 0) return QString();
	int j = idx + 5;
	while (j < upperNoSpace.size() && (upperNoSpace[j] == QLatin1Char('=') || upperNoSpace[j].isSpace()))
		++j;
	int k = j;
	while (k < upperNoSpace.size() && upperNoSpace[k] != QLatin1Char(','))
		++k;
	return upperNoSpace.mid(j, k - j);
}

// Abaqus/CalculiX C3D4 局部面角点（0-based 单元内节点索引）
// S1=1,2,3  S2=1,4,2  S3=2,4,3  S4=3,4,1
static const int kC3D4LocalFaces[4][3] = {
    {0, 1, 2},
    {0, 3, 1},
    {1, 3, 2},
    {2, 3, 0},
};

static QString mcCanonicalFaceKey(QVector<int> cornerNodeIds) {
	std::sort(cornerNodeIds.begin(), cornerNodeIds.end());
	QString key;
	key.reserve(static_cast<int>(cornerNodeIds.size()) * 12 + 4);
	for (int v : cornerNodeIds) {
		key += QLatin1Char('_');
		key += QString::number(v);
	}
	return key;
}

static void mcRegisterFace(const QVector<int>& cornerNodeIds,
                           QHash<QString, int>* faceCount,
                           QHash<QString, QVector<int>>* firstFaceNodes) {
	const QString key = mcCanonicalFaceKey(cornerNodeIds);
	(*faceCount)[key] += 1;
	if (!firstFaceNodes->contains(key))
		firstFaceNodes->insert(key, cornerNodeIds);
}

static void mcExpandSolidFaces(const QString& typeToken,
                               const QVector<int>& nn,
                               QHash<QString, int>* faceCount,
                               QHash<QString, QVector<int>>* firstFaceNodes) {
	if (typeToken == QLatin1String("C3D4") && nn.size() >= 4) {
		for (int fi = 0; fi < 4; ++fi) {
			QVector<int> c = {nn[kC3D4LocalFaces[fi][0]], nn[kC3D4LocalFaces[fi][1]], nn[kC3D4LocalFaces[fi][2]]};
			mcRegisterFace(c, faceCount, firstFaceNodes);
		}
		return;
	}
	if (typeToken == QLatin1String("C3D8") && nn.size() >= 8) {
		const int f[6][4] = {
		    {0, 1, 2, 3}, {4, 7, 6, 5}, {0, 4, 5, 1}, {1, 5, 6, 2}, {2, 6, 7, 3}, {0, 3, 7, 4}};
		for (int fi = 0; fi < 6; ++fi) {
			QVector<int> c = {nn[f[fi][0]], nn[f[fi][1]], nn[f[fi][2]], nn[f[fi][3]]};
			mcRegisterFace(c, faceCount, firstFaceNodes);
		}
		return;
	}
	if (typeToken == QLatin1String("C3D6") && nn.size() >= 6) {
		mcRegisterFace(QVector<int>{nn[0], nn[1], nn[2]}, faceCount, firstFaceNodes);
		mcRegisterFace(QVector<int>{nn[3], nn[5], nn[4]}, faceCount, firstFaceNodes);
		mcRegisterFace(QVector<int>{nn[0], nn[3], nn[4], nn[1]}, faceCount, firstFaceNodes);
		mcRegisterFace(QVector<int>{nn[1], nn[4], nn[5], nn[2]}, faceCount, firstFaceNodes);
		mcRegisterFace(QVector<int>{nn[0], nn[2], nn[5], nn[3]}, faceCount, firstFaceNodes);
	}
}

static QVector<GearNodeXYZ> gearUniqueFaceCorners(const QVector<int>& face,
                                                  const QMap<int, GearNodeXYZ>& nodePos) {
	QVector<GearNodeXYZ> pts;
	QSet<int>            seen;
	for (int nid : face) {
		if (nid <= 0 || seen.contains(nid))
			continue;
		const auto it = nodePos.constFind(nid);
		if (it == nodePos.constEnd())
			continue;
		seen.insert(nid);
		pts.append(it.value());
	}
	return pts;
}

static bool gearFaceCentroidAndNormal(const QVector<GearNodeXYZ>& pts, double* mx, double* my, double* mz,
                                      double* nx, double* ny, double* nz) {
	if (pts.size() < 3)
		return false;
	double sx = 0, sy = 0, sz = 0;
	for (const GearNodeXYZ& p : pts) {
		sx += p.x;
		sy += p.y;
		sz += p.z;
	}
	const double inv = 1.0 / static_cast<double>(pts.size());
	*mx = sx * inv;
	*my = sy * inv;
	*mz = sz * inv;
	const GearNodeXYZ& a = pts[0];
	const GearNodeXYZ& b = pts[1];
	const GearNodeXYZ& c = pts[2];
	const double        v1x = b.x - a.x, v1y = b.y - a.y, v1z = b.z - a.z;
	const double        v2x = c.x - a.x, v2y = c.y - a.y, v2z = c.z - a.z;
	const double        cx = v1y * v2z - v1z * v2y;
	const double        cy = v1z * v2x - v1x * v2z;
	const double        cz = v1x * v2y - v1y * v2x;
	const double        clen = std::sqrt(cx * cx + cy * cy + cz * cz);
	if (clen < 1e-18)
		return false;
	*nx = cx / clen;
	*ny = cy / clen;
	*nz = cz / clen;
	return true;
}

static bool gearFaceIsEndCap(const QVector<GearNodeXYZ>& pts, const GearToothPickContext& ctx, double zTol,
                             double mx, double my, double mz, double nzAbs) {
	if (pts.isEmpty())
		return true;

	double fzMin = 1e100;
	double fzMax = -1e100;
	for (const GearNodeXYZ& p : pts) {
		fzMin = std::min(fzMin, p.z);
		fzMax = std::max(fzMax, p.z);
	}

	const bool onBottomFace = std::fabs(fzMin - ctx.zMin) < zTol && std::fabs(fzMax - ctx.zMin) < zTol;
	const bool onTopFace    = std::fabs(fzMin - ctx.zMax) < zTol && std::fabs(fzMax - ctx.zMax) < zTol;
	if (onBottomFace || onTopFace)
		return true;

	if (nzAbs >= 0.65 && (mz <= ctx.zMin + zTol || mz >= ctx.zMax - zTol))
		return true;

	return false;
}

static double gearRadialLo(const GearToothPickContext& ctx, GearToothFacePickMode mode) {
	switch (mode) {
	case GearToothFacePickMode::ToothBand:
		return std::max(ctx.hubR * 1.02, ctx.rootR - 0.08 * ctx.module);
	case GearToothFacePickMode::RelaxedRadial:
		return std::max(ctx.hubR * 1.02, ctx.rootR - 0.38 * ctx.module);
	case GearToothFacePickMode::ExteriorShell:
		return ctx.hubR * 1.01;
	}
	return ctx.hubR * 1.01;
}

static double gearNormalizeAngle(double a) {
	while (a > M_PI)
		a -= 2.0 * M_PI;
	while (a < -M_PI)
		a += 2.0 * M_PI;
	return a;
}

static double gearTargetAngleRad(const GearToothPickContext& ctx) {
	if (ctx.targetAngleRad != 0.0)
		return ctx.targetAngleRad;
	return 0.5 * M_PI;
}

static bool gearFaceInLoadToothAngleWindow(double mx, double my, const GearToothPickContext& ctx) {
	if (ctx.teethCount < 1)
		return true;

	const double toothPitchAngle = 2.0 * M_PI / static_cast<double>(ctx.teethCount);
	const double halfWindow      = 0.5 * ctx.loadToothCount * toothPitchAngle;
	const double targetAngle     = gearTargetAngleRad(ctx);
	const double faceAngle       = std::atan2(my - ctx.axisY, mx - ctx.axisX);
	const double dAngle          = gearNormalizeAngle(faceAngle - targetAngle);
	return std::fabs(dAngle) <= halfWindow;
}

static int gearComputeCenterToothIndex(const GearToothPickContext& ctx) {
	if (ctx.teethCount < 1)
		return 0;
	const double toothPitchAngle = 2.0 * M_PI / static_cast<double>(ctx.teethCount);
	double       targetAngle     = gearTargetAngleRad(ctx);
	while (targetAngle < 0.0)
		targetAngle += 2.0 * M_PI;
	while (targetAngle >= 2.0 * M_PI)
		targetAngle -= 2.0 * M_PI;
	int idx = static_cast<int>(std::lround(targetAngle / toothPitchAngle)) % ctx.teethCount;
	if (idx < 0)
		idx += ctx.teethCount;
	return idx;
}

static int gearToothIndexRelativeToCenter(double mx, double my, const GearToothPickContext& ctx) {
	if (ctx.teethCount < 1)
		return 0;
	const double toothPitchAngle = 2.0 * M_PI / static_cast<double>(ctx.teethCount);
	const double targetAngle     = gearTargetAngleRad(ctx);
	const double faceAngle       = std::atan2(my - ctx.axisY, mx - ctx.axisX);
	const double dAngle            = gearNormalizeAngle(faceAngle - targetAngle);
	return static_cast<int>(std::lround(dAngle / toothPitchAngle));
}

static QList<int> gearCoveredToothIndices(int centerToothIndex, int halfSpan, int teethCount) {
	QList<int> out;
	if (teethCount < 1)
		return out;
	for (int d = -halfSpan; d <= halfSpan; ++d) {
		int t = (centerToothIndex + d) % teethCount;
		if (t < 0)
			t += teethCount;
		out.append(t);
	}
	std::sort(out.begin(), out.end());
	out.erase(std::unique(out.begin(), out.end()), out.end());
	return out;
}

static QString gearFormatToothRangeText(int centerToothIndex, int halfSpan, int teethCount) {
	const QList<int> teeth = gearCoveredToothIndices(centerToothIndex, halfSpan, teethCount);
	if (teeth.isEmpty())
		return QStringLiteral("(empty)");
	QStringList parts;
	for (int t : teeth)
		parts << QString::number(t);
	return QStringLiteral("center=%1 span=±%2 indices=[%3]")
	    .arg(centerToothIndex)
	    .arg(halfSpan)
	    .arg(parts.join(QLatin1Char(',')));
}

static bool gearFaceInLocalContactToothRange(double mx,
                                             double my,
                                             const GearToothPickContext& ctx,
                                             int* outRelativeIndex = nullptr,
                                             bool* outIsHalfTooth   = nullptr) {
	if (!ctx.localContactSurface || ctx.teethCount < 1)
		return true;

	const int halfSpan = std::max(0, ctx.contactToothHalfSpan);
	const int rel      = gearToothIndexRelativeToCenter(mx, my, ctx);
	if (outRelativeIndex)
		*outRelativeIndex = rel;
	if (std::abs(rel) > halfSpan) {
		if (outIsHalfTooth)
			*outIsHalfTooth = false;
		return false;
	}

	if (ctx.useHalfToothTransition && (rel == -halfSpan || rel == halfSpan)) {
		const double toothPitchAngle = 2.0 * M_PI / static_cast<double>(ctx.teethCount);
		const double targetAngle     = gearTargetAngleRad(ctx);
		const double faceAngle       = std::atan2(my - ctx.axisY, mx - ctx.axisX);
		const double dAngle          = gearNormalizeAngle(faceAngle - targetAngle);
		const double toothCenterAngle = static_cast<double>(rel) * toothPitchAngle;
		const double offsetInTooth    = gearNormalizeAngle(dAngle - toothCenterAngle);

		if (rel == -halfSpan) {
			if (offsetInTooth < -0.5 * toothPitchAngle || offsetInTooth > 0.0) {
				if (outIsHalfTooth)
					*outIsHalfTooth = false;
				return false;
			}
		} else {
			if (offsetInTooth > 0.5 * toothPitchAngle || offsetInTooth < 0.0) {
				if (outIsHalfTooth)
					*outIsHalfTooth = false;
				return false;
			}
		}
		if (outIsHalfTooth)
			*outIsHalfTooth = true;
		return true;
	}

	if (outIsHalfTooth)
		*outIsHalfTooth = false;
	return true;
}

static double gearRadialHi(const GearToothPickContext& ctx, GearToothFacePickMode mode) {
	switch (mode) {
	case GearToothFacePickMode::ToothBand:
		return ctx.tipR * 1.05;
	case GearToothFacePickMode::RelaxedRadial:
		return ctx.tipR * 1.12;
	case GearToothFacePickMode::ExteriorShell:
		return ctx.tipR * 1.60;
	}
	return ctx.tipR * 1.60;
}

/// 从 KEY 同源的 solid 外边界面筛齿面/外圆周侧面；排除上下端面、孔柱、轮毂接触面。
static void collectToothNodesFromOuterContourFaces(const QVector<QVector<int>>& boundaryFaces,
                                                    const QMap<int, GearNodeXYZ>& nodePos,
                                                    const QSet<int>& hubIds, const GearToothPickContext& ctx,
                                                    GearToothFacePickMode mode, QSet<int>* acc) {
	if (!acc)
		return;
	acc->clear();

	const double zSpan = std::max(1e-9, ctx.zMax - ctx.zMin);
	const double zTol  = std::max(1e-6, 0.02 * zSpan);
	const double rLo   = gearRadialLo(ctx, mode);
	const double rHi   = gearRadialHi(ctx, mode);
	const double boreR = ctx.hubR + ctx.boreTol;

	for (QVector<int> face : boundaryFaces) {
		if (face.size() == 3)
			face.append(face.last());
		if (face.size() != 4)
			continue;

		bool touchesHub = false;
		for (int nid : face) {
			if (hubIds.contains(nid)) {
				touchesHub = true;
				break;
			}
		}
		if (touchesHub)
			continue;

		const QVector<GearNodeXYZ> pts = gearUniqueFaceCorners(face, nodePos);
		if (pts.size() < 3)
			continue;

		double mx = 0, my = 0, mz = 0, nx = 0, ny = 0, nz = 0;
		if (!gearFaceCentroidAndNormal(pts, &mx, &my, &mz, &nx, &ny, &nz))
			continue;

		const double rC    = std::hypot(mx - ctx.axisX, my - ctx.axisY);
		const double nzAbs = std::fabs(nz);

		if (gearFaceIsEndCap(pts, ctx, zTol, mx, my, mz, nzAbs))
			continue;

		if (rC <= boreR)
			continue;

		if (mode != GearToothFacePickMode::ExteriorShell) {
			if (rC < rLo || rC > rHi)
				continue;
		}

		if (!gearFaceInLoadToothAngleWindow(mx, my, ctx))
			continue;

		for (int nid : face) {
			if (nid > 0 && nodePos.contains(nid))
				acc->insert(nid);
		}
	}
}

static bool collectBoundaryFacesFromMeshInp(const QString& meshInpPath, QVector<QVector<int>>* outFaces,
                                            QString* diag) {
	auto fail = [&](const QString& msg) -> bool {
		if (diag)
			*diag = msg;
		if (outFaces)
			outFaces->clear();
		return false;
	};
	if (!outFaces)
		return fail(QStringLiteral("outFaces null"));
	outFaces->clear();

	QFile mf(meshInpPath);
	if (!mf.open(QIODevice::ReadOnly | QIODevice::Text))
		return fail(QStringLiteral("cannot open mesh.inp"));

	QStringList lines;
	{
		QTextStream mts(&mf);
		while (!mts.atEnd())
			lines << mts.readLine();
	}

	QHash<QString, int>          faceCount;
	QHash<QString, QVector<int>> firstFaceNodes;
	bool                         sawSupportedSolid = false;

	int i = 0;
	while (i < lines.size()) {
		const QString tline = lines[i].trimmed();
		if (tline.isEmpty() || tline.startsWith(QStringLiteral("**"))) {
			++i;
			continue;
		}
		QString upper = tline.toUpper();
		upper.remove(QLatin1Char(' '));
		if (!upper.startsWith(QLatin1String("*ELEMENT"))) {
			++i;
			continue;
		}
		if (mcMeshInpLineIsSurfaceElementHeader(upper)) {
			++i;
			continue;
		}

		const QString typeTok = mcExtractElementTypeToken(upper);
		if (typeTok != QLatin1String("C3D4") && typeTok != QLatin1String("C3D8")
		    && typeTok != QLatin1String("C3D6")) {
			++i;
			continue;
		}

		sawSupportedSolid = true;
		int j = i + 1;
		while (j < lines.size()) {
			const QString dat = lines[j].trimmed();
			if (dat.isEmpty()) {
				++j;
				continue;
			}
			if (dat.startsWith(QLatin1Char('*')))
				break;

			const QStringList toks = dat.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
			if (toks.size() < 2) {
				++j;
				continue;
			}
			QVector<int> nn;
			nn.reserve(toks.size() - 1);
			for (int k = 1; k < toks.size(); ++k) {
				bool ok = false;
				const int v = toks[k].toInt(&ok);
				if (ok)
					nn.append(v);
			}
			mcExpandSolidFaces(typeTok, nn, &faceCount, &firstFaceNodes);
			++j;
		}
		i = j;
	}

	if (!sawSupportedSolid)
		return fail(QStringLiteral("no C3D4/C3D8/C3D6 volume *ELEMENT block in mesh.inp"));

	for (auto it = faceCount.constBegin(); it != faceCount.constEnd(); ++it) {
		if (it.value() != 1)
			continue;
		const QVector<int> face = firstFaceNodes.value(it.key());
		if (!face.isEmpty())
			outFaces->append(face);
	}

	if (outFaces->isEmpty())
		return fail(QStringLiteral("no boundary faces (face key count==1)"));
	if (diag)
		diag->clear();
	return true;
}

static QString mcElsetFromElementHeader(const QString& upperNoSpace) {
	return extractElsetName(upperNoSpace);
}

static bool mcIsSupportedVolumeType(const QString& typeTok)
{
	return typeTok == QLatin1String("C3D4") || typeTok == QLatin1String("C3D8")
	       || typeTok == QLatin1String("C3D6") || typeTok == QLatin1String("C3D10");
}

static void mcCollectNodesForElementIds(const QStringList& lines,
                                        const QSet<int>&   elemIds,
                                        QSet<int>*         nodeIds)
{
	if (!nodeIds || elemIds.isEmpty())
		return;
	int i = 0;
	while (i < lines.size()) {
		const QString tline = lines[i].trimmed();
		if (tline.isEmpty() || tline.startsWith(QStringLiteral("**"))) {
			++i;
			continue;
		}
		QString upper = tline.toUpper();
		upper.remove(QLatin1Char(' '));
		if (!upper.startsWith(QLatin1String("*ELEMENT")) || mcMeshInpLineIsSurfaceElementHeader(upper)) {
			++i;
			continue;
		}
		const QString typeTok = mcExtractElementTypeToken(upper);
		if (!mcIsSupportedVolumeType(typeTok)) {
			++i;
			continue;
		}
		int j = i + 1;
		while (j < lines.size()) {
			const QString dat = lines[j].trimmed();
			if (dat.isEmpty()) {
				++j;
				continue;
			}
			if (dat.startsWith(QLatin1Char('*')))
				break;
			const QStringList toks = dat.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
			if (!toks.isEmpty()) {
				bool okE = false;
				const int eid = toks[0].toInt(&okE);
				if (okE && elemIds.contains(eid)) {
					for (int k = 1; k < toks.size(); ++k) {
						bool okN = false;
						const int nid = toks[k].toInt(&okN);
						if (okN)
							nodeIds->insert(nid);
					}
				}
			}
			++j;
		}
		i = j;
	}
}

static bool mcParseVolumeElsetElements(const QString& meshInpPath,
                                       const QString& elsetWant,
                                       QSet<int>* elemIds,
                                       QSet<int>* nodeIds) {
	if (!elemIds)
		return false;
	elemIds->clear();
	if (nodeIds)
		nodeIds->clear();

	bool ok = false;
	const QStringList lines = readLinesKeepEol(meshInpPath, &ok);
	if (!ok)
		return false;
	const QString want = elsetWant.toUpper();

	int i = 0;
	while (i < lines.size()) {
		const QString tline = lines[i].trimmed();
		if (tline.isEmpty() || tline.startsWith(QStringLiteral("**"))) {
			++i;
			continue;
		}
		QString upper = tline.toUpper();
		upper.remove(QLatin1Char(' '));
		if (!upper.startsWith(QLatin1String("*ELEMENT"))) {
			++i;
			continue;
		}
		if (mcMeshInpLineIsSurfaceElementHeader(upper)) {
			++i;
			continue;
		}
		const QString elset   = mcElsetFromElementHeader(upper);
		const QString typeTok = mcExtractElementTypeToken(upper);
		if (!mcIsSupportedVolumeType(typeTok)) {
			++i;
			continue;
		}
		if (elset.toUpper() != want) {
			++i;
			continue;
		}
		int j = i + 1;
		while (j < lines.size()) {
			const QString dat = lines[j].trimmed();
			if (dat.isEmpty()) {
				++j;
				continue;
			}
			if (dat.startsWith(QLatin1Char('*')))
				break;
			const QStringList toks = dat.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
			if (!toks.isEmpty()) {
				bool ok2 = false;
				const int eid = toks[0].toInt(&ok2);
				if (ok2)
					elemIds->insert(eid);
			}
			++j;
		}
		i = j;
	}

	// Gmsh 也可能只写 *ELSET, ELSET=GEAR1（单元号列表），*ELEMENT 上为 Volume1 等
	if (elemIds->isEmpty()) {
		i = 0;
		while (i < lines.size()) {
			const QString tline = lines[i].trimmed();
			QString       upper = tline.toUpper();
			upper.remove(QLatin1Char(' '));
			if (upper.startsWith(QLatin1String("*ELSET"))) {
				const QString name = extractSetName(upper, QLatin1String("ELSET="));
				if (name.toUpper() == want)
					i = collectIdColumn(lines, i + 1, elemIds);
				else
					++i;
				continue;
			}
			++i;
		}
	}

	if (elemIds->isEmpty())
		return false;

	if (nodeIds) {
		mcCollectNodesForElementIds(lines, *elemIds, nodeIds);
		// 若 *ELEMENT 块内联 ELSET 时未填 nodeIds，上面会补全
		if (nodeIds->isEmpty()) {
			i = 0;
			while (i < lines.size()) {
				const QString tline = lines[i].trimmed();
				if (tline.isEmpty() || tline.startsWith(QStringLiteral("**"))) {
					++i;
					continue;
				}
				QString upper = tline.toUpper();
				upper.remove(QLatin1Char(' '));
				if (!upper.startsWith(QLatin1String("*ELEMENT"))
				    || mcMeshInpLineIsSurfaceElementHeader(upper)) {
					++i;
					continue;
				}
				const QString elset   = mcElsetFromElementHeader(upper);
				const QString typeTok = mcExtractElementTypeToken(upper);
				if (!mcIsSupportedVolumeType(typeTok) || elset.toUpper() != want) {
					++i;
					continue;
				}
				int j = i + 1;
				while (j < lines.size()) {
					const QString dat = lines[j].trimmed();
					if (dat.isEmpty()) {
						++j;
						continue;
					}
					if (dat.startsWith(QLatin1Char('*')))
						break;
					const QStringList toks = dat.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
					if (!toks.isEmpty()) {
						bool ok2 = false;
						const int eid = toks[0].toInt(&ok2);
						if (ok2 && elemIds->contains(eid)) {
							for (int k = 1; k < toks.size(); ++k) {
								bool ok3 = false;
								const int nid = toks[k].toInt(&ok3);
								if (ok3)
									nodeIds->insert(nid);
							}
						}
					}
					++j;
				}
				i = j;
			}
		}
	}
	return !elemIds->isEmpty();
}

static bool collectBoundaryFacesForElset(const QString& meshInpPath,
                                         const QString& elsetName,
                                         QVector<QVector<int>>* outFaces,
                                         QString* diag) {
	auto fail = [&](const QString& msg) -> bool {
		if (diag)
			*diag = msg;
		if (outFaces)
			outFaces->clear();
		return false;
	};
	if (!outFaces)
		return fail(QStringLiteral("outFaces null"));
	outFaces->clear();

	QSet<int> elsetElems;
	if (!mcParseVolumeElsetElements(meshInpPath, elsetName, &elsetElems, nullptr))
		return fail(QStringLiteral("elset has no volume elements"));

	bool ok = false;
	const QStringList lines = readLinesKeepEol(meshInpPath, &ok);
	if (!ok)
		return fail(QStringLiteral("cannot read mesh.inp"));

	QHash<QString, int>          faceCount;
	QHash<QString, QVector<int>> firstFaceNodes;

	int i = 0;
	while (i < lines.size()) {
		const QString tline = lines[i].trimmed();
		if (tline.isEmpty() || tline.startsWith(QStringLiteral("**"))) {
			++i;
			continue;
		}
		QString upper = tline.toUpper();
		upper.remove(QLatin1Char(' '));
		if (!upper.startsWith(QLatin1String("*ELEMENT"))) {
			++i;
			continue;
		}
		if (mcMeshInpLineIsSurfaceElementHeader(upper)) {
			++i;
			continue;
		}
		const QString typeTok = mcExtractElementTypeToken(upper);
		if (!mcIsSupportedVolumeType(typeTok)) {
			++i;
			continue;
		}

		int j = i + 1;
		while (j < lines.size()) {
			const QString dat = lines[j].trimmed();
			if (dat.isEmpty()) {
				++j;
				continue;
			}
			if (dat.startsWith(QLatin1Char('*')))
				break;

			const QStringList toks = dat.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
			if (toks.size() < 2) {
				++j;
				continue;
			}
			bool okE = false;
			const int eid = toks[0].toInt(&okE);
			if (!okE || !elsetElems.contains(eid)) {
				++j;
				continue;
			}
			QVector<int> nn;
			for (int k = 1; k < toks.size(); ++k) {
				bool okN = false;
				const int v = toks[k].toInt(&okN);
				if (okN)
					nn.append(v);
			}
			mcExpandSolidFaces(typeTok, nn, &faceCount, &firstFaceNodes);
			++j;
		}
		i = j;
	}

	for (auto it = faceCount.constBegin(); it != faceCount.constEnd(); ++it) {
		if (it.value() != 1)
			continue;
		const QVector<int> face = firstFaceNodes.value(it.key());
		if (!face.isEmpty())
			outFaces->append(face);
	}

	if (outFaces->isEmpty())
		return fail(QStringLiteral("no elset boundary faces"));
	if (diag)
		diag->clear();
	return true;
}

struct McElemFaceRef {
	int     elemId   = 0;
	QString faceLabel;  ///< S1..S6
};

struct McElsetFaceRecord {
	int           elemId          = 0;
	QString       typeTok;
	QVector<int> faceNodes;
	int           localFaceIndex0 = -1;  ///< 生成阶段确定的局部面索引，写入时直接转 S1..S6
};

static int mcSolidLocalFaceCount(const QString& typeTok) {
	if (typeTok == QLatin1String("C3D4"))
		return 4;
	if (typeTok == QLatin1String("C3D8"))
		return 6;
	if (typeTok == QLatin1String("C3D6"))
		return 5;
	return 0;
}

static QVector<int> mcGetSolidLocalFaceNodes(const QString& typeTok, const QVector<int>& nn, int faceIndex0) {
	if (typeTok == QLatin1String("C3D4") && nn.size() >= 4) {
		if (faceIndex0 < 0 || faceIndex0 >= 4)
			return {};
		return {nn[kC3D4LocalFaces[faceIndex0][0]], nn[kC3D4LocalFaces[faceIndex0][1]],
		        nn[kC3D4LocalFaces[faceIndex0][2]]};
	}
	if (typeTok == QLatin1String("C3D8") && nn.size() >= 8) {
		const int f[6][4] = {
		    {0, 1, 2, 3}, {4, 7, 6, 5}, {0, 4, 5, 1}, {1, 5, 6, 2}, {2, 6, 7, 3}, {0, 3, 7, 4}};
		if (faceIndex0 < 0 || faceIndex0 >= 6)
			return {};
		return {nn[f[faceIndex0][0]], nn[f[faceIndex0][1]], nn[f[faceIndex0][2]], nn[f[faceIndex0][3]]};
	}
	if (typeTok == QLatin1String("C3D6") && nn.size() >= 6) {
		const QVector<QVector<int>> faces = {
		    {nn[0], nn[1], nn[2]},
		    {nn[3], nn[5], nn[4]},
		    {nn[0], nn[3], nn[4], nn[1]},
		    {nn[1], nn[4], nn[5], nn[2]},
		    {nn[0], nn[2], nn[5], nn[3]},
		};
		if (faceIndex0 < 0 || faceIndex0 >= faces.size())
			return {};
		return faces[faceIndex0];
	}
	return {};
}

static QString mcFaceLabelFromIndex0(int faceIndex0) {
	return QStringLiteral("S") + QString::number(faceIndex0 + 1);
}

static void mcRegisterSolidFace(int elemId, const QString& typeTok, int faceIndex0,
                                const QVector<int>& cornerNodeIds, const QSet<int>& elsetElems,
                                QHash<QString, int>* globalFaceCount,
                                QHash<QString, McElsetFaceRecord>* elsetFaceByKey) {
	const QString key = mcCanonicalFaceKey(cornerNodeIds);
	(*globalFaceCount)[key] += 1;
	if (elsetElems.contains(elemId) && !elsetFaceByKey->contains(key)) {
		McElsetFaceRecord rec;
		rec.elemId          = elemId;
		rec.typeTok         = typeTok;
		rec.faceNodes       = cornerNodeIds;
		rec.localFaceIndex0 = faceIndex0;
		elsetFaceByKey->insert(key, rec);
	}
}

static void mcExpandSolidFacesForSurfaceCollect(const QString& typeToken, int elemId, const QVector<int>& nn,
                                                const QSet<int>& elsetElems, QHash<QString, int>* globalFaceCount,
                                                QHash<QString, McElsetFaceRecord>* elsetFaceByKey) {
	const int nFace = mcSolidLocalFaceCount(typeToken);
	for (int fi = 0; fi < nFace; ++fi) {
		const QVector<int> c = mcGetSolidLocalFaceNodes(typeToken, nn, fi);
		if (c.size() >= 3)
			mcRegisterSolidFace(elemId, typeToken, fi, c, elsetElems, globalFaceCount, elsetFaceByKey);
	}
}

enum class SurfaceFilterMode {
	AbsoluteExterior  ///< faceCount==1：体单元绝对外表面，不含内部共享面
};

struct McFilteredSurfaceFaces {
	QList<McElemFaceRef> elemRefs;
	QList<QVector<int>>  faceNodeLists;  ///< debug：与 elemRefs 一一对应的角点节点（不经 S*）
};

static bool mcBuildSolidFaceMaps(const QString& meshInpPath,
                                 const QSet<int>& elsetElems,
                                 QHash<QString, int>* globalFaceCount,
                                 QHash<QString, McElsetFaceRecord>* elsetFaceByKey) {
	if (!globalFaceCount || !elsetFaceByKey)
		return false;
	globalFaceCount->clear();
	elsetFaceByKey->clear();

	bool ok = false;
	const QStringList lines = readLinesKeepEol(meshInpPath, &ok);
	if (!ok)
		return false;

	int i = 0;
	while (i < lines.size()) {
		const QString tline = lines[i].trimmed();
		if (tline.isEmpty() || tline.startsWith(QStringLiteral("**"))) {
			++i;
			continue;
		}
		QString upper = tline.toUpper();
		upper.remove(QLatin1Char(' '));
		if (!upper.startsWith(QLatin1String("*ELEMENT"))) {
			++i;
			continue;
		}
		if (mcMeshInpLineIsSurfaceElementHeader(upper)) {
			++i;
			continue;
		}
		const QString typeTok = mcExtractElementTypeToken(upper);
		if (!mcIsSupportedVolumeType(typeTok)) {
			++i;
			continue;
		}

		int j = i + 1;
		while (j < lines.size()) {
			const QString dat = lines[j].trimmed();
			if (dat.isEmpty()) {
				++j;
				continue;
			}
			if (dat.startsWith(QLatin1Char('*')))
				break;
			const QStringList toks = dat.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
			if (toks.size() < 2) {
				++j;
				continue;
			}
			bool okE = false;
			const int eid = toks[0].toInt(&okE);
			if (!okE)
				continue;
			QVector<int> nn;
			for (int k = 1; k < toks.size(); ++k) {
				bool okN = false;
				const int v = toks[k].toInt(&okN);
				if (okN)
					nn.append(v);
			}
			mcExpandSolidFacesForSurfaceCollect(typeTok, eid, nn, elsetElems, globalFaceCount, elsetFaceByKey);
			++j;
		}
		i = j;
	}
	return true;
}

static QList<QVector<int>> mcCollectAbsoluteExteriorFaceNodes(const QHash<QString, int>& globalFaceCount,
                                                              const QHash<QString, McElsetFaceRecord>& elsetFaceByKey) {
	QList<QVector<int>> out;
	for (auto it = elsetFaceByKey.constBegin(); it != elsetFaceByKey.constEnd(); ++it) {
		if (globalFaceCount.value(it.key()) != 1)
			continue;
		if (it.value().faceNodes.size() >= 3)
			out.append(it.value().faceNodes);
	}
	return out;
}

static bool mcComputeUnitFaceNormal(const QVector<int>& faceNodeIds,
                                    const QMap<int, GearNodeXYZ>& nodePos,
                                    double* nx,
                                    double* ny,
                                    double* nz) {
	if (!nx || !ny || !nz || faceNodeIds.size() < 3)
		return false;
	const auto get = [&](int id) -> const GearNodeXYZ* {
		const auto it = nodePos.constFind(id);
		return it == nodePos.constEnd() ? nullptr : &(*it);
	};
	const GearNodeXYZ* p0 = get(faceNodeIds[0]);
	const GearNodeXYZ* p1 = get(faceNodeIds[1]);
	const GearNodeXYZ* p2 = get(faceNodeIds[2]);
	if (!p0 || !p1 || !p2)
		return false;
	const double ax = p1->x - p0->x;
	const double ay = p1->y - p0->y;
	const double az = p1->z - p0->z;
	const double bx = p2->x - p0->x;
	const double by = p2->y - p0->y;
	const double bz = p2->z - p0->z;
	const double fnx = ay * bz - az * by;
	const double fny = az * bx - ax * bz;
	const double fnz = ax * by - ay * bx;
	const double len = std::hypot(fnx, std::hypot(fny, fnz));
	if (len < 1e-15)
		return false;
	*nx = fnx / len;
	*ny = fny / len;
	*nz = fnz / len;
	return true;
}

static McFilteredSurfaceFaces mcFilterElsetExteriorFaces(const QHash<QString, int>& globalFaceCount,
                                                         const QHash<QString, McElsetFaceRecord>& elsetFaceByKey,
                                                         const QSet<int>* targetNodeIds,
                                                         bool excludeEndFaces,
                                                         const QMap<int, GearNodeXYZ>* nodePos,
                                                         SurfaceFilterMode mode,
                                                         const QString& logLabel) {
	McFilteredSurfaceFaces out;
	Q_UNUSED(mode);

	int exteriorCount = 0;
	int nodesInSetCount = 0;
	int afterEndFaceCull = 0;

	for (auto it = elsetFaceByKey.constBegin(); it != elsetFaceByKey.constEnd(); ++it) {
		const QString key = it.key();
		if (globalFaceCount.value(key) != 1)
			continue;
		++exteriorCount;

		const McElsetFaceRecord rec = it.value();
		if (rec.faceNodes.size() < 3)
			continue;

		if (targetNodeIds) {
			bool allNodesInRegion = true;
			for (int nid : rec.faceNodes) {
				if (!targetNodeIds->contains(nid)) {
					allNodesInRegion = false;
					break;
				}
			}
			if (!allNodesInRegion)
				continue;
		}
		++nodesInSetCount;

		if (excludeEndFaces && nodePos) {
			double fnx = 0.0;
			double fny = 0.0;
			double fnz = 0.0;
			if (mcComputeUnitFaceNormal(rec.faceNodes, *nodePos, &fnx, &fny, &fnz)
			    && std::fabs(fnz) > 0.75) {
				continue;
			}
		}
		++afterEndFaceCull;

		if (rec.localFaceIndex0 < 0)
			continue;

		McElemFaceRef ref;
		ref.elemId    = rec.elemId;
		ref.faceLabel = mcFaceLabelFromIndex0(rec.localFaceIndex0);
		out.elemRefs.append(ref);
		out.faceNodeLists.append(rec.faceNodes);
	}

	if (!logLabel.isEmpty()) {
		if (excludeEndFaces && nodePos) {
	GEAR_OPT_DEBUG_NOQUOTE
			    << QStringLiteral("[GearOpt][Mesh] %1: exterior=%2, allNodesInSet=%3, afterEndFaceCull=%4, final=%5")
			           .arg(logLabel)
			           .arg(exteriorCount)
			           .arg(nodesInSetCount)
			           .arg(afterEndFaceCull)
			           .arg(out.elemRefs.size());
		} else {
	GEAR_OPT_DEBUG_NOQUOTE
			    << QStringLiteral("[GearOpt][Mesh] %1: exterior=%2, allNodesInSet=%3, final=%4")
			           .arg(logLabel)
			           .arg(exteriorCount)
			           .arg(nodesInSetCount)
			           .arg(out.elemRefs.size());
		}
	}

	return out;
}

static bool mcWriteDebugSurfaceTrianglesVtk(const QString& vtkPath,
                                            const QList<QVector<int>>& faceNodeLists,
                                            const QMap<int, GearNodeXYZ>& nodePos) {
	struct VtkTri {
		int a = 0;
		int b = 0;
		int c = 0;
	};
	QHash<int, int>           nidToPt;
	QVector<GearNodeXYZ>      pts;
	QVector<VtkTri>           tris;

	auto ptIndex = [&](int nid) -> int {
		const auto found = nidToPt.constFind(nid);
		if (found != nidToPt.constEnd())
			return found.value();
		const auto it = nodePos.constFind(nid);
		if (it == nodePos.constEnd())
			return -1;
		const int idx = pts.size();
		nidToPt.insert(nid, idx);
		pts.append(it.value());
		return idx;
	};

	for (const QVector<int>& face : faceNodeLists) {
		if (face.size() == 3) {
			const int a = ptIndex(face[0]);
			const int b = ptIndex(face[1]);
			const int c = ptIndex(face[2]);
			if (a < 0 || b < 0 || c < 0)
				continue;
			tris.append({a, b, c});
		} else if (face.size() >= 4) {
			const int a = ptIndex(face[0]);
			const int b = ptIndex(face[1]);
			const int c = ptIndex(face[2]);
			const int d = ptIndex(face[3]);
			if (a < 0 || b < 0 || c < 0 || d < 0)
				continue;
			tris.append({a, b, c});
			tris.append({a, c, d});
		}
	}

	if (tris.isEmpty())
		return false;

	QFile f(vtkPath);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts << "# vtk DataFile Version 3.0\n";
	ts << "GearAutoOpt debug surface triangles\n";
	ts << "ASCII\n";
	ts << "DATASET POLYDATA\n";
	ts << "POINTS " << pts.size() << " float\n";
	for (const GearNodeXYZ& p : pts)
		ts << p.x << " " << p.y << " " << p.z << "\n";
	ts << "POLYGONS " << tris.size() << " " << (tris.size() * 4) << "\n";
	for (const VtkTri& t : tris)
		ts << "3 " << t.a << " " << t.b << " " << t.c << "\n";
	return f.error() == QFile::NoError;
}

static QString mcDebugVtkPath(const QString& meshInpPath, const QString& baseName) {
	return QFileInfo(meshInpPath).absolutePath() + QLatin1Char('/') + baseName + QStringLiteral("_debug_surface_triangles.vtk");
}

static QString mcDebugExteriorVtkPath(const QString& meshInpPath, const QString& elsetName) {
	return QFileInfo(meshInpPath).absolutePath() + QLatin1Char('/') + elsetName
	       + QStringLiteral("_debug_exterior_triangles.vtk");
}

static void mcExportDebugExteriorVtkForElset(const QString& meshInpPath,
                                             const QString& elsetName,
                                             const QSet<int>& elsetElems,
                                             const QMap<int, GearNodeXYZ>& nodePos) {
	QHash<QString, int>              globalFaceCount;
	QHash<QString, McElsetFaceRecord> elsetFaceByKey;
	if (!mcBuildSolidFaceMaps(meshInpPath, elsetElems, &globalFaceCount, &elsetFaceByKey))
		return;
	const QList<QVector<int>> faces = mcCollectAbsoluteExteriorFaceNodes(globalFaceCount, elsetFaceByKey);
	const QString             path  = mcDebugExteriorVtkPath(meshInpPath, elsetName);
	if (mcWriteDebugSurfaceTrianglesVtk(path, faces, nodePos)) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh][DebugVTK] %1 triangles=%2 -> %3")
		                              .arg(elsetName)
		                              .arg(faces.size())
		                              .arg(path);
	}
}

static void mcExportDebugSurfaceVtk(const QString& meshInpPath,
                                    const QString& surfName,
                                    const QList<QVector<int>>& faceNodeLists,
                                    const QMap<int, GearNodeXYZ>& nodePos) {
	const QString path = mcDebugVtkPath(meshInpPath, surfName);
	if (mcWriteDebugSurfaceTrianglesVtk(path, faceNodeLists, nodePos)) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh][DebugVTK] %1 triangles=%2 -> %3")
		                              .arg(surfName)
		                              .arg(faceNodeLists.size())
		                              .arg(path);
	}
}

static McFilteredSurfaceFaces collectElementSurfacesGeneric(const QString& meshInpPath,
                                                            const QSet<int>& elsetElems,
                                                            const QSet<int>& targetNodeIds,
                                                            SurfaceFilterMode mode,
                                                            const QMap<int, GearNodeXYZ>* nodePos,
                                                            bool excludeEndFaces,
                                                            const QString& logLabel) {
	McFilteredSurfaceFaces empty;
	if (elsetElems.isEmpty() || targetNodeIds.isEmpty())
		return empty;

	QHash<QString, int>              globalFaceCount;
	QHash<QString, McElsetFaceRecord> elsetFaceByKey;
	if (!mcBuildSolidFaceMaps(meshInpPath, elsetElems, &globalFaceCount, &elsetFaceByKey))
		return empty;

	return mcFilterElsetExteriorFaces(globalFaceCount, elsetFaceByKey, &targetNodeIds, excludeEndFaces, nodePos,
	                                  mode, logLabel);
}

enum class ContactSurfaceMode {
	MatingToothBand,       ///< 啮合角窗口 + TOOTH_OUTER 节点（旧逻辑，仅 TOOTH_OUTER NSET）
	LocalContactToothBand  ///< 啮合中心齿 ±N 局部外周齿面（GEAR*_TOOTH_SURF）
};

static bool mcFaceAllNodesNearGearZPlane(const QVector<int>& faceNodeIds,
                                         const QMap<int, GearNodeXYZ>& nodePos,
                                         double zPlane,
                                         double zTol) {
	if (faceNodeIds.isEmpty())
		return false;
	for (int nid : faceNodeIds) {
		const auto it = nodePos.constFind(nid);
		if (it == nodePos.constEnd())
			return false;
		if (std::fabs(it->z - zPlane) > zTol)
			return false;
	}
	return true;
}

static bool mcFaceIsGearEndCap(const QVector<int>& faceNodeIds,
                                 const QMap<int, GearNodeXYZ>& nodePos,
                                 double zMin,
                                 double zMax,
                                 double zTol) {
	return mcFaceAllNodesNearGearZPlane(faceNodeIds, nodePos, zMin, zTol)
	       || mcFaceAllNodesNearGearZPlane(faceNodeIds, nodePos, zMax, zTol);
}

static double mcFaceAvgRadial(const QVector<int>& faceNodeIds,
                              const QMap<int, GearNodeXYZ>& nodePos,
                              double axisX,
                              double axisY) {
	double sum = 0.0;
	int    n   = 0;
	for (int nid : faceNodeIds) {
		const auto it = nodePos.constFind(nid);
		if (it == nodePos.constEnd())
			continue;
		sum += std::hypot(it->x - axisX, it->y - axisY);
		++n;
	}
	return n > 0 ? sum / static_cast<double>(n) : 0.0;
}

/// GEAR*_TOOTH_SURF：ELSET 绝对外表面，剔除端面与孔内壁，并按啮合中心齿局部齿号筛选。
static McFilteredSurfaceFaces collectLocalContactToothSurfacesForElset(const QString& meshInpPath,
                                                                       const QString& elsetName,
                                                                       const QString& surfName,
                                                                       const QMap<int, GearNodeXYZ>& nodePos,
                                                                       const GearToothPickContext& pickCtx,
                                                                       double axisX,
                                                                       double axisY,
                                                                       double hubR,
                                                                       double hubTol,
                                                                       double zMin,
                                                                       double zMax,
                                                                       QString* outToothRangeText) {
	McFilteredSurfaceFaces out;
	QSet<int> elsetElems;
	if (!mcParseVolumeElsetElements(meshInpPath, elsetName, &elsetElems, nullptr) || elsetElems.isEmpty())
		return out;

	QHash<QString, int>               globalFaceCount;
	QHash<QString, McElsetFaceRecord> elsetFaceByKey;
	if (!mcBuildSolidFaceMaps(meshInpPath, elsetElems, &globalFaceCount, &elsetFaceByKey))
		return out;

	const double zSpan         = std::max(1e-9, zMax - zMin);
	const double zTol          = std::max(1e-6, 0.02 * zSpan);
	const double hubRThreshold = hubR + hubTol;

	const int centerToothIndex = gearComputeCenterToothIndex(pickCtx);
	const int halfSpan         = std::max(0, pickCtx.contactToothHalfSpan);

	int fullExterior   = 0;
	int removedEnd     = 0;
	int removedHub     = 0;
	int removedTooth   = 0;
	int halfToothFaces = 0;

	for (auto it = elsetFaceByKey.constBegin(); it != elsetFaceByKey.constEnd(); ++it) {
		if (globalFaceCount.value(it.key()) != 1)
			continue;
		++fullExterior;

		const McElsetFaceRecord rec = it.value();
		if (rec.faceNodes.size() < 3 || rec.localFaceIndex0 < 0)
			continue;

		if (mcFaceIsGearEndCap(rec.faceNodes, nodePos, zMin, zMax, zTol)) {
			++removedEnd;
			continue;
		}

		const double rAvg = mcFaceAvgRadial(rec.faceNodes, nodePos, axisX, axisY);
		if (rAvg < hubRThreshold) {
			++removedHub;
			continue;
		}

		double mx = 0.0;
		double my = 0.0;
		double mz = 0.0;
		for (int nid : rec.faceNodes) {
			const auto np = nodePos.constFind(nid);
			if (np == nodePos.constEnd())
				continue;
			mx += np->x;
			my += np->y;
			mz += np->z;
		}
		const double inv = 1.0 / static_cast<double>(rec.faceNodes.size());
		mx *= inv;
		my *= inv;
		mz *= inv;
		Q_UNUSED(mz);

		bool isHalfTooth = false;
		if (!gearFaceInLocalContactToothRange(mx, my, pickCtx, nullptr, &isHalfTooth)) {
			++removedTooth;
			continue;
		}
		if (isHalfTooth)
			++halfToothFaces;

		McElemFaceRef ref;
		ref.elemId    = rec.elemId;
		ref.faceLabel = mcFaceLabelFromIndex0(rec.localFaceIndex0);
		out.elemRefs.append(ref);
		out.faceNodeLists.append(rec.faceNodes);
	}

	const QString rangeText = gearFormatToothRangeText(centerToothIndex, halfSpan, pickCtx.teethCount);
	if (outToothRangeText)
		*outToothRangeText = rangeText;

	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1 full outer exterior faces = %2")
	                              .arg(elsetName)
	                              .arg(fullExterior);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1 removed end faces = %2")
	                              .arg(elsetName)
	                              .arg(removedEnd);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1 removed hub inner faces = %2")
	                              .arg(elsetName)
	                              .arg(removedHub);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1 removed non-contact tooth faces = %2")
	                              .arg(surfName)
	                              .arg(removedTooth);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1 local contact mode = %2 half-tooth transition = %3")
	                              .arg(surfName)
	                              .arg(pickCtx.localContactSurface ? QStringLiteral("true")
	                                                               : QStringLiteral("false"))
	                              .arg(pickCtx.useHalfToothTransition ? QStringLiteral("true")
	                                                                  : QStringLiteral("false"));
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1 tooth range: %2 half-tooth faces = %3")
	                              .arg(surfName)
	                              .arg(rangeText)
	                              .arg(halfToothFaces);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1 final faces = %2")
	                              .arg(surfName)
	                              .arg(out.elemRefs.size());

	return out;
}

static McFilteredSurfaceFaces collectHubElementSurfacesForElset(const QString& meshInpPath,
                                                                const QString& elsetName,
                                                                const QSet<int>& hubNodeIds,
                                                                const QString& logLabel) {
	QSet<int> elsetElems;
	if (!mcParseVolumeElsetElements(meshInpPath, elsetName, &elsetElems, nullptr) || hubNodeIds.isEmpty())
		return {};
	return collectElementSurfacesGeneric(meshInpPath, elsetElems, hubNodeIds,
	                                     SurfaceFilterMode::AbsoluteExterior, nullptr, false, logLabel);
}

static QSet<int> collectHubNodesForGear(const QMap<int, GearNodeXYZ>& nodePos,
                                        const QSet<int>& gearNodeIds,
                                        double axisX,
                                        double axisY,
                                        double hubR,
                                        double boreTol) {
	auto pickShell = [&](double tol) -> QSet<int> {
		QSet<int> out;
		for (int nid : gearNodeIds) {
			const auto it = nodePos.constFind(nid);
			if (it == nodePos.constEnd())
				continue;
			const double r = std::hypot(it->x - axisX, it->y - axisY);
			if (std::fabs(r - hubR) <= tol)
				out.insert(nid);
		}
		return out;
	};

	QSet<int> hub = pickShell(boreTol);
	if (hub.isEmpty())
		hub = pickShell(boreTol * 2.5);
	if (hub.isEmpty()) {
		for (int nid : gearNodeIds) {
			const auto it = nodePos.constFind(nid);
			if (it == nodePos.constEnd())
				continue;
			const double r = std::hypot(it->x - axisX, it->y - axisY);
			if (r <= hubR * 1.12)
				hub.insert(nid);
		}
	}
	return hub;
}

static bool mcParseAllNodes(const QString& meshInpPath, QMap<int, GearNodeXYZ>* nodePos) {
	if (!nodePos)
		return false;
	nodePos->clear();
	bool ok = false;
	const QStringList lines = readLinesKeepEol(meshInpPath, &ok);
	if (!ok)
		return false;

	int i = 0;
	while (i < lines.size()) {
		const QString trimmed = lines[i].trimmed();
		QString       upper   = trimmed.toUpper();
		upper.remove(QLatin1Char(' '));
		if (upper.startsWith(QLatin1String("*NODE")) && !upper.startsWith(QLatin1String("*NODEPRINT"))
		    && !upper.startsWith(QLatin1String("*NODEFILE"))) {
			int j = i + 1;
			while (j < lines.size()) {
				const QString dat = lines[j].trimmed();
				if (dat.isEmpty()) {
					++j;
					continue;
				}
				if (dat.startsWith(QLatin1Char('*')))
					break;
				const QStringList toks = dat.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
				if (toks.size() >= 4) {
					bool okId = false;
					const int nid = toks[0].toInt(&okId);
					if (okId) {
						GearNodeXYZ g;
						g.x = toks[1].toDouble();
						g.y = toks[2].toDouble();
						g.z = toks[3].toDouble();
						nodePos->insert(nid, g);
					}
				}
				++j;
			}
			i = j;
			continue;
		}
		++i;
	}
	return !nodePos->isEmpty();
}

static bool mcAppendSetBlock(const QString& meshInpPath,
                             const QString& card,  // NSET or ELSET
                             const QString& setName,
                             const QList<int>& ids) {
	QFile f(meshInpPath);
	if (!f.open(QIODevice::Append | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts << "\n** GearAutoOpt: " << setName << "\n";
	ts << "*" << card << ", " << card << "=" << setName << "\n";
	for (int i = 0; i < ids.size(); ++i) {
		ts << ids[i];
		if ((i + 1) % 16 == 0 || i == ids.size() - 1)
			ts << "\n";
		else
			ts << ", ";
	}
	return f.error() == QFile::NoError;
}

static bool mcAppendSurfaceBlock(const QString& meshInpPath,
                                 const QString& surfName,
                                 const QList<McElemFaceRef>& faces) {
	QFile f(meshInpPath);
	if (!f.open(QIODevice::Append | QIODevice::Text))
		return false;
	QTextStream ts(&f);
	ts << "\n** GearAutoOpt: " << surfName << "\n";
	ts << "*SURFACE, NAME=" << surfName << ", TYPE=ELEMENT\n";
	for (const McElemFaceRef& ref : faces) {
		ts << ref.elemId << ", " << ref.faceLabel << "\n";
	}
	return f.error() == QFile::NoError;
}

static QList<int> sortedIds(const QSet<int>& ids) {
	QList<int> list = QList<int>::fromSet(ids);
	std::sort(list.begin(), list.end());
	return list;
}

static bool stripGearAutoOptAppendedSections(const QString& meshInpPath)
{
	bool ok = false;
	QStringList lines = readLinesKeepEol(meshInpPath, &ok);
	if (!ok)
		return false;

	QStringList out;
	out.reserve(lines.size());
	for (int i = 0; i < lines.size(); ++i) {
		const QString t = lines[i].trimmed();
		if (t.startsWith(QStringLiteral("** GearAutoOpt:"))) {
			while (i + 1 < lines.size()) {
				const QString nt = lines[i + 1].trimmed();
				if (nt.startsWith(QStringLiteral("** GearAutoOpt:")))
					break;
				++i;
			}
			continue;
		}
		out << lines[i];
	}
	return writeFile(meshInpPath, out.join(QString()));
}

static bool inferAxisFromNodeSet(const QMap<int, GearNodeXYZ>& nodePos,
                                 const QSet<int>&             gearNodeIds,
                                 double*                      axisX,
                                 double*                      axisY)
{
	if (!axisX || !axisY || gearNodeIds.isEmpty())
		return false;
	double sx = 0.0;
	double sy = 0.0;
	int    n  = 0;
	for (int nid : gearNodeIds) {
		const auto it = nodePos.constFind(nid);
		if (it == nodePos.constEnd())
			continue;
		sx += it->x;
		sy += it->y;
		++n;
	}
	if (n <= 0)
		return false;
	*axisX = sx / static_cast<double>(n);
	*axisY = sy / static_cast<double>(n);
	return true;
}

static double mcMatingAngleRad(double fromX, double fromY, double toX, double toY) {
	return std::atan2(toY - fromY, toX - fromX);
}

static bool collectToothNodesFromSolidExteriorFacesForElset(const QString& meshInpPath,
                                                            const QMap<int, GearNodeXYZ>& nodePos,
                                                            const QSet<int>& hubIds,
                                                            const GearToothPickContext& ctx,
                                                            GearToothFacePickMode mode,
                                                            const QString& elsetName,
                                                            QList<int>* outNodes,
                                                            QString* diag);

static bool enrichOneGearSets(const QString& meshInpPath,
                              const QMap<int, GearNodeXYZ>& nodePos,
                              const QString& elsetName,
                              const QString& hubNset,
                              const QString& toothNset,
                              const QString& surfName,
                              const QString& contactSurfAlias,
                              const QString& hubSurfName,
                              const GearMeshEnrichParams& params,
                              int z,
                              double xShift,
                              double axisX,
                              double axisY,
                              double targetAngleRad,
                              int* hubCount,
                              int* toothCount,
                              int* surfCount,
                              bool* localContactSurf,
                              int* centerToothIndex,
                              QString* toothRangeText) {
	const double m      = params.module;
	const double pitchR = z * m / 2.0;
	// 与 GeoCommandCreateGear 一致：孔径 = 0.4×分度圆直径 → 孔半径 = 0.4×pitchR（不用 dp.hubRatio，避免优化变量与几何不一致）
	const double hubR   = 0.4 * pitchR;
	const double tipR   = (z / 2.0 + 1.0 + xShift) * m;
	const double rootR  = std::max(0.1 * m, pitchR - (1.25 - xShift) * m);
	const double meshSz = params.meshSize > 0.0 ? params.meshSize : (0.5 * m);
	const double boreTol = std::max(0.28 * meshSz, 0.025 * std::max(hubR, 1e-6));

	QSet<int> gearNodes;
	QSet<int> gearElems;
	if (!mcParseVolumeElsetElements(meshInpPath, elsetName, &gearElems, &gearNodes)) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1: no volume elements in elset %2")
		                              .arg(hubNset, elsetName);
		return false;
	}

	const QSet<int> hubIds =
	    collectHubNodesForGear(nodePos, gearNodes, axisX, axisY, hubR, boreTol);

	double zMin = 1e300;
	double zMax = -1e300;
	for (int nid : gearNodes) {
		const auto it = nodePos.constFind(nid);
		if (it == nodePos.constEnd())
			continue;
		zMin = std::min(zMin, it->z);
		zMax = std::max(zMax, it->z);
	}

	GearToothPickContext pickCtx;
	pickCtx.module          = m;
	pickCtx.rootR           = rootR;
	pickCtx.pitchR          = pitchR;
	pickCtx.tipR            = tipR;
	pickCtx.hubR            = hubR;
	pickCtx.boreTol         = boreTol;
	pickCtx.zMin            = zMin;
	pickCtx.zMax            = zMax;
	pickCtx.axisX           = axisX;
	pickCtx.axisY           = axisY;
	pickCtx.teethCount      = z;
	pickCtx.loadToothCount  = 3.0;
	pickCtx.targetAngleRad  = targetAngleRad;
	pickCtx.localContactSurface    = true;
	pickCtx.contactToothHalfSpan   = 3;
	pickCtx.useHalfToothTransition = false;

	mcExportDebugExteriorVtkForElset(meshInpPath, elsetName, gearElems, nodePos);

	QSet<int> toothAcc;
	QString   tdiag;
	for (GearToothFacePickMode mode :
	     {GearToothFacePickMode::ToothBand, GearToothFacePickMode::RelaxedRadial,
	      GearToothFacePickMode::ExteriorShell}) {
		QList<int> tmp;
		if (collectToothNodesFromSolidExteriorFacesForElset(meshInpPath, nodePos, hubIds, pickCtx,
		                                                    mode, elsetName, &tmp, &tdiag)) {
			for (int nid : tmp)
				toothAcc.insert(nid);
		}
	}
	QList<int> toothNodes = sortedIds(toothAcc);
	QString    surfRangeText;
	const McFilteredSurfaceFaces toothFaces =
	    collectLocalContactToothSurfacesForElset(meshInpPath, elsetName, surfName, nodePos, pickCtx, axisX, axisY,
	                                             hubR, boreTol, zMin, zMax, &surfRangeText);
	const QList<McElemFaceRef>& surfRefs = toothFaces.elemRefs;

	if (localContactSurf)
		*localContactSurf = pickCtx.localContactSurface;
	if (centerToothIndex)
		*centerToothIndex = gearComputeCenterToothIndex(pickCtx);
	if (toothRangeText)
		*toothRangeText = surfRangeText;

	if (kExportToothSurfDebugVtk)
		mcExportDebugSurfaceVtk(meshInpPath, surfName, toothFaces.faceNodeLists, nodePos);

	if (hubIds.isEmpty()) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1: hub nodes empty (axis=%2,%3 hubR=%4 tol=%5 nodes=%6)")
		                              .arg(hubNset)
		                              .arg(axisX, 0, 'g', 6)
		                              .arg(axisY, 0, 'g', 6)
		                              .arg(hubR, 0, 'g', 6)
		                              .arg(boreTol, 0, 'g', 6)
		                              .arg(gearNodes.size());
		return false;
	}
	if (toothNodes.isEmpty()) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1: tooth outer nodes empty (diag=%2 elems=%3)")
		                              .arg(toothNset, tdiag)
		                              .arg(gearElems.size());
		return false;
	}
	if (surfRefs.isEmpty()) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1: local contact tooth surface empty (elset=%2 elems=%3 range=%4)")
		                              .arg(surfName, elsetName)
		                              .arg(gearElems.size())
		                              .arg(surfRangeText);
		return false;
	}

	if (!mcAppendSetBlock(meshInpPath, QStringLiteral("NSET"), hubNset, sortedIds(hubIds)))
		return false;
	if (!mcAppendSetBlock(meshInpPath, QStringLiteral("NSET"), toothNset, toothNodes))
		return false;
	if (!mcAppendSurfaceBlock(meshInpPath, surfName, surfRefs))
		return false;
	if (!contactSurfAlias.isEmpty()) {
		if (!mcAppendSurfaceBlock(meshInpPath, contactSurfAlias, surfRefs))
			return false;
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1 debug surface faces = %2 (alias for contact)")
		                              .arg(surfName)
		                              .arg(surfRefs.size());
	}

	if (!hubSurfName.isEmpty()) {
		const McFilteredSurfaceFaces hubFaces =
		    collectHubElementSurfacesForElset(meshInpPath, elsetName, hubIds, hubSurfName);
		const QList<McElemFaceRef>& hubSurfRefs = hubFaces.elemRefs;
		mcExportDebugSurfaceVtk(meshInpPath, hubSurfName, hubFaces.faceNodeLists, nodePos);
		if (hubSurfRefs.isEmpty()) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] %1: hub surface faces empty (hub nodes=%2)")
			                              .arg(hubSurfName)
			                              .arg(hubIds.size());
			return false;
		}
		if (!mcAppendSurfaceBlock(meshInpPath, hubSurfName, hubSurfRefs)) {
			return false;
		}
	}

	if (hubCount)
		*hubCount = hubIds.size();
	if (toothCount)
		*toothCount = toothNodes.size();
	if (surfCount)
		*surfCount = surfRefs.size();
	return true;
}

bool meshInpHasElset(const QString& meshInpPath, const QString& elsetName) {
	QSet<int> ids;
	if (mcParseVolumeElsetElements(meshInpPath, elsetName, &ids, nullptr) && !ids.isEmpty())
		return true;

	bool ok = false;
	const QStringList lines = readLinesKeepEol(meshInpPath, &ok);
	if (!ok)
		return false;
	const QString want = elsetName.toUpper();
	int           i    = 0;
	while (i < lines.size()) {
		const QString t = lines[i].trimmed();
		QString       u = t.toUpper();
		u.remove(QLatin1Char(' '));
		if (u.startsWith(QLatin1String("*ELSET"))) {
			const QString name = extractSetName(u, QLatin1String("ELSET="));
			if (name == want) {
				QSet<int> eids;
				collectIdColumn(lines, i + 1, &eids);
				return !eids.isEmpty();
			}
		}
		++i;
	}
	return false;
}

bool meshInpHasNset(const QString& meshInpPath, const QString& nsetName)
{
	return countNsetNodes(meshInpPath, nsetName) > 0;
}

bool meshInpNeedsGearSetEnrich(const QString& meshInpPath)
{
	if (!meshInpHasElset(meshInpPath, QStringLiteral("GEAR1"))
	    || !meshInpHasElset(meshInpPath, QStringLiteral("GEAR2")))
		return false;
	if (!meshInpHasNset(meshInpPath, QStringLiteral("GEAR1_HUB")))
		return true;
	if (countSurfaceFaces(meshInpPath, QStringLiteral("GEAR1_HUB_SURF")) <= 0)
		return true;
	return countSurfaceFaces(meshInpPath, QStringLiteral("master")) <= 0
	       || countSurfaceFaces(meshInpPath, QStringLiteral("slave")) <= 0;
}

bool enrichGearMeshInp(const QString& meshInpPath,
                       const GearMeshEnrichParams& params,
                       DualGearMeshReport* report) {
	if (!meshInpHasElset(meshInpPath, QStringLiteral("GEAR1"))
	    || !meshInpHasElset(meshInpPath, QStringLiteral("GEAR2"))) {
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] enrichGearMeshInp: GEAR1/GEAR2 volume elset not found";
		return false;
	}

	stripGearAutoOptAppendedSections(meshInpPath);

	QMap<int, GearNodeXYZ> nodePos;
	if (!mcParseAllNodes(meshInpPath, &nodePos))
		return false;

	DualGearMeshReport local;
	QSet<int>          e1;
	QSet<int>          e2;
	QSet<int>          n1;
	QSet<int>          n2;
	mcParseVolumeElsetElements(meshInpPath, QStringLiteral("GEAR1"), &e1, &n1);
	mcParseVolumeElsetElements(meshInpPath, QStringLiteral("GEAR2"), &e2, &n2);
	local.gear1ElementCount = e1.size();
	local.gear2ElementCount = e2.size();
	local.gear1NodeCount    = n1.size();
	local.gear2NodeCount    = n2.size();

	GearMeshEnrichParams p = params;
	if (p.module <= 0.0 || p.z1 <= 0 || p.z2 <= 0) {
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] enrichGearMeshInp: invalid module/z1/z2 in params";
		return false;
	}

	inferAxisFromNodeSet(nodePos, n1, &p.axis1X, &p.axis1Y);
	inferAxisFromNodeSet(nodePos, n2, &p.axis2X, &p.axis2Y);

	const double angle1 = mcMatingAngleRad(p.axis1X, p.axis1Y, p.axis2X, p.axis2Y);
	const double angle2 = mcMatingAngleRad(p.axis2X, p.axis2Y, p.axis1X, p.axis1Y);

	GEAR_OPT_DEBUG << "[GearOpt][Mesh] enrich from GEAR1/GEAR2 axis1=" << p.axis1X << p.axis1Y << "axis2=" << p.axis2X
	         << p.axis2Y;

	if (!enrichOneGearSets(meshInpPath, nodePos, QStringLiteral("GEAR1"), QStringLiteral("GEAR1_HUB"),
	                       QStringLiteral("GEAR1_TOOTH_OUTER"), QStringLiteral("GEAR1_TOOTH_SURF"),
	                       QStringLiteral("master"), QStringLiteral("GEAR1_HUB_SURF"), p, p.z1, p.x1, p.axis1X,
	                       p.axis1Y, angle1, &local.gear1HubNodeCount, &local.gear1ToothNodeCount,
	                       &local.gear1ToothSurfCount, &local.gear1LocalContactSurf, &local.gear1CenterToothIndex,
	                       &local.gear1ToothRangeText)) {
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] enrichOneGearSets GEAR1 failed";
		return false;
	}

	if (!enrichOneGearSets(meshInpPath, nodePos, QStringLiteral("GEAR2"), QStringLiteral("GEAR2_HUB"),
	                       QStringLiteral("GEAR2_TOOTH_OUTER"), QStringLiteral("GEAR2_TOOTH_SURF"),
	                       QStringLiteral("slave"), QStringLiteral("GEAR2_HUB_SURF"), p, p.z2, p.x2, p.axis2X,
	                       p.axis2Y, angle2, &local.gear2HubNodeCount, &local.gear2ToothNodeCount,
	                       &local.gear2ToothSurfCount, &local.gear2LocalContactSurf, &local.gear2CenterToothIndex,
	                       &local.gear2ToothRangeText)) {
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] enrichOneGearSets GEAR2 failed";
		return false;
	}

	GEAR_OPT_DEBUG << "[GearOpt][Mesh] GEAR1_HUB node count =" << local.gear1HubNodeCount;
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] GEAR2_HUB node count =" << local.gear2HubNodeCount;
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] GEAR1_TOOTH_OUTER node count =" << local.gear1ToothNodeCount;
	GEAR_OPT_DEBUG << "[GearOpt][Mesh] GEAR2_TOOTH_OUTER node count =" << local.gear2ToothNodeCount;
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] GEAR1_TOOTH_SURF face count = %1 localContact = %2 range: %3")
	                              .arg(local.gear1ToothSurfCount)
	                              .arg(local.gear1LocalContactSurf ? QStringLiteral("true")
	                                                               : QStringLiteral("false"))
	                              .arg(local.gear1ToothRangeText);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] GEAR2_TOOTH_SURF face count = %1 localContact = %2 range: %3")
	                              .arg(local.gear2ToothSurfCount)
	                              .arg(local.gear2LocalContactSurf ? QStringLiteral("true")
	                                                               : QStringLiteral("false"))
	                              .arg(local.gear2ToothRangeText);
	const int masterFaces = countSurfaceFaces(meshInpPath, QStringLiteral("master"));
	const int slaveFaces  = countSurfaceFaces(meshInpPath, QStringLiteral("slave"));
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] contact surface names: slave, master");
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] master face count = %1  slave face count = %2")
	                              .arg(masterFaces)
	                              .arg(slaveFaces);
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][Mesh] local contact teeth: center ±3, full 7 teeth");

	if (report)
		*report = local;
	return true;
}

static bool collectToothNodesFromSolidExteriorFacesForElset(const QString& meshInpPath,
                                                            const QMap<int, GearNodeXYZ>& nodePos,
                                                            const QSet<int>& hubIds,
                                                            const GearToothPickContext& ctx,
                                                            GearToothFacePickMode mode,
                                                            const QString& elsetName,
                                                            QList<int>* outNodes,
                                                            QString* diag) {
	auto fail = [&](const QString& msg) -> bool {
		if (diag)
			*diag = msg;
		if (outNodes)
			outNodes->clear();
		return false;
	};
	if (!outNodes)
		return fail(QStringLiteral("outNodes null"));
	outNodes->clear();

	QVector<QVector<int>> boundaryFaces;
	QString               bdiag;
	if (!collectBoundaryFacesForElset(meshInpPath, elsetName, &boundaryFaces, &bdiag))
		return fail(bdiag);

	QSet<int> acc;
	collectToothNodesFromOuterContourFaces(boundaryFaces, nodePos, hubIds, ctx, mode, &acc);
	if (acc.isEmpty())
		return fail(QStringLiteral("outer_contour_tooth_pick_empty"));

	QList<int> sorted = acc.values();
	std::sort(sorted.begin(), sorted.end());
	*outNodes = sorted;
	if (diag)
		diag->clear();
	return true;
}

bool collectToothNodesFromSolidExteriorFaces(const QString& meshInpPath, const QMap<int, GearNodeXYZ>& nodePos,
                                             const QSet<int>& hubIds, const GearToothPickContext& ctx,
                                             GearToothFacePickMode mode, QList<int>* outNodes, QString* diag) {
	auto fail = [&](const QString& msg) -> bool {
		if (diag)
			*diag = msg;
		if (outNodes)
			outNodes->clear();
		return false;
	};
	if (!outNodes)
		return fail(QStringLiteral("outNodes null"));
	outNodes->clear();

	QVector<QVector<int>> boundaryFaces;
	QString               bdiag;
	if (!collectBoundaryFacesFromMeshInp(meshInpPath, &boundaryFaces, &bdiag))
		return fail(bdiag);

	QSet<int> acc;
	collectToothNodesFromOuterContourFaces(boundaryFaces, nodePos, hubIds, ctx, mode, &acc);
	if (acc.isEmpty())
		return fail(QStringLiteral("outer_contour_tooth_pick_empty"));

	QList<int> sorted = acc.values();
	std::sort(sorted.begin(), sorted.end());
	*outNodes = sorted;
	if (diag)
		diag->clear();
	return true;
}

namespace {

// CalculiX C3D4 面号（节点 n1..n4 为单元数据行顺序）：
// S1=n2,n3,n4  S2=n1,n4,n3  S3=n1,n2,n4  S4=n1,n3,n2
static const int kVerifyC3D4FaceLocal[4][3] = {
    {1, 2, 3},
    {0, 3, 2},
    {0, 1, 3},
    {0, 2, 1},
};

static QVector<int> verifyC3d4FaceNodeIds(const QVector<int>& elemNodes, int sideIndex0)
{
	if (elemNodes.size() < 4 || sideIndex0 < 0 || sideIndex0 >= 4)
		return {};
	return {elemNodes[kVerifyC3D4FaceLocal[sideIndex0][0]],
	        elemNodes[kVerifyC3D4FaceLocal[sideIndex0][1]],
	        elemNodes[kVerifyC3D4FaceLocal[sideIndex0][2]]};
}

static int verifyFaceSideIndexFromLabel(const QString& label, bool* okSide)
{
	if (okSide)
		*okSide = false;
	const QString u = label.trimmed().toUpper();
	if (u.size() != 2 || u.at(0) != QLatin1Char('S'))
		return -1;
	bool okN = false;
	const int n = u.mid(1).toInt(&okN);
	if (!okN || n < 1 || n > 4)
		return -1;
	if (okSide)
		*okSide = true;
	return n - 1;
}

static bool verifyLoadInpSurfaceElemFaces(const QString& inpPath,
                                          const QString& surfaceName,
                                          QList<McElemFaceRef>* outFaces)
{
	if (!outFaces)
		return false;
	outFaces->clear();

	bool            ok = false;
	const QStringList lines = readLinesKeepEol(inpPath, &ok);
	if (!ok)
		return false;

	const QString want = surfaceName.toUpper();
	int           i    = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(QLatin1Char(' '));
		if (!upperNoSpace.startsWith(QLatin1String("*SURFACE"))
		    || !upperNoSpace.contains(QLatin1String("TYPE=ELEMENT"))) {
			++i;
			continue;
		}
		const QString name = extractSetName(upperNoSpace, QLatin1String("NAME="));
		if (name != want) {
			++i;
			continue;
		}
		++i;
		while (i < lines.size()) {
			const QString trimmed = lines[i].trimmed();
			if (trimmed.isEmpty()) {
				++i;
				continue;
			}
			if (trimmed.startsWith(QLatin1Char('*')))
				break;
			const QStringList toks = trimmed.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
			if (toks.size() >= 2) {
				bool okE = false;
				const int eid = toks[0].toInt(&okE);
				if (okE) {
					McElemFaceRef ref;
					ref.elemId    = eid;
					ref.faceLabel = toks[1];
					outFaces->append(ref);
				}
			}
			++i;
		}
		return !outFaces->isEmpty();
	}
	return false;
}

static bool verifyParseC3d4Elements(const QString& meshInpPath, QHash<int, QVector<int>>* outElems)
{
	if (!outElems)
		return false;
	outElems->clear();

	bool            ok = false;
	const QStringList lines = readLinesKeepEol(meshInpPath, &ok);
	if (!ok)
		return false;

	int i = 0;
	while (i < lines.size()) {
		const QString tline = lines[i].trimmed();
		if (tline.isEmpty() || tline.startsWith(QStringLiteral("**"))) {
			++i;
			continue;
		}
		QString upper = tline.toUpper();
		upper.remove(QLatin1Char(' '));
		if (!upper.startsWith(QLatin1String("*ELEMENT")) || mcMeshInpLineIsSurfaceElementHeader(upper)) {
			++i;
			continue;
		}
		const QString typeTok = mcExtractElementTypeToken(upper);
		if (typeTok != QLatin1String("C3D4")) {
			++i;
			continue;
		}
		int j = i + 1;
		while (j < lines.size()) {
			const QString dat = lines[j].trimmed();
			if (dat.isEmpty()) {
				++j;
				continue;
			}
			if (dat.startsWith(QLatin1Char('*')))
				break;
			const QStringList toks = dat.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
			if (toks.size() >= 5) {
				bool okE = false;
				const int eid = toks[0].toInt(&okE);
				if (okE) {
					QVector<int> nn;
					nn.reserve(4);
					for (int k = 1; k <= 4; ++k) {
						bool okN = false;
						const int nid = toks[k].toInt(&okN);
						if (okN)
							nn.append(nid);
					}
					if (nn.size() == 4)
						outElems->insert(eid, nn);
				}
			}
			++j;
		}
		i = j;
	}
	return !outElems->isEmpty();
}

static GearNodeXYZ verifyNodeOrZero(const QMap<int, GearNodeXYZ>& nodePos, int nid)
{
	const auto it = nodePos.constFind(nid);
	if (it == nodePos.constEnd())
		return {};
	return it.value();
}

static bool verifyAccumPointStats(const GearNodeXYZ& p,
                                  bool*              hasBbox,
                                  double             bboxMin[3],
                                  double             bboxMax[3],
                                  bool*              hasRadius,
                                  double*            radiusMin,
                                  double*            radiusMax,
                                  bool*              hasZ,
                                  double*            zMin,
                                  double*            zMax)
{
	if (hasBbox) {
		if (!*hasBbox) {
			bboxMin[0] = bboxMax[0] = p.x;
			bboxMin[1] = bboxMax[1] = p.y;
			bboxMin[2] = bboxMax[2] = p.z;
			*hasBbox   = true;
		} else {
			bboxMin[0] = std::min(bboxMin[0], p.x);
			bboxMin[1] = std::min(bboxMin[1], p.y);
			bboxMin[2] = std::min(bboxMin[2], p.z);
			bboxMax[0] = std::max(bboxMax[0], p.x);
			bboxMax[1] = std::max(bboxMax[1], p.y);
			bboxMax[2] = std::max(bboxMax[2], p.z);
		}
	}
	const double r = std::hypot(p.x, p.y);
	if (hasRadius) {
		if (!*hasRadius) {
			*radiusMin   = r;
			*radiusMax   = r;
			*hasRadius = true;
		} else {
			*radiusMin = std::min(*radiusMin, r);
			*radiusMax = std::max(*radiusMax, r);
		}
	}
	if (hasZ) {
		if (!*hasZ) {
			*zMin = *zMax = p.z;
			*hasZ = true;
		} else {
			*zMin = std::min(*zMin, p.z);
			*zMax = std::max(*zMax, p.z);
		}
	}
	return true;
}

} // anonymous namespace

bool verifyCcxElementSurfaceFaces(const QString& meshInpPath,
                                  const QString& jobInpPath,
                                  const QString& surfaceName,
                                  CcxSurfaceVerifyReport* report,
                                  QString*                  errorMsg)
{
	auto fail = [&](const QString& msg) -> bool {
		if (errorMsg)
			*errorMsg = msg;
		return false;
	};

	if (meshInpPath.isEmpty() || jobInpPath.isEmpty() || surfaceName.isEmpty())
		return fail(QStringLiteral("verifyCcxElementSurfaceFaces: empty path or surface name"));

	QMap<int, GearNodeXYZ> nodePos;
	if (!mcParseAllNodes(meshInpPath, &nodePos) || nodePos.isEmpty())
		return fail(QStringLiteral("verifyCcxElementSurfaceFaces: no *NODE in mesh.inp"));

	QHash<int, QVector<int>> c3d4Elems;
	if (!verifyParseC3d4Elements(meshInpPath, &c3d4Elems))
		return fail(QStringLiteral("verifyCcxElementSurfaceFaces: no C3D4 *ELEMENT in mesh.inp"));

	QList<McElemFaceRef> surfFaces;
	if (!verifyLoadInpSurfaceElemFaces(jobInpPath, surfaceName, &surfFaces)
	    && !verifyLoadInpSurfaceElemFaces(meshInpPath, surfaceName, &surfFaces))
		return fail(QStringLiteral("verifyCcxElementSurfaceFaces: *SURFACE not found in job.inp or mesh.inp"));

	CcxSurfaceVerifyReport local;
	local.surfaceName = surfaceName;
	local.faceCount   = surfFaces.size();

	QList<QVector<int>> vtkFaceNodeLists;
	vtkFaceNodeLists.reserve(surfFaces.size());

	bool   hasBbox = false;
	double bboxMin[3] = {0, 0, 0};
	double bboxMax[3] = {0, 0, 0};
	bool   hasRadius  = false;
	double radiusMin  = 0.0;
	double radiusMax  = 0.0;
	bool   hasZ       = false;
	double zMin       = 0.0;
	double zMax       = 0.0;

	for (const McElemFaceRef& ref : surfFaces) {
		bool okSide = false;
		const int sideIndex0 = verifyFaceSideIndexFromLabel(ref.faceLabel, &okSide);
		if (!okSide) {
			++local.invalidSide;
			continue;
		}

		const auto eit = c3d4Elems.constFind(ref.elemId);
		if (eit == c3d4Elems.constEnd()) {
			++local.invalidElem;
			continue;
		}

		const QVector<int> faceNodes = verifyC3d4FaceNodeIds(eit.value(), sideIndex0);
		if (faceNodes.size() != 3) {
			++local.invalidSide;
			continue;
		}

		GearNodeXYZ p1 = verifyNodeOrZero(nodePos, faceNodes[0]);
		GearNodeXYZ p2 = verifyNodeOrZero(nodePos, faceNodes[1]);
		GearNodeXYZ p3 = verifyNodeOrZero(nodePos, faceNodes[2]);
		if (!nodePos.contains(faceNodes[0]) || !nodePos.contains(faceNodes[1])
		    || !nodePos.contains(faceNodes[2])) {
			++local.invalidElem;
			continue;
		}

		vtkFaceNodeLists.append(faceNodes);

		for (int nid : faceNodes) {
			const GearNodeXYZ p = verifyNodeOrZero(nodePos, nid);
			verifyAccumPointStats(p, &hasBbox, bboxMin, bboxMax, &hasRadius, &radiusMin, &radiusMax,
			                      &hasZ, &zMin, &zMax);
		}

		GearNodeXYZ elemCenter{0, 0, 0};
		for (int nid : eit.value()) {
			const GearNodeXYZ p = verifyNodeOrZero(nodePos, nid);
			elemCenter.x += p.x;
			elemCenter.y += p.y;
			elemCenter.z += p.z;
		}
		elemCenter.x *= 0.25;
		elemCenter.y *= 0.25;
		elemCenter.z *= 0.25;

		GearNodeXYZ faceCenter{
		    (p1.x + p2.x + p3.x) / 3.0,
		    (p1.y + p2.y + p3.y) / 3.0,
		    (p1.z + p2.z + p3.z) / 3.0,
		};

		const double ax = p2.x - p1.x;
		const double ay = p2.y - p1.y;
		const double az = p2.z - p1.z;
		const double bx = p3.x - p1.x;
		const double by = p3.y - p1.y;
		const double bz = p3.z - p1.z;
		const double nx = ay * bz - az * by;
		const double ny = az * bx - ax * bz;
		const double nz = ax * by - ay * bx;

		const double dx = faceCenter.x - elemCenter.x;
		const double dy = faceCenter.y - elemCenter.y;
		const double dz = faceCenter.z - elemCenter.z;
		const double dot = nx * dx + ny * dy + nz * dz;
		if (dot > 0.0)
			++local.outwardCount;
		else if (dot < 0.0)
			++local.inwardCount;
	}

	const bool exportToothSurfVtk =
	    kExportToothSurfDebugVtk && surfaceName.contains(QStringLiteral("TOOTH_SURF"), Qt::CaseInsensitive);
	if (exportToothSurfVtk) {
		const QString vtkPath =
		    QFileInfo(jobInpPath).absolutePath() + QLatin1Char('/')
		    + QStringLiteral("debug_inp_") + surfaceName + QStringLiteral(".vtk");
		if (!mcWriteDebugSurfaceTrianglesVtk(vtkPath, vtkFaceNodeLists, nodePos))
			return fail(QStringLiteral("verifyCcxElementSurfaceFaces: failed to write VTK"));
		local.vtkPath = vtkPath;
	}
	local.hasBbox        = hasBbox;
	local.hasRadiusRange = hasRadius;
	local.hasZRange      = hasZ;
	if (hasBbox) {
		local.bboxMin[0] = bboxMin[0];
		local.bboxMin[1] = bboxMin[1];
		local.bboxMin[2] = bboxMin[2];
		local.bboxMax[0] = bboxMax[0];
		local.bboxMax[1] = bboxMax[1];
		local.bboxMax[2] = bboxMax[2];
	}
	if (hasRadius) {
		local.radiusMin = radiusMin;
		local.radiusMax = radiusMax;
	}
	if (hasZ) {
		local.zMin = zMin;
		local.zMax = zMax;
	}

	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[SurfaceVerify] %1 faces=%2 outward=%3 inward=%4 invalidElem=%5 invalidSide=%6")
	                              .arg(local.surfaceName)
	                              .arg(local.faceCount)
	                              .arg(local.outwardCount)
	                              .arg(local.inwardCount)
	                              .arg(local.invalidElem)
	                              .arg(local.invalidSide);
	if (local.hasBbox) {
		GEAR_OPT_DEBUG_NOQUOTE
		    << QStringLiteral("[SurfaceVerify] %1 bbox=(%2,%3,%4)-(%5,%6,%7)")
		           .arg(local.surfaceName)
		           .arg(local.bboxMin[0], 0, 'g', 8)
		           .arg(local.bboxMin[1], 0, 'g', 8)
		           .arg(local.bboxMin[2], 0, 'g', 8)
		           .arg(local.bboxMax[0], 0, 'g', 8)
		           .arg(local.bboxMax[1], 0, 'g', 8)
		           .arg(local.bboxMax[2], 0, 'g', 8);
	}
	if (local.hasRadiusRange) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[SurfaceVerify] %1 radiusRange=[%2,%3]")
		                              .arg(local.surfaceName)
		                              .arg(local.radiusMin, 0, 'g', 8)
		                              .arg(local.radiusMax, 0, 'g', 8);
	}
	if (local.hasZRange) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[SurfaceVerify] %1 zRange=[%2,%3]")
		                              .arg(local.surfaceName)
		                              .arg(local.zMin, 0, 'g', 8)
		                              .arg(local.zMax, 0, 'g', 8);
	}
	if (exportToothSurfVtk) {
	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[SurfaceVerify] %1 vtk=%2 triangles=%3")
		                              .arg(local.surfaceName)
		                              .arg(local.vtkPath)
		                              .arg(vtkFaceNodeLists.size());
	}

	if (report)
		*report = local;
	return true;
}

} // namespace GearAutoOpt
