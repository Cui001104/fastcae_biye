#include "MeshConverter.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QTextStream>

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

int countNsetNodes(const QString& inpPath, const QString& nsetName) {
	bool ok = false;
	const QStringList lines = readLinesKeepEol(inpPath, &ok);
	if (!ok) return -1;
	const QString want = nsetName.toUpper();

	QSet<int> ids;
	int i = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
		if (upperNoSpace.startsWith(QLatin1String("*NSET"))) {
			const QString name = extractSetName(upperNoSpace, QLatin1String("NSET="));
			if (name == want) {
				i = collectIdColumn(lines, i + 1, &ids);
				continue;
			}
		}
		++i;
	}
	return ids.isEmpty() ? -1 : ids.size();
}

int countElsetNodes(const QString& inpPath, const QString& elsetName) {
	bool ok = false;
	const QStringList lines = readLinesKeepEol(inpPath, &ok);
	if (!ok) return -1;
	const QString want = elsetName.toUpper();

	// 两步：先收集 elset 中的 element id；然后扫所有 *ELEMENT 块捕获节点 id
	QSet<int> elemIds;
	{
		int i = 0;
		while (i < lines.size()) {
			const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
			if (upperNoSpace.startsWith(QLatin1String("*ELSET"))) {
				const QString name = extractSetName(upperNoSpace, QLatin1String("ELSET="));
				if (name == want) {
					i = collectIdColumn(lines, i + 1, &elemIds);
					continue;
				}
			}
			// *ELEMENT,...,ELSET=NAME 的内联 elset 也算（gmsh 默认就是这样）
			if (upperNoSpace.startsWith(QLatin1String("*ELEMENT"))) {
				if (extractElsetName(upperNoSpace) == want) {
					// 收集 element id（第 0 列）
					int j = i + 1;
					while (j < lines.size() && !lines[j].trimmed().startsWith('*')) {
						const QString trimmed = lines[j].trimmed();
						if (!trimmed.isEmpty()) {
							const QStringList toks = trimmed.split(
								QRegExp("[,\\s]+"), QString::SkipEmptyParts);
							if (!toks.isEmpty()) {
								bool ok2 = false;
								int v = toks.first().toInt(&ok2);
								if (ok2) elemIds.insert(v);
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
	}
	if (elemIds.isEmpty()) return -1;

	QSet<int> nodeIds;
	int i = 0;
	while (i < lines.size()) {
		const QString upperNoSpace = lines[i].toUpper().trimmed().remove(' ');
		if (!upperNoSpace.startsWith(QLatin1String("*ELEMENT"))) {
			++i;
			continue;
		}
		// 扫元素块
		int j = i + 1;
		while (j < lines.size() && !lines[j].trimmed().startsWith('*')) {
			const QString trimmed = lines[j].trimmed();
			if (!trimmed.isEmpty()) {
				const QStringList toks = trimmed.split(QRegExp("[,\\s]+"),
				                                       QString::SkipEmptyParts);
				if (!toks.isEmpty()) {
					bool ok2 = false;
					int eid = toks.first().toInt(&ok2);
					if (ok2 && elemIds.contains(eid)) {
						for (int k = 1; k < toks.size(); ++k) {
							bool ok3 = false;
							int n = toks[k].toInt(&ok3);
							if (ok3) nodeIds.insert(n);
						}
					}
				}
			}
			++j;
		}
		i = j;
	}
	return nodeIds.isEmpty() ? -1 : nodeIds.size();
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

} // namespace GearAutoOpt
