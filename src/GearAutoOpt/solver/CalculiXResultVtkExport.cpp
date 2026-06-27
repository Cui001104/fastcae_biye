#include "CalculiXResultVtkExport.h"
#include "GearAutoOpt/data/GearOptLog.h"
#include "CCXResultParser.h"
#include "MeshConverter.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QHash>
#include <QSet>
#include <QTextStream>
#include <QVector>

#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>

#include <algorithm>
#include <cmath>

namespace GearAutoOpt {

namespace {

struct SolidElem {
	int         id     = 0;
	int         vtkType = VTK_TETRA;
	QVector<int> nodes;
};

struct MeshSolid {
	QHash<int, NodeXYZ> nodes;
	QVector<SolidElem>  elems;
};

QString upperNoSpace(const QString& line)
{
	QString u = line.toUpper();
	u.remove(QLatin1Char(' '));
	return u;
}

QString elementTypeToken(const QString& upperNoSpace)
{
	const int idx = upperNoSpace.indexOf(QLatin1String("TYPE="));
	if (idx < 0)
		return QString();
	int j = idx + 5;
	while (j < upperNoSpace.size()
	       && (upperNoSpace[j] == QLatin1Char('=') || upperNoSpace[j].isSpace()))
		++j;
	int k = j;
	while (k < upperNoSpace.size() && upperNoSpace[k] != QLatin1Char(','))
		++k;
	return upperNoSpace.mid(j, k - j);
}

bool isSurfaceElementHeader(const QString& upperNoSpace)
{
	return upperNoSpace.contains(QLatin1String("TYPE=CPS"))
	       || upperNoSpace.contains(QLatin1String("TYPE=S3"))
	       || upperNoSpace.contains(QLatin1String("TYPE=S4"))
	       || upperNoSpace.contains(QLatin1String("TYPE=S6"))
	       || upperNoSpace.contains(QLatin1String("TYPE=S8"));
}

bool parseMeshInpSolids(const QString& meshInpPath, MeshSolid* out, QString* err)
{
	if (!out)
		return false;
	out->nodes.clear();
	out->elems.clear();

	QFile f(meshInpPath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
		if (err)
			*err = QStringLiteral("cannot open mesh.inp");
		return false;
	}

	bool inNode = false;
	while (!f.atEnd()) {
		const QString line = f.readLine().trimmed();
		if (line.isEmpty() || line.startsWith(QStringLiteral("**")))
			continue;
		if (line.startsWith(QLatin1Char('*'), Qt::CaseInsensitive)) {
			inNode = line.startsWith(QStringLiteral("*NODE"), Qt::CaseInsensitive)
			         && !line.startsWith(QStringLiteral("*NODE PRINT"), Qt::CaseInsensitive)
			         && !line.startsWith(QStringLiteral("*NODE FILE"), Qt::CaseInsensitive);
			continue;
		}
		if (!inNode)
			continue;
		const QStringList parts = line.split(QLatin1Char(','));
		if (parts.size() < 4)
			continue;
		bool ok = false;
		const int id = parts[0].trimmed().toInt(&ok);
		if (!ok)
			continue;
		NodeXYZ p{parts[1].trimmed().toDouble(), parts[2].trimmed().toDouble(),
		          parts[3].trimmed().toDouble()};
		out->nodes.insert(id, p);
	}

	QStringList lines;
	{
		f.seek(0);
		QTextStream ts(&f);
		while (!ts.atEnd())
			lines << ts.readLine();
	}

	int i = 0;
	while (i < lines.size()) {
		const QString tline = lines[i].trimmed();
		if (tline.isEmpty() || tline.startsWith(QStringLiteral("**"))) {
			++i;
			continue;
		}
		const QString upper = upperNoSpace(tline);
		if (!upper.startsWith(QLatin1String("*ELEMENT")) || isSurfaceElementHeader(upper)) {
			++i;
			continue;
		}

		const QString typeTok = elementTypeToken(upper);
		int           vtkType = 0;
		int           nnodes  = 0;
		if (typeTok == QLatin1String("C3D4")) {
			vtkType = VTK_TETRA;
			nnodes  = 4;
		} else if (typeTok == QLatin1String("C3D8")) {
			vtkType = VTK_HEXAHEDRON;
			nnodes  = 8;
		} else {
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
			if (toks.size() < 1 + nnodes) {
				++j;
				continue;
			}
			bool okId = false;
			const int eid = toks[0].toInt(&okId);
			if (!okId) {
				++j;
				continue;
			}
			SolidElem el;
			el.id      = eid;
			el.vtkType = vtkType;
			el.nodes.reserve(nnodes);
			for (int k = 1; k <= nnodes; ++k) {
				bool okN = false;
				const int nid = toks[k].toInt(&okN);
				if (!okN) {
					el.nodes.clear();
					break;
				}
				el.nodes.append(nid);
			}
			if (el.nodes.size() == nnodes)
				out->elems.append(el);
			++j;
		}
		i = j;
	}

	if (out->nodes.isEmpty() || out->elems.isEmpty()) {
		if (err)
			*err = QStringLiteral("no C3D4/C3D8 volume elements in mesh.inp");
		return false;
	}
	return true;
}

struct StressAcc {
	double sxx = 0, syy = 0, szz = 0, sxy = 0, sxz = 0, syz = 0;
	int    n   = 0;
	void add(const StressSample& s)
	{
		sxx += s.sxx;
		syy += s.syy;
		szz += s.szz;
		sxy += s.sxy;
		sxz += s.sxz;
		syz += s.syz;
		++n;
	}
	StressSample mean() const
	{
		StressSample s;
		if (n <= 0)
			return s;
		const double inv = 1.0 / static_cast<double>(n);
		s.sxx = sxx * inv;
		s.syy = syy * inv;
		s.szz = szz * inv;
		s.sxy = sxy * inv;
		s.sxz = sxz * inv;
		s.syz = syz * inv;
		return s;
	}
};

QHash<int, StressSample> buildElementStressFromDat(const QString& datPath)
{
	QHash<int, StressAcc> acc;
	const QStringList elsets = {QStringLiteral("GEAR1"), QStringLiteral("GEAR2"),
	                            QStringLiteral("VOLUME1"), QStringLiteral("BEAM")};
	for (const QString& el : elsets) {
		const QVector<StressSample> samples = parseDatSamples(datPath, el);
		for (const StressSample& s : samples)
			acc[s.elemId].add(s);
	}

	QHash<int, StressSample> out;
	for (auto it = acc.constBegin(); it != acc.constEnd(); ++it) {
		if (it.value().n > 0)
			out.insert(it.key(), it.value().mean());
	}
	return out;
}

void fillNodeStressFromElements(const MeshSolid& mesh,
                                const QHash<int, StressSample>& elemStress,
                                QHash<int, NodeStress>*         nodeStress)
{
	if (!nodeStress)
		return;
	struct NAcc {
		double sxx = 0, syy = 0, szz = 0, sxy = 0, syz = 0, szx = 0;
		int    n = 0;
	};
	QHash<int, NAcc> acc;
	for (const SolidElem& el : mesh.elems) {
		const auto it = elemStress.constFind(el.id);
		if (it == elemStress.constEnd())
			continue;
		const StressSample& s = it.value();
		for (int nid : el.nodes) {
			NAcc& a = acc[nid];
			a.sxx += s.sxx;
			a.syy += s.syy;
			a.szz += s.szz;
			a.sxy += s.sxy;
			a.syz += s.syz;
			a.szx += s.sxz;
			++a.n;
		}
	}
	for (auto it = acc.constBegin(); it != acc.constEnd(); ++it) {
		if (it.value().n <= 0)
			continue;
		const double inv = 1.0 / static_cast<double>(it.value().n);
		NodeStress ns;
		ns.sxx = it.value().sxx * inv;
		ns.syy = it.value().syy * inv;
		ns.szz = it.value().szz * inv;
		ns.sxy = it.value().sxy * inv;
		ns.syz = it.value().syz * inv;
		ns.szx = it.value().szx * inv;
		nodeStress->insert(it.key(), ns);
	}
}

vtkSmartPointer<vtkDoubleArray> makeScalarArray(const char* name, vtkIdType n)
{
	vtkSmartPointer<vtkDoubleArray> a = vtkSmartPointer<vtkDoubleArray>::New();
	a->SetName(name);
	a->SetNumberOfComponents(1);
	a->SetNumberOfTuples(n);
	a->Fill(0.0);
	return a;
}

} // anonymous namespace

bool exportCalculixResultToVTK(const QString& meshInpPath,
                               const QString& frdPath,
                               const QString& datPath,
                               const QString& outVtuPath,
                               QString*       errorMsg)
{
	MeshSolid mesh;
	if (!parseMeshInpSolids(meshInpPath, &mesh, errorMsg))
		return false;

	if (!QFile::exists(frdPath)) {
		if (errorMsg)
			*errorMsg = QStringLiteral("job.frd not found");
		return false;
	}

	const FrdResult frd = parseFrd(frdPath);

	QSet<int> usedNodes;
	for (const SolidElem& el : mesh.elems) {
		for (int nid : el.nodes)
			usedNodes.insert(nid);
	}

	QList<int> nodeIds;
	nodeIds.reserve(usedNodes.size());
	for (int nid : usedNodes)
		nodeIds.append(nid);
	std::sort(nodeIds.begin(), nodeIds.end());

	QHash<int, vtkIdType> ccxToVtkPt;
	ccxToVtkPt.reserve(nodeIds.size());

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->SetNumberOfPoints(static_cast<vtkIdType>(nodeIds.size()));

	for (int i = 0; i < nodeIds.size(); ++i) {
		const int nid = nodeIds[i];
		const auto it = mesh.nodes.constFind(nid);
		if (it == mesh.nodes.constEnd()) {
			if (errorMsg)
				*errorMsg = QStringLiteral("mesh node %1 missing in *NODE").arg(nid);
			return false;
		}
		const NodeXYZ& p = it.value();
		const vtkIdType ptIdx = points->InsertNextPoint(p.x, p.y, p.z);
		ccxToVtkPt.insert(nid, ptIdx);
	}

	QVector<int> vtkCellElemId;
	vtkCellElemId.reserve(mesh.elems.size());

	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
	grid->SetPoints(points);
	grid->Allocate(static_cast<vtkIdType>(mesh.elems.size()));

	for (const SolidElem& el : mesh.elems) {
		vtkIdType ids[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		if (el.nodes.size() > 8) {
			if (errorMsg)
				*errorMsg = QStringLiteral("unsupported element node count");
			return false;
		}
		for (int k = 0; k < el.nodes.size(); ++k) {
			const auto pit = ccxToVtkPt.constFind(el.nodes[k]);
			if (pit == ccxToVtkPt.constEnd()) {
				if (errorMsg)
					*errorMsg = QStringLiteral("element %1 references unknown node %2")
					                .arg(el.id)
					                .arg(el.nodes[k]);
				return false;
			}
			ids[k] = pit.value();
		}
		grid->InsertNextCell(el.vtkType, el.nodes.size(), ids);
		vtkCellElemId.append(el.id);
	}

	const vtkIdType nPts  = points->GetNumberOfPoints();
	const vtkIdType nCell = grid->GetNumberOfCells();

	vtkSmartPointer<vtkDoubleArray> disp = vtkSmartPointer<vtkDoubleArray>::New();
	disp->SetName("displacement");
	disp->SetNumberOfComponents(3);
	disp->SetNumberOfTuples(nPts);
	disp->Fill(0.0);

	auto sxx = makeScalarArray("SXX", nPts);
	auto syy = makeScalarArray("SYY", nPts);
	auto szz = makeScalarArray("SZZ", nPts);
	auto sxy = makeScalarArray("SXY", nPts);
	auto syz = makeScalarArray("SYZ", nPts);
	auto szx = makeScalarArray("SZX", nPts);
	auto vm  = makeScalarArray("vonMises", nPts);

	QHash<int, NodeStress> nodeStress = frd.stresses;
	if (nodeStress.isEmpty() && QFile::exists(datPath)) {
		const QHash<int, StressSample> elemStress = buildElementStressFromDat(datPath);
		fillNodeStressFromElements(mesh, elemStress, &nodeStress);
	}

	for (int i = 0; i < nodeIds.size(); ++i) {
		const int nid = nodeIds[i];
		const vtkIdType pt = ccxToVtkPt.value(nid);

		const auto dit = frd.displacements.constFind(nid);
		if (dit != frd.displacements.constEnd()) {
			disp->SetTuple3(pt, dit.value().ux, dit.value().uy, dit.value().uz);
		}

		const auto sit = nodeStress.constFind(nid);
		if (sit != nodeStress.constEnd()) {
			const NodeStress& s = sit.value();
			sxx->SetValue(pt, s.sxx);
			syy->SetValue(pt, s.syy);
			szz->SetValue(pt, s.szz);
			sxy->SetValue(pt, s.sxy);
			syz->SetValue(pt, s.syz);
			szx->SetValue(pt, s.szx);
			vm->SetValue(pt, s.vonMises());
		}
	}

	grid->GetPointData()->AddArray(disp);
	grid->GetPointData()->SetActiveVectors("displacement");
	grid->GetPointData()->AddArray(sxx);
	grid->GetPointData()->AddArray(syy);
	grid->GetPointData()->AddArray(szz);
	grid->GetPointData()->AddArray(sxy);
	grid->GetPointData()->AddArray(syz);
	grid->GetPointData()->AddArray(szx);
	grid->GetPointData()->AddArray(vm);
	grid->GetPointData()->SetActiveScalars("vonMises");

	{
		auto contactSurf = makeScalarArray("contact_surface", nPts);
		QSet<int>        surfNodes;
		QSet<int>        tmp;
		if (loadNsetNodeIds(meshInpPath, QStringLiteral("GEAR1_TOOTH_SURF"), &tmp)) {
			for (int nid : tmp)
				surfNodes.insert(nid);
			tmp.clear();
		}
		if (loadNsetNodeIds(meshInpPath, QStringLiteral("GEAR2_TOOTH_SURF"), &tmp)) {
			for (int nid : tmp)
				surfNodes.insert(nid);
		}
		for (int nid : surfNodes) {
			const auto pit = ccxToVtkPt.constFind(nid);
			if (pit != ccxToVtkPt.constEnd())
				contactSurf->SetValue(pit.value(), 1.0);
		}
		grid->GetPointData()->AddArray(contactSurf);
	}

	if (QFile::exists(datPath)) {
		const QHash<int, StressSample> elemStress = buildElementStressFromDat(datPath);
		auto cSxx = makeScalarArray("SXX", nCell);
		auto cSyy = makeScalarArray("SYY", nCell);
		auto cSzz = makeScalarArray("SZZ", nCell);
		auto cSxy = makeScalarArray("SXY", nCell);
		auto cSxz = makeScalarArray("SXZ", nCell);
		auto cSyz = makeScalarArray("SYZ", nCell);
		auto cVm  = makeScalarArray("vonMises", nCell);

		for (vtkIdType ci = 0; ci < nCell; ++ci) {
			const auto it = elemStress.constFind(vtkCellElemId[static_cast<int>(ci)]);
			if (it == elemStress.constEnd())
				continue;
			const StressSample& s = it.value();
			cSxx->SetValue(ci, s.sxx);
			cSyy->SetValue(ci, s.syy);
			cSzz->SetValue(ci, s.szz);
			cSxy->SetValue(ci, s.sxy);
			cSxz->SetValue(ci, s.sxz);
			cSyz->SetValue(ci, s.syz);
			cVm->SetValue(ci, s.vonMises());
		}

		grid->GetCellData()->AddArray(cSxx);
		grid->GetCellData()->AddArray(cSyy);
		grid->GetCellData()->AddArray(cSzz);
		grid->GetCellData()->AddArray(cSxy);
		grid->GetCellData()->AddArray(cSxz);
		grid->GetCellData()->AddArray(cSyz);
		grid->GetCellData()->AddArray(cVm);
	}

	vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer =
	    vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
	writer->SetFileName(QDir::toNativeSeparators(outVtuPath).toUtf8().constData());
	writer->SetInputData(grid);
	writer->SetDataModeToBinary();
	if (writer->Write() == 0) {
		if (errorMsg)
			*errorMsg = QStringLiteral("vtkXMLUnstructuredGridWriter failed");
		return false;
	}

	GEAR_OPT_DEBUG_NOQUOTE << QStringLiteral("[GearOpt][VTK] exported %1: points=%2 cells=%3 (C3D4/C3D8)")
	                              .arg(outVtuPath)
	                              .arg(nPts)
	                              .arg(nCell);
	return true;
}

} // namespace GearAutoOpt
