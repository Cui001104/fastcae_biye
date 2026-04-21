#include "GmshThread.h"
#include "PythonModule/PyAgent.h"
#include "GmshPy.h"
#include "Geometry/geometryData.h"
#include "Geometry/geometrySet.h"
#include "Geometry/geometryParaGear.h"
#include <QApplication>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <TopExp.hxx>
#include <QDir>
#include <QMessageBox>
#include <QTextCodec>
#include <vtkDataSetReader.h>
#include <vtkCell.h>
#include <vtkCell3D.h>
#include "MeshData/meshKernal.h"
#include "MeshData/meshSet.h"
#include "MeshData/meshSingleton.h"
#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkUnstructuredGrid.h>
#include <QDebug>
#include <algorithm>
#include <cmath>
#include <limits>
#include "MainWindow/MainWindow.h"
#include "MainWindow/SubWindowManager.h"
#include "MainWidgets/preWindow.h"
#include "GmshModule.h"
#include "ModuleBase/processBar.h"
#include "DataProperty/ParameterString.h"
#include "DataProperty/ParameterInt.h"
#include "DataProperty/ParameterDouble.h"
//#include "IO/TemplateReplacer.h"
#include "MeshReader.h"
#include "Geometry/GeoCommon.h"

#include "GmshSettingData.h"
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <string>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <vtkUnstructuredGrid.h>
#include <vtkPoints.h>
#include <vtkIdList.h>
#include <QHash>
#include <QQueue>
#include <QSet>
#include "GeometryCommand/GeoCommandCommon.h"
#include "FluidMeshPreProcess.h"
#include "GmshScriptWriter.h"

namespace Gmsh
{
namespace
{
	struct GearBoreSpec
	{
		QString name{};
		double holeRadius{0.0};
	};

	QVector<GearBoreSpec> collectGearBoreSpecs()
	{
		QVector<GearBoreSpec> specs;
		auto *geoData = Geometry::GeometryData::getInstance();
		if (geoData == nullptr)
			return specs;

		for (int i = 0; i < geoData->getGeometrySetCount(); ++i)
		{
			auto *set = geoData->getGeometrySetAt(i);
			if (set == nullptr)
				continue;

			auto *gearPara = dynamic_cast<Geometry::GeometryParaGear *>(set->getParameter());
			if (gearPara == nullptr)
				continue;

			const double module = gearPara->getModule();
			const double holeRadius1 = 0.2 * module * static_cast<double>(gearPara->getNumberOfTeeth());
			if (holeRadius1 > 0.0)
			{
				GearBoreSpec spec;
				spec.name = QStringLiteral("gear1_bore_nodes");
				spec.holeRadius = holeRadius1;
				specs.append(spec);
			}

			const double holeRadius2 = 0.2 * module * static_cast<double>(gearPara->getNumberOfSecondTeeth());
			if (holeRadius2 > 0.0)
			{
				GearBoreSpec spec;
				spec.name = QStringLiteral("gear2_bore_nodes");
				spec.holeRadius = holeRadius2;
				specs.append(spec);
			}
			break;
		}
		return specs;
	}

	int detectAxialAxis(vtkDataSet *dataset)
	{
		if (dataset == nullptr)
			return 2;
		double bounds[6] = {0.0};
		dataset->GetBounds(bounds);
		int axis = 0;
		double minSpan = bounds[1] - bounds[0];
		for (int a = 1; a < 3; ++a)
		{
			const double span = bounds[2 * a + 1] - bounds[2 * a];
			if (span < minSpan)
			{
				minSpan = span;
				axis = a;
			}
		}
		return axis;
	}

	int detectSeparationAxis(vtkDataSet *dataset, int axialAxis)
	{
		double bounds[6] = {0.0};
		dataset->GetBounds(bounds);
		QVector<int> transverseAxes;
		for (int a = 0; a < 3; ++a)
		{
			if (a != axialAxis)
				transverseAxes.append(a);
		}
		if (transverseAxes.size() != 2)
			return (axialAxis + 1) % 3;

		const double span0 = bounds[2 * transverseAxes[0] + 1] - bounds[2 * transverseAxes[0]];
		const double span1 = bounds[2 * transverseAxes[1] + 1] - bounds[2 * transverseAxes[1]];
		return span0 >= span1 ? transverseAxes[0] : transverseAxes[1];
	}

	void computeGroupCenter(vtkDataSet *dataset, const QVector<int> &pointIds, double center[3])
	{
		center[0] = center[1] = center[2] = 0.0;
		if (dataset == nullptr || pointIds.isEmpty())
			return;

		double minCoord[3] = {
			(std::numeric_limits<double>::max)(),
			(std::numeric_limits<double>::max)(),
			(std::numeric_limits<double>::max)()};
		double maxCoord[3] = {
			-(std::numeric_limits<double>::max)(),
			-(std::numeric_limits<double>::max)(),
			-(std::numeric_limits<double>::max)()};

		double point[3] = {0.0, 0.0, 0.0};
		for (int pid : pointIds)
		{
			dataset->GetPoint(pid, point);
			for (int a = 0; a < 3; ++a)
			{
				minCoord[a] = qMin(minCoord[a], point[a]);
				maxCoord[a] = qMax(maxCoord[a], point[a]);
			}
		}

		for (int a = 0; a < 3; ++a)
			center[a] = 0.5 * (minCoord[a] + maxCoord[a]);
	}

	double radialDistanceToAxis(const double point[3], const double center[3], int axis)
	{
		const int a0 = (axis + 1) % 3;
		const int a1 = (axis + 2) % 3;
		const double d0 = point[a0] - center[a0];
		const double d1 = point[a1] - center[a1];
		return std::sqrt(d0 * d0 + d1 * d1);
	}

	QVector<QVector<int>> collectBodyPointGroups(vtkDataSet *dataset)
	{
		QVector<QVector<int>> groups;
		if (dataset == nullptr)
			return groups;

		const int nCells = dataset->GetNumberOfCells();
		const int nPoints = dataset->GetNumberOfPoints();
		if (nCells <= 0 || nPoints <= 0)
			return groups;

		QHash<int, QVector<int>> pointToCells;
		QVector<QVector<int>> cellPoints(nCells);
		for (int ci = 0; ci < nCells; ++ci)
		{
			vtkCell *cell = dataset->GetCell(ci);
			if (cell == nullptr)
				continue;
			vtkIdList *pids = cell->GetPointIds();
			if (pids == nullptr)
				continue;
			QVector<int> &pts = cellPoints[ci];
			pts.reserve(static_cast<int>(pids->GetNumberOfIds()));
			for (vtkIdType j = 0; j < pids->GetNumberOfIds(); ++j)
			{
				const int pid = static_cast<int>(pids->GetId(j));
				if (pid < 0 || pid >= nPoints)
					continue;
				pts.append(pid);
				pointToCells[pid].append(ci);
			}
		}

		QVector<char> visited(nCells, 0);
		for (int start = 0; start < nCells; ++start)
		{
			if (visited[start] || cellPoints[start].isEmpty())
				continue;

			QQueue<int> queue;
			QSet<int> bodyPointSet;
			queue.enqueue(start);
			visited[start] = 1;

			while (!queue.isEmpty())
			{
				const int current = queue.dequeue();
				const QVector<int> &pts = cellPoints[current];
				for (int pid : pts)
				{
					bodyPointSet.insert(pid);
					const QVector<int> neighbors = pointToCells.value(pid);
					for (int next : neighbors)
					{
						if (next < 0 || next >= nCells || visited[next] || cellPoints[next].isEmpty())
							continue;
						visited[next] = 1;
						queue.enqueue(next);
					}
				}
			}

			if (!bodyPointSet.isEmpty())
				groups.append(bodyPointSet.values().toVector());
		}

		return groups;
	}

	QList<int> selectBoreNodeIds1Based(vtkDataSet *dataset, const QVector<int> &pointIds, const double center[3], int axialAxis,
		double holeRadius, double tolerance)
	{
		QList<int> nodeIds;
		if (dataset == nullptr || pointIds.isEmpty() || holeRadius <= 0.0 || tolerance <= 0.0)
			return nodeIds;

		double point[3] = {0.0, 0.0, 0.0};
		for (int pid : pointIds)
		{
			dataset->GetPoint(pid, point);
			const double radius = radialDistanceToAxis(point, center, axialAxis);
			if (std::fabs(radius - holeRadius) <= tolerance)
				nodeIds.append(pid + 1);
		}

		std::sort(nodeIds.begin(), nodeIds.end());
		nodeIds.erase(std::unique(nodeIds.begin(), nodeIds.end()), nodeIds.end());
		return nodeIds;
	}

	QStringList createGearBoreNodeSets(MeshData::MeshData *meshData, MeshData::MeshKernal *kernal, vtkDataSet *dataset,
		double configuredTolerance, double fallbackSize)
	{
		QStringList createdNames;
		if (meshData == nullptr || kernal == nullptr || dataset == nullptr)
			return createdNames;

		const QVector<GearBoreSpec> specs = collectGearBoreSpecs();
		if (specs.isEmpty())
			return createdNames;

		const int axialAxis = detectAxialAxis(dataset);
		QVector<QVector<int>> bodyPointGroups = collectBodyPointGroups(dataset);
		if (bodyPointGroups.isEmpty())
			return createdNames;

		// ??????????????? gear1 / gear2 ??
		const int separationAxis = detectSeparationAxis(dataset, axialAxis);
		std::sort(bodyPointGroups.begin(), bodyPointGroups.end(), [&](const QVector<int> &a, const QVector<int> &b) {
			double centerA[3] = {0.0, 0.0, 0.0};
			double centerB[3] = {0.0, 0.0, 0.0};
			computeGroupCenter(dataset, a, centerA);
			computeGroupCenter(dataset, b, centerB);
			return centerA[separationAxis] < centerB[separationAxis];
		});

		const int bodyCount = qMin(specs.size(), bodyPointGroups.size());
		for (int i = 0; i < bodyCount; ++i)
		{
			double center[3] = {0.0, 0.0, 0.0};
			computeGroupCenter(dataset, bodyPointGroups[i], center);
			const double tolerance = configuredTolerance > 0.0
				? configuredTolerance
				: qMax(1.0e-6, fallbackSize > 0.0 ? 0.5 * fallbackSize : 0.01 * specs[i].holeRadius);
			const QList<int> nodeIds1Based = selectBoreNodeIds1Based(dataset, bodyPointGroups[i], center, axialAxis,
				specs[i].holeRadius, tolerance);
			if (nodeIds1Based.isEmpty())
				continue;

			auto *set = new MeshData::MeshSet(specs[i].name, MeshData::Node);
			const int kernelId = kernal->getID();
			for (int nodeId1Based : nodeIds1Based)
				set->appendMember(kernelId, nodeId1Based - 1);
			meshData->appendMeshSet(set);
			createdNames.append(specs[i].name);
		}
		return createdNames;
	}
}
	GmshThread::GmshThread(GUI::MainWindow *mw, MainWidget::PreWindow *pre, GmshModule *m, int dim)
		: _mainwindow(mw), _preWindow(pre), _gmshModule(m), _dim(dim)
	{

		connect(&_process, SIGNAL(readyReadStandardOutput()), this, SLOT(readProcessOutput()));
		connect(this, SIGNAL(sendMessage(QString)), mw, SIGNAL(printMessageToMessageWindow(QString)));
		connect(&_process, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(processFinished(int, QProcess::ExitStatus)));
		connect(this, SIGNAL(updateMeshActor()), _preWindow, SIGNAL(updateMeshActorSig()));

		_compounnd = new TopoDS_Compound;
		_fluidMeshProcess = new FluidMeshPreProcess();
		_scriptWriter = new GmshScriptWriter;
		//???????
		// 		QString exelPath = QCoreApplication::applicationDirPath();
		// 		const QString tempDir = exelPath + "/../temp/";
		// 		DataProperty::ParameterString* s = new DataProperty::ParameterString();
		// 		s->setDescribe("TempPath");
		// 		s->setValue(tempDir);
		// 		this->appendParameter(s);

		if (dim == 3) //????????
		{
			_smoothIteration = 20;
			// 			DataProperty::ParameterInt* sm = new DataProperty::ParameterInt();
			// 			sm->setDescribe("Smooth");
			// 			sm->setValue(20);
			// 			this->appendParameter(sm);
		}
	}

	GmshThread::~GmshThread()
	{
		if (_compounnd != nullptr)
			delete _compounnd;
		if (_fluidMeshProcess != nullptr)
			delete _fluidMeshProcess;
		if (_scriptWriter != nullptr)
			delete _scriptWriter;
	}
	void GmshThread::appendSolid(int id, int index)
	{
		_solidHash.insert(id, index);
	}

	void GmshThread::appendSurface(int geo, int face)
	{
		_surfaceHash.insert(geo, face);
	}

	void GmshThread::setElementType(QString t)
	{
		_elementType = t;
		int type = 0;
		if (t.toLower() == "quad")
			type = 1;
		else if (t.toLower() == "hex")
			type = 2;

		// 		DataProperty::ParameterInt* pi = new DataProperty::ParameterInt();
		// 		pi->setDescribe("ElementType");
		// 		pi->setValue(type);
		// 		this->appendParameter(pi);
	}

	void GmshThread::setElementOrder(int order)
	{
		_elementOrder = order;

		// 		DataProperty::ParameterInt* pi = new DataProperty::ParameterInt();
		// 		pi->setDescribe("ElementOrder");
		// 		pi->setValue(order);
		// 		this->appendParameter(pi);
	}

	void GmshThread::setMethod(int m)
	{
		_method = m;

		// 		DataProperty::ParameterInt* pi = new DataProperty::ParameterInt();
		// 		pi->setDescribe("Method");
		// 		pi->setValue(m);
		// 		this->appendParameter(pi);
	}

	void GmshThread::setSizeFactor(double f)
	{
		_sizeFactor = f;
		DataProperty::ParameterDouble *pd = new DataProperty::ParameterDouble();
		pd->setDescribe("SizeFactor");
		pd->setValue(f);
		this->appendParameter(pd);
	}

	void GmshThread::setMinSize(double min)
	{
		_minSize = min;
		// 		DataProperty::ParameterDouble* pd = new DataProperty::ParameterDouble();
		// 		pd->setDescribe("SizeMin");
		// 		pd->setValue(min);
		// 		this->appendParameter(pd);
	}

	void GmshThread::setMaxSize(double max)
	{
		_maxSize = max;
		// 		DataProperty::ParameterDouble* pd = new DataProperty::ParameterDouble();
		// 		pd->setDescribe("SizeMax");
		// 		pd->setValue(max);
		// 		this->appendParameter(pd);
	}

	void GmshThread::isCleanGeo(bool c)
	{
		_geoclean = c;
		// 		int gc = 0;
		// 		if (_geoclean)  gc = 1;

		// 		DataProperty::ParameterInt* pi = new DataProperty::ParameterInt();
		// 		pi->setDescribe("GeoClean");
		// 		pi->setValue(gc);
		// 		this->appendParameter(pi);
	}

	void GmshThread::setSmoothIteration(int it)
	{
		_smoothIteration = it;
		// 		DataProperty::ParameterInt* sm = new DataProperty::ParameterInt();
		// 		sm->setDescribe("Smooth");
		// 		sm->setValue(it);
		// 		this->appendParameter(sm);
	}

	void GmshThread::run()
	{
		this->mergeGeometry();
		this->initGmshEnvoirment();
		this->generate();
		// 		this->readMesh();
	}

	void GmshThread::mergeGeometry()
	{
		QString exelPath = QCoreApplication::applicationDirPath();
		const QString tempDir = exelPath + "/../temp/";
		QDir dir(tempDir);
		if (!dir.exists())
			dir.mkpath(tempDir);

		const QString meshfilename = exelPath + "/../temp/mesh.vtk";
		if (QFile::exists(meshfilename))
			QFile::remove(meshfilename);

		const QString geofilename = exelPath + "/../temp/geometry.brep";
		if (QFile::exists(geofilename))
			QFile::remove(geofilename);

		const QString gmshfilename = exelPath + "/../temp/gmsh.Geo";
		if (QFile::exists(gmshfilename))
			QFile::remove(gmshfilename);

		const QString tempPath = tempDir + QString("geometry.brep");

		if (_fluidMesh)
			_fluidMeshProcess->mergeFluidField(_compounnd, _solidHash);
		else if (_selectall)
			mergeAllGeo();
		else if (_selectvisible)
			mergeVisibleGeo();
		else
			mergeSelectGeo();
		QByteArray arr = tempPath.toLatin1();
		BRepTools::Write(*_compounnd, arr.data());
	}

	void GmshThread::readMesh()
	{
		MeshData::MeshData *data = MeshData::MeshData::getInstance();
		QString exelPath = QCoreApplication::applicationDirPath();
		const QString fileName = exelPath + "/../temp/mesh.vtk";

		QTextCodec *codec = QTextCodec::codecForName("GB18030");
		QByteArray ba = codec->fromUnicode(fileName);
		vtkSmartPointer<vtkDataSetReader> vtkReader = vtkSmartPointer<vtkDataSetReader>::New();
		vtkReader->SetFileName(ba);
		vtkReader->Update();
		vtkDataSet *dataset = vtkReader->GetOutput();
		if (dataset == nullptr)
			return;

		if (!_isSaveToKernal)
			emit writeToSolveFileSig(dataset);
		else
		{
			// 			vtkDataSet* vtkset = nullptr;
			// 			if (_cellTypeList.size() > 0)
			// 				vtkset = deleteSpecifiedCells(dataset);

			auto k = new MeshData::MeshKernal();
			k->setName(QString("Mesh_%1").arg(k->getID()));

			// 			if (vtkset != nullptr)
			// 				k->setMeshData(vtkset);
			// 			else
			k->setMeshData(dataset);

			data->appendMeshKernal(k);
			const QStringList boreSetNames = createGearBoreNodeSets(data, k, dataset, _boreNodeTolerance, _minSize);
			if (!boreSetNames.isEmpty())
				emit sendMessage(QString("Auto bore node sets created: %1").arg(boreSetNames.join(" / ")));
			// ???????????????FixedEnd / DriveEnd?? Z ?????????
			double bounds[6] = {0.0};
			dataset->GetBounds(bounds);
			const double zMin = bounds[4];
			const double zMax = bounds[5];
			const double zSpan = zMax - zMin;
			if (std::fabs(zSpan) > 1e-12)
			{
				const double tol = std::max(1e-6, zSpan * 1e-3);
				auto* fixedSet = new MeshData::MeshSet(QString("%1_FixedEnd").arg(k->getName()), MeshData::Node);
				auto* driveSet = new MeshData::MeshSet(QString("%1_DriveEnd").arg(k->getName()), MeshData::Node);
				double p[3] = {0.0, 0.0, 0.0};
				const int kid = k->getID();
				const int np = dataset->GetNumberOfPoints();
				for (int i = 0; i < np; ++i)
				{
					dataset->GetPoint(i, p);
					if (std::fabs(p[2] - zMin) <= tol) fixedSet->appendMember(kid, i);
					if (std::fabs(p[2] - zMax) <= tol) driveSet->appendMember(kid, i);
				}
				bool hasAutoSet = false;
				if (fixedSet->getAllCount() > 0) { data->appendMeshSet(fixedSet); hasAutoSet = true; }
				else delete fixedSet;
				if (driveSet->getAllCount() > 0) { data->appendMeshSet(driveSet); hasAutoSet = true; }
				else delete driveSet;
				if (hasAutoSet)
					emit sendMessage("Auto BC sets created: FixedEnd / DriveEnd");
			}

			// ???????????????????????Y ??????????????????????
			{
				const int np = dataset->GetNumberOfPoints();
				if (np > 0)
				{
					double bb[6] = {0.0};
					dataset->GetBounds(bb);
					const double yMidSplit = 0.5 * (bb[2] + bb[3]);

					double p[3] = {0.0, 0.0, 0.0};
					double c1x = 0.0, c1y = 0.0, c2x = 0.0, c2y = 0.0;
					int n1 = 0, n2 = 0;
					for (int i = 0; i < np; ++i)
					{
						dataset->GetPoint(i, p);
						if (p[1] <= yMidSplit) { c1x += p[0]; c1y += p[1]; ++n1; }
						else { c2x += p[0]; c2y += p[1]; ++n2; }
					}
					if (n1 > 0 && n2 > 0)
					{
						c1x /= n1; c1y /= n1;
						c2x /= n2; c2y /= n2;

						double r1Max = 0.0, r2Max = 0.0;
						for (int i = 0; i < np; ++i)
						{
							dataset->GetPoint(i, p);
							if (p[1] <= yMidSplit)
							{
								const double r = std::sqrt((p[0] - c1x) * (p[0] - c1x) + (p[1] - c1y) * (p[1] - c1y));
								if (r > r1Max) r1Max = r;
							}
							else
							{
								const double r = std::sqrt((p[0] - c2x) * (p[0] - c2x) + (p[1] - c2y) * (p[1] - c2y));
								if (r > r2Max) r2Max = r;
							}
						}

						const double outerBand = 0.90; // ????????/???/???????
						auto* gear1Face = new MeshData::BoundMeshSet();
						auto* gear2Face = new MeshData::BoundMeshSet();
						gear1Face->setType(MeshData::Element);
						gear2Face->setType(MeshData::Element);
						gear1Face->setName(QStringLiteral("set_gear1_surf"));
						gear2Face->setName(QStringLiteral("set_gear2_surf"));
						const int kid = k->getID();
						const int nc = dataset->GetNumberOfCells();
						QMap<QString, int> faceCount;
						QMap<QString, QPair<int, int>> firstFace; // key -> (cellIdx, localFaceIdx)
						for (int ci = 0; ci < nc; ++ci)
						{
							vtkCell* cell = dataset->GetCell(ci);
							auto* c3d = vtkCell3D::SafeDownCast(cell);
							if (c3d == nullptr) continue;
							const int faceNum = c3d->GetNumberOfFaces();
							for (int fi = 0; fi < faceNum; ++fi)
							{
								vtkCell* fcell = c3d->GetFace(fi);
								if (fcell == nullptr) continue;
								vtkIdList* pids = fcell->GetPointIds();
								if (pids == nullptr || pids->GetNumberOfIds() < 3) continue;
								QVector<int> face;
								for (vtkIdType j = 0; j < pids->GetNumberOfIds() && j < 4; ++j)
									face.append(static_cast<int>(pids->GetId(j)));
								if (face.size() == 3) face.append(face.last());
								if (face.size() != 4) continue;
								QVector<int> sortedFace = face;
								std::sort(sortedFace.begin(), sortedFace.end());
								const QString key = QStringLiteral("%1_%2_%3_%4")
									.arg(sortedFace[0]).arg(sortedFace[1]).arg(sortedFace[2]).arg(sortedFace[3]);
								faceCount[key] += 1;
								if (!firstFace.contains(key))
									firstFace.insert(key, qMakePair(ci, fi));
							}
						}

						QMap<int, QVector<int>> gear1CellFaces;
						QMap<int, QVector<int>> gear2CellFaces;
						for (auto it = faceCount.constBegin(); it != faceCount.constEnd(); ++it)
						{
							if (it.value() != 1 || !firstFace.contains(it.key())) continue; // ????
							const QPair<int, int> cf = firstFace.value(it.key());
							const int ci = cf.first;
							const int fi = cf.second;
							vtkCell* cell = dataset->GetCell(ci);
							auto* c3d = vtkCell3D::SafeDownCast(cell);
							if (c3d == nullptr) continue;
							vtkCell* fcell = c3d->GetFace(fi);
							if (fcell == nullptr || fcell->GetNumberOfPoints() <= 0) continue;
							double fx = 0.0, fy = 0.0;
							for (vtkIdType j = 0; j < fcell->GetNumberOfPoints(); ++j)
							{
								dataset->GetPoint(fcell->GetPointId(j), p);
								fx += p[0];
								fy += p[1];
							}
							fx /= fcell->GetNumberOfPoints();
							fy /= fcell->GetNumberOfPoints();
							if (fy <= yMidSplit)
							{
								const double r = std::sqrt((fx - c1x) * (fx - c1x) + (fy - c1y) * (fy - c1y));
								if (r >= outerBand * r1Max) gear1CellFaces[ci].append(fi);
							}
							else
							{
								const double r = std::sqrt((fx - c2x) * (fx - c2x) + (fy - c2y) * (fy - c2y));
								if (r >= outerBand * r2Max) gear2CellFaces[ci].append(fi);
							}
						}

						for (auto it = gear1CellFaces.constBegin(); it != gear1CellFaces.constEnd(); ++it)
							gear1Face->appendMember(kid, it.key());
						for (auto it = gear2CellFaces.constBegin(); it != gear2CellFaces.constEnd(); ++it)
							gear2Face->appendMember(kid, it.key());
						gear1Face->setCellFaces(gear1CellFaces);
						gear2Face->setCellFaces(gear2CellFaces);

						bool hasContactSet = false;
						if (!gear1CellFaces.isEmpty()) { data->appendMeshSet(gear1Face); hasContactSet = true; }
						else delete gear1Face;
						if (!gear2CellFaces.isEmpty()) { data->appendMeshSet(gear2Face); hasContactSet = true; }
						else delete gear2Face;
						if (hasContactSet)
							emit sendMessage("Auto surface sets created: set_gear1_surf / set_gear2_surf (boundary faces)");
					}
				}
			}

			if (!_fluidMesh)
				setGmshSettingData(k);

			emit _gmshModule->updateMeshTree();
			emit _gmshModule->updateSetTree();
			//			emit _preWindow->updateMeshActorSig();
			emit _gmshModule->updateActions();
			emit updateMeshActor();
		}
	}

	void GmshThread::initGmshEnvoirment()
	{
		// 		_pyAgent->backstageExec("Mesher.initGmsh()");
		// 		_pyAgent->backstageExec("Mesher.mergeGeoToGmsh()");
		QString exelPath = QCoreApplication::applicationDirPath();
		const QString tempDir = exelPath + "/../temp/";
		// const QString gmshDir = exelPath + "/gmsh/";
		QFile::remove(tempDir + "gmsh.Geo");

		// 		QFile::copy(gmshDir + "gmsh.Geo", tempDir + "gmsh.Geo");
		//
		// 		IO::TempalteReplacer replacer(this);
		// 		replacer.appendFile(tempDir + "gmsh.Geo");
		// 		replacer.replace();
		// 		if (_fluidMesh)
		// 			writeFluidMeshScript(tempDir);
		// 		else
		// 			appendScript(tempDir);
		_scriptWriter->setCompound(_compounnd);
		setGmshScriptData();

		if (_fluidMesh)
		{
			QList<int> curve = _fluidMeshProcess->getInerMember(1);
			QList<int> surface = _fluidMeshProcess->getInerMember(2);
			_scriptWriter->writeFluidMeshScript(tempDir, _solidHash, curve, surface);
		}
		else
			_scriptWriter->writeGmshScript(tempDir);
	}

	void GmshThread::generate()
	{
		// 		_pyAgent->backstageExec(QString("gmsh.model.mesh.generate(%1)").arg(_dim));
		// 		_pyAgent->backstageExec(QString("Mesher.finalizeGmsh()"));
		QString exelPath = QCoreApplication::applicationDirPath();
		const QString tempDir = exelPath + "/../temp/";
		// const QString gmshDir = exelPath + "/gmsh/";
		QString gmshexe = exelPath + "/gmsh";

		bool ok = false;
#ifdef Q_OS_WIN
		ok = QFile::exists(gmshexe + ".exe");
#endif
#ifdef Q_OS_LINUX
		ok = QFile::exists(gmshexe);
#endif
		if (!ok)
		{
			QMessageBox::warning(_mainwindow, QString(tr("Warning")), QString(tr("Gmsh is not exist !")));
			return;
		}

		// 		QString oldDir = QDir::currentPath();
		// 		QDir::setCurrent(gmshDir);

		QString startProcess = QString("%1 %2 -format vtk -bin -o %3 -%4").arg(gmshexe).arg(tempDir + "gmsh.Geo").arg(tempDir + "mesh.vtk").arg(_dim);

		if (gmshexe.contains(" "))
			startProcess = QString("\"%1\"").arg(startProcess);
		qDebug() << startProcess;

		_process.start(startProcess);
	}

	void GmshThread::processFinished(int exitCode, QProcess::ExitStatus exitStatus)
	{
		Q_UNUSED(exitCode)
		switch (exitStatus)
		{
		case QProcess::NormalExit:
			emit sendMessage("************************************");
			emit sendMessage("********* Gmsh finished ************");
			emit sendMessage("************************************");

			break;
		case QProcess::CrashExit:
			emit sendMessage("************************************");
			emit sendMessage("********* Gmsh Crashed ************");
			emit sendMessage("************************************");
			break;
		default:
			emit sendMessage("***********************************");
			emit sendMessage("********* Gmsh Stopped ************");
			emit sendMessage("***********************************");
			break;
		}

		readMesh();
		// 		if (this->isRunning())
		// 			this->wait();
		emit threadFinished(this);
		Py::PythonAgent::getInstance()->unLock();
	}

	void GmshThread::readProcessOutput()
	{
		QString outputBuffer = QString::fromLocal8Bit(_process.readAll());

		emit sendMessage(outputBuffer);
	}

	void GmshThread::stop()
	{
		_process.kill();
	}

	void GmshThread::setPara(GMshPara *para)
	{
		this->setSolid(para->_solidHash);
		this->setSurface(para->_surfaceHash);

		if (_dim == 2)
			this->setSmoothIteration(para->_smoothIteration);
		this->setElementType(para->_elementType);
		this->setElementOrder(para->_elementOrder);
		this->setMethod(para->_method);
		this->setMinSize(para->_minSize);
		this->setMaxSize(para->_maxSize);
		this->setSizeFactor(para->_sizeFactor);
		this->isCleanGeo(para->_geoclean);
		this->setGridCoplanar(para->_isGridCoplanar);
		this->setSizeAtPoint(para->_sizeAtPoints);
		this->setSizeFields(para->_sizeFields);
		// this->setPhysicals(para->_physicals);
		this->setSelectedAll(para->_selectall);
		this->setSelectedVisible(para->_selectvisible);
		this->setBoreNodeTolerance(para->_boreNodeTolerance);
		this->setMeshID(para->_meshID);
		this->setFluidMesh(para->_fluidMesh);
		// this->setCellTypeList(para->_cells);
		//	this->setFluidField(para->_fluidField);

		_fluidMeshProcess->setFluidField(para->_fluidField);

		// this->setPhysicals(para->_physicals);

		//_scriptWriter->setFluidField(para->_fluidField);
	}

	void GmshThread::setSolid(QMultiHash<int, int> s)
	{
		_solidHash = s;
	}

	void GmshThread::setSurface(QMultiHash<int, int> s)
	{
		_surfaceHash = s;
	}

	// 	void GmshThread::physicalsGroup(QTextStream* out)
	// 	{
	// 		if (_physicals.isEmpty())
	// 			return;
	//
	// 		QMultiHash<QString,int> points{};
	// 		QMultiHash<QString,int> curves{};
	// 		QMultiHash<QString,int> surfaces{};
	// 		QMultiHash<QString,int> solids{};
	//
	// 		QStringList glist = _physicals.split(";");
	// 		for (QString g : glist)
	// 		{
	// 			QStringList plist = g.split(",");
	// 			if (plist.size() != 4)
	// 				continue;
	//
	// 			bool b = false;
	// 			int type = plist.at(0).toInt(&b);
	// 			if (!b) continue;
	// 			if (type <= 0 || type > 4) continue;
	// 			int set = plist.at(2).toInt(&b);
	// 			if (!b) continue;
	// 			int d = plist.at(3).toInt(&b);
	// 			if (!b) continue;
	//
	// 			int index = getShhapeIndexInCompound(set, d, type);
	// 			if (index == 0) continue;
	//
	// 			switch (type)
	// 			{
	// 			case 1:
	// 				points.insert(plist.at(1), index);
	// 				break;
	// 			case 2:
	// 				curves.insert(plist.at(1), index);
	// 				break;
	// 			case 3:
	// 				surfaces.insert(plist.at(1), index);
	// 				break;
	// 			case 4:
	// 				solids.insert(plist.at(1), index);
	// 				break;
	// 			}
	// 		}
	//
	// 		if (points.size() > 0)
	// 			physicalsScript(out, "Point", points);
	// 		if (curves.size() > 0)
	// 			physicalsScript(out, "Curve", curves);
	// 		if (surfaces.size() > 0)
	// 			physicalsScript(out, "Surface", surfaces);
	// 		if (solids.size() > 0)
	// 			physicalsScript(out, "Volume", solids);
	//
	//
	// 	}
	//
	// 	void GmshThread::physicalsScript(QTextStream* out, QString type, QMultiHash<QString, int> pHash)
	// 	{
	// 		QList<QString> namelist = pHash.uniqueKeys();
	// 		for (QString name : namelist)
	// 		{
	// 			QList<int> indexlist = pHash.values(name);
	// 			QStringList list{};
	// 			for (int index : indexlist)
	// 			{
	// 				list.append(QString::number(index));
	// 			}
	// 			QString slist = list.join(",");
	//
	// 			*out << "//+" << endl;
	// 			*out << "Physical " << type << "(\"" << name << "\") = {" << slist << "};" << endl;
	// 		}
	// 	}

	void GmshThread::setGridCoplanar(bool gc)
	{
		_isGridCoplanar = gc;
	}

	void GmshThread::setSizeAtPoint(QString ps)
	{
		_sizeAtPoints = ps;
	}

	void GmshThread::setSizeFields(QString fs)
	{
		_sizeFields = fs;
	}

	void GmshThread::setBoreNodeTolerance(double tol)
	{
		_boreNodeTolerance = tol;
	}

	void GmshThread::setMeshID(int id)
	{
		_meshID = id;
	}

	// 	void GmshThread::setPhysicals(QString ps)
	// 	{
	// 		_physicals = ps;
	// 	}

	void GmshThread::setSelectedAll(bool al)
	{
		_selectall = al;
	}

	void GmshThread::setSelectedVisible(bool sv)
	{
		_selectvisible = sv;
	}

	void GmshThread::setFluidMesh(bool fm)
	{
		_fluidMesh = fm;
	}

	void GmshThread::setCellTypeList(QString cells)
	{
		QStringList celllist = cells.split(",");
		for (QString cell : celllist)
		{
			bool ok = false;
			int type = cell.toInt(&ok);
			if (!ok)
				continue;
			_cellTypeList.append(type);
		}
	}

	void GmshThread::mergeAllGeo()
	{
		BRep_Builder aBuilder;
		aBuilder.MakeCompound(*_compounnd);

		Geometry::GeometryData *data = Geometry::GeometryData::getInstance();
		const int nset = data->getGeometrySetCount();
		for (int i = 0; i < nset; ++i)
		{
			auto gset = data->getGeometrySetAt(i);
			TopoDS_Shape *shape = gset->getShape();
			if (shape == nullptr)
				continue;
			aBuilder.Add(*_compounnd, *shape);
		}
	}

	void GmshThread::mergeVisibleGeo()
	{
		BRep_Builder aBuilder;
		aBuilder.MakeCompound(*_compounnd);

		Geometry::GeometryData *data = Geometry::GeometryData::getInstance();
		const int nset = data->getGeometrySetCount();
		for (int i = 0; i < nset; ++i)
		{
			auto gset = data->getGeometrySetAt(i);
			if (!gset->isVisible())
				continue;
			TopoDS_Shape *shape = gset->getShape();
			if (shape == nullptr)
				continue;
			aBuilder.Add(*_compounnd, *shape);
		}
	}

	void GmshThread::mergeSelectGeo()
	{
		BRep_Builder aBuilder;
		aBuilder.MakeCompound(*_compounnd);

		Geometry::GeometryData *data = Geometry::GeometryData::getInstance();
		QList<int> setList = _surfaceHash.uniqueKeys();
		for (int setid : setList)
		{
			Geometry::GeometrySet *set = data->getGeometrySetByID(setid);
			if (set == nullptr)
				continue;
			TopoDS_Shape *shape = set->getShape();
			if (shape == nullptr)
				continue;

			QList<int> indexList = _surfaceHash.values(setid);
			for (int faceindex : indexList)
			{
				TopExp_Explorer faceExp(*shape, TopAbs_FACE);
				for (int index = 0; index < faceindex && faceExp.More(); faceExp.Next(), ++index)
					;

				const TopoDS_Shape &faceShape = faceExp.Current();
				aBuilder.Add(*_compounnd, faceShape);
			}
		}

		setList = _solidHash.uniqueKeys();
		for (int setid : setList)
		{
			Geometry::GeometrySet *set = data->getGeometrySetByID(setid);
			if (set == nullptr)
				continue;
			TopoDS_Shape *shape = set->getShape();
			if (shape == nullptr)
				continue;

			QList<int> indexList = _solidHash.values(setid);
			for (int solidIndex : indexList)
			{
				TopExp_Explorer solidexp(*shape, TopAbs_SOLID);
				for (int index = 0; index < solidIndex && solidexp.More(); solidexp.Next(), ++index)
					;

				const TopoDS_Shape &solidshape = solidexp.Current();
				aBuilder.Add(*_compounnd, solidshape);
			}
		}
	}

	// 	QList<int> GmshThread::getShapeIndexListInFluidField(int itype)
	// 	{
	// 		QList<int> indexList{};
	//
	// 		QList<gp_Pnt> pntlist{};
	// 		//gp_Pnt pt0(-50, -50, -50); gp_Pnt pt1(50, 50, 50);
	// 		gp_Pnt pt0(_fluidField[0][0], _fluidField[0][1], _fluidField[0][2]); gp_Pnt pt1(_fluidField[1][0], _fluidField[1][1], _fluidField[1][2]);
	// 		TopoDS_Shape bigBox = BRepPrimAPI_MakeBox(pt0, pt1).Shape();
	// 		if (bigBox.IsNull()) return indexList;
	// 		QList<Handle(TopoDS_TShape)> tshapelist;
	// 		TopExp_Explorer explor(bigBox, TopAbs_VERTEX);
	// 		for (; explor.More(); explor.Next())
	// 		{
	// 			const TopoDS_Shape& oneshape = explor.Current();
	// 			Handle(TopoDS_TShape) ts = oneshape.TShape();
	// 			if (tshapelist.contains(ts)) continue;
	// 			tshapelist.append(ts);
	// 			const TopoDS_Vertex& vertex = TopoDS::Vertex(oneshape);
	// 			gp_Pnt pt = BRep_Tool::Pnt(vertex);
	// 			pntlist.append(pt);
	// 		}
	//
	// 		TopAbs_ShapeEnum type;
	// 		switch (itype)
	// 		{
	// 		case 1: type = TopAbs_VERTEX; break;
	// 		case 2: type = TopAbs_EDGE;
	// 			return indexList = getInerEdgeIndexs(pntlist);
	// 			break;
	// 		case 3: type = TopAbs_FACE;
	// 			return indexList = getInerFaceIndexs(pntlist);
	// 			break;
	// 		case 4: type = TopAbs_SOLID; break;
	// 		default: break;
	// 		}
	//
	//
	// 	}
	//
	// 	QList<int> GmshThread::getInerFaceIndexs(QList<gp_Pnt> pntlist)
	// 	{
	// 		QList<int> indexList{};
	// 		TopExp_Explorer exp(*_compounnd, TopAbs_FACE);
	// 		for (; exp.More(); exp.Next())
	// 		{
	// 			const TopoDS_Shape& faceShape = exp.Current();
	// 			TopoDS_Face face = TopoDS::Face(faceShape);
	// 			const Handle(Geom_Surface) geosurface = BRep_Tool::Surface(face);
	// 			bool isOnBox{ false };
	// 			for (gp_Pnt pt : pntlist)
	// 			{
	// 				GeomAPI_ProjectPointOnSurf project(pt, geosurface);
	// 				double distance = project.LowerDistance();
	// 				if (distance < 1e-6) { isOnBox = true; break; }
	// 			}
	// 			if (!isOnBox)
	// 			{
	// 				TopTools_IndexedMapOfShape mapper;
	// 				TopExp::MapShapes(*_compounnd, TopAbs_FACE, mapper);
	// 				int resindex = mapper.FindIndex(faceShape);
	// 				indexList.push_back(resindex);
	// 			}
	// 		}
	// 		return indexList;
	// 	}

	// 	QList<int> GmshThread::getInerEdgeIndexs(QList<gp_Pnt> pntlist)
	// 	{
	// 		QList<int> indexList{};
	// 		TopExp_Explorer exp(*_setCompound, TopAbs_SOLID);
	// 		TopoDS_Shape fauShape{};
	// 		for (; exp.More();exp.Next())
	// 		{
	// 			BRepAlgoAPI_Fuse fau(fauShape, exp.Current());
	//
	// 			if (!fau.IsDone()) return indexList;
	// 			const TopoDS_Shape& aFusedShape = fau.Shape();
	//
	// 			if (aFusedShape.IsNull()) return indexList;
	// 			//TopoDS_Shape shape = Command::GeoCommandCommon::removeSplitter(aFusedShape);
	// 			fauShape = aFusedShape;
	// 		}
	// 		const char* ch = "D://FusedShape.brep";
	// 		BRepTools::Write(fauShape, ch);
	//
	//
	// 		return indexList;
	// 	}

	QList<itemInfo> GmshThread::generateGeoIds(vtkDataSet *dataset)
	{
		QList<itemInfo> infoList{};
		// QMultiHash<int, QList<int>> elementHash{};
		if (_surfaceHash.size() < 1 || dataset == nullptr)
			return infoList;

		Geometry::GeometryData *data = Geometry::GeometryData::getInstance();
		QMultiHash<int, int>::iterator it = _surfaceHash.begin();
		for (; it != _surfaceHash.end(); it++)
		{
			if (it.key() < 0 || it.value() < 0)
				continue;
			TopoDS_Compound aRes;
			BRep_Builder aBuilder;
			aBuilder.MakeCompound(aRes);
			Geometry::GeometrySet *set = data->getGeometrySetByID(it.key());
			TopoDS_Shape *body = set->getShape();

			TopExp_Explorer exper(*body, TopAbs_FACE);
			for (int index = 0; index < it.value() && exper.More(); exper.Next(), ++index)
				;
			const TopoDS_Shape &s = exper.Current();
			aBuilder.Add(aRes, s);
			QList<int> inids{};
			if (!aRes.IsNull())
				inids = GeoCommon::getD2ElementsInShape(dataset, &aRes);

			infoList.append(itemInfo{it.key(), it.value(), inids});
			// elementHash.insert(it.key(), inids);
		}
		return infoList;

		/*QList<int> geoList = _surfaceHash.uniqueKeys();
					for (int setid : geoList)
					{
							if (setid < 0) continue;
							TopoDS_Compound aRes;
							BRep_Builder aBuilder;
							aBuilder.MakeCompound(aRes);
							Geometry::GeometrySet* set = data->getGeometrySetByID(setid);
							TopoDS_Shape* body = set->getShape();
							QList<int> member = _surfaceHash.values(setid);
							for (int m : member)
							{
									if (m < 0) continue;
									TopExp_Explorer exper(*body, TopAbs_FACE);
									for (int index = 0; index < m && exper.More(); exper.Next(), ++index);
									const TopoDS_Shape& s = exper.Current();
									aBuilder.Add(aRes, s);
							}
							QList<int> inids;
							if (!aRes.IsNull())	inids = GeoCommon::getD2ElementsInShape(dataset, &aRes);
							elementHash.insert(setid, inids);
					}*/
	}

	void GmshThread::isSaveDataToKernal(bool save)
	{
		_isSaveToKernal = save;
	}

	void GmshThread::setGmshSettingData(MeshData::MeshKernal *k)
	{
		GmshSettingData *setting = new GmshSettingData;
		setting->setID(_dim);
		setting->setSolidHash(_solidHash);
		setting->setSurfaceHash(_surfaceHash);
		setting->setElementOrder(_elementOrder);
		setting->setElementType(_elementType);
		setting->setGeoClean(_geoclean);
		setting->setGridCoplanar(_isGridCoplanar);
		setting->setMaxSize(_maxSize);
		setting->setMinSize(_minSize);
		setting->setSizeFactor(_sizeFactor);
		setting->setSelectAll(_selectall);
		setting->setSelectVisiable(_selectvisible);
		setting->setSmoothIteration(_smoothIteration);
		setting->setMethod(_method);
		setting->setSizeAtPoints(_sizeAtPoints);
		setting->setSizeFields(_sizeFields);
		setting->setBoreNodeTolerance(_boreNodeTolerance);
		setting->setMeshID(_meshID);
		setting->setCells(_cellTypeList);

		k->setGmshSetting(setting);
	}

	void GmshThread::setGmshScriptData()
	{
		_scriptWriter->setSmooth(_smoothIteration);
		_scriptWriter->setElementType(_elementType);
		_scriptWriter->setElementOrder(_elementOrder);
		_scriptWriter->setMethod(_method);
		_scriptWriter->setMinSize(_minSize);
		_scriptWriter->setMaxSize(_maxSize);
		_scriptWriter->setFactor(_sizeFactor);
		_scriptWriter->setGeoClean(_geoclean);
		_scriptWriter->setGridCoplanar(_isGridCoplanar);
		_scriptWriter->setSizePoints(_sizeAtPoints);
		_scriptWriter->setSizeFields(_sizeFields);
	}

	vtkDataSet *GmshThread::deleteSpecifiedCells(vtkDataSet *dataset)
	{
		if (dataset == nullptr)
			return dataset;
		// int index = 0;
		vtkUnstructuredGrid *ung = vtkUnstructuredGrid::New();
		vtkPoints *points = vtkPoints::New();

		const int nNode = dataset->GetNumberOfPoints();
		for (int i = 0; i < nNode; i++)
		{
			double *coor = dataset->GetPoint(i);
			points->InsertNextPoint(coor);
		}
		ung->SetPoints(points);

		const int ncell = dataset->GetNumberOfCells();
		for (int i = 0; i < ncell; i++)
		{
			vtkCell *cell = dataset->GetCell(i);
			if (cell == nullptr)
				continue;
			VTKCellType type = (VTKCellType)cell->GetCellType();
			vtkIdList *idlist = vtkIdList::New();
			if (isSpecifiedCell(type))
			{
				idlist = cell->GetPointIds();
				ung->InsertNextCell(type, idlist);
			}
		}

		return ung;
	}

	bool GmshThread::isSpecifiedCell(VTKCellType type)
	{
		int flag = -1;
		switch (type)
		{
		case VTKCellType::VTK_VERTEX:
		case VTKCellType::VTK_POLY_VERTEX:
			flag = 0;
			break;
		case VTKCellType::VTK_LINE:
		case VTKCellType::VTK_POLY_LINE:
		case VTKCellType::VTK_QUADRATIC_EDGE:
			flag = 1;
			break;
		case VTKCellType::VTK_TRIANGLE:
		case VTKCellType::VTK_QUAD:
		case VTKCellType::VTK_TRIANGLE_STRIP:
		case VTKCellType::VTK_PIXEL:
		case VTKCellType::VTK_POLYGON:
		case VTKCellType::VTK_QUADRATIC_TRIANGLE:
		case VTKCellType::VTK_QUADRATIC_LINEAR_QUAD:
		case VTKCellType::VTK_QUADRATIC_QUAD:
		case VTKCellType::VTK_BIQUADRATIC_QUAD:
			flag = 2;
			break;
		case VTKCellType::VTK_TETRA:
		case VTKCellType::VTK_HEXAHEDRON:
		case VTKCellType::VTK_VOXEL:
		case VTKCellType::VTK_WEDGE:
		case VTKCellType::VTK_PYRAMID:
		case VTKCellType::VTK_PENTAGONAL_PRISM:
		case VTKCellType::VTK_HEXAGONAL_PRISM:
		case VTKCellType::VTK_QUADRATIC_TETRA:
		case VTKCellType::VTK_QUADRATIC_PYRAMID:
		case VTKCellType::VTK_QUADRATIC_HEXAHEDRON:
		case VTKCellType::VTK_BIQUADRATIC_QUADRATIC_HEXAHEDRON:
		case VTKCellType::VTK_TRIQUADRATIC_HEXAHEDRON:
		case VTKCellType::VTK_QUADRATIC_LINEAR_WEDGE:
		case VTKCellType::VTK_QUADRATIC_WEDGE:
		case VTKCellType::VTK_BIQUADRATIC_QUADRATIC_WEDGE:
			flag = 3;
			break;
		default:
			break;
		}

		if (_cellTypeList.contains(flag))
			return true;

		return false;
	}

}








