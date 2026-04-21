#include "KEYdataExchange.h"
#include "meshDataExchangePlugin.h"
#include "MeshData/meshSingleton.h"
#include "MeshData/meshKernal.h"
#include "MeshData/meshSet.h"
#include "Geometry/geometryData.h"
#include "Geometry/geometrySet.h"
#include "Geometry/geometryParaGear.h"
#include <QFile>
#include <QHash>
#include <QMap>
#include <QQueue>
#include <QSet>
#include <QRegExp>
#include <QString>
#include <QTextStream>
#include <QDebug>
#include <QList>
#include <QVector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vtkCell.h>
#include <vtkCell3D.h>
#include <vtkIdList.h>
#include <vtkSmartPointer.h>
#include <vtkQuad.h>
#include <vtkTriangle.h>
#include <vtkHexahedron.h>
#include <vtkTetra.h>
#include <vtkQuadraticTetra.h>
#include <vtkQuadraticHexahedron.h>

namespace
{
	QString hmPadRight(const QString &s, int width)
	{
		QString t = s;
		if (t.size() > width)
			t = t.left(width);
		return t + QString(width - t.size(), QChar(' '));
	}

	// ??????????????????????????7.850000e-09210000??
	QString hmNumCol(const QString &s, int minWidth)
	{
		const int w = qMax(minWidth, s.size() + 1);
		return s.rightJustified(w, QChar(' '));
	}

	QString hmInt8(qint64 v)
	{
		return QString::number(v).rightJustified(8, QChar(' '));
	}

	QString hmInt10(qint64 v)
	{
		return QString::number(v).rightJustified(10, QChar(' '));
	}

	QString hmFloat16(double v, int prec = 8)
	{
		return QString::number(v, 'f', prec).rightJustified(16, QChar(' '));
	}

	struct ExportValidationState
	{
		QSet<int> nodeIds{};
		QSet<int> elemIds{};
		QSet<int> partIds{};
		QSet<int> secIds{};
		QSet<int> matIds{};
	};

	constexpr bool kExportShellElements = true;

	bool isShellCellType(int ct)
	{
		return ct == VTK_TRIANGLE || ct == VTK_QUAD;
	}

	bool isSolidCellType(int ct)
	{
		return ct == VTK_HEXAHEDRON || ct == VTK_VOXEL ||
			   ct == VTK_TETRA || ct == VTK_QUADRATIC_TETRA ||
			   ct == VTK_QUADRATIC_HEXAHEDRON || ct == VTK_TRIQUADRATIC_HEXAHEDRON ||
			   ct == VTK_BIQUADRATIC_QUADRATIC_HEXAHEDRON;
	}

	// vtkVoxel ??vtkHexahedron ??????????????????
	constexpr int vtkVoxelToHexVertex[8] = {0, 1, 3, 2, 4, 5, 7, 6};

	bool pointIdsInRange(vtkIdList *pids, vtkIdType nPts, int nCorner)
	{
		if (pids == nullptr || pids->GetNumberOfIds() < nCorner)
			return false;
		for (int j = 0; j < nCorner; ++j)
		{
			const vtkIdType pid = pids->GetId(j);
			if (pid < 0 || pid >= nPts)
				return false;
		}
		return true;
	}

	// ??writeShellPart + writeCellPart ??????? VTK ??????????EID?????? EID??
	// ?????? GetId=-1????????EID????deck????????0??
	void buildVtkToExportEidMap(vtkDataSet *grid, QVector<int> &out)
	{
		const int nc = grid->GetNumberOfCells();
		out.resize(nc);
		out.fill(0);
		const vtkIdType nPts = grid->GetNumberOfPoints();
		int next = 1;

		for (int i = 0; i < nc; ++i)
		{
			vtkCell *cell = grid->GetCell(i);
			if (cell == nullptr)
				continue;
			vtkIdList *p = cell->GetPointIds();
			const int ct = cell->GetCellType();
			if (ct == VTK_QUAD)
			{
				if (!pointIdsInRange(p, nPts, 4))
					continue;
				out[i] = next++;
			}
			else if (ct == VTK_TRIANGLE)
			{
				if (!pointIdsInRange(p, nPts, 3))
					continue;
				out[i] = next++;
			}
		}

		for (int i = 0; i < nc; ++i)
		{
			vtkCell *cell = grid->GetCell(i);
			if (cell == nullptr)
				continue;
			const int ct = cell->GetCellType();
			if (ct == VTK_TRIANGLE || ct == VTK_QUAD)
				continue;
			vtkIdList *p = cell->GetPointIds();
			if (p == nullptr)
				continue;

			if (ct == VTK_HEXAHEDRON)
			{
				if (!pointIdsInRange(p, nPts, 8))
					continue;
				out[i] = next++;
			}
			else if (ct == VTK_VOXEL)
			{
				if (!pointIdsInRange(p, nPts, 8))
					continue;
				out[i] = next++;
			}
			else if (ct == VTK_TETRA || ct == VTK_QUADRATIC_TETRA)
			{
				if (!pointIdsInRange(p, nPts, 4))
					continue;
				out[i] = next++;
			}
			else if (ct == VTK_QUADRATIC_HEXAHEDRON || ct == VTK_TRIQUADRATIC_HEXAHEDRON ||
					 ct == VTK_BIQUADRATIC_QUADRATIC_HEXAHEDRON)
			{
				if (!pointIdsInRange(p, nPts, 8))
					continue;
				out[i] = next++;
			}
		}
	}

	void buildVtkPointToExportNidMap(vtkDataSet *grid, QVector<int> &out)
	{
		const int nPts = (grid == nullptr) ? 0 : static_cast<int>(grid->GetNumberOfPoints());
		out.resize(nPts);
		for (int i = 0; i < nPts; ++i)
			out[i] = i + 1;
	}

	QString previewIntList(const QList<int> &values, int maxCount = 12)
	{
		QStringList parts;
		const int n = qMin(values.size(), maxCount);
		for (int i = 0; i < n; ++i)
			parts.append(QString::number(values.at(i)));
		if (values.size() > maxCount)
			parts.append(QStringLiteral("..."));
		return parts.join(QStringLiteral(","));
	}

} // namespace

namespace MeshData
{
	KEYdataExchange::KEYdataExchange(const QString &fileName, MeshOperation operation, GUI::MainWindow *mw, int modelId) : _meshData(MeshData::getInstance()),
																														   _operation(operation),
																														   _fileName(fileName),
																														   _stream(nullptr),
																														   MeshThreadBase(fileName, operation, mw),
																														   _modelId(modelId)
	{
	}

	KEYdataExchange::~KEYdataExchange()
	{
		if (_stream != nullptr)
		{
			delete _stream;
			_stream = nullptr;
		}
	}

	void KEYdataExchange::run()
	{
		ModuleBase::ThreadTask::run();
		bool result = false;
		switch (_operation)
		{
		case MESH_READ:
			emit showInformation(tr("Import KEY Mesh File From \"%1\"").arg(_fileName));
			result = read();
			setReadResult(result);
			break;
		case MESH_WRITE:
			emit showInformation(tr("Export KEY Mesh File From \"%1\"").arg(_fileName));
			result = write();
			setWriteResult(result);
			break;
		}
		defaultMeshFinished();
	}

	bool KEYdataExchange::read()
	{
		QFile file(_fileName);
		if (!file.open(QIODevice::ReadOnly))
			return false;
		_stream = new QTextStream(&file);

		QString aLine{};
		vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
		auto mk = new MeshKernal;
		mk->setMeshData(grid);
		const int mkID = mk->getID();
		while (!_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			readLine(aLine);
			if (aLine.startsWith("*NODE"))
				readNodes(grid, aLine);
			else if (aLine.startsWith("*ELEMENT_SHELL"))
				readShellElements(grid, aLine, mkID);
			else if (aLine.startsWith("*ELEMENT_SOLID"))
				readElements(grid, aLine, mkID);
			else if (aLine.startsWith("*SET_NODE_LIST"))
				readNodesGroup(mkID, aLine);
		}
		if (grid->GetNumberOfPoints() < 1 || grid->GetNumberOfCells() < 1)
			return false;

		mk->setName(QString("KEY_%1").arg(mkID));
		mk->setPath(_fileName);
		mk->appendProperty("Points", (int)grid->GetNumberOfPoints());
		mk->appendProperty("Cells", (int)grid->GetNumberOfCells());
		_meshData->appendMeshKernal(mk);
		_meshData->generateDisplayDataSet();
		file.close();
		return true;
	}

	bool KEYdataExchange::readNodes(vtkSmartPointer<vtkUnstructuredGrid> grid, QString &aLine)
	{
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		while (!_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			readLine(aLine);
			if (aLine.startsWith("$"))
				continue;

			// ???? *NODE ????
			// 1) ??????KEYWORD ????????
			// 2) ??/?????HyperMesh/??????
			QStringList parts = aLine.simplified().split(QRegExp("\\s+"), Qt::SkipEmptyParts);
			if (parts.size() < 4)
				continue;
			// nodeID ????????????????????????????????????
			const int nodeID = parts.at(0).toInt();
			(void)nodeID;
			const double nodeX = parts.at(1).toDouble();
			const double nodeY = parts.at(2).toDouble();
			const double nodeZ = parts.at(3).toDouble();
			points->InsertNextPoint(nodeX, nodeY, nodeZ);
		}
		grid->SetPoints(points);
		return true;
	}

	bool KEYdataExchange::readShellElements(vtkSmartPointer<vtkUnstructuredGrid> grid, QString &aLine, int /*mkId*/)
	{
		// HyperMesh????EID PID N1 N2 N3 N4?? modeltest/standard2.k????????5 ???????? N3
		while (!_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			readLine(aLine);
			if (aLine.startsWith("*"))
				break;
			if (aLine.startsWith("$"))
				continue;
			const QStringList lineList = aLine.simplified().split(' ');
			if (lineList.size() < 5)
				continue;
			if (lineList.size() >= 6)
			{
				vtkSmartPointer<vtkQuad> q = vtkSmartPointer<vtkQuad>::New();
				q->GetPointIds()->SetId(0, lineList.at(2).toInt() - 1);
				q->GetPointIds()->SetId(1, lineList.at(3).toInt() - 1);
				q->GetPointIds()->SetId(2, lineList.at(4).toInt() - 1);
				q->GetPointIds()->SetId(3, lineList.at(5).toInt() - 1);
				grid->InsertNextCell(q->GetCellType(), q->GetPointIds());
			}
			else
			{
				vtkSmartPointer<vtkTriangle> t = vtkSmartPointer<vtkTriangle>::New();
				t->GetPointIds()->SetId(0, lineList.at(2).toInt() - 1);
				t->GetPointIds()->SetId(1, lineList.at(3).toInt() - 1);
				t->GetPointIds()->SetId(2, lineList.at(4).toInt() - 1);
				grid->InsertNextCell(t->GetCellType(), t->GetPointIds());
			}
		}
		return true;
	}

	bool KEYdataExchange::readElements(vtkSmartPointer<vtkUnstructuredGrid> grid, QString &aLine, int mkId)
	{
		while (!_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			readLine(aLine);
			if (aLine.startsWith("*"))
				break;
			if (aLine.startsWith("$"))
				continue;
			setGridCells(grid, aLine, mkId);
		}
		return true;
	}

	bool KEYdataExchange::readNodesGroup(const int mkID, QString &aLine)
	{
		MeshSet *ms = new MeshSet("", Node);
		if (ms == nullptr)
			return false;
		_meshData->appendMeshSet(ms);
		int setID = ms->getID();
		while (!_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			readLine(aLine);
			if (aLine.startsWith("*"))
				return false;
			if (aLine.startsWith("$"))
				continue;
			if (aLine.contains(QStringLiteral("MECH")))
			{
				const QStringList t = aLine.simplified().split(QRegExp("\\s+"), Qt::SkipEmptyParts);
				if (!t.isEmpty())
					setID = t.at(0).toInt();
				break;
			}
			if (aLine.contains(QLatin1Char(',')) && aLine.contains(QStringLiteral("MECH")))
			{
				const QStringList t = aLine.split(QLatin1Char(','));
				if (!t.isEmpty())
					setID = t.at(0).toInt();
				break;
			}
		}
		ms->setName(QString("NodeSet_%1").arg(setID));
		ms->appendProperty("K_ID", setID);
		while (!_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			readLine(aLine);
			if (aLine.startsWith("*"))
				break;
			if (aLine.startsWith("$"))
				continue;
			QStringList lineList = aLine.simplified().split(' ');
			int startIdx = 0;
			if (lineList.size() > 1 && lineList.at(0).toInt() == setID)
				startIdx = 1;
			for (int li = startIdx; li < lineList.size(); ++li)
			{
				bool ok = false;
				const int v = lineList.at(li).toInt(&ok);
				if (!ok)
					continue;
				const int cellIndex = v - 1;
				ms->appendMember(mkID, cellIndex);
			}
		}
		return true;
	}

	void KEYdataExchange::setGridCells(vtkSmartPointer<vtkUnstructuredGrid> grid, QString &aLine, int mKid)
	{
		QMap<int, QVector<int>> cells{};
		while (!_stream->atEnd())
		{
			if (!_threadRuning)
				return;
			if (aLine.startsWith("$") || aLine.startsWith("*"))
				break;
			QStringList lineList = aLine.simplified().split(" ");
			// HyperMesh????EID PID?????N1..N8?? N1..N4??
			if (lineList.size() == 2)
			{
				readLine(aLine);
				if (aLine.startsWith("$") || aLine.startsWith("*"))
					break;
				const QStringList line2 = aLine.simplified().split(" ");
				lineList.append(line2);
			}
			const int cellID = lineList.at(1).toInt();
			const int numOfPonits = lineList.size() - 2;
			switch (numOfPonits)
			{
			case 8:
			{
				const int vtkCellIdx = static_cast<int>(grid->GetNumberOfCells());
				// HyperMesh??????*ELEMENT_SOLID ????8 ????N5..N8 ????N4?? modeltest/0403-h.k??
				const int n5 = lineList.at(6).toInt();
				const int n6 = lineList.at(7).toInt();
				const int n7 = lineList.at(8).toInt();
				const int n8 = lineList.at(9).toInt();
				if (n5 == n6 && n6 == n7 && n7 == n8)
				{
					vtkSmartPointer<vtkTetra> aTetraCell = vtkSmartPointer<vtkTetra>::New();
					aTetraCell->GetPointIds()->SetId(0, lineList.at(2).toInt() - 1);
					aTetraCell->GetPointIds()->SetId(1, lineList.at(3).toInt() - 1);
					aTetraCell->GetPointIds()->SetId(2, lineList.at(4).toInt() - 1);
					aTetraCell->GetPointIds()->SetId(3, lineList.at(5).toInt() - 1);
					grid->InsertNextCell(aTetraCell->GetCellType(), aTetraCell->GetPointIds());
				}
				else
				{
					vtkSmartPointer<vtkHexahedron> aHexahedronCell = vtkSmartPointer<vtkHexahedron>::New();
					aHexahedronCell->GetPointIds()->SetId(0, lineList.at(2).toInt() - 1);
					aHexahedronCell->GetPointIds()->SetId(1, lineList.at(3).toInt() - 1);
					aHexahedronCell->GetPointIds()->SetId(2, lineList.at(4).toInt() - 1);
					aHexahedronCell->GetPointIds()->SetId(3, lineList.at(5).toInt() - 1);
					aHexahedronCell->GetPointIds()->SetId(4, lineList.at(6).toInt() - 1);
					aHexahedronCell->GetPointIds()->SetId(5, lineList.at(7).toInt() - 1);
					aHexahedronCell->GetPointIds()->SetId(6, lineList.at(8).toInt() - 1);
					aHexahedronCell->GetPointIds()->SetId(7, lineList.at(9).toInt() - 1);
					grid->InsertNextCell(aHexahedronCell->GetCellType(), aHexahedronCell->GetPointIds());
				}
				cells[cellID].append(vtkCellIdx);
			}
			break;
			case 4:
			{
				const int vtkCellIdx = static_cast<int>(grid->GetNumberOfCells());
				vtkSmartPointer<vtkTetra> aTetraCell = vtkSmartPointer<vtkTetra>::New();
				aTetraCell->GetPointIds()->SetId(0, lineList.at(2).toInt() - 1);
				aTetraCell->GetPointIds()->SetId(1, lineList.at(3).toInt() - 1);
				aTetraCell->GetPointIds()->SetId(2, lineList.at(4).toInt() - 1);
				aTetraCell->GetPointIds()->SetId(3, lineList.at(5).toInt() - 1);
				grid->InsertNextCell(aTetraCell->GetCellType(), aTetraCell->GetPointIds());
				cells[cellID].append(vtkCellIdx);
			}
			break;
			case 10:
			{
				const int vtkCellIdx = static_cast<int>(grid->GetNumberOfCells());
				vtkSmartPointer<vtkQuadraticTetra> aQTetraCell = vtkSmartPointer<vtkQuadraticTetra>::New();
				aQTetraCell->GetPointIds()->SetId(0, lineList.at(2).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(1, lineList.at(3).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(2, lineList.at(4).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(3, lineList.at(5).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(4, lineList.at(6).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(5, lineList.at(7).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(6, lineList.at(8).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(7, lineList.at(9).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(8, lineList.at(10).toInt() - 1);
				aQTetraCell->GetPointIds()->SetId(9, lineList.at(11).toInt() - 1);
				grid->InsertNextCell(aQTetraCell->GetCellType(), aQTetraCell->GetPointIds());
				cells[cellID].append(vtkCellIdx);
			}
			break;
			case 20:
			{
				const int vtkCellIdx = static_cast<int>(grid->GetNumberOfCells());
				vtkSmartPointer<vtkQuadraticHexahedron> aQHexCell = vtkSmartPointer<vtkQuadraticHexahedron>::New();
				aQHexCell->GetPointIds()->SetId(0, lineList.at(2).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(1, lineList.at(3).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(2, lineList.at(4).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(3, lineList.at(5).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(4, lineList.at(6).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(5, lineList.at(7).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(6, lineList.at(8).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(7, lineList.at(9).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(8, lineList.at(10).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(9, lineList.at(11).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(10, lineList.at(12).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(11, lineList.at(13).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(12, lineList.at(14).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(13, lineList.at(15).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(14, lineList.at(16).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(15, lineList.at(17).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(16, lineList.at(18).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(17, lineList.at(19).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(18, lineList.at(20).toInt() - 1);
				aQHexCell->GetPointIds()->SetId(19, lineList.at(21).toInt() - 1);
				grid->InsertNextCell(aQHexCell->GetCellType(), aQHexCell->GetPointIds());
				cells[cellID].append(vtkCellIdx);
			}
			break;
			default:
				break;
			}
			readLine(aLine);
		}
		QMapIterator<int, QVector<int>> it(cells);
		while (it.hasNext())
		{
			it.next();
			const int id = it.key();
			MeshSet *ms = new MeshSet("", Element);
			for (auto &id : it.value())
			{
				ms->appendMember(mKid, id);
			}
			_meshData->appendMeshSet(ms);

			ms->setName(QString("CellSet_%1").arg(id));
			ms->appendProperty("K_ID", id);
		}
	}

	void KEYdataExchange::readLine(QString &aLine)
	{
		while (_threadRuning)
		{
			if (_stream->atEnd())
			{
				//_threadRuning = false;
				break;
			}
			aLine = _stream->readLine();
			if (aLine.isEmpty())
				continue;
			break;
		}
	}

	bool KEYdataExchange::write()
	{
		if (_stream != nullptr)
		{
			delete _stream;
			_stream = nullptr;
		}
		const int nk = _meshData->getKernalCount();
		if (nk <= 0)
			return false;
		

		MeshKernal* mk = nullptr;
		if (_modelId == -1)
			mk = _meshData->getKernalAt(nk - 1);
		else
			mk = _meshData->getKernalByID(_modelId);

		if (mk == nullptr)
			return false;
		

		auto grid = mk->getMeshData();
		if (grid == nullptr || grid->GetNumberOfPoints() <= 0 || grid->GetNumberOfCells() <= 0)
			return false;
		

		// ?? modelId=-1????????????ID ??????
		_modelId = mk->getID();

		// HyperMesh 971_R12 ???? modeltest/0403-h.k??+??????modeltest/standard2.k
		const QString partBaseName = mk->getName().isEmpty() ? QStringLiteral("FastCAE_Part") : mk->getName();
		int secidSolid = 0;
		int secidShell = 0;
		writeSectionCards(grid, secidSolid, secidShell);
		QVector<int> vtkToEid;
		buildVtkToExportEidMap(grid, vtkToEid);
		QVector<int> vtkPointToExportNid;
		buildVtkPointToExportNidMap(grid, vtkPointToExportNid);
		QList<ExportBodyInfo> bodies;
		QVector<int> vtkCellToBody;
		if (!buildExportBodies(grid, vtkToEid, partBaseName, bodies, vtkCellToBody))
			return false;
		if (!validateExportDeck(grid, vtkToEid, vtkCellToBody, bodies, secidSolid, secidShell, 1))
			return false;

		QFile file(_fileName);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
			return false;
		_stream = new QTextStream(&file);

		writeHyperMeshHeader();
		writeMaterialElasticStub(1);
		writeSectionCards(grid, secidSolid, secidShell);
		writePartCards(bodies, secidSolid, secidShell, 1);
		const bool okPoint = writePointPart(grid, vtkPointToExportNid);
		const bool okShell = writeShellPart(grid, vtkToEid, vtkCellToBody, bodies, vtkPointToExportNid);
		const bool okCell = writeCellPart(grid, vtkToEid, vtkCellToBody, bodies, vtkPointToExportNid);
		const bool okSet = writeMeshPart(grid, vtkToEid, bodies, vtkPointToExportNid);
		*_stream << "*END" << endl;
		file.close();
		delete _stream;
		_stream = nullptr;
		return okPoint && okShell && okCell && okSet;
	}

	void KEYdataExchange::writeHyperMeshHeader()
	{
		*_stream << "$$ HM_OUTPUT_DECK created by FastCAE" << endl;
		*_stream << "$$ Ls-dyna Input Deck Generated by FastCAE" << endl;
		*_stream << "$$ Generated using HyperMesh-Ls-dyna 971_R12.0 Template Version : FastCAE" << endl;
		*_stream << "*KEYWORD" << endl;
		*_stream << "*TITLE" << endl;
		*_stream << hmPadRight(QStringLiteral("FastCAE mesh export"), 80) << endl;
	}

	void KEYdataExchange::writeMaterialElasticStub(int mid)
	{
		// ??????????modeltest/standard2.k??HMNAME / $HWCOLOR / ????+ ?????
		const double rho = 7.85e-9;
		const double E = 210000.0;
		const double pr = 0.30;
		*_stream << "*MAT_ELASTIC" << endl;
		*_stream << "$      MID       RHO         E        PR        DA        DB         K" << endl;
		QString matRow;
		matRow += hmInt10(mid);
		matRow += hmNumCol(QString::number(rho, 'e', 6), 10);
		matRow += hmNumCol(QString::number(E, 'e', 6), 10);
		matRow += hmNumCol(QString::number(pr, 'f', 6), 10);
		matRow += hmInt10(0);
		matRow += hmInt10(0);
		matRow += hmInt10(0);
		*_stream << matRow << endl;
	}

	void KEYdataExchange::writeSectionCards(vtkDataSet *grid, int &outSecidSolid, int &outSecidShell)
	{
		outSecidSolid = 0;
		outSecidShell = 0;
		bool hasSolid = false;
		bool hasShell = false;
		const int numOfCell = grid->GetNumberOfCells();
		for (int i = 0; i < numOfCell; ++i)
		{
			vtkCell *cell = grid->GetCell(i);
			if (cell == nullptr)
				continue;
			const int ctype = cell->GetCellType();
			if (isSolidCellType(ctype))
				hasSolid = true;
			else if (kExportShellElements && isShellCellType(ctype))
				hasShell = true;
		}

		if (hasSolid)
		{
			outSecidSolid = 1;
			if (_stream != nullptr)
			{
				*_stream << "*SECTION_SOLID" << endl;
				*_stream << "$   SECID    ELFORM       AET" << endl;
				*_stream << hmInt10(outSecidSolid)
						 << hmInt10(1)
						 << hmInt10(0)
						 << endl;
			}
		}
		if (hasShell)
		{
			outSecidShell = hasSolid ? 2 : 1;
			if (_stream != nullptr)
			{
				*_stream << "*SECTION_SHELL" << endl;
				*_stream << "$    SECID    ELFORM      SHRF       NIP     PROPT        QR     ICOMP     SETYP" << endl;
				*_stream << hmInt10(outSecidShell)
						 << hmInt10(2)
						 << hmNumCol(QString::number(0.833333, 'f', 6), 12)
						 << hmInt10(5)
						 << hmInt10(0)
						 << hmInt10(0)
						 << hmInt10(0)
						 << hmInt10(0)
						 << endl;
				*_stream << "$       T1        T2        T3        T4      NLOC     MAREA      IDOF    EDGSET" << endl;
				*_stream << hmNumCol(QString::number(1.0, 'f', 6), 10)
						 << hmNumCol(QString::number(1.0, 'f', 6), 10)
						 << hmNumCol(QString::number(1.0, 'f', 6), 10)
						 << hmNumCol(QString::number(1.0, 'f', 6), 10)
						 << hmInt10(0)
						 << hmInt10(0)
						 << hmInt10(0)
						 << hmInt10(0)
						 << endl;
			}
		}
	}

	bool KEYdataExchange::buildExportBodies(vtkDataSet *grid, const QVector<int> &vtkCellToExportEid, const QString &baseName,
											QList<KEYdataExchange::ExportBodyInfo> &bodies, QVector<int> &vtkCellToBody)
	{
		const int nCells = grid->GetNumberOfCells();
		vtkCellToBody.resize(nCells);
		vtkCellToBody.fill(-1);

		QHash<int, QVector<int>> pointToCells;
		QVector<int> exportCells;
		exportCells.reserve(nCells);
		const vtkIdType nPts = grid->GetNumberOfPoints();
		for (int i = 0; i < nCells && i < vtkCellToExportEid.size(); ++i)
		{
			if (vtkCellToExportEid[i] <= 0)
				continue;
			vtkCell *cell = grid->GetCell(i);
			if (cell == nullptr)
				continue;
			const int ctype = cell->GetCellType();
			if (!isSolidCellType(ctype) && !(kExportShellElements && isShellCellType(ctype)))
				continue;
			vtkIdList *pids = cell->GetPointIds();
			if (pids == nullptr)
				continue;
			const int nCorner = (ctype == VTK_TRIANGLE) ? 3 :
								(ctype == VTK_QUAD) ? 4 :
								(ctype == VTK_TETRA || ctype == VTK_QUADRATIC_TETRA) ? 4 : 8;
			if (!pointIdsInRange(pids, nPts, nCorner))
				continue;
			exportCells.append(i);
			for (vtkIdType j = 0; j < pids->GetNumberOfIds(); ++j)
				pointToCells[static_cast<int>(pids->GetId(j))].append(i);
		}

		for (int cellIdx : exportCells)
		{
			if (vtkCellToBody[cellIdx] >= 0)
				continue;
			const int bodyIndex = bodies.size();
			ExportBodyInfo body;
			QQueue<int> queue;
			queue.enqueue(cellIdx);
			vtkCellToBody[cellIdx] = bodyIndex;
			while (!queue.isEmpty())
			{
				const int current = queue.dequeue();
				body.cellIndices.append(current);
				vtkCell *cell = grid->GetCell(current);
				if (cell == nullptr)
					continue;
				const int ctype = cell->GetCellType();
				body.hasSolid = body.hasSolid || isSolidCellType(ctype);
				body.hasShell = body.hasShell || (kExportShellElements && isShellCellType(ctype));
				vtkIdList *pids = cell->GetPointIds();
				if (pids == nullptr)
					continue;
				for (vtkIdType j = 0; j < pids->GetNumberOfIds(); ++j)
				{
					const int pid0 = static_cast<int>(pids->GetId(j));
					body.nodeIds1Based.append(pid0 + 1);
					const QVector<int> neighbors = pointToCells.value(pid0);
					for (int next : neighbors)
					{
						if (vtkCellToBody[next] >= 0)
							continue;
						vtkCellToBody[next] = bodyIndex;
						queue.enqueue(next);
					}
				}
			}
			bodies.append(body);
		}

		if (bodies.isEmpty())
			return false;

		std::sort(bodies.begin(), bodies.end(), [](const ExportBodyInfo &a, const ExportBodyInfo &b) {
			return a.cellIndices.isEmpty() ? false : (b.cellIndices.isEmpty() ? true : a.cellIndices.first() < b.cellIndices.first());
		});
		for (int newIndex = 0; newIndex < bodies.size(); ++newIndex)
		{
			for (int cellIdx : bodies[newIndex].cellIndices)
				vtkCellToBody[cellIdx] = newIndex;
		}

		for (int i = 0; i < bodies.size(); ++i)
		{
			auto &body = bodies[i];
			std::sort(body.cellIndices.begin(), body.cellIndices.end());
			std::sort(body.nodeIds1Based.begin(), body.nodeIds1Based.end());
			body.nodeIds1Based.erase(std::unique(body.nodeIds1Based.begin(), body.nodeIds1Based.end()), body.nodeIds1Based.end());
			body.name = QStringLiteral("%1_Gear%2").arg(baseName).arg(i + 1);

			QMap<QString, QVector<int>> firstFace;
			QMap<QString, int> faceCount;
			for (int cellIdx : body.cellIndices)
			{
				const int eid = (cellIdx >= 0 && cellIdx < vtkCellToExportEid.size()) ? vtkCellToExportEid[cellIdx] : 0;
				vtkCell *cell = grid->GetCell(cellIdx);
				if (cell == nullptr)
					continue;
				const int ctype = cell->GetCellType();
				if (eid > 0 && isSolidCellType(ctype))
					body.elemIds1Based.append(eid);
				if (isShellCellType(ctype))
				{
					vtkIdList *pids = cell->GetPointIds();
					if (pids == nullptr)
						continue;
					QVector<int> face;
					const int count = (ctype == VTK_TRIANGLE) ? 3 : 4;
					for (int j = 0; j < count; ++j)
						face.append(static_cast<int>(pids->GetId(j) + 1));
					if (count == 3)
						face.append(face.last());
					body.boundaryFaces1Based.append(face);
					continue;
				}
				auto *c3d = vtkCell3D::SafeDownCast(cell);
				if (c3d == nullptr)
					continue;
				const int faceNum = c3d->GetNumberOfFaces();
				for (int fi = 0; fi < faceNum; ++fi)
				{
					vtkCell *faceCell = c3d->GetFace(fi);
					if (faceCell == nullptr)
						continue;
					vtkIdList *pids = faceCell->GetPointIds();
					if (pids == nullptr)
						continue;
					QVector<int> face;
					for (vtkIdType j = 0; j < pids->GetNumberOfIds() && j < 4; ++j)
						face.append(static_cast<int>(pids->GetId(j) + 1));
					if (face.size() == 3)
						face.append(face.last());
					if (face.size() != 4)
						continue;
					QVector<int> sortedFace = face;
					std::sort(sortedFace.begin(), sortedFace.end());
					QString key = QStringLiteral("%1_%2_%3_%4").arg(sortedFace[0]).arg(sortedFace[1]).arg(sortedFace[2]).arg(sortedFace[3]);
					faceCount[key] += 1;
					if (!firstFace.contains(key))
						firstFace.insert(key, face);
				}
			}

			if (body.hasSolid)
			{
				for (auto it = faceCount.constBegin(); it != faceCount.constEnd(); ++it)
				{
					if (it.value() == 1 && firstFace.contains(it.key()))
						body.boundaryFaces1Based.append(firstFace.value(it.key()));
				}
			}
			std::sort(body.elemIds1Based.begin(), body.elemIds1Based.end());
			body.elemIds1Based.erase(std::unique(body.elemIds1Based.begin(), body.elemIds1Based.end()), body.elemIds1Based.end());
		}

		assignBodyExportIds(bodies);
		return true;
	}

	void KEYdataExchange::assignBodyExportIds(QList<KEYdataExchange::ExportBodyInfo> &bodies)
	{
		int nextPid = 1;
		for (int i = 0; i < bodies.size(); ++i)
		{
			auto &body = bodies[i];
			if (body.hasSolid)
				body.solidPid = nextPid++;
			if (kExportShellElements && body.hasShell)
				body.shellPid = nextPid++;
			body.nodeSetId = 1000 + i + 1;
			body.elemSetId = 2000 + i + 1;
			body.segmentSetId = 3000 + i + 1;
		}
	}

	void KEYdataExchange::writePartCards(const QList<KEYdataExchange::ExportBodyInfo> &bodies, int secidSolid, int secidShell, int mid)
	{
		auto writeOnePart = [&](int pid, int secid, const QString &title) {
			*_stream << "*PART" << endl;
			*_stream << hmPadRight(title, 80) << endl;
			*_stream << "$     PID     SECID       MID     EOSID      HGID      GRAV    ADPOPT      TMID" << endl;
			*_stream << hmInt10(pid)
					 << hmInt10(secid)
					 << hmInt10(mid)
					 << hmInt10(0)
					 << hmInt10(0)
					 << hmInt10(0)
					 << hmInt10(0)
					 << hmInt10(0)
					 << endl;
		};

		for (const auto &body : bodies)
		{
			if (body.hasSolid && secidSolid > 0 && body.solidPid > 0)
				writeOnePart(body.solidPid, secidSolid, body.name + QStringLiteral("_solid"));
			if (kExportShellElements && body.hasShell && secidShell > 0 && body.shellPid > 0)
				writeOnePart(body.shellPid, secidShell, body.name + QStringLiteral("_shell"));
		}
	}

	bool KEYdataExchange::validateExportDeck(vtkDataSet *grid, const QVector<int> &vtkCellToExportEid,
											const QVector<int> &vtkCellToBody, const QList<KEYdataExchange::ExportBodyInfo> &bodies,
											int secidSolid, int secidShell, int mid)
	{
		ExportValidationState state;
		const vtkIdType nPts = grid->GetNumberOfPoints();
		for (vtkIdType i = 0; i < nPts; ++i)
			state.nodeIds.insert(static_cast<int>(i + 1));

		if (mid > 0)
			state.matIds.insert(mid);
		if (secidSolid > 0)
			state.secIds.insert(secidSolid);
		if (kExportShellElements && secidShell > 0)
			state.secIds.insert(secidShell);
		for (const auto &body : bodies)
		{
			if (body.solidPid > 0)
				state.partIds.insert(body.solidPid);
			if (body.shellPid > 0)
				state.partIds.insert(body.shellPid);
		}

		if (state.partIds.isEmpty() || state.secIds.isEmpty() || state.matIds.isEmpty())
			return false;

		auto fail = [&](const QString &msg) -> bool {
			emit showInformation(msg);
			return false;
		};

		const int nCells = grid->GetNumberOfCells();
		for (int i = 0; i < nCells && i < vtkCellToExportEid.size(); ++i)
		{
			const int eid = vtkCellToExportEid[i];
			if (eid <= 0)
				continue;
			if (state.elemIds.contains(eid))
				return fail(tr("KEY export validation failed: duplicate element id %1").arg(eid));
			state.elemIds.insert(eid);

			vtkCell *cell = grid->GetCell(i);
			if (cell == nullptr)
				return fail(tr("KEY export validation failed: null cell at index %1").arg(i));

			const int ctype = cell->GetCellType();
			const bool isShell = isShellCellType(ctype);
			const bool isSolid = isSolidCellType(ctype);
			if (!isShell && !isSolid)
				return fail(tr("KEY export validation failed: unsupported cell type %1").arg(ctype));
			if (isShell && !kExportShellElements)
				continue;

			const int bodyIndex = (i >= 0 && i < vtkCellToBody.size()) ? vtkCellToBody[i] : -1;
			if (bodyIndex < 0 || bodyIndex >= bodies.size())
				return fail(tr("KEY export validation failed: cell %1 has no body id").arg(i));
			const auto &body = bodies.at(bodyIndex);
			const int pid = isShell ? body.shellPid : body.solidPid;
			const int secid = isShell ? secidShell : secidSolid;
			if (!state.partIds.contains(pid))
				return fail(tr("KEY export validation failed: missing PID %1").arg(pid));
			if (!state.secIds.contains(secid))
				return fail(tr("KEY export validation failed: missing SECID %1").arg(secid));
			if (!state.matIds.contains(mid))
				return fail(tr("KEY export validation failed: missing MID %1").arg(mid));

			vtkIdList *pids = cell->GetPointIds();
			if (pids == nullptr)
				return fail(tr("KEY export validation failed: null point id list for element %1").arg(eid));

			int nCorner = 0;
			if (ctype == VTK_TRIANGLE)
				nCorner = 3;
			else if (ctype == VTK_QUAD)
				nCorner = 4;
			else if (ctype == VTK_TETRA || ctype == VTK_QUADRATIC_TETRA)
				nCorner = 4;
			else
				nCorner = 8;

			if (!pointIdsInRange(pids, nPts, nCorner))
				return fail(tr("KEY export validation failed: element %1 references missing node").arg(eid));
		}
		return true;
	}

	bool KEYdataExchange::writePointPart(vtkDataSet *grid, const QVector<int> &vtkPointToExportNid)
	{
		*_stream << "*NODE" << endl;
		*_stream << "$    NID               X               Y               Z      TC      RC" << endl;
		if (grid == nullptr)
			return false;
		const vtkIdType numOfPoint = grid->GetNumberOfPoints();
		// *NODE：NID 与坐标必须对应同一 VTK 点下标 vtkPointId。
		// 约定（与 buildVtkPointToExportNidMap 及单元 connectivity 一致）：NID = vtkPointId + 1。
		// 使用 GetPoint(vtkId, double[3])，禁止依赖 GetPoint 返回的指针与循环变量不同步。
		for (vtkIdType vtkPid = 0; vtkPid < numOfPoint; ++vtkPid)
		{
			if (!_threadRuning)
				return false;
			const int nidCanonical = static_cast<int>(vtkPid) + 1;
			if (static_cast<int>(vtkPid) < vtkPointToExportNid.size())
			{
				const int nidFromMap = vtkPointToExportNid.at(static_cast<int>(vtkPid));
				if (nidFromMap > 0 && nidFromMap != nidCanonical)
					qWarning() << "[KEY] vtkPointToExportNid mismatch at vtkPid" << vtkPid
							   << ": map" << nidFromMap << "canonical" << nidCanonical
							   << "- *NODE uses canonical NID + GetPoint(same vtkPid)";
			}

			double pt[3]{};
			grid->GetPoint(vtkPid, pt);
			*_stream << hmInt8(nidCanonical)
					 << hmFloat16(pt[0])
					 << hmFloat16(pt[1])
					 << hmFloat16(pt[2])
					 << QString(16, QChar(' '))
					 << endl;
		}
		return true;
	}

	bool KEYdataExchange::writeShellPart(vtkDataSet *grid, const QVector<int> &vtkCellToExportEid,
										const QVector<int> &vtkCellToBody, const QList<KEYdataExchange::ExportBodyInfo> &bodies,
										const QVector<int> &vtkPointToExportNid)
	{
		if (!kExportShellElements)
			return true;
		const int numOfCell = grid->GetNumberOfCells();
		const vtkIdType nPts = grid->GetNumberOfPoints();
		bool hasShell = false;
		for (int i = 0; i < numOfCell && i < vtkCellToExportEid.size(); ++i)
		{
			vtkCell *c = grid->GetCell(i);
			if (c == nullptr)
				continue;
			const int ct = c->GetCellType();
			if (isShellCellType(ct) && vtkCellToExportEid[i] > 0)
			{
				hasShell = true;
				break;
			}
		}
		if (!hasShell)
			return true;

		*_stream << "*ELEMENT_SHELL" << endl;
		*_stream << "$    EID     PID      N1      N2      N3      N4" << endl;
		for (int index = 0; index < numOfCell; ++index)
		{
			if (!_threadRuning)
				return false;
			if (index >= vtkCellToExportEid.size())
				break;
			const int eid = vtkCellToExportEid[index];
			if (eid <= 0)
				continue;
			vtkCell *cell = grid->GetCell(index);
			if (cell == nullptr)
				continue;
			vtkIdList *pids = cell->GetPointIds();
			if (pids == nullptr)
				continue;
			const int ctype = cell->GetCellType();
			if (ctype == VTK_QUAD)
			{
				if (!pointIdsInRange(pids, nPts, 4))
					continue;
				const int bodyIndex = (index >= 0 && index < vtkCellToBody.size()) ? vtkCellToBody[index] : -1;
				if (bodyIndex < 0 || bodyIndex >= bodies.size())
					continue;
				const int partPid = bodies.at(bodyIndex).shellPid;
				if (partPid <= 0)
					continue;
				QString line;
				line += hmInt8(eid);
				line += hmInt8(partPid);
				for (int j = 0; j < 4; ++j)
				{
					const vtkIdType pid = pids->GetId(j);
					const int nid = (pid >= 0 && pid < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid)) : 0;
					line += hmInt8(nid);
				}
				*_stream << line << endl;
			}
			else if (ctype == VTK_TRIANGLE)
			{
				if (!pointIdsInRange(pids, nPts, 3))
					continue;
				const int bodyIndex = (index >= 0 && index < vtkCellToBody.size()) ? vtkCellToBody[index] : -1;
				if (bodyIndex < 0 || bodyIndex >= bodies.size())
					continue;
				const int partPid = bodies.at(bodyIndex).shellPid;
				if (partPid <= 0)
					continue;
				QString line;
				line += hmInt8(eid);
				line += hmInt8(partPid);
				const vtkIdType pid1 = pids->GetId(0);
				const vtkIdType pid2 = pids->GetId(1);
				const vtkIdType pid3 = pids->GetId(2);
				const int n1 = (pid1 >= 0 && pid1 < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid1)) : 0;
				const int n2 = (pid2 >= 0 && pid2 < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid2)) : 0;
				const int n3 = (pid3 >= 0 && pid3 < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid3)) : 0;
				line += hmInt8(static_cast<qint64>(n1));
				line += hmInt8(static_cast<qint64>(n2));
				line += hmInt8(static_cast<qint64>(n3));
				line += hmInt8(static_cast<qint64>(n3));
				*_stream << line << endl;
			}
		}
		return true;
	}

	bool KEYdataExchange::writeCellPart(vtkDataSet *grid, const QVector<int> &vtkCellToExportEid,
									   const QVector<int> &vtkCellToBody, const QList<KEYdataExchange::ExportBodyInfo> &bodies,
									   const QVector<int> &vtkPointToExportNid)
	{
		const int numOfCell = grid->GetNumberOfCells();
		const vtkIdType nPts = grid->GetNumberOfPoints();

		bool headerDone = false;
		for (int index = 0; index < numOfCell; ++index)
		{
			if (!_threadRuning)
				return false;
			if (index >= vtkCellToExportEid.size())
				break;
			const int eid = vtkCellToExportEid[index];
			if (eid <= 0)
				continue;
			vtkCell *cell = grid->GetCell(index);
			if (cell == nullptr)
				continue;
			const int ctype = cell->GetCellType();
			if (!isSolidCellType(ctype))
				continue;
			vtkIdList *PointIdIndexs = cell->GetPointIds();
			if (PointIdIndexs == nullptr)
				continue;
			const int nCorner = (ctype == VTK_TETRA || ctype == VTK_QUADRATIC_TETRA) ? 4 : 8;
			if (!pointIdsInRange(PointIdIndexs, nPts, nCorner))
				continue;
			const int bodyIndex = (index >= 0 && index < vtkCellToBody.size()) ? vtkCellToBody[index] : -1;
			if (bodyIndex < 0 || bodyIndex >= bodies.size())
				continue;
			const int partPid = bodies.at(bodyIndex).solidPid;
			if (partPid <= 0)
				continue;

			if (!headerDone)
			{
				*_stream << "*ELEMENT_SOLID" << endl;
				*_stream << "$    EID     PID      N1      N2      N3      N4      N5      N6      N7      N8" << endl;
				headerDone = true;
			}

			QString line;
			line += hmInt8(eid);
			line += hmInt8(partPid);

			if (ctype == VTK_HEXAHEDRON)
			{
				for (int j = 0; j < 8; ++j)
				{
					const vtkIdType pid = PointIdIndexs->GetId(j);
					const int nid = (pid >= 0 && pid < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid)) : 0;
					line += hmInt8(nid);
				}
			}
			else if (ctype == VTK_VOXEL)
			{
				for (int j = 0; j < 8; ++j)
				{
					const vtkIdType pid = PointIdIndexs->GetId(vtkVoxelToHexVertex[j]);
					const int nid = (pid >= 0 && pid < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid)) : 0;
					line += hmInt8(nid);
				}
			}
			else if (ctype == VTK_TETRA || ctype == VTK_QUADRATIC_TETRA)
			{
				for (int j = 0; j < 4; ++j)
				{
					const vtkIdType pid = PointIdIndexs->GetId(j);
					const int nid = (pid >= 0 && pid < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid)) : 0;
					line += hmInt8(nid);
				}
				const vtkIdType pid4 = PointIdIndexs->GetId(3);
				const int n4 = (pid4 >= 0 && pid4 < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid4)) : 0;
				for (int j = 0; j < 4; ++j)
					line += hmInt8(static_cast<qint64>(n4));
			}
			else if (ctype == VTK_QUADRATIC_HEXAHEDRON || ctype == VTK_TRIQUADRATIC_HEXAHEDRON ||
					 ctype == VTK_BIQUADRATIC_QUADRATIC_HEXAHEDRON)
			{
				for (int j = 0; j < 8; ++j)
				{
					const vtkIdType pid = PointIdIndexs->GetId(j);
					const int nid = (pid >= 0 && pid < vtkPointToExportNid.size()) ? vtkPointToExportNid.at(static_cast<int>(pid)) : 0;
					line += hmInt8(nid);
				}
			}
			*_stream << line << endl;
		}
		return true;
	}

	void KEYdataExchange::writeHmSetNodeList(int sid, const QString &name, const QList<int> &nodeIds1Based)
	{
		// HyperMesh 模板：每行 8 个节点号，每个字段固定 10 列右对齐（与 HM 导出 stan.k 一致，非 8 列）
		constexpr int kHmSetNodeListFieldWidth = 10;
		constexpr int kHmSetNodeListColsPerRow = 8;

		*_stream << "*SET_NODE_LIST" << endl;
		*_stream << "$HMSET" << endl;
		*_stream << hmPadRight(QStringLiteral("$HMNAME SETS       %1%2").arg(sid).arg(name), 80) << endl;
		*_stream << "$      SID       DA1       DA2       DA3       DA4SOLVER    " << endl;
		*_stream << hmInt10(sid)
				 << QString(40, QChar(' '))
				 << QStringLiteral("MECH")
				 << endl;
		int i = 0;
		const int n = nodeIds1Based.size();
		while (i < n)
		{
			QString row;
			int col = 0;
			while (i < n && col < kHmSetNodeListColsPerRow)
			{
				row += QString::number(nodeIds1Based.at(i)).rightJustified(kHmSetNodeListFieldWidth, QChar(' '));
				++i;
				++col;
			}
			*_stream << row << endl;
		}
	}

	void KEYdataExchange::writeHmSetSolid(int sid, const QString &name, const QList<int> &elemIds1Based)
	{
		*_stream << "*SET_SOLID" << endl;
		*_stream << "$HMSET" << endl;
		*_stream << hmPadRight(QStringLiteral("$HMNAME SETS       %1%2").arg(sid).arg(name), 80) << endl;
		*_stream << "$      SIDSOLVER    " << endl;
		*_stream << QStringLiteral("         %1          ").arg(sid) << endl;
		int i = 0;
		const int n = elemIds1Based.size();
		while (i < n)
		{
			QString row = QStringLiteral("    ");
			int col = 0;
			while (i < n && col < 8)
			{
				row += QString::number(elemIds1Based.at(i)).rightJustified(8, QChar(' '));
				++i;
				++col;
			}
			*_stream << row << endl;
		}
	}

	void KEYdataExchange::writeSegmentSetByFaces(int sid, const QString &name, const QVector<QVector<int>> &faces1Based)
	{
		if (faces1Based.isEmpty())
			return;
		*_stream << "*SET_SEGMENT_TITLE" << endl;
		*_stream << name << endl;
		*_stream << "*SET_SEGMENT" << endl;
		*_stream << hmInt10(sid) << endl;
		for (const auto &face : faces1Based)
		{
			if (face.size() != 4)
				continue;
			QString row;
			row += hmInt10(face.at(0));
			row += hmInt10(face.at(1));
			row += hmInt10(face.at(2));
			row += hmInt10(face.at(3));
			*_stream << row << endl;
		}
	}

	void KEYdataExchange::writeSegmentSet(BoundMeshSet *bm, vtkDataSet *grid, int sid)
	{
		*_stream << "*SET_SEGMENT_TITLE" << endl;
		*_stream << bm->getName() << endl;
		*_stream << "*SET_SEGMENT" << endl;
		*_stream << QString("%1").arg(sid) << endl;
		const vtkIdType nPts = grid->GetNumberOfPoints();
		const QMap<int, QVector<int>> cf = bm->getCellFaces();
		QStringList line;
		for (auto it = cf.constBegin(); it != cf.constEnd(); ++it)
		{
			const int cidx = it.key();
			vtkCell *cell = grid->GetCell(cidx);
			if (cell == nullptr)
				continue;
			for (int fi : it.value())
			{
				auto *c3d = vtkCell3D::SafeDownCast(cell);
				if (c3d == nullptr)
					continue;
				vtkCell *faceCell = c3d->GetFace(fi);
				if (faceCell == nullptr)
					continue;
				vtkIdList *pids = faceCell->GetPointIds();
				if (pids == nullptr)
					continue;
				const int n = static_cast<int>(pids->GetNumberOfIds());
				auto nid1 = [&](int k) -> int {
					const vtkIdType pid = pids->GetId(k);
					if (pid < 0 || pid >= nPts)
						return -1;
					return static_cast<int>(pid + 1);
				};
				if (n == 3)
				{
					const int a = nid1(0), b = nid1(1), c = nid1(2);
					if (a <= 0 || b <= 0 || c <= 0)
						continue;
					line.append(QString::number(a));
					line.append(QString::number(b));
					line.append(QString::number(c));
					line.append(QString::number(c));
				}
				else if (n >= 4)
				{
					const int a = nid1(0), b = nid1(1), c = nid1(2), d = nid1(3);
					if (a <= 0 || b <= 0 || c <= 0 || d <= 0)
						continue;
					line.append(QString::number(a));
					line.append(QString::number(b));
					line.append(QString::number(c));
					line.append(QString::number(d));
				}
				else
					continue;
				if (line.size() == 8)
				{
					*_stream << line.join(", ") << endl;
					line.clear();
				}
			}
		}
		if (!line.isEmpty())
			*_stream << line.join(", ") << endl;
	}

bool KEYdataExchange::writeMeshPart(vtkDataSet *grid, const QVector<int> &vtkCellToExportEid,
								   const QList<KEYdataExchange::ExportBodyInfo> &bodies,
								   const QVector<int> &vtkPointToExportNid)
	{
		Q_UNUSED(vtkCellToExportEid);
		*_stream << "$" << endl;
		*_stream << "$ Sets Defined For Export Bodies" << endl;
		*_stream << "$" << endl;
		for (const auto &body : bodies)
		{
			if (!body.nodeIds1Based.isEmpty())
				writeHmSetNodeList(body.nodeSetId, body.name + QStringLiteral("_nodes"), body.nodeIds1Based);
			if (!body.elemIds1Based.isEmpty())
				writeHmSetSolid(body.elemSetId, body.name + QStringLiteral("_elements"), body.elemIds1Based);
			if (!body.boundaryFaces1Based.isEmpty())
				writeSegmentSetByFaces(body.segmentSetId, body.name + QStringLiteral("_surface"), body.boundaryFaces1Based);
		}

		int nextBoreSetId = 4001;
		auto writeExplicitBoreNodeSetByCandidates = [&](const QStringList &candidateNames)
		{
			qDebug() << "[KEY-DEBUG] Begin bore set export. candidates=" << candidateNames
					 << ", modelId(kernelId)=" << _modelId;
			const int setCount = _meshData->getMeshSetCount();
			bool exported = false;
			for (const QString &setName : candidateNames)
			{
				for (int i = 0; i < setCount; ++i)
				{
					auto *set = _meshData->getMeshSetAt(i);
					if (set == nullptr || set->getSetType() != Node)
						continue;
					if (set->getName() != setName)
						continue;

					const QList<int> kernelIds = set->getKernals();
					qDebug() << "[KEY-DEBUG] Matched set name. setId=" << set->getID()
							 << ", setName=" << setName << ", kernels=" << kernelIds;

					if (!set->isContainsKernal(_modelId))
					{
						qDebug() << "[KEY-DEBUG] Skip: set has no members under current kernel."
								 << " requiredKernel=" << _modelId;
						continue;
					}

					QList<int> members = set->getKernalMembers(_modelId);
					if (members.isEmpty())
					{
						qDebug() << "[KEY-DEBUG] Skip: members empty for kernel" << _modelId;
						continue;
					}
					qDebug() << "[KEY-DEBUG] Raw members from MeshSet count=" << members.size()
							 << ", preview=" << previewIntList(members);

					// MeshSet::_members 中 Node 集第二个参数约定为：当前 MeshKernal 对应 vtkDataSet 的
					// vtkPointId（0-based）。Gmsh 自动集、findConplanarPorC 建集均符合此约定。
					// *NODE 写出：NID = vtkPointToExportNid[vtkPid]（与 vtkPid+1 一致当映射为恒等时）。
					// 此处严禁对越界 member 调用 vtkPointToExportNid.at(member)，否则会产生随机巨大 NID。
					const vtkIdType nPts = (grid == nullptr) ? 0 : grid->GetNumberOfPoints();
					QList<int> nodeIds1Based;
					nodeIds1Based.reserve(members.size());
					int skippedInvalid = 0;
					for (int member : members)
					{
						int vtkPid = -1;
						if (member >= 0 && static_cast<vtkIdType>(member) < nPts)
							vtkPid = member;
						else if (member >= 1 && static_cast<vtkIdType>(member) <= nPts)
						{
							// 兼容误存为 1-based 显示号（与 *NODE 中 NID 一致）的情况
							vtkPid = member - 1;
						}
						else
						{
							++skippedInvalid;
							continue;
						}
						if (vtkPid < 0 || vtkPid >= vtkPointToExportNid.size())
						{
							++skippedInvalid;
							continue;
						}
						const int exportNid = vtkPointToExportNid.at(vtkPid);
						if (exportNid > 0)
							nodeIds1Based.append(exportNid);
					}
					if (skippedInvalid > 0)
						qWarning() << "[KEY] gear bore set" << setName << ": skipped" << skippedInvalid
								   << "invalid member ids (not in [0," << (nPts - 1) << "] as vtk index nor [1,"
								   << nPts << "] as 1-based NID). nPts=" << nPts;
					if (nodeIds1Based.isEmpty())
					{
						qDebug() << "[KEY-DEBUG] Skip: no valid export node ids after vtk index validation.";
						continue;
					}
					std::sort(nodeIds1Based.begin(), nodeIds1Based.end());
					nodeIds1Based.erase(std::unique(nodeIds1Based.begin(), nodeIds1Based.end()), nodeIds1Based.end());
					qDebug() << "[KEY-DEBUG] Converted export NIDs count=" << nodeIds1Based.size()
							 << ", preview=" << previewIntList(nodeIds1Based)
							 << ", nPts=" << nPts << ", exportMapSize=" << vtkPointToExportNid.size();

					writeHmSetNodeList(nextBoreSetId++, setName, nodeIds1Based);
					qDebug() << "[KEY-DEBUG] Bore set written into KEY. setName=" << setName
							 << ", sid=" << (nextBoreSetId - 1);
					exported = true;
					break;
				}
				if (exported)
					break;
			}

			if (!exported)
				qDebug() << "[KEY-DEBUG] Bore set not exported. checked candidates=" << candidateNames;
		};

		// ?????????????????????????
		writeExplicitBoreNodeSetByCandidates(
			QStringList() << QStringLiteral("set1")
						  << QStringLiteral("gear_1_bore_nodes")
						  << QStringLiteral("gear1_bore_nodes"));
		writeExplicitBoreNodeSetByCandidates(
			QStringList() << QStringLiteral("set2")
						  << QStringLiteral("gear_2_bore_nodes")
						  << QStringLiteral("gear2_bore_nodes"));
		return true;
	}
}



