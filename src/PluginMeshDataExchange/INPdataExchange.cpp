#include "INPdataExchange.h"
#include "MainWindow/MainWindow.h"
#include "MeshData/meshSingleton.h"
#include "MeshData/meshKernal.h"
#include "MeshData/meshSet.h"
#include <vtkCell.h>
#include "BCBase/BCUserDef.h"
#include "ModelData/modelDataSingleton.h"
#include "ModelData/modelDataBaseExtend.h"
#include "DataProperty/ParameterGroup.h"
#include "DataProperty/ParameterDouble.h"
#include "DataProperty/ParameterString.h"
#include "DataProperty/PropertyString.h"
#include "Material/Material.h"
#include "ConfigOptions/ConfigOptions.h"
#include "ConfigOptions/MaterialConfig.h"
#include "Material/MaterialFactory.h"
#include "Material/MaterialSingletion.h"
#include "ModuleBase/ThreadTaskManager.h"
#include <vtkUnstructuredGrid.h>
#include <QFileInfo>
#include <QMessageBox>
#include <QDebug>
#include <omp.h>
#include <QFileDialog>

#ifdef Q_OS_WIN
#define ENDL "\r\n"
#else
#define ENDL "\n"
#endif

namespace
{
	QString abaqusFaceLabel(vtkCell *c, int vtkFaceIdx)
	{
		if (c == nullptr)
			return QStringLiteral("S1");
		switch (c->GetCellType())
		{
		case VTK_HEXAHEDRON:
		{
			static const char *labels[] = {"S1", "S2", "S3", "S4", "S5", "S6"};
			if (vtkFaceIdx >= 0 && vtkFaceIdx < 6)
				return QString::fromLatin1(labels[vtkFaceIdx]);
		}
		break;
		case VTK_TETRA:
		{
			static const char *labels[] = {"S1", "S2", "S3", "S4"};
			if (vtkFaceIdx >= 0 && vtkFaceIdx < 4)
				return QString::fromLatin1(labels[vtkFaceIdx]);
		}
		break;
		default:
			break;
		}
		return QStringLiteral("S1");
	}

	QString abaqusSurfName(const QString &s)
	{
		QString t = s;
		t.replace(QLatin1Char(' '), QLatin1Char('_'));
		return t;
	}

	void buildInpSolidElementMap(vtkDataSet *data, QVector<int> &cellToElem, int &nSolid)
	{
		nSolid = 0;
		if (data == nullptr)
			return;
		const int n = data->GetNumberOfCells();
		cellToElem.resize(n);
		cellToElem.fill(-1);
		int eid = 0;
		for (int i = 0; i < n; ++i)
		{
			vtkCell *c = data->GetCell(i);
			if (c == nullptr)
				continue;
			if (c->GetCellType() == VTK_TETRA)
			{
				eid++;
				cellToElem[i] = eid;
			}
		}
		for (int i = 0; i < n; ++i)
		{
			vtkCell *c = data->GetCell(i);
			if (c == nullptr)
				continue;
			if (c->GetCellType() == VTK_HEXAHEDRON)
			{
				eid++;
				cellToElem[i] = eid;
			}
		}
		nSolid = eid;
	}
} // namespace

namespace MeshData
{
	INPdataExchange::INPdataExchange(const QString &fileName, MeshOperation operation, GUI::MainWindow *mw, int modelId) : MeshThreadBase(fileName, operation, mw),
																														   _fileName(fileName),
																														   _meshData(MeshData::getInstance()),
																														   _operation(operation),
																														   _modelId(modelId),
																														   _mw(mw)
	{
		if (modelId != -1)
		{
			ModelData::ModelDataBase *model = ModelData::ModelDataSingleton::getinstance()->getModelByID(modelId);
			_Case = dynamic_cast<ModelData::ModelDataBaseExtend *>(model);
		}
	}
	INPdataExchange::~INPdataExchange()
	{
		if (_stream != nullptr)
			delete _stream;
	}

	// 	void INPdataExchange::destroyThread()
	// 	{
	// 		ThreadTask::destroyThread();
	// 		if (_stream != nullptr) delete _stream;
	// 	}

	void INPdataExchange::readLine(QString &line)
	{
		while (_threadRuning && !_stream->atEnd())
		{
			// xuxinwie  20200519
			// 			if (_stream->atEnd())
			// 			{
			// 				//_threadRuning = false;
			// 				break;
			// 			}

			line = _stream->readLine().simplified().toLower();
			if (line.startsWith("**") && line.size() == 2)
				continue;
			break;
		}
	}

	bool INPdataExchange::read()
	{
		QFileInfo info(_fileName);
		if (!info.exists())
			return false;
		QString name = info.fileName();
		QString path = info.filePath();
		QFile file(_fileName);
		if (!file.open(QIODevice::ReadOnly))
			return false;
		_stream = new QTextStream(&file);
		vtkUnstructuredGrid *dataset = vtkUnstructuredGrid::New();
		MeshKernal *k = new MeshKernal;
		QString line;

		QList<int> inpSetIds;
		QStringList materialName, density, elastic;
		QStringList bcSetIds, bcName, bcType;
		QList<double> displacement, rotation;

		do
		{
			if (!_threadRuning)
			{
				file.close();
				return false;
			}
			this->readLine(line);
			if (line.startsWith("*node") /*&& line.size() < 6*/)
			{
				// 				if (!readNodes(dataset, line))
				// 				{
				// 					dataset->Delete();
				// 					file.close();
				// 					return false;
				// 				}
				if (!readNode(dataset, line, inpSetIds, k))
				{
					delete k;
					dataset->Delete();
					file.close();
					return false;
				}
			}
			if (line.startsWith("*element"))
			{
				// 				if (!readElements(dataset, line))
				// 				{
				// 					delete k;
				// 					dataset->Delete();
				// 					file.close();
				// 					return false;
				// 				}
				if (!readElement(dataset, line, inpSetIds, k))
				{
					delete k;
					dataset->Delete();
					file.close();
					return false;
				}

				if (dataset != nullptr)
				{
					/*MeshKernal* k = new MeshKernal;*/
					k->setName(name);
					k->setPath(path);
					k->setMeshData((vtkDataSet *)dataset);
					_meshData->appendMeshKernal(k);
				}
			}
			if (line.startsWith("*nset"))
			{
				this->readNSet(line, inpSetIds);
			}
			if (line.startsWith("*elset"))
			{
				this->readElSet(line, inpSetIds);
			}

			// xuxinwie  20200519
			// if (_stream->atEnd())
			//	_threadRuning = false;
			if (_modelId == -1)
				continue;
			if (line.startsWith("*material"))
				readMaterial(line);
			// readMaterial(line, materialName, density, elastic);
			if (line.startsWith("** boundary"))
				readBoundary(line, bcSetIds, bcName, bcType, displacement, rotation);

		} while (_threadRuning && !_stream->atEnd());

		if (_modelId != -1)
		{
			addINPComponents(inpSetIds);
			// addINPMaterials(materialName, density, elastic);
			addINPMaterials();
			addINPBCs(bcSetIds, bcName, bcType, displacement, rotation);
			emit _mw->updatePhysicsTreeSignal();
		}
		file.close();
		return true;
	}

	bool INPdataExchange::write()
	{
		int kId = -1;
		vtkDataSet *data = nullptr;

		if (_modelId == -1)
		{
			// 点击状态栏按钮导出：默认导出最后一个网格核（避免直接返回导致0字节文件）
			const int kc = _meshData->getKernalCount();
			if (kc <= 0)
				return false;
			auto k = _meshData->getKernalAt(kc - 1);
			if (k == nullptr)
				return false;
			kId = k->getID();
			data = k->getMeshData();
		}
		else
		{
			// 点击Case中菜单导出：仅导出当前Case绑定的一个kernal
			if (_Case == nullptr || _Case->getMeshKernalList().size() != 1)
				return false;
			kId = _Case->getMeshKernalList().at(0);
			auto k = _meshData->getKernalByID(kId);
			if (k == nullptr)
				return false;
			data = k->getMeshData();
		}

		if (!data)
			return false;

		QFile file(_fileName);
		if (!file.open(QIODevice::Truncate | QIODevice::WriteOnly))
			return false;
		_stream = new QTextStream(&file);

		*_stream << "*Heading" << endl;
		*_stream << "** FastCAE mesh export" << endl;
		*_stream << "*Part, name=PART-1" << endl;

		QVector<int> cellToElem;
		int nSolid = 0;
		buildInpSolidElementMap(data, cellToElem, nSolid);

		writePoint(data);
		writeCell(data, cellToElem);
		writeAllSolidElset(nSolid);
		writeMeshSetsForKernel(kId, cellToElem);
		writeBoundSurfaces(data, kId, cellToElem);
		if (_Case != nullptr && !_Case->getInpMaterialIds().isEmpty())
			writeSolidSection();

		*_stream << "*End Part" << endl;
		*_stream << "*Assembly, name=ASSEMBLY-1" << endl;
		*_stream << "*Instance, name=PART-1-1, part=PART-1" << endl;
		*_stream << "*End Instance" << endl;
		*_stream << "*End Assembly" << endl;

		if (_Case != nullptr)
		{
			writeMaterial();
			writeBoundary();
		}

		file.close();
		return true;
	}

	bool INPdataExchange::readNode(vtkUnstructuredGrid *g, QString &line, QList<int> &inpSetIds, MeshKernal *k)
	{
		if (line.size() < 6)
		{
			if (!readNodes(g, line))
				return false;
		}
		else if (line.contains("nset"))
		{
			if (k == nullptr)
				return false;
			const int kid = k->getID();

			QStringList sinfo = line.split(",");
			QString name = sinfo.at(1).simplified();
			name.remove("nset=");

			MeshSet *set = new MeshSet(name, Node);

			vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
			bool ok = false;
			int index = 0;
			while (_threadRuning && !_stream->atEnd())
			{
				if (!_threadRuning)
					return false;
				readLine(line);
				if (line.contains("*"))
					break;

				double coor[3] = {0};
				QStringList scoor = line.split(",");
				if (scoor.size() != 4)
					continue;
				for (int j = 0; j < 3; ++j)
				{
					coor[j] = scoor.at(j + 1).toDouble(&ok);
					if (!ok)
						return false;
				}

				points->InsertNextPoint(coor);
				int id = scoor.at(0).toInt(&ok);
				if (ok)
					_nodeIDIndex[id] = index;
				index++;

				set->appendMember(kid, index);
			}

			_meshData->appendMeshSet(set);
			inpSetIds.append(set->getID());

			if (points->GetNumberOfPoints() > 2)
			{
				g->SetPoints(points);
				return true;
			}
			return false;
		}

		return true;
	}

	bool INPdataExchange::readNodes(vtkUnstructuredGrid *g, QString &line)
	{
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		bool ok = false;
		int index = 0;
		while (_threadRuning && !_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			readLine(line);
			if (line.contains("*"))
				break;

			double coor[3] = {0};
			QStringList scoor = line.split(",");
			if (scoor.size() != 4)
				continue;
			for (int j = 0; j < 3; ++j)
			{
				coor[j] = scoor.at(j + 1).toDouble(&ok);
				if (!ok)
					return false;
			}

			points->InsertNextPoint(coor);
			int id = scoor.at(0).toInt(&ok);
			if (ok)
				_nodeIDIndex[id] = index;
			index++;

			// xuxinwie  20200519
			// 			if (_stream->atEnd())
			// 				_threadRuning = false;
		}

		if (points->GetNumberOfPoints() > 2)
		{
			g->SetPoints(points);
			return true;
		}
		return false;
	}

	bool INPdataExchange::readElement(vtkUnstructuredGrid *g, QString &line, QList<int> &inpSetIds, MeshKernal *k)
	{
		if (line.contains("elset"))
		{
			if (k == nullptr)
				return false;
			const int kid = k->getID();

			QStringList stype = line.split(",");
			QString st = stype.at(1);
			st = st.remove("type=").remove(" ").simplified();
			VTKCellType tp = VTK_EMPTY_CELL;

			if (st == "c3d4")
			{
				tp = VTK_TETRA;
			}
			else if (st == "f3d8")
			{
				tp = VTK_HEXAHEDRON;
			}

			if (tp == VTK_EMPTY_CELL)
				return false;

			QStringList sinfo = line.split(",");
			QString name = sinfo.at(1).simplified();
			name.remove("elset=");

			MeshSet *set = new MeshSet(name, Element);

			bool ok = false;
			int index = 0;

			while (_threadRuning && !_stream->atEnd())
			{
				if (!_threadRuning)
					return false;
				this->readLine(line);
				if (line.startsWith("*"))
					break;
				vtkSmartPointer<vtkIdList> indexList = vtkSmartPointer<vtkIdList>::New();

				QStringList sids = line.split(",");
				for (int j = 1; j < sids.size(); ++j)
				{
					int nodeid = sids.at(j).toInt(&ok);
					assert(ok);
					int nodeindex = _nodeIDIndex.value(nodeid);
					indexList->InsertNextId(nodeindex);
				}

				g->InsertNextCell(tp, indexList);
				int eleID = sids.at(0).toInt(&ok);
				_elemIDIndex[eleID] = index;
				index++;

				set->appendMember(kid, index);
			}

			_meshData->appendMeshSet(set);
			inpSetIds.append(set->getID());
		}
		else
			readElements(g, line);

		return true;
	}

	bool INPdataExchange::readElements(vtkUnstructuredGrid *g, QString &line)
	{
		QStringList stype = line.split(",");
		// if (stype.size() != 2) return false;
		QString st = stype.at(1);
		st = st.remove("type=").remove(" ").simplified();
		VTKCellType tp = VTK_EMPTY_CELL;
		//		qDebug() << st;
		if (st == "c3d4")
		{
			tp = VTK_TETRA;
		}
		else if (st == "f3d8")
		{
			tp = VTK_HEXAHEDRON;
		}

		if (tp == VTK_EMPTY_CELL)
			return false;

		bool ok = false;
		int index = 0;

		while (_threadRuning && !_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			this->readLine(line);
			if (line.startsWith("*"))
				break;
			vtkSmartPointer<vtkIdList> indexList = vtkSmartPointer<vtkIdList>::New();

			QStringList sids = line.split(",");
			for (int j = 1; j < sids.size(); ++j)
			{
				int nodeid = sids.at(j).toInt(&ok);
				assert(ok);
				int nodeindex = _nodeIDIndex.value(nodeid);
				indexList->InsertNextId(nodeindex);
			}

			g->InsertNextCell(tp, indexList);
			int eleID = sids.at(0).toInt(&ok);
			_elemIDIndex[eleID] = index;
			index++;

			// xuxinwie  20200519
			// 			if (_stream->atEnd())
			// 				_threadRuning = false;
		}
		return true;
	}

	bool INPdataExchange::readNSet(QString &line, QList<int> &inpSetIds)
	{
		bool isgen = false;
		if (line.contains("generate"))
			isgen = true;

		QStringList sinfo = line.split(",");
		QString name = sinfo.at(1).simplified();
		name.remove("nset=");

		if (isgen)
			name = name + "_gen";

		MeshSet *set = new MeshSet(name, Node);
		const int c = _meshData->getKernalCount();
		if (c <= 0)
			return false;
		MeshKernal *k = _meshData->getKernalAt(c - 1);
		if (k == nullptr)
			return false;
		// 		vtkDataSet* dataset = k->getMeshData();
		// 		set->setDataSet(dataset);
		const int kid = k->getID();

		//		vtkSmartPointer<vtkIdTypeArray> array = vtkSmartPointer<vtkIdTypeArray>::New();
		while (_threadRuning && !_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			this->readLine(line);
			if (line.startsWith("*"))
			{
				//				set->setIDList(array);
				_meshData->appendMeshSet(set);
				inpSetIds.append(set->getID());
			}
			if (line.startsWith("*nset"))
			{
				this->readNSet(line, inpSetIds);
			}
			if (line.startsWith("*elset"))
			{
				this->readElSet(line, inpSetIds);
			}
			if (line.startsWith("*") && (!line.startsWith("*nset")))
				return true;

			QStringList sid = line.split(",");
			if (isgen)
			{
				int beg = sid.at(0).toInt();
				int end = sid.at(1).toInt();
				for (int i = beg; i <= end; ++i)
				{
					if (!_nodeIDIndex.contains(i))
						continue;
					int index = _nodeIDIndex.value(i);
					//					array->InsertNextValue(index);
					set->appendMember(kid, index);
				}
				continue;
			}

			for (int i = 0; i < sid.size(); ++i)
			{
				if (sid.at(i).size() == 0)
					continue;
				int id = sid.at(i).toInt();
				int index = _nodeIDIndex.value(id);
				//				array->InsertNextValue(index);
				set->appendMember(kid, index);
			}
		}
		return true;
	}

	bool INPdataExchange::readElSet(QString &line, QList<int> &inpSetIds)
	{
		bool isgen = false;
		if (line.contains("generate"))
			isgen = true;

		QStringList sinfo = line.split(",");
		QString name = sinfo.at(1).simplified().remove("elset=");

		if (isgen)
			name = name + "_gen";

		MeshSet *set = new MeshSet(name, Element);
		const int c = _meshData->getKernalCount();
		if (c <= 0)
			return false;
		MeshKernal *k = _meshData->getKernalAt(c - 1);
		if (k == nullptr)
			return false;
		//		vtkDataSet* dataset = k->getMeshData();
		//		set->setDataSet(dataset);
		const int kid = k->getID();

		//		vtkSmartPointer<vtkIdTypeArray> array = vtkSmartPointer<vtkIdTypeArray>::New();
		while (_threadRuning && !_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			this->readLine(line);
			if (line.startsWith("*"))
			{
				//				set->setIDList(array);
				_meshData->appendMeshSet(set);
				inpSetIds.append(set->getID());
			}
			if (line.startsWith("*elset"))
			{
				this->readElSet(line, inpSetIds);
			}
			if (line.startsWith("*nset"))
			{
				this->readNSet(line, inpSetIds);
			}
			if (line.startsWith("*") && (!line.startsWith("*elset")))
				return true;

			QStringList sid = line.split(",");
			if (isgen)
			{
				int beg = sid.at(0).toInt();
				int end = sid.at(1).toInt();
				for (int i = beg; i <= end; ++i)
				{
					if (!_elemIDIndex.contains(i))
						continue;
					int index = _elemIDIndex.value(i);
					//					array->InsertNextValue(index);
					set->appendMember(kid, index);
				}
				continue;
			}

			for (int i = 0; i < sid.size(); ++i)
			{
				if (sid.at(i).size() == 0)
					continue;
				int id = sid.at(i).toInt();
				int index = _elemIDIndex.value(id);
				//				array->InsertNextValue(index);
				set->appendMember(kid, index);
			}
		}
		return true;
	}

	bool INPdataExchange::readMaterial(QString &line, QStringList &materialName, QStringList &density, QStringList &elastic)
	{
		if (line.contains("** boundary"))
			return true;
		materialName.append(line.split('=').at(1));
		while (_threadRuning && !_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			readLine(line);
			readLine(line);
			density.append(line.left(line.size() - 1));
			readLine(line);
			readLine(line);
			if (line.contains(".,"))
				elastic.append(line.remove(".,"));
			else
				elastic.append(line.remove(','));
			//	elastic.append(line.split(","));
			readLine(line);
			if (line.startsWith("*material"))
				readMaterial(line, materialName, density, elastic);
			if (line.contains("** boundary"))
				return true;
		}
		return false;
	}

	bool INPdataExchange::readMaterial(QString &line)
	{
		// 		if (line.startsWith("**") || line.startsWith("*sloid"))
		// 			return true;
		QString materialname = line.split("=").at(1);
		QString name;
		QMultiHash<QString, QString> attibutehash;
		while (_threadRuning && !_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			this->readLine(line);
			if (line.startsWith("*material"))
			{
				m_materialHash.insert(materialname, attibutehash);
				readMaterial(line);
			}
			if (line.startsWith("**") || line.startsWith("*solid"))
			{
				m_materialHash.insert(materialname, attibutehash);
				return true;
			}
			if (line.startsWith("*"))
			{
				name = line.remove("*");
				continue;
			}
			attibutehash.insert(name, line);
		}
		return false;
	}

	bool INPdataExchange::readBoundary(QString &line, QStringList &bcSetIds, QStringList &bcName, QStringList &bcType, QList<double> &displacement, QList<double> &rotation)
	{
		if (line.startsWith("** output"))
			return true;

		QStringList lineList;
		auto bcConfig = ConfigOption::ConfigOption::getInstance()->getBCConfig();
		while (_threadRuning && !_stream->atEnd())
		{
			if (!_threadRuning)
				return false;
			if (!line.startsWith("** name"))
			{
				readLine(line);
				continue;
			}
			else
			{
				lineList = line.split(' ');
				bcName.append(lineList.at(2));
				bcType.append(lineList.at(4));
				readLine(line);
			}

			QStringList lineList;
			for (int i = 0; i < 6; i++)
			{
				readLine(line);
				lineList = line.split(',');

				auto set = _meshData->getMeshSetByName(lineList.at(0));
				if (!set)
					continue;
				bcSetIds.append(QString::number(set->getID()));

				if ((i == 0 || i == 1 || i == 2) && lineList.size() == 3)
					displacement << 0;
				else if ((i == 0 || i == 1 || i == 2) && lineList.size() == 4)
					displacement << lineList.at(3).toDouble();
				else if ((i == 3 || i == 4 || i == 5) && lineList.size() == 3)
					rotation << 0;
				else if ((i == 3 || i == 4 || i == 5) && lineList.size() == 4)
					rotation << lineList.at(3).toDouble();
			}
			bcSetIds.removeDuplicates();

			readLine(line);
			if (line.startsWith("** name"))
				readBoundary(line, bcSetIds, bcName, bcType, displacement, rotation);
			if (line.startsWith("** output"))
				return true;
		}
		return false;
	}

	void INPdataExchange::addINPComponents(const QList<int> &inpSetIds)
	{
		if (inpSetIds.size() <= 0)
			return;
		auto kId = _meshData->getMeshSetByID(inpSetIds.at(0))->getKernals();
		_Case->setMeshKernelList(kId);
		_Case->setComponentIDList(inpSetIds);
	}

	void INPdataExchange::addINPMaterials(const QStringList &materialName, const QStringList &density, const QStringList &elastic)
	{
		QList<int> inpMaIds;
		QString stype = QObject::tr("INP Material");
		//#pragma omp parallel for firstprivate(stype)
		auto materSteWard = Material::MaterialSingleton::getInstance();
		for (int i = 0; i < materialName.size(); i++)
		{
			// 			qDebug() << stype;
			// 			qDebug() << omp_get_thread_num();
			// 			qDebug() << _materialName;

			QStringList elasticList = elastic.at(i).split(" ");
			Material::Material *material = new Material::Material;
			material->setType(stype);
			material->setName(materialName.at(i));
			material->appendProperty(QObject::tr("Density"), density.at(i));
			material->appendProperty(QObject::tr("Elasticity Modulus"), QString(elasticList.at(0)));
			material->appendProperty(QObject::tr("Poisson Ratio"), QString(elasticList.at(1)));
			materSteWard->appendMaterial(material);
			inpMaIds << material->getID();
		}
		_Case->bindInpMaterialIds(inpMaIds);
	}

	void INPdataExchange::addINPMaterials()
	{
		QList<int> inpMaIds;
		QString stype = QObject::tr("INP Material");

		auto materSteWard = Material::MaterialSingleton::getInstance();

		QStringList materiallist = m_materialHash.keys();
		for (QString name : materiallist)
		{

			Material::Material *material = new Material::Material;
			material->setType(stype);
			material->setName(name);

			QMultiHash<QString, QString> attributehash = m_materialHash.value(name);
			QStringList attributelist = attributehash.uniqueKeys();
			for (QString attribute : attributelist)
			{
				QStringList valuelist = attributehash.values(attribute);
				addINPMaterialAttribute(material, attribute, valuelist);
			}

			materSteWard->appendMaterial(material);
			inpMaIds << material->getID();
		}
		_Case->bindInpMaterialIds(inpMaIds);
	}

	void INPdataExchange::addINPMaterialAttribute(Material::Material *m, QString name, QStringList list)
	{
		for (QString para : list)
		{
			QStringList paralist = para.split(",");
			paralist.removeAll("");
			if (paralist.size() == 1)
			{
				DataProperty::ParameterBase *val = getMaterialParameter(paralist.at(0).simplified());
				val->setDescribe(name);
				/*val->setValue(paralist.at(0));*/
				m->appendParameter(val);
			}
			else
			{
				DataProperty::ParameterGroup *group = new DataProperty::ParameterGroup;
				group->setDescribe(name);

				if ((name.toLower() == "elastic") && (paralist.size() == 2))
				{
					DataProperty::ParameterBase *ela = getMaterialParameter(paralist.at(0).simplified());
					DataProperty::ParameterBase *pra = getMaterialParameter(paralist.at(1).simplified());
					ela->setDescribe(tr("Elasticity Modulus"));
					pra->setDescribe(tr("Poisson Ratio"));
					group->appendParameter(ela);
					group->appendParameter(pra);
				}
				else
				{
					for (int i = 0; i < paralist.size(); i++)
					{
						DataProperty::ParameterBase *val = getMaterialParameter(paralist.at(i).simplified());
						val->setDescribe(tr("%1_%2").arg(name).arg(i + 1));
						group->appendParameter(val);
					}
				}

				m->appendParameterGroup(group);
			}
		}
	}

	DataProperty::ParameterBase *INPdataExchange::getMaterialParameter(QString s)
	{
		bool ok = false;
		double val = s.toDouble(&ok);
		if (ok)
		{
			DataProperty::ParameterDouble *para = new DataProperty::ParameterDouble;
			para->setValue(val);
			return para;
		}
		else
		{
			DataProperty::ParameterString *para = new DataProperty::ParameterString;
			para->setValue(s);
			return para;
		}

		return nullptr;
	}

	void INPdataExchange::addINPBCs(const QStringList &bcSetIds, const QStringList &bcName, const QStringList &bcType, const QList<double> &displacement, const QList<double> &rotation)
	{
		QStringList paraDescribes;
		paraDescribes << "X"
					  << "Y"
					  << "Z";

		for (int i = 0; i < bcSetIds.size(); i++)
		{
			BCBase::BCUserDef *bc = new BCBase::BCUserDef;
			bc->bingdingComponentID(bcSetIds.at(i).toInt());
			bc->setName(bcName.at(i));
			_Case->appeendBC(bc);

			QString qdis, qrot;
			DataProperty::ParameterGroup *pagDis = new DataProperty::ParameterGroup;
			DataProperty::ParameterGroup *pagRot = new DataProperty::ParameterGroup;
			pagDis->setDescribe(bcType.at(i).split('/').at(0));
			pagRot->setDescribe(bcType.at(i).split('/').at(1));

			for (int j = 0; j < 3; j++)
			{
				qdis = QString("Displacement%1").arg(paraDescribes.at(j));
				qrot = QString("Rotation%1").arg(paraDescribes.at(j));

				auto dispara = new DataProperty::ParameterDouble;
				auto rotpara = new DataProperty::ParameterDouble;

				dispara->setDataID(_modelId);
				rotpara->setDataID(_modelId);

				dispara->setDescribe(qdis);
				rotpara->setDescribe(qrot);

				dispara->setValue(displacement.at(3 * i + j));
				rotpara->setValue(rotation.at(3 * i + j));

				pagDis->appendParameter(dispara);
				pagRot->appendParameter(rotpara);
			}

			bc->appendParameterGroup(pagDis);
			bc->appendParameterGroup(pagRot);
			bc->generateParaInfo();
		}
	}

	void INPdataExchange::writePoint(vtkDataSet *data)
	{
		*_stream << "*Node" << endl;
		int pIndex = 0;
		int nPoint = data->GetNumberOfPoints();
		double *xyz = NULL;
		QString qPointIdxyz;
		while (pIndex < nPoint)
		{
			if (!_threadRuning)
				return;
			xyz = data->GetPoint(pIndex);
			pIndex++;
			qPointIdxyz.append(QString::number(pIndex) + ",");
			qPointIdxyz.append(QString::number(xyz[0]) + ",");
			qPointIdxyz.append(QString::number(xyz[1]) + ",");
			qPointIdxyz.append(QString::number(xyz[2]) + ENDL);

			if (qPointIdxyz.size() > 1024)
			{
				*_stream << qPointIdxyz;
				qPointIdxyz.clear();
			}
			if (pIndex == nPoint && qPointIdxyz.size() > 0)
				*_stream << qPointIdxyz;
		}
	}

	void INPdataExchange::writeCell(vtkDataSet *data, const QVector<int> &cellToElem)
	{
		const int nCell = data->GetNumberOfCells();
		if (nCell <= 0)
			return;

		QString tetraLines;
		QString hexaLines;
		bool hasTetra = false;
		bool hasHexa = false;

		for (int cIndex = 0; cIndex < nCell; ++cIndex)
		{
			if (!_threadRuning)
				return;

			auto cell = data->GetCell(cIndex);
			if (cell == nullptr)
				continue;

			auto ptIdIndexs = cell->GetPointIds();
			if (ptIdIndexs == nullptr)
				continue;

			const int eid = (cIndex < cellToElem.size()) ? cellToElem[cIndex] : -1;
			if (eid <= 0)
				continue;

			const int cellType = cell->GetCellType();
			if (cellType == VTK_TETRA && ptIdIndexs->GetNumberOfIds() >= 4)
			{
				hasTetra = true;
				tetraLines.append(QString::number(eid) + ",");
				tetraLines.append(QString::number(ptIdIndexs->GetId(0) + 1) + ",");
				tetraLines.append(QString::number(ptIdIndexs->GetId(1) + 1) + ",");
				tetraLines.append(QString::number(ptIdIndexs->GetId(2) + 1) + ",");
				tetraLines.append(QString::number(ptIdIndexs->GetId(3) + 1) + ENDL);
			}
			else if (cellType == VTK_HEXAHEDRON && ptIdIndexs->GetNumberOfIds() >= 8)
			{
				hasHexa = true;
				hexaLines.append(QString::number(eid) + ",");
				hexaLines.append(QString::number(ptIdIndexs->GetId(0) + 1) + ",");
				hexaLines.append(QString::number(ptIdIndexs->GetId(1) + 1) + ",");
				hexaLines.append(QString::number(ptIdIndexs->GetId(2) + 1) + ",");
				hexaLines.append(QString::number(ptIdIndexs->GetId(3) + 1) + ",");
				hexaLines.append(QString::number(ptIdIndexs->GetId(4) + 1) + ",");
				hexaLines.append(QString::number(ptIdIndexs->GetId(5) + 1) + ",");
				hexaLines.append(QString::number(ptIdIndexs->GetId(6) + 1) + ",");
				hexaLines.append(QString::number(ptIdIndexs->GetId(7) + 1) + ENDL);
			}
		}

		if (hasTetra)
		{
			*_stream << "*Element, type=C3D4, elset=ALL_SOLID" << endl;
			*_stream << tetraLines;
		}
		if (hasHexa)
		{
			*_stream << "*Element, type=C3D8, elset=ALL_SOLID" << endl;
			*_stream << hexaLines;
		}
	}

	void INPdataExchange::writeAllSolidElset(int nElem)
	{
		if (nElem <= 0)
			return;
		*_stream << "*Elset, elset=ALL_SOLID, generate" << endl;
		*_stream << QString("1, %1, 1").arg(nElem) << endl;
	}

	void INPdataExchange::writeMeshSetsForKernel(int kId, const QVector<int> &cellToElem)
	{
		QList<int> setIds = _meshData->getSetIDFromKernal(kId);
		int nSet = setIds.size();
		if (nSet == 0)
			return;

		QString qSet, inpSetName;
		int index = 0, nMember = 0, head = -1, tail = -1;
		QList<int> members;

		while (index < nSet)
		{
			if (!_threadRuning)
				return;
			auto inpSet = _meshData->getMeshSetByID(setIds.at(index));
			index++;
			if (inpSet == nullptr)
				continue;
			if (dynamic_cast<BoundMeshSet *>(inpSet) != nullptr)
				continue;
			if (inpSet->getSetType() != Node && inpSet->getSetType() != Element)
				continue;
			if (inpSet->getSetType() == Node)
				qSet.append("*Nset,nset=");
			else if (inpSet->getSetType() == Element)
				qSet.append("*Elset,elset=");

			members = inpSet->getKernalMembers(kId);
			if (inpSet->getSetType() == Element)
			{
				QList<int> mapped;
				for (int m : members)
				{
					if (m >= 0 && m < cellToElem.size() && cellToElem[m] > 0)
						mapped.append(cellToElem[m] - 1);
				}
				members = mapped;
			}
			nMember = members.size();
			if (nMember <= 0)
				continue;

			head = members.at(0) + 1;
			tail = members.at(nMember - 1) + 1;
			if (head > tail)
			{
				int temp = head;
				head = tail;
				tail = temp;
			}

			inpSetName = inpSet->getName().toUpper();
			if (inpSetName.endsWith("_GEN"))
				endsWithGEN(inpSetName, qSet, head, tail);
			else
				notEndsWithGEN(inpSetName, qSet, nMember, members);

			if (qSet.size() > 1024)
			{
				*_stream << qSet;
				qSet.clear();
			}
			if (index == nSet && qSet.size() > 0)
				*_stream << qSet;
		}
	}

	void INPdataExchange::writeBoundSurfaces(vtkDataSet *data, int kId, const QVector<int> &cellToElem)
	{
		QList<int> setIds = _meshData->getSetIDFromKernal(kId);
		for (int sid : setIds)
		{
			auto ms = _meshData->getMeshSetByID(sid);
			auto bm = dynamic_cast<BoundMeshSet *>(ms);
			if (bm == nullptr)
				continue;
			const QMap<int, QVector<int>> cf = bm->getCellFaces();
			if (cf.isEmpty())
				continue;
			const QString surfName = abaqusSurfName(ms->getName());
			*_stream << "*Surface, type=ELEMENT, name=" << surfName << endl;
			for (auto it = cf.constBegin(); it != cf.constEnd(); ++it)
			{
				const int cellIdx = it.key();
				const int elid = (cellIdx >= 0 && cellIdx < cellToElem.size()) ? cellToElem[cellIdx] : -1;
				if (elid <= 0)
					continue;
				vtkCell *c = data->GetCell(cellIdx);
				if (c == nullptr)
					continue;
				for (int fi : it.value())
					*_stream << QString("%1, %2").arg(elid).arg(abaqusFaceLabel(c, fi)) << endl;
			}
		}
	}

	void INPdataExchange::writeSolidSection()
	{
		if (_Case == nullptr)
			return;
		const QList<int> inpMaterIds = _Case->getInpMaterialIds();
		if (inpMaterIds.isEmpty())
			return;
		auto materSteWard = Material::MaterialSingleton::getInstance();
		auto inpMaterial = materSteWard->getMaterialByID(inpMaterIds.first());
		if (inpMaterial == nullptr)
			return;
		*_stream << "*Solid Section, elset=ALL_SOLID, material=" << inpMaterial->getName() << ENDL << ENDL;
	}

	void INPdataExchange::writeComponent(int kId)
	{
		MeshKernal *k = _meshData->getKernalByID(kId);
		if (k == nullptr)
			return;
		vtkDataSet *data = k->getMeshData();
		QVector<int> cellToElem;
		int nSolid = 0;
		buildInpSolidElementMap(data, cellToElem, nSolid);
		(void)nSolid;
		writeMeshSetsForKernel(kId, cellToElem);
	}

	void INPdataExchange::writeMaterial()
	{
		auto materSteWard = Material::MaterialSingleton::getInstance();
		const QList<int> inpMaterIds = _Case->getInpMaterialIds();
		DataProperty::PropertyString *strPro = NULL;
		QString qMaterial;
		int index = 0;
		for (int inpMaterId : inpMaterIds)
		{
			index++;
			qMaterial.append("*Material,name=");

			auto inpMaterial = materSteWard->getMaterialByID(inpMaterId);
			qMaterial.append(inpMaterial->getName() + ENDL + "*Density" + ENDL);

			strPro = dynamic_cast<DataProperty::PropertyString *>(inpMaterial->getPropertyByName(QObject::tr("Density")));
			qMaterial.append(strPro->getValue() + "," + ENDL + "*Elastic" + ENDL);

			strPro = dynamic_cast<DataProperty::PropertyString *>(inpMaterial->getPropertyByName(QObject::tr("Elasticity Modulus")));
			qMaterial.append(strPro->getValue() + ", ");

			strPro = dynamic_cast<DataProperty::PropertyString *>(inpMaterial->getPropertyByName(QObject::tr("Poisson Ratio")));
			qMaterial.append(strPro->getValue() + ENDL);

			if (qMaterial.size() > 1024)
			{
				*_stream << qMaterial;
				qMaterial.clear();
			}

			if (index == inpMaterIds.size() && qMaterial.size() > 0)
				*_stream << qMaterial;
		}
	}

	void INPdataExchange::writeBoundary()
	{
		//每个set只能由一种kernal构成, 不能是两个或多个
		//导出边界时如果边界绑定的组件们所对应的kernalId必须一样,
		//因为边界中记录的是点或单元的Id, 两个不同的kernal点或单元的id会重复
		int nBC = _Case->getBCCount();
		if (nBC <= 0)
			return;
		*_stream << "** BOUNDARY CONDITIONS" << endl;

		BCBase::BCUserDef *inpBC = NULL;
		DataProperty::ParameterGroup *pagDis = NULL;
		DataProperty::ParameterGroup *pagRot = NULL;
		DataProperty::ParameterDouble *paraDis = NULL;
		DataProperty::ParameterDouble *paraRot = NULL;

		QString qBC, inpSetName;
		for (int i = 0; i < nBC; i++)
		{
			inpBC = dynamic_cast<BCBase::BCUserDef *>(_Case->getBCAt(i));
			if (!inpBC || inpBC->getParameterGroupCount() != 2)
				continue;
			inpSetName = _meshData->getMeshSetByID(inpBC->getComponentID())->getName();

			pagDis = inpBC->getParameterGroupAt(0);
			pagRot = inpBC->getParameterGroupAt(1);

			qBC.append("** Name: " + inpBC->getName() + " Type: ");
			qBC.append(pagDis->getDescribe() + "/" + pagRot->getDescribe() + ENDL + "*Boundary" + ENDL);

			QString qDis, qRot;
			for (int j = 0; j < 3; j++)
			{
				paraDis = dynamic_cast<DataProperty::ParameterDouble *>(pagDis->getParameterAt(j));
				paraRot = dynamic_cast<DataProperty::ParameterDouble *>(pagRot->getParameterAt(j));

				qDis.append(inpSetName + "," + QString::number(j + 1) + "," + QString::number(j + 1) + ",");
				qDis.append(QString::number(paraDis->getValue()) + ENDL);

				qRot.append(inpSetName + "," + QString::number(j + 4) + "," + QString::number(j + 4) + ",");
				qRot.append(QString::number(paraRot->getValue()) + ENDL);
			}
			qBC.append(qDis + qRot);
			if (qBC.size() > 1024)
			{
				*_stream << qBC;
				qBC.clear();
			}
			if (i == nBC - 1 && qBC.size() > 0)
				*_stream << qBC;
		}
	}

	void INPdataExchange::endsWithGEN(QString &inpSetName, QString &qSet, int head, int tail)
	{
		inpSetName.remove("_GEN");
		qSet.append(inpSetName + ",generate").append(ENDL);
		qSet.append(QString::number(head) + ",");
		qSet.append(QString::number(tail) + ",");
		qSet.append(ENDL);
	}

	void INPdataExchange::notEndsWithGEN(QString &inpSetName, QString &qSet, int nMember, const QList<int> &members)
	{
		qSet.append(inpSetName).append(ENDL);
		if (nMember == 1)
		{
			qSet.append(QString::number(members.at(0) + 1) + ",");
			qSet.append(ENDL);
			return;
		}

		int num = nMember / 16;
		for (int i = 0; i < num; i++)
		{
			for (int j = 0; j < 16; j++)
			{
				qSet.append(QString::number(members.at(16 * i + j) + 1));
				if (j != 15)
					qSet.append(",");
			}
			qSet.append(ENDL);
			if (qSet.size() > 1024)
			{
				*_stream << qSet;
				qSet.clear();
			}
		}

		int remainder = nMember % 16;
		if (remainder != 0)
		{
			for (int m = nMember - remainder; m < nMember; m++)
			{
				qSet.append(QString::number(members.at(m) + 1));
				if (m != nMember - 1)
					qSet.append(",");
			}
			qSet.append(ENDL);
		}
	}

	void INPdataExchange::run()
	{
		ModuleBase::ThreadTask::run();
		bool result = false;
		switch (_operation)
		{
		case MESH_READ:
			emit showInformation(tr("Importing INP Mesh File From \"%1\"").arg(_fileName));
			result = read();
			setReadResult(result);
			break;
		case MESH_WRITE:
			emit showInformation(tr("Exporting INP Mesh File To \"%1\"").arg(_fileName));
			result = write();
			setWriteResult(result);
			break;
		}
		defaultMeshFinished();
	}
}
