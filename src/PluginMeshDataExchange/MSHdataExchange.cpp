#include "MSHdataExchange.h"
#include "MeshData/meshSingleton.h"
#include "MeshData/meshKernal.h"
#include "MeshData/meshSet.h"
#include <QFileInfo>
#include <QTextStream>
#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <QFile>
#include <QMessageBox>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <set>

namespace MeshData {
	MSHdataExchange::MSHdataExchange(const QString& fileName, MeshOperation operation,
									 GUI::MainWindow* mw, int modelId)
		: //	_fileName(fileName),
		_operation(operation)
		, _meshData(MeshData::getInstance())
		, _totalNumber(0)
		, MeshThreadBase(fileName, operation, mw)
		, _modelId(modelId)
	{
		_fileName = fileName;
		_file	  = new QFile(_fileName);
	}

	MSHdataExchange::~MSHdataExchange()
	{
		if(_stream != nullptr)
			delete _stream;
		if(_file != nullptr)
			delete _file;
		_file	= nullptr;
		_stream = nullptr;
	}


	bool MSHdataExchange::write()
	{
		// 当前仅实现了MSH读入，写出流程尚未实现
		return false;
	}

	void MSHdataExchange::run()
	{
		ModuleBase::ThreadTask::run();
		bool result = false;
		switch(_operation) {
			case MESH_READ:
				emit showInformation(tr("Import MSH Mesh File From \"%1\"").arg(_fileName));
				result = read();
				setReadResult(result);
				break;
			case MESH_WRITE:
				emit showInformation(tr("Export MSH Mesh File From \"%1\"").arg(_fileName));
				result = write();
				setWriteResult(result);
				break;
		}
		defaultMeshFinished();
	}

	bool MSHdataExchange::read()
	{
		_file = new QFile();
		QFileInfo info(_fileName);
		if(!info.exists())
			return false;
		_baseFileName = info.fileName();
		_filePath	  = info.filePath();
		_file->setFileName(_fileName);
		if(!_file->open(QIODevice::ReadOnly))
			return false;
		_stream					 = new QTextStream(_file);
		CFDMeshReader*	  reader = new CFDMeshReader;
		CFDMshFileParser* parser = new CFDMshFileParser;
		reader->setMeshFileParser(parser);
		bool ok = reader->readFile(_fileName.toStdString());
		if(!ok)
			return ok;
		vtkSmartPointer<vtkUnstructuredGrid> dataset = reader->getGrid();
		MeshKernal*							 k		 = new MeshKernal;
		k->setName(_baseFileName);
		k->setPath(_filePath);
		k->setMeshData((vtkDataSet*)dataset);
		_meshData->appendMeshKernal(k);
		int									 kid   = k->getID();
		std::unordered_map<int, std::string> zones = parser->getZones();
		QMap<int, std::string>				 zoneNames{};
		for(auto zone : zones) {
			zoneNames.insert(zone.first, zone.second);
		}
		std::unordered_map<int, std::pair<int, int>> zoneIndexes = parser->getCellZones();
		for(auto it : zoneIndexes) {
			int		 setid = it.first;
			MeshSet* set   = new MeshSet;
			set->setType(SetType::Element);
			if(zoneNames.contains(setid))
				set->setName(QString::fromStdString(zoneNames[setid]));
			else
				set->setName("ElementSet" + QString::number(it.first));
			MeshSet* s = _idsetList.value(setid);
			if(s != nullptr) {
				delete s;
				_idsetList.remove(setid);
			}
			_idsetList[setid] = set;
			int firstIndex	  = it.second.first;
			int lastIndex	  = it.second.second;
			for(int i = firstIndex; i <= lastIndex; ++i)
				set->appendTempMem(i - 1);
		}
		std::unordered_map<int, std::pair<int, int>> faceZones = parser->getPointZones();
		for(auto it : faceZones) {
			int		 setid = it.first;
			MeshSet* set   = new MeshSet;
			set->setType(SetType::Node);
			if(zoneNames.contains(setid))
				set->setName(QString::fromStdString(zoneNames[setid]));
			else
				set->setName("PointSet" + QString::number(it.first));
			MeshSet* s = _idsetList.value(setid);
			if(s != nullptr) {
				delete s;
				_idsetList.remove(setid);
			}
			_idsetList[setid] = set;
			int firstIndex	  = it.second.first;
			int lastIndex	  = it.second.second;
			for(int i = firstIndex; i <= lastIndex; ++i)
				set->appendTempMem(i - 1);
		}
		std::unordered_map<int, std::pair<int, int>> face2CellIndex =
			parser->getFaceIDtoCellIndex();
		for(auto faceZone : parser->getFaceZones()) {
			int zoneId = faceZone.first;
			if(zoneNames.contains(zoneId)) {
				BoundMeshSet* set = new BoundMeshSet;
				set->setName(QString::fromStdString(zoneNames[zoneId]));
				QMap<int, QVector<int>> cellFaces{};
				int						firstIndex = faceZone.second.first;
				int						lastIndex  = faceZone.second.second;
				for(int i = firstIndex; i <= lastIndex; ++i) {
					int cellIndex = face2CellIndex[i].first;
					int index	  = face2CellIndex[i].second;
					cellFaces[cellIndex - 1].push_back(index - 1);
					set->appendTempMem(i - 1);
				}
				MeshSet* s = _idsetList.value(zoneId);
				if(s != nullptr) {
					delete s;
					_idsetList.remove(zoneId);
				}
				_idsetList[zoneId] = set;
			}
		}
		QList<MeshSet*> setList = _idsetList.values();
		for(int i = 0; i < _idsetList.size(); ++i) {
			MeshSet* set = setList.at(i);
			set->setKeneralID(kid);
			_meshData->appendMeshSet(set);
		}
		delete parser;
		delete reader;
		return ok;
		// if (mMeshType == typeGambit)
		//{
		//	if (!readGambitFile(dataset))
		//		return false;
		//	else
		//		return true;
		// }
		// else if (mMeshType == typeFluent)
		//{
		//	if (!readFluentFile(dataset))
		//		return false;
		// }
		// return true;
	}

	bool CFDMshFileParser::setFile(const std::string& file)
	{
		m_File = file;
		return this->read();
	}

	std::vector<double> CFDMshFileParser::getPoints()
	{
		return m_Points;
	}

	std::unordered_map<int, std::vector<int>> CFDMshFileParser::getFaces()
	{
		return m_Faces;
	}

	std::unordered_map<int, std::vector<int>> CFDMshFileParser::getCells()
	{
		return m_Cells;
	}

	std::unordered_map<int, int> CFDMshFileParser::getCellsType()
	{
		return m_CellsType;
	}

	std::unordered_map<int, std::pair<int, int>> CFDMshFileParser::getPointZones()
	{
		return m_PointZones;
	}

	std::unordered_map<int, std::pair<int, int>> CFDMshFileParser::getFaceZones()
	{
		return m_FaceZones;
	}

	std::unordered_map<int, std::pair<int, int>> CFDMshFileParser::getCellZones()
	{
		return m_CellZones;
	}

	std::unordered_map<int, std::pair<int, int>> CFDMshFileParser::getFaceIDtoCellIndex()
	{
		return m_FaceIdToCellFaceIndex;
	}

	std::unordered_map<int, std::string> CFDMshFileParser::getZones()
	{
		return m_Zones;
	}

	bool CFDMshFileParser::read()
	{
		std::ifstream in{};
		in.open(m_File, std::ios_base::in);
		if(!in.is_open()) {
			return false;
		}
		std::stringstream buffer;
		buffer << in.rdbuf();
		m_ByteArray = std::string(buffer.str());
		in.close();
		bool p = this->readPoints();
		if(!p)
			return p;
		bool f = this->readFaces();
		if(!f)
			return f;
		bool c = this->readCellsType();
		if(!c)
			return c;
		this->readZones();
		return true;
	}

	bool CFDMshFileParser::readPoints()
	{
		// 读取点
		const std::string nodeLabel{ "\n(10" };
		int				  nlSize = nodeLabel.size();
		int				  first	 = m_ByteArray.find(nodeLabel);
		if(first == -1)
			return false;
		while(first != -1) {
			int end = m_ByteArray.find("\n", first + nlSize);
			if(end == -1)
				return false;
			std::string element{ m_ByteArray.substr(first + 1, end - first) };
			stringTrimmed(element);
			std::vector<std::string> list{};
			this->stringSplit(element, ' ', list);
			if(list.size() < 5)
				return false;
			bool		ok;
			std::string zoneIdStr = list.at(1).substr(1);
			int			zoneID{ -1 };
			sscanf(zoneIdStr.c_str(), "%X", &zoneID);
			std::string firstIndex = list.at(2);
			std::string lastIndex  = list.at(3);
			int			fIndex{ 0 }, lIndex{ 0 };
			sscanf(firstIndex.c_str(), "%X", &fIndex);
			sscanf(lastIndex.c_str(), "%X", &lIndex);
			if(zoneID == 0) {
				std::string pnStr = list.at(3);
				m_PointNumber	  = strtol(pnStr.c_str(), 0, 16);
			} else {
				m_PointZones[zoneID] = std::make_pair(fIndex, lIndex);
				int first			 = m_ByteArray.find(")", end + 1);
				if(first == -1)
					break;
				std::string				 nodes = m_ByteArray.substr(end + 1, first - end);
				std::vector<std::string> arrays{};
				stringSplit(nodes, '\n', arrays);
				for(auto& array : arrays) {
					std::string coords{ array };
					stringTrimmed(coords);
					std::vector<std::string> cs{};
					stringSplit(coords, ' ', cs);
					if(cs.size() == 3) {
						m_Points.push_back(atof(cs.at(0).c_str()));
						m_Points.push_back(atof(cs.at(1).c_str()));
						m_Points.push_back(atof(cs.at(2).c_str()));
					}
				}
				end = first;
			}
			first = m_ByteArray.find(nodeLabel, end);
		}
		return true;
	}

	bool CFDMshFileParser::readZones()
	{
		std::string zoneLable{ "\n(39" };
		int			first = m_ByteArray.find(zoneLable);
		if(first == -1) {
			zoneLable = "\n(45";
			first	  = m_ByteArray.find(zoneLable);
		}
		if(first == -1)
			return false;
		int clSize = zoneLable.size();
		while(first != -1) {
			int			end	 = m_ByteArray.find("\n", first + clSize);
			std::string line = m_ByteArray.substr(first + clSize, end - first - clSize);
			int			i	 = line.find("(");
			if(i == -1)
				return false;
			int j = line.find(")");
			if(j == -1)
				return false;
			std::string element = line.substr(i + 1, j - i - 1);
			stringTrimmed(element);
			std::vector<std::string> segs{};
			stringSplit(element, ' ', segs);
			if(segs.size() >= 3) {
				std::string name   = segs.at(2);
				int			zoneID = strtol(segs.at(0).c_str(), 0, 10);
				m_Zones[zoneID]	   = name;
			}
			first = m_ByteArray.find(zoneLable, end);
		}
		return true;
	}

	bool CFDMshFileParser::readFaces()
	{
		// 读取面及单元
		const std::string faceLable{ "\n(13" };
		int				  flSize = faceLable.size();
		int				  first	 = m_ByteArray.find(faceLable);
		if(first == -1)
			return false;
		while(first != -1) {
			int end = m_ByteArray.find("\n", first + flSize);
			if(end == -1)
				return false;
			// 以（13开头的整行
			std::string element{ m_ByteArray.substr(first + 1, end - first) };
			int			a = element.find("(", 3);
			int			b = element.find_last_of(")");
			element		  = element.substr(a + 1, b - a - 1);
			stringTrimmed(element);
			std::vector<std::string> list{};
			stringSplit(element, ' ', list);
			if(list.size() < 4)
				return false;
			std::string zoneStr = list.at(0);
			int			zoneId{ -1 };
			sscanf(zoneStr.c_str(), "%x", &zoneId);
			int startFaceIndex = strtol(list.at(1).c_str(), 0, 16);
			int endFaceIndex   = strtol(list.at(2).c_str(), 0, 16);
			if(zoneId == 0) {
				m_FaceNumber = strtol(list.at(2).c_str(), 0, 16);
			} else {
				m_FaceZones[zoneId] = std::make_pair(startFaceIndex, endFaceIndex);
				int bcType;
				sscanf(list.at(3).c_str(), "%d", &bcType);
				int faceType;
				sscanf(list.at(4).c_str(), "%d", &faceType);
				int first = m_ByteArray.find(")", end + 1);
				if(first == -1)
					break;
				std::string				 faces = m_ByteArray.substr(end + 1, first - end);
				std::vector<std::string> arrays{};
				stringSplit(faces, '\n', arrays);
				for(auto& array : arrays) {
					std::string coords{ array };
					stringTrimmed(coords);
					std::vector<std::string> cs{};
					stringSplit(coords, ' ', cs);
					this->parseFaceAndCells(startFaceIndex, cs, faceType);
					++startFaceIndex;
				}
				end = first;
			}
			first = m_ByteArray.find(faceLable, end);
		}
		return true;
	}

	bool CFDMshFileParser::readCellsType()
	{
		// 读取单元类型
		const std::string cellLable{ "\n(12" };
		int				  clSize = cellLable.size();
		int				  first	 = m_ByteArray.find(cellLable);
		if(first == -1)
			return false;
		while(first != -1) {
			int end = m_ByteArray.find("\n", first + clSize);
			if(end == -1)
				return false;
			std::string element{ m_ByteArray.substr(first + cellLable.size(),
													end - first - cellLable.size()) };
			stringTrimmed(element);
			std::vector<std::string> list{};
			stringSplit(element, ' ', list);
			if(list.size() < 4)
				return false;
			bool ok;
			int	 zoneId		= strtol(list.at(0).substr(1).c_str(), 0, 16);
			int	 startIndex = strtol(list.at(1).c_str(), 0, 16);
			int	 endIndex	= strtol(list.at(2).c_str(), 0, 16);
			int	 type		= strtol(list.at(3).c_str(), 0, 16);
			if(zoneId == 0)
				;
			else {
				m_CellZones[zoneId] = std::make_pair(startIndex, endIndex);
				std::string str		= list.at(4);
				int			elementType{ -1 };
				std::string etStr = str.substr(0, str.find_first_of(")"));
				sscanf(etStr.c_str(), "%x", &elementType);
				if(elementType == 0) {
					int bodyEnd = m_ByteArray.find(")", end + 1);
					if(bodyEnd != -1) {
						std::string body{ m_ByteArray.substr(end + 1, bodyEnd - end - 1) };
						stringTrimmed(body);
						if(body.at(0) == '(')
							body = body.substr(1);
						std::vector<std::string> types{};
						stringSplit(body, ' ', types);
						for(const std::string& ty : types) {
							int type{};
							sscanf(ty.c_str(), "%d", &type);
							m_CellsType[startIndex] = type;
							++startIndex;
						}
						end = bodyEnd;
					}
				} else {
					for(int i = startIndex; i <= endIndex; ++i)
						m_CellsType[i] = elementType;
				}
			}
			first = m_ByteArray.find(cellLable, end);
		}
		return true;
	}

	void CFDMshFileParser::parseFaceAndCells(int							 faceIndex,
											 const std::vector<std::string>& faceLineSegs,
											 int							 faceType)
	{
		bool ok;
		switch(faceType) {
			case 0:
			case 5: {
				if(faceLineSegs.size() >= 5) {
					std::string pnStr = faceLineSegs.front();
					int			pointNum{ 0 };
					sscanf(pnStr.c_str(), "%d", &pointNum);
					std::vector<int> pointIDs{};
					int				 i = 1;
					for(; i <= pointNum; ++i) {
						std::string idStr = faceLineSegs.at(i);
						int			id	  = strtol(idStr.c_str(), 0, 16);
						pointIDs.push_back(id);
					}
					m_Faces.insert(make_pair(faceIndex, pointIDs));
					std::string ownerStr = faceLineSegs.at(i);
					int			owner	 = strtol(ownerStr.c_str(), 0, 16);
					++i;
					std::string nbStr	= faceLineSegs.at(i);
					int			neigbur = strtol(nbStr.c_str(), 0, 16);
					if(owner != 0) {
						m_Cells[owner].push_back(faceIndex);
						m_FaceIdToCellFaceIndex[faceIndex] =
							std::make_pair(owner, m_Cells[owner].size());
					}
					if(neigbur != 0) {
						m_Cells[neigbur].push_back(faceIndex);
						m_FaceIdToCellFaceIndex[faceIndex] =
							std::make_pair(neigbur, m_Cells[neigbur].size());
					}
				}
			} break;
			case 3: {
				if(faceLineSegs.size() != 5)
					return;
				std::vector<int> pointIds{};
				for(int i = 0; i < 3; ++i) {
					int id = strtol(faceLineSegs.at(i).c_str(), 0, 16);
					pointIds.push_back(id);
				}
				m_Faces.insert(make_pair(faceIndex, pointIds));
				std::string ownerStr{ faceLineSegs.at(3) };
				int			owner = strtol(ownerStr.c_str(), 0, 16);
				std::string nbStr{ faceLineSegs.at(4) };
				int			neighber = strtol(nbStr.c_str(), 0, 16);
				if(owner != 0) {
					m_Cells[owner].push_back(faceIndex);
					m_FaceIdToCellFaceIndex[faceIndex] =
						std::make_pair(owner, m_Cells[owner].size());
				}
				if(neighber != 0) {
					m_Cells[neighber].push_back(faceIndex);
					m_FaceIdToCellFaceIndex[faceIndex] =
						std::make_pair(neighber, m_Cells[neighber].size());
				}
			} break;
			case 4: {
				if(faceLineSegs.size() != 6)
					return;
				std::vector<int> pointIds{};
				for(int i = 0; i < 4; ++i) {
					int id = strtol(faceLineSegs.at(i).c_str(), 0, 16);
					pointIds.push_back(id);
				}
				m_Faces.insert(make_pair(faceIndex, pointIds));
				std::string ownerStr{ faceLineSegs.at(4) };
				int			owner = strtol(ownerStr.c_str(), 0, 16);
				std::string nbStr{ faceLineSegs.at(5) };
				int			neighber = strtol(nbStr.c_str(), 0, 16);
				if(owner != 0) {
					m_Cells[owner].push_back(faceIndex);
					m_FaceIdToCellFaceIndex[faceIndex] =
						std::make_pair(owner, m_Cells[owner].size());
				}
				if(neighber != 0) {
					m_Cells[neighber].push_back(faceIndex);
					m_FaceIdToCellFaceIndex[faceIndex] =
						std::make_pair(neighber, m_Cells[neighber].size());
				}
			} break;
			default:
				break;
		}
	}

	void CFDMshFileParser::stringSplit(const std::string str, const char split,
									   std::vector<std::string>& vec)
	{
		std::istringstream iss(str);
		std::string		   token;
		while(getline(iss, token, split)) {
			if(token.size() != 0)
				vec.push_back(token);
		}
	}

	void CFDMshFileParser::stringTrimmed(std::string& str)
	{
		int first = str.find_first_not_of(" ");
		str.erase(0, first);
		int last = str.find_last_not_of(" ");
		if(last + 1 < str.size())
			str.erase(last + 1);
	}

	void CFDMeshReader::setMeshFileParser(CFDMshFileParser* parser)
	{
		m_Parser = parser;
	}

	bool CFDMeshReader::readFile(const std::string& file)
	{
		if(m_Parser == nullptr)
			return false;
		bool ok = m_Parser->setFile(file);
		if(!ok)
			return ok;
		m_ugrid = vtkSmartPointer<vtkUnstructuredGrid>::New();
		vtkSmartPointer<vtkPoints>						points = vtkSmartPointer<vtkPoints>::New();
		const std::vector<double>						ps	   = m_Parser->getPoints();
		const std::unordered_map<int, std::vector<int>> cells  = m_Parser->getCells();
		const std::unordered_map<int, std::vector<int>> faces  = m_Parser->getFaces();
		const std::unordered_map<int, int>				cellTyps = m_Parser->getCellsType();
		std::set<int>									usedFaces{};
		int												num = ps.size() / 3;
		for(int i = 0; i < num; ++i) {
			double coord[3]{ ps[i * 3], ps[i * 3 + 1], ps[i * 3 + 2] };
			points->InsertNextPoint(coord);
		}
		m_ugrid->SetPoints(points);
		std::unordered_map<int, std::vector<int>>::const_iterator it = cells.cbegin();
		while(it != cells.cend()) {
			std::vector<vtkIdType>	   pointIds{};
			vtkSmartPointer<vtkIdList> facesIDS = vtkSmartPointer<vtkIdList>::New();
			std::vector<int>		   faceids	= it->second;
			int						   index	= it->first;
			for(int id : faceids) {
				auto it = faces.find(id);
				if(it == faces.cend())
					continue;
				const std::vector<int> pids = it->second;
				usedFaces.insert(it->first);
				facesIDS->InsertNextId(pids.size());
				for(int pid : pids) {
					pointIds.push_back(pid - 1);
					facesIDS->InsertNextId(pid - 1);
				}
			}

			std::sort(pointIds.begin(), pointIds.end());
			auto ii = std::unique(pointIds.begin(), pointIds.end());
			pointIds.erase(ii, pointIds.end());
			m_ugrid->InsertNextCell(VTK_POLYHEDRON, pointIds.size(), pointIds.data(),
									faceids.size(), facesIDS->GetPointer(0));
			++it;
		}
		return true;
	}

	vtkSmartPointer<vtkUnstructuredGrid> CFDMeshReader::getGrid()
	{
		return m_ugrid;
	}

} // namespace MeshData