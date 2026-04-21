#ifndef _KEYDATAEXCHANGE_H_
#define _KEYDATAEXCHANGE_H_

#include "MeshThreadBase.h"
#include "meshDataExchangePlugin.h"
#include <QList>
#include <QString>
#include <QVector>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

class QTextStream;

namespace MeshData
{
	class MeshData;
	class BoundMeshSet;
	class MESHDATAEXCHANGEPLUGINAPI KEYdataExchange : public MeshThreadBase
	{
		public:		
			KEYdataExchange(const QString &fileName, MeshOperation operation, GUI::MainWindow *mw, int modelId = -1);
			~KEYdataExchange();

			void run();
			bool read();
			bool write();

		private:			
			struct ExportBodyInfo
			{
				QString name{};
				QList<int> cellIndices{};
				QList<int> nodeIds1Based{};
				QList<int> elemIds1Based{};
				QVector<QVector<int>> boundaryFaces1Based{};
				int solidPid{0};
				int shellPid{0};
				int nodeSetId{0};
				int elemSetId{0};
				int segmentSetId{0};
				bool hasSolid{false};
				bool hasShell{false};
			};

			void readLine(QString&);
			bool readNodes(vtkSmartPointer<vtkUnstructuredGrid>, QString&);
			bool readShellElements(vtkSmartPointer<vtkUnstructuredGrid>, QString&, int);
			bool readElements(vtkSmartPointer<vtkUnstructuredGrid>, QString&, int);
			bool readNodesGroup(const int, QString&);
			void setGridCells(vtkSmartPointer<vtkUnstructuredGrid>, QString&, int);

			void writeHyperMeshHeader();
			void writeMaterialElasticStub(int mid);
			void writeSectionCards(vtkDataSet *grid, int &outSecidSolid, int &outSecidShell);
			bool buildExportBodies(vtkDataSet *grid, const QVector<int> &vtkCellToExportEid, const QString &baseName,
								   QList<ExportBodyInfo> &bodies, QVector<int> &vtkCellToBody);
			void assignBodyExportIds(QList<ExportBodyInfo> &bodies);
			void writePartCards(const QList<ExportBodyInfo> &bodies, int secidSolid, int secidShell, int mid);
			bool validateExportDeck(vtkDataSet *grid, const QVector<int> &vtkCellToExportEid,
									const QVector<int> &vtkCellToBody, const QList<ExportBodyInfo> &bodies,
									int secidSolid, int secidShell, int mid);
			bool writePointPart(vtkDataSet *, const QVector<int> &vtkPointToExportNid);
			bool writeShellPart(vtkDataSet *, const QVector<int> &vtkCellToExportEid,
								const QVector<int> &vtkCellToBody, const QList<ExportBodyInfo> &bodies,
								const QVector<int> &vtkPointToExportNid);
			bool writeCellPart(vtkDataSet *, const QVector<int> &vtkCellToExportEid,
							   const QVector<int> &vtkCellToBody, const QList<ExportBodyInfo> &bodies,
							   const QVector<int> &vtkPointToExportNid);
			bool writeMeshPart(vtkDataSet *grid, const QVector<int> &vtkCellToExportEid,
							   const QList<ExportBodyInfo> &bodies, const QVector<int> &vtkPointToExportNid);
			void writeHmSetNodeList(int sid, const QString &name, const QList<int> &nodeIds1Based);
			void writeHmSetSolid(int sid, const QString &name, const QList<int> &elemIds1Based);
			void writeSegmentSetByFaces(int sid, const QString &name, const QVector<QVector<int>> &faces1Based);
			void writeSegmentSet(BoundMeshSet *bm, vtkDataSet *grid, int sid);

		private:
			MeshOperation _operation;
			const QString _fileName;
			MeshData* _meshData{};
			QTextStream* _stream{};
			QString _preLine{};
			int _modelId;
	};
}

#endif


