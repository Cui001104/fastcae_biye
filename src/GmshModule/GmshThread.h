#ifndef _GMSHTHREAD_H_
#define _GMSHTHREAD_H_

#include <QList>
#include <QMultiHash>
#include <QProcess>
#include <QString>
#include <QTextStream>
#include <QThread>
#include <QTime>
#include <vtkCellType.h>
#include "DataProperty/DataBase.h"
#include "GmshModuleAPI.h"

class TopoDS_Compound;
class TopoDS_Shape;
class gp_Pnt;
class vtkDataSet;

namespace GUI
{
	class MainWindow;
}

namespace MainWidget
{
	class PreWindow;
}

namespace ModuleBase
{
	class ProcessBar;
}

namespace Py
{
	class PythonAgent;
}

namespace MeshData
{
	class MeshKernal;
}

namespace Gmsh
{
	typedef struct
	{
		int geoSetID;
		int itemIndex;
		QList<int> cellIndexs;
	} itemInfo;

	class GmshModule;
	class FluidMeshPreProcess;
	class GmshScriptWriter;

	class GMshPara
	{
	public:
		int _dim{-1};
		QMultiHash<int, int> _solidHash{};
		QMultiHash<int, int> _surfaceHash{};
		QString _elementType{};
		int _elementOrder{-1};
		int _method{-1};
		double _sizeFactor{0.0};
		double _minSize{0.0};
		double _maxSize{0.0};
		bool _geoclean{false};
		int _smoothIteration{0};
		bool _isGridCoplanar{false};
		QString _sizeAtPoints{};
		QString _sizeFields{};
		bool _selectall{false};
		bool _selectvisible{false};
		double _boreNodeTolerance{-1.0};
		int _meshID{-1};
		QList<double *> _fluidField{};
		bool _fluidMesh{false};
		QString _cells{};
	};

	class GMSHAPI GmshThread : public DataProperty::DataBase
	{
		Q_OBJECT
	public:
		GmshThread(GUI::MainWindow *mw, MainWidget::PreWindow *pre, GmshModule *mod, int dim);
		~GmshThread();

		void setPara(GMshPara *para);
		void appendSolid(int id, int index);
		void setSolid(QMultiHash<int, int> s);
		void appendSurface(int geo, int face);
		void setSurface(QMultiHash<int, int> s);
		void setElementType(QString t);
		void setElementOrder(int order);
		void setMethod(int m);
		void setSizeFactor(double f);
		void setMinSize(double min);
		void setMaxSize(double max);
		void isCleanGeo(bool c);
		void setSmoothIteration(int it);
		void setGridCoplanar(bool gc);
		void setSizeAtPoint(QString ps);
		void setSizeFields(QString fs);
		void setBoreNodeTolerance(double tol);
		void setMeshID(int id);
		void setSelectedAll(bool al);
		void setSelectedVisible(bool sv);
		void setFluidMesh(bool fm);
		void setCellTypeList(QString cells);

		void run();
		void stop();
		void isSaveDataToKernal(bool save);
		QList<itemInfo> generateGeoIds(vtkDataSet *dataset);

	signals:
		void threadFinished(GmshThread *t);
		void sendMessage(QString);
		void writeToSolveFileSig(vtkDataSet *);
		void updateMeshActor();

	private slots:
		void processFinished(int, QProcess::ExitStatus);
		void readProcessOutput();

	private:
		void mergeGeometry();
		void initGmshEnvoirment();
		void submitParaToGmsh();
		void generate();
		void readMesh();
		void mergeAllGeo();
		void mergeVisibleGeo();
		void mergeSelectGeo();
		void setGmshSettingData(MeshData::MeshKernal *k);
		void setGmshScriptData();
		vtkDataSet *deleteSpecifiedCells(vtkDataSet *dataset);
		bool isSpecifiedCell(VTKCellType type);

	private:
		GUI::MainWindow *_mainwindow{};
		MainWidget::PreWindow *_preWindow{};
		GmshModule *_gmshModule{};
		FluidMeshPreProcess *_fluidMeshProcess{};
		QProcess _process{};
		ModuleBase::ProcessBar *_processBar{};
		int _dim{-1};
		TopoDS_Compound *_compounnd{};
		QMultiHash<int, int> _solidHash{};
		QMultiHash<int, int> _surfaceHash{};
		QString _elementType{};
		int _elementOrder{-1};
		int _method{-1};
		double _sizeFactor{0.0};
		double _minSize{0.0};
		double _maxSize{0.0};
		bool _geoclean{false};
		int _smoothIteration{0};
		bool _isGridCoplanar{false};
		QString _sizeAtPoints{};
		QString _sizeFields{};
		double _boreNodeTolerance{-1.0};
		bool _selectall{false};
		bool _selectvisible{false};
		bool _isSaveToKernal{true};
		int _meshID{-1};
		bool _fluidMesh{false};
		QList<int> _cellTypeList{};
		GmshScriptWriter *_scriptWriter{};
	};
}

#endif
