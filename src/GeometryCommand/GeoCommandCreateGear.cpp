#include "GeoCommandCreateGear.h"
#include "Geometry/geometryData.h"
#include "Geometry/geometrySet.h"
#include "Geometry/geometryParaGear.h"

#include <cmath>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>

#include <Geom_BezierCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <GC_MakeArcOfCircle.hxx>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Command {
	GeoCommandCreateGear::GeoCommandCreateGear(GUI::MainWindow* m, MainWidget::PreWindow* p)
		: GeoCommandBase(m, p)
	{
	}

	void GeoCommandCreateGear::setName(QString name)
	{
		_name = name;
	}

	void GeoCommandCreateGear::setNumberOfTeeth(int n)
	{
		_numberOfTeeth = n;
	}

	void GeoCommandCreateGear::setModule(double m)
	{
		_module = m;
	}

	void GeoCommandCreateGear::setPressureAngle(double angle)
	{
		_pressureAngle = angle;
	}

	void GeoCommandCreateGear::setAddendumCoefficient(double coeff)
	{
		_addendumCoeff = coeff;
	}

	void GeoCommandCreateGear::setDedendumCoefficient(double coeff)
	{
		_dedendumCoeff = coeff;
	}

	void GeoCommandCreateGear::setFilletCoefficient(double coeff)
	{
		_filletCoeff = coeff;
	}

	void GeoCommandCreateGear::setThickness(double t)
	{
		_thickness = t;
	}

	void GeoCommandCreateGear::setExternalGear(bool external)
	{
		_externalGear = external;
	}

	// 渐开线极坐标角度计算
	static double involuteAngle(double Rb, double R)
	{
		return std::sqrt(R * R - Rb * Rb) / Rb - std::acos(Rb / R);
	}

	// 渐开线点计算 (基于参数theta)
	static gp_Pnt involutePoint(double Rb, double theta)
	{
		double x = Rb * (std::cos(theta) + theta * std::sin(theta));
		double y = Rb * (std::sin(theta) - theta * std::cos(theta));
		return gp_Pnt(x, y, 0);
	}

	// 旋转点
	static gp_Pnt rotatePoint(const gp_Pnt& pt, double angle)
	{
		double cosA = std::cos(angle);
		double sinA = std::sin(angle);
		double x	= pt.X() * cosA - pt.Y() * sinA;
		double y	= pt.X() * sinA + pt.Y() * cosA;
		return gp_Pnt(x, y, pt.Z());
	}

	// 镜像点 (关于X轴)
	static gp_Pnt mirrorPoint(const gp_Pnt& pt)
	{
		return gp_Pnt(pt.X(), -pt.Y(), pt.Z());
	}

	TopoDS_Wire GeoCommandCreateGear::createGearProfile()
	{
		// 齿轮基本参数计算
		double m	= _module;
		int	   Z	= _numberOfTeeth;
		double phi	= _pressureAngle * M_PI / 180.0; // 转换为弧度

		// 各圆半径计算
		double Rref = Z * m / 2.0;				 // 分度圆半径
		double Rb	= Rref * std::cos(phi);		 // 基圆半径
		double Ra	= Rref + _addendumCoeff * m; // 齿顶圆半径
		double Rf	= Rref - _dedendumCoeff * m; // 齿根圆半径

		// 确保齿根圆不小于一个合理值
		if(Rf < 0)
			Rf = 0.1 * m;

		// 角度计算
		double angularPitch			   = 2.0 * M_PI / Z;	  // 齿距角
		double invAlpha				   = std::tan(phi) - phi; // 渐开线函数

		// 齿厚半角 (在分度圆上)
		double toothThicknessHalfAngle = angularPitch / 4.0;

		// 渐开线参数范围
		double thetaStart			   = 0.0;
		if(Rf > Rb) {
			// 齿根圆在基圆外，渐开线从齿根圆开始
			thetaStart = std::sqrt((Rf * Rf - Rb * Rb)) / Rb;
		}
		double				thetaEnd  = std::sqrt((Ra * Ra - Rb * Rb)) / Rb;

		// 生成单个齿的渐开线点
		const int			numPoints = 20;
		std::vector<gp_Pnt> involuteLeft;
		std::vector<gp_Pnt> involuteRight;

		for(int i = 0; i <= numPoints; ++i) {
			double t		  = (double)i / numPoints;
			double theta	  = thetaStart + t * (thetaEnd - thetaStart);
			gp_Pnt pt		  = involutePoint(Rb, theta);

			// 计算渐开线在分度圆处的角度偏移
			double angleAtRef = involuteAngle(Rb, Rref);

			// 旋转使齿对称于X轴
			gp_Pnt ptRotated  = rotatePoint(pt, -angleAtRef - toothThicknessHalfAngle);
			involuteLeft.push_back(ptRotated);

			// 镜像得到另一侧渐开线
			involuteRight.push_back(mirrorPoint(ptRotated));
		}

		BRepBuilderAPI_MakeWire wireBuilder;

		// 为每个齿创建轮廓
		for(int tooth = 0; tooth < Z; ++tooth) {
			double toothAngle = tooth * angularPitch;

			// 渐开线左侧 (从齿根到齿顶)
			for(size_t i = 0; i < involuteLeft.size() - 1; ++i) {
				gp_Pnt p1 = rotatePoint(involuteLeft[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteLeft[i + 1], toothAngle);
				if(p1.Distance(p2) > 1e-6) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
					wireBuilder.Add(edge);
				}
			}

			// 齿顶圆弧
			gp_Pnt tipLeft	= rotatePoint(involuteLeft.back(), toothAngle);
			gp_Pnt tipRight = rotatePoint(involuteRight.back(), toothAngle);

			if(tipLeft.Distance(tipRight) > 1e-6) {
				// 使用圆弧连接齿顶
				gp_Pnt tipMid((tipLeft.X() + tipRight.X()) / 2.0 * Ra
								  / std::sqrt(std::pow((tipLeft.X() + tipRight.X()) / 2.0, 2)
											  + std::pow((tipLeft.Y() + tipRight.Y()) / 2.0, 2)),
							  (tipLeft.Y() + tipRight.Y()) / 2.0 * Ra
								  / std::sqrt(std::pow((tipLeft.X() + tipRight.X()) / 2.0, 2)
											  + std::pow((tipLeft.Y() + tipRight.Y()) / 2.0, 2)),
							  0);
				try {
					GC_MakeArcOfCircle arcMaker(tipLeft, tipMid, tipRight);
					if(arcMaker.IsDone()) {
						TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker.Value());
						wireBuilder.Add(arcEdge);
					} else {
						// 如果圆弧失败，使用直线
						TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(tipLeft, tipRight);
						wireBuilder.Add(edge);
					}
				} catch(...) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(tipLeft, tipRight);
					wireBuilder.Add(edge);
				}
			}

			// 渐开线右侧 (从齿顶到齿根)
			for(int i = (int)involuteRight.size() - 1; i > 0; --i) {
				gp_Pnt p1 = rotatePoint(involuteRight[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteRight[i - 1], toothAngle);
				if(p1.Distance(p2) > 1e-6) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
					wireBuilder.Add(edge);
				}
			}

			// 齿根圆弧 (连接到下一个齿)
			gp_Pnt rootRight	= rotatePoint(involuteRight.front(), toothAngle);
			gp_Pnt nextRootLeft = rotatePoint(involuteLeft.front(), toothAngle + angularPitch);

			if(rootRight.Distance(nextRootLeft) > 1e-6) {
				// 计算齿根圆弧的起始和结束角度
				double rootRightAngle	 = std::atan2(rootRight.Y(), rootRight.X());
				double nextRootLeftAngle = std::atan2(nextRootLeft.Y(), nextRootLeft.X());

				// 确保角度连续（处理跨越0度的情况）
				if(nextRootLeftAngle < rootRightAngle) {
					nextRootLeftAngle += 2.0 * M_PI;
				}

				// 计算齿根圆弧的角度跨度
				double rootArcSpan = nextRootLeftAngle - rootRightAngle;

				// 如果圆弧角度超过180度，限制为180度
				if(rootArcSpan > M_PI) {
					// 计算180度圆弧的终点角度
					double limitedEndAngle = rootRightAngle + M_PI;
					// 圆弧中点（恰好在起点偏移90度处）
					double rootMidAngle	   = rootRightAngle + M_PI / 2.0;
					gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
					// 180度圆弧的终点
					gp_Pnt arcEnd(Rf * std::cos(limitedEndAngle), Rf * std::sin(limitedEndAngle),
								  0);

					try {
						// 创建180度圆弧
						GC_MakeArcOfCircle arcMaker(rootRight, rootMid, arcEnd);
						if(arcMaker.IsDone()) {
							wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
						}
					} catch(...) {
					}

					// 用直线连接剩余部分（从圆弧终点到下一个齿的渐开线起点）
					if(arcEnd.Distance(nextRootLeft) > 1e-6) {
						TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(arcEnd, nextRootLeft);
						wireBuilder.Add(edge);
					}
				} else {
					// 圆弧角度不超过180度，正常处理
					double rootMidAngle = (rootRightAngle + nextRootLeftAngle) / 2.0;
					gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);

					try {
						GC_MakeArcOfCircle arcMaker(rootRight, rootMid, nextRootLeft);
						if(arcMaker.IsDone()) {
							TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker.Value());
							wireBuilder.Add(arcEdge);
						} else {
							TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft);
							wireBuilder.Add(edge);
						}
					} catch(...) {
						TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft);
						wireBuilder.Add(edge);
					}
				}
			}
		}

		return wireBuilder.Wire();
	}

	TopoDS_Shape GeoCommandCreateGear::extrudeProfile(const TopoDS_Wire& profile)
	{
		// 在XY平面上创建面
		gp_Pln					plane(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
		BRepBuilderAPI_MakeFace faceMaker(plane, profile,
										  Standard_True); // Standard_True 表示检查并修复 wire
		if(!faceMaker.IsDone()) {
			return TopoDS_Shape();
		}

		TopoDS_Face			  face = faceMaker.Face();

		// 沿Z轴拉伸
		gp_Vec				  extrusionDir(0, 0, _thickness);
		BRepPrimAPI_MakePrism prismMaker(face, extrusionDir);

		if(!prismMaker.IsDone()) {
			return TopoDS_Shape();
		}

		return prismMaker.Shape();
	}

	bool GeoCommandCreateGear::execute()
	{
		// 创建齿轮2D轮廓
		TopoDS_Wire profile = createGearProfile();
		if(profile.IsNull()) {
			return false;
		}

		// 拉伸生成3D齿轮
		TopoDS_Shape gearShape = extrudeProfile(profile);
		if(gearShape.IsNull()) {
			return false;
		}

		// 创建形状指针
		TopoDS_Shape* shape		   = new TopoDS_Shape;
		*shape					   = gearShape;

		// 创建几何集对象
		Geometry::GeometrySet* set = new Geometry::GeometrySet(Geometry::STEP);
		set->setShape(shape);
		_res = set;

		if(_isEdit) {
			set->setName(_editSet->getName());
			_geoData->replaceSet(set, _editSet);
			emit removeDisplayActor(_editSet);
		} else {
			set->setName(_name);
			_geoData->appendGeometrySet(set);
		}

		// 创建参数对象
		Geometry::GeometryParaGear* para = new Geometry::GeometryParaGear;
		para->setName(_name);
		para->setNumberOfTeeth(_numberOfTeeth);
		para->setModule(_module);
		para->setPressureAngle(_pressureAngle);
		para->setAddendumCoefficient(_addendumCoeff);
		para->setDedendumCoefficient(_dedendumCoeff);
		para->setFilletCoefficient(_filletCoeff);
		para->setThickness(_thickness);
		para->setExternalGear(_externalGear);
		_res->setParameter(para);

		GeoCommandBase::execute();
		emit updateGeoTree();
		emit showSet(set);

		return true;
	}

	void GeoCommandCreateGear::undo()
	{
		emit removeDisplayActor(_res);
		if(_isEdit) {
			_geoData->replaceSet(_editSet, _res);
			emit showSet(_editSet);
		} else {
			_geoData->removeTopGeometrySet(_res);
		}
		GeoCommandBase::undo();
		emit updateGeoTree();
	}

	void GeoCommandCreateGear::redo()
	{
		if(_isEdit) {
			_geoData->replaceSet(_res, _editSet);
			emit removeDisplayActor(_editSet);
		} else {
			_geoData->appendGeometrySet(_res);
		}
		emit updateGeoTree();
		emit showSet(_res);
	}

	void GeoCommandCreateGear::releaseResult()
	{
		if(_res != nullptr)
			delete _res;
		_res = nullptr;
	}
} // namespace Command
