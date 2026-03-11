#include "GeoCommandCreateGear.h"
#include "Geometry/geometryData.h"
#include "Geometry/geometrySet.h"
#include "Geometry/geometryParaGear.h"

#include <cmath>
#include <vector>
#include <QDebug>

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
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <gp_Trsf.hxx>

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

	void GeoCommandCreateGear::setTipReliefAmount(double amount)
	{
		_tipReliefAmount = amount;
	}

	void GeoCommandCreateGear::setTipReliefLength(double length)
	{
		_tipReliefLength = length;
	}

	void GeoCommandCreateGear::setNumberOfSecondTeeth(int n)
	{
		_numberOfSecondTeeth = n;
	}

	void GeoCommandCreateGear::setTipReliefAmount2(double amount)
	{
		_tipReliefAmount2 = amount;
	}

	void GeoCommandCreateGear::setTipReliefLength2(double length)
	{
		_tipReliefLength2 = length;
	}
	void GeoCommandCreateGear::setprofileShiftCoefficient1(double coefficient1)
	{
		_x1 = coefficient1;
	}
	void GeoCommandCreateGear::setprofileShiftCoefficient2(double coefficient2)
	{
		_x2 = coefficient2;
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
    double m  = _module;
    int    Z  = _numberOfTeeth;
    double Z2 = _numberOfSecondTeeth;  // 第二个齿轮的齿数
    double phi = _pressureAngle * M_PI / 180.0; // 转换为弧度
    
    // 变位系数
    double x1 = _x1;  // 第一个齿轮的变位系数
    double x2 = _x2;  // 第二个齿轮的变位系数（用于计算中心距等）
    double centerDistance;
    double y_delt = 0.0;  // 齿顶高变动系数
    double alphaPrime = phi;  // 啮合角，默认为压力角

    // 计算中心距和变位相关参数
    if (x1 + x2 == 0)
    {
        qDebug() << "第一个齿轮：不用变位，直接计算中心距";
        centerDistance = (Z + Z2) * m / 2;
    }
    else
    {
        qDebug() << "第一个齿轮：使用变位计算中心距及相关参数";
        
        // 1. 计算未变位时的中心距
        double a = (Z + Z2) * m / 2;
        
        // 2. 计算总变位系数
        double x_sig = x1 + x2;
        
        // 3. 计算啮合角 α'
        double invAlpha = std::tan(phi) - phi; // 渐开线函数 invα
        double invAlphaPrime = invAlpha + 2 * x_sig * std::tan(phi) / (Z + Z2);
        
        // 求解啮合角 α'（牛顿迭代法）
        alphaPrime = phi; // 初始值设为压力角
        double tolerance = 1e-10;
        int maxIterations = 100;
        
        for (int i = 0; i < maxIterations; i++)
        {
            double f = std::tan(alphaPrime) - alphaPrime - invAlphaPrime;
            double fPrime = 1.0 / (std::cos(alphaPrime) * std::cos(alphaPrime)) - 1.0;
            
            double delta = f / fPrime;
            alphaPrime -= delta;
            
            if (std::abs(delta) < tolerance)
                break;
        }
        
        // 4. 计算实际中心距 a'
        centerDistance = a * std::cos(phi) / std::cos(alphaPrime);
        
        // 5. 计算中心距变动系数 y
        double y = (centerDistance - a) / m;
        
        // 6. 计算齿顶高变动系数 y_delt
        y_delt = x_sig - y;
        
        qDebug() << "第一个齿轮变位参数: x_sig =" << x_sig 
                 << ", y =" << y 
                 << ", y_delt =" << y_delt;
        qDebug() << "啮合角 α' =" << (alphaPrime * 180.0 / M_PI) << "度";
        qDebug() << "实际中心距 a' =" << centerDistance;
    }

    // 各圆半径计算（区分变位和非变位情况）
    double Rref = Z * m / 2.0; // 分度圆半径
    
    // 对于变位齿轮，齿顶高需要减去齿顶高变动系数
    double ha = _addendumCoeff * m;
    if (x1 + x2 != 0)
    {
        ha = (_addendumCoeff + x1 - y_delt) * m; // 齿1的齿顶高
    }
    
    double Rb = Rref * std::cos(phi); // 基圆半径
    double Ra = Rref + ha;            // 齿顶圆半径
    
    // 齿根高计算
    double hf = _dedendumCoeff * m;
    if (x1 + x2 != 0)
    {
        hf = (_dedendumCoeff - x1) * m; // 齿1的齿根高
    }
    
    double Rf = Rref - hf; // 齿根圆半径

    // 确保齿根圆不小于一个合理值
    if (Rf < 0)
        Rf = 0.1 * m;

    // 角度计算
    double angularPitch = 2.0 * M_PI / Z; // 齿距角
    
    // 齿厚半角 (在分度圆上)
    // 对于变位齿轮，齿厚会变化
    double toothThicknessHalfAngle;
    if (x1 + x2 == 0)
    {
        toothThicknessHalfAngle = angularPitch / 4.0;
    }
    else
    {
        // 变位齿轮的齿厚半角：s = m(π/2 + 2x tanφ)
        double s = m * (M_PI / 2.0 + 2 * x1 * std::tan(phi));
        toothThicknessHalfAngle = s / (2.0 * Rref);
    }

    // 渐开线参数范围
    double thetaStart = 0.0;
    if (Rf > Rb)
    {
        // 齿根圆在基圆外，渐开线从齿根圆开始
        thetaStart = std::sqrt((Rf * Rf - Rb * Rb)) / Rb;
    }
    double thetaEnd = std::sqrt((Ra * Ra - Rb * Rb)) / Rb;

    // 生成单个齿的渐开线点
    const int numPoints = 20;
    std::vector<gp_Pnt> involuteLeft;
    std::vector<gp_Pnt> involuteRight;

    // 修型起点半径（从齿顶向下 _tipReliefLength 距离）
    double R_relief_start = Ra - _tipReliefLength;

    // ===== 调试输出：齿轮参数 =====
    qDebug() << "========== 第一个齿轮参数 ==========";
    qDebug() << "齿数 Z =" << Z;
    qDebug() << "变位系数 x1 =" << x1;
    qDebug() << "齿顶圆半径 Ra =" << Ra << "mm";
    qDebug() << "齿根圆半径 Rf =" << Rf << "mm";
    qDebug() << "基圆半径 Rb =" << Rb << "mm";
    qDebug() << "分度圆半径 Rref =" << Rref << "mm";
    qDebug() << "齿厚半角 =" << (toothThicknessHalfAngle * 180.0 / M_PI) << "度";
    qDebug() << "渐开线参数范围: thetaStart =" << thetaStart 
             << ", thetaEnd =" << thetaEnd;
    qDebug() << "===================================";

    // ===== 调试输出：修型参数 =====
    qDebug() << "========== 齿轮修型参数 ==========";
    qDebug() << "齿顶圆半径 Ra =" << Ra << "mm";
    qDebug() << "基圆半径 Rb =" << Rb << "mm";
    qDebug() << "修型量 Ca =" << _tipReliefAmount << "mm";
    qDebug() << "修型长度 Lca =" << _tipReliefLength << "mm";
    qDebug() << "修型起点半径 R_start =" << R_relief_start << "mm";
    qDebug() << "===================================";

		int reliefPointCount = 0; // 统计被修型的点数

		for(int i = 0; i <= numPoints; ++i) {
			double t		 = (double)i / numPoints;
			double theta	 = thetaStart + t * (thetaEnd - thetaStart);
			gp_Pnt pt		 = involutePoint(Rb, theta);

			// 计算当前点的半径
			double R_current = std::sqrt(pt.X() * pt.X() + pt.Y() * pt.Y());

			// ===== 抛物线修型 =====
			// 如果启用修型且当前点在修型区域内
			if(_tipReliefAmount > 0 && _tipReliefLength > 0 && R_current > R_relief_start) {
				// 到修型起点的距离
				double y	   = R_current - R_relief_start;
				// 抛物线修型量: δ = Ca * (y/Lca)²
				double delta   = _tipReliefAmount * (y / _tipReliefLength) * (y / _tipReliefLength);

				// 计算该点的压力角
				double alpha_y = std::acos(Rb / R_current);

				// 简化处理：沿径向向内偏移
				double nx	   = pt.X() / R_current; // 径向单位向量
				double ny	   = pt.Y() / R_current;

				// 调试输出：每个被修型的点
				if(reliefPointCount < 5) { // 只输出前5个点避免刷屏
					qDebug() << "点" << i << ": R=" << R_current << "mm, y=" << y
							 << "mm, delta=" << delta << "mm";
					qDebug() << "  原坐标:(" << pt.X() << "," << pt.Y() << ")";
				}

				// 向内偏移 delta 距离
				pt.SetX(pt.X() - delta * nx);
				pt.SetY(pt.Y() - delta * ny);

				if(reliefPointCount < 5) {
					qDebug() << "  修型后:(" << pt.X() << "," << pt.Y() << ")";
				}

				reliefPointCount++;
			}

			// 计算渐开线在分度圆处的角度偏移
			double angleAtRef = involuteAngle(Rb, Rref);

			// 旋转使齿对称于X轴
			gp_Pnt ptRotated  = rotatePoint(pt, -angleAtRef - toothThicknessHalfAngle);
			involuteLeft.push_back(ptRotated);

			// 镜像得到另一侧渐开线
			involuteRight.push_back(mirrorPoint(ptRotated));
		}

		// 调试输出：修型统计
		qDebug() << "修型点数:" << reliefPointCount << "/" << (numPoints + 1);
		if(reliefPointCount == 0 && _tipReliefAmount > 0) {
			qDebug() << "警告: 没有点被修型! 请检查修型长度是否太小或齿顶圆半径计算是否正确";
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

	TopoDS_Wire GeoCommandCreateGear::createSecondGearProfile()
	{
		// 齿轮基本参数计算
		double m	= _module;
		int	   Z	= _numberOfSecondTeeth;
		double Z1   = _numberOfTeeth;
		double phi	= _pressureAngle * M_PI / 180.0; // 转换为弧度
		double centerDistance;

		double x1 = _x1;
		double x2 = _x2;
		double _y_delt;
		qDebug()<<"变位系数"<<x1<<","<<x2;
		if (x1 + x2 == 0)
		{
			qDebug()<<"不用变位 直接计算中心距";
			centerDistance = (Z1 + Z)*m/2;
			
		}
		
	  else
	  {
		qDebug() << "使用变位计算中心距及相关参数";
        
        // 1. 计算未变位时的中心距
        double a = (Z1 + Z) * m / 2;
        
        // 2. 计算总变位系数
        double x_sig = x1 + x2;
        
        // 3. 计算啮合角 α'
        // 根据公式：invα' = invα + 2 * (x1 + x2) * tan(φ) / (Z1 + Z)
        double invAlpha = std::tan(phi) - phi; // 渐开线函数 invα
        double invAlphaPrime = invAlpha + 2 * x_sig * std::tan(phi) / (Z1 + Z);
        
        // 求解啮合角 α'（需要迭代求解，这里使用牛顿法）
        double alphaPrime = phi; // 初始值设为压力角
        double tolerance = 1e-10;
        int maxIterations = 100;
        
        for (int i = 0; i < maxIterations; i++)
        {
            double f = std::tan(alphaPrime) - alphaPrime - invAlphaPrime;
            double fPrime = 1.0 / (std::cos(alphaPrime) * std::cos(alphaPrime)) - 1.0;
            
            double delta = f / fPrime;
            alphaPrime -= delta;
            
            if (std::abs(delta) < tolerance)
                break;
        }
        
        // 4. 计算实际中心距 a'
        centerDistance = a * std::cos(phi) / std::cos(alphaPrime);
        
        // 5. 计算中心距变动系数 y
        double y = (centerDistance - a) / m;
        
        // 6. 计算齿顶高变动系数 y_delt
        double y_delt = x_sig - y;
        
        // 存储计算得到的变位参数（如果需要）
        double _y = y;
        _y_delt = y_delt;
        double _alphaPrime = alphaPrime * 180.0 / M_PI; // 转换为角度
        
        qDebug() << "变位参数: x_sig =" << x_sig << ", y =" << y << ", y_delt =" << y_delt;
        qDebug() << "啮合角 α' =" << _alphaPrime << "度";
        qDebug() << "实际中心距 a' =" << centerDistance;
    }

    // 各圆半径计算（需要区分变位和非变位情况）
    double Rref = Z * m / 2.0; // 分度圆半径
    
    // 对于变位齿轮，齿顶高需要减去齿顶高变动系数
    double ha = _addendumCoeff * m;
    if (x1 + x2 != 0)
    {
		qDebug() << "使用变位计算齿顶高";
        ha = (_addendumCoeff + x2 - (_y_delt)) * m; // 齿2的齿顶高
    }
    
    double Rb = Rref * std::cos(phi); // 基圆半径
    double Ra = Rref + ha;            // 齿顶圆半径
    
    // 齿根高计算
    double hf = _dedendumCoeff * m;
    if (x1 + x2 != 0)
    {
        hf = (_dedendumCoeff - x2) * m; // 齿2的齿根高
    }
    
    double Rf = Rref - hf; // 齿根圆半径

    // 确保齿根圆不小于一个合理值
    if (Rf < 0)
        Rf = 0.1 * m;

    // 角度计算
    double angularPitch = 2.0 * M_PI / Z; // 齿距角
    
    // 齿厚半角 (在分度圆上)
    // 对于变位齿轮，齿厚会变化
    double toothThicknessHalfAngle;
    if (x1 + x2 == 0)
    {
        toothThicknessHalfAngle = angularPitch / 4.0;
    }
    else
    {
        // 变位齿轮的齿厚半角：s = m(π/2 + 2x tanφ)
        double s = m * (M_PI / 2.0 + 2 * x2 * std::tan(phi));
        toothThicknessHalfAngle = s / (2.0 * Rref);
    }

    // 渐开线参数范围
    double thetaStart = 0.0;
    if (Rf > Rb)
    {
        // 齿根圆在基圆外，渐开线从齿根圆开始
        thetaStart = std::sqrt((Rf * Rf - Rb * Rb)) / Rb;
    }
    double thetaEnd = std::sqrt((Ra * Ra - Rb * Rb)) / Rb;

		// 生成单个齿的渐开线点
		const int			numPoints = 20;
		std::vector<gp_Pnt> involuteLeft;
		std::vector<gp_Pnt> involuteRight;

		// 修型起点半径（从齿顶向下 _tipReliefLength2 距离）
		double				R_relief_start = Ra - _tipReliefLength2;

		// ===== 调试输出：修型参数 =====
		qDebug() << "========== 第二个齿轮修型参数 ==========";
		qDebug() << "齿顶圆半径 Ra =" << Ra << "mm";
		qDebug() << "基圆半径 Rb =" << Rb << "mm";
		qDebug() << "修型量 Ca =" << _tipReliefAmount2 << "mm";
		qDebug() << "修型长度 Lca =" << _tipReliefLength2 << "mm";
		qDebug() << "修型起点半径 R_start =" << R_relief_start << "mm";
		qDebug() << "中心距 a =" << centerDistance << "mm";
		qDebug() << "===================================";

		int reliefPointCount = 0; // 统计被修型的点数

		for(int i = 0; i <= numPoints; ++i) {
			double t		 = (double)i / numPoints;
			double theta	 = thetaStart + t * (thetaEnd - thetaStart);
			gp_Pnt pt		 = involutePoint(Rb, theta);

			// 计算当前点的半径
			double R_current = std::sqrt(pt.X() * pt.X() + pt.Y() * pt.Y());

			// ===== 抛物线修型 =====
			// 如果启用修型且当前点在修型区域内
			if(_tipReliefAmount2 > 0 && _tipReliefLength2 > 0 && R_current > R_relief_start) {
				// 到修型起点的距离
				double y	   = R_current - R_relief_start;
				// 抛物线修型量: δ = Ca * (y/Lca)²
				double delta   = _tipReliefAmount2 * (y / _tipReliefLength2) * (y / _tipReliefLength2);

				// 计算该点的压力角
				double alpha_y = std::acos(Rb / R_current);

				// 简化处理：沿径向向内偏移
				double nx	   = pt.X() / R_current; // 径向单位向量
				double ny	   = pt.Y() / R_current;

				// 调试输出：每个被修型的点
				if(reliefPointCount < 5) { // 只输出前5个点避免刷屏
					qDebug() << "点" << i << ": R=" << R_current << "mm, y=" << y
							 << "mm, delta=" << delta << "mm";
					qDebug() << "  原坐标:(" << pt.X() << "," << pt.Y() << ")";
				}

				// 向内偏移 delta 距离
				pt.SetX(pt.X() - delta * nx);
				pt.SetY(pt.Y() - delta * ny);

				if(reliefPointCount < 5) {
					qDebug() << "  修型后:(" << pt.X() << "," << pt.Y() << ")";
				}

				reliefPointCount++;
			}

			// 计算渐开线在分度圆处的角度偏移
			double angleAtRef = involuteAngle(Rb, Rref);

			// 旋转使齿对称于X轴
			gp_Pnt ptRotated  = rotatePoint(pt, -angleAtRef - toothThicknessHalfAngle);
			involuteLeft.push_back(ptRotated);

			// 镜像得到另一侧渐开线
			involuteRight.push_back(mirrorPoint(ptRotated));
		}

		// 调试输出：修型统计
		qDebug() << "修型点数:" << reliefPointCount << "/" << (numPoints + 1);
		if(reliefPointCount == 0 && _tipReliefAmount2 > 0) {
			qDebug() << "警告: 没有点被修型! 请检查修型长度是否太小或齿顶圆半径计算是否正确";
		}

		BRepBuilderAPI_MakeWire wireBuilder;

		// 为每个齿创建轮廓（先在 (0,0,0) 创建）
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
					gp_Pnt arcEnd(Rf * std::cos(limitedEndAngle), Rf * std::sin(limitedEndAngle), 0);

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

		// 先在 (0,0,0) 创建 Wire
		TopoDS_Wire wire = wireBuilder.Wire();

		// 最后统一平移到 (0, centerDistance, 0)
		gp_Trsf transform;
		transform.SetTranslation(gp_Vec(0, centerDistance, 0));
		BRepBuilderAPI_Transform transformMaker(wire, transform);
		TopoDS_Wire transformedWire = TopoDS::Wire(transformMaker.Shape());

		return transformedWire;
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
		// 使用与 GeoCommandMakeExtrusion 相同的参数：Copy=true, Canonize=false
		// 这样可以确保生成 Solid 而不是 Shell
		gp_Vec				  extrusionDir(0, 0, _thickness);
		BRepPrimAPI_MakePrism prismMaker(face, extrusionDir, true, false);

		if(!prismMaker.IsDone()) {
			return TopoDS_Shape();
		}

		return prismMaker.Shape();
	}

	bool GeoCommandCreateGear::execute()
	{
		// 创建第一个齿轮2D轮廓
		TopoDS_Wire profile = createGearProfile();
		if(profile.IsNull()) {
			return false;
		}

		// 拉伸生成3D齿轮
		TopoDS_Shape gearShape = extrudeProfile(profile);
		if(gearShape.IsNull()) {
			return false;
		}

		// 创建第二个齿轮2D轮廓
		TopoDS_Wire secondProfile = createSecondGearProfile();
		if(secondProfile.IsNull()) {
			return false;
		}

		// 拉伸生成第二个3D齿轮
		TopoDS_Shape secondGearShape = extrudeProfile(secondProfile);
		if(secondGearShape.IsNull()) {
			return false;
		}
		// 需要计算中心距 - 根据齿轮参数
    double m = _module;
    int Z1 = _numberOfTeeth;
    int Z2 = _numberOfSecondTeeth;
    double x1 = _x1;
    double x2 = _x2;
    double phi = _pressureAngle * M_PI / 180.0;
    
    // 计算中心距
    double centerDistance;
    if (x1 + x2 == 0)
    {
        // 非变位齿轮的标准中心距
        centerDistance = (Z1 + Z2) * m / 2.0;
    }
    else
    {
        // 变位齿轮的中心距计算
        // 1. 计算未变位时的中心距
        double a = (Z1 + Z2) * m / 2.0;
        
        // 2. 计算总变位系数
        double x_sig = x1 + x2;
        
        // 3. 计算啮合角 α'
        double invAlpha = std::tan(phi) - phi;
        double invAlphaPrime = invAlpha + 2 * x_sig * std::tan(phi) / (Z1 + Z2);
        
        // 求解啮合角 α'（牛顿迭代法）
        double alphaPrime = phi;
        double tolerance = 1e-10;
        int maxIterations = 100;
        
        for (int i = 0; i < maxIterations; i++)
        {
            double f = std::tan(alphaPrime) - alphaPrime - invAlphaPrime;
            double fPrime = 1.0 / (std::cos(alphaPrime) * std::cos(alphaPrime)) - 1.0;
            
            double delta = f / fPrime;
            alphaPrime -= delta;
            
            if (std::abs(delta) < tolerance)
                break;
        }
        
        // 4. 计算实际中心距 a'
        centerDistance = a * std::cos(phi) / std::cos(alphaPrime);
    }
    
    // 计算旋转角度
    double rotationAngle = 0.0;
    
    // 根据齿数计算旋转角度
    // 旋转半个齿的角度（对于第二个齿轮）
    rotationAngle = M_PI / Z2;
	// 对第二个齿轮进行旋转
    if(std::abs(rotationAngle) > 1e-6) {
        gp_Trsf rotateTransform;
        gp_Ax1 rotationAxis(gp_Pnt(0, centerDistance, 0), gp_Dir(0, 0, 1));
        rotateTransform.SetRotation(gp_Ax1(gp_Pnt(0, centerDistance, 0), gp_Dir(0, 0, 1)), rotationAngle);
        BRepBuilderAPI_Transform rotateMaker(secondGearShape, rotateTransform);
        if(rotateMaker.IsDone()) {
            secondGearShape = rotateMaker.Shape();
        }
    }
		// 合并两个齿轮
		BRepAlgoAPI_Fuse fuseMaker(gearShape, secondGearShape);
		if(!fuseMaker.IsDone()) {
			return false;
		}
		TopoDS_Shape combinedShape = fuseMaker.Shape();

		// 创建形状指针
		TopoDS_Shape* shape		   = new TopoDS_Shape;
		*shape					   = combinedShape;

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
		para->setTipReliefAmount(_tipReliefAmount);
		para->setTipReliefLength(_tipReliefLength);
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
