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
#include <Standard_Failure.hxx>

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

	// 娓愬紑绾挎瀬鍧愭爣瑙掑害璁＄畻
	static double involuteAngle(double Rb, double R)
	{
		return std::sqrt(R * R - Rb * Rb) / Rb - std::acos(Rb / R);
	}

	// 娓愬紑绾跨偣璁＄畻 (鍩轰簬鍙傛暟theta)
	static gp_Pnt involutePoint(double Rb, double theta)
	{
		double x = Rb * (std::cos(theta) + theta * std::sin(theta));
		double y = Rb * (std::sin(theta) - theta * std::cos(theta));
		return gp_Pnt(x, y, 0);
	}

	// 鏃嬭浆鐐?
	static gp_Pnt rotatePoint(const gp_Pnt& pt, double angle)
	{
		double cosA = std::cos(angle);
		double sinA = std::sin(angle);
		double x	= pt.X() * cosA - pt.Y() * sinA;
		double y	= pt.X() * sinA + pt.Y() * cosA;
		return gp_Pnt(x, y, pt.Z());
	}

	// 闀滃儚鐐?(鍏充簬X杞?
	static gp_Pnt mirrorPoint(const gp_Pnt& pt)
	{
		return gp_Pnt(pt.X(), -pt.Y(), pt.Z());
	}

	TopoDS_Wire GeoCommandCreateGear::createGearProfile()
	{
		// 榻胯疆鍩烘湰鍙傛暟璁＄畻
    double m  = _module;
    int    Z  = _numberOfTeeth;
    double Z2 = _numberOfSecondTeeth;  // 绗簩涓娇杞殑榻挎暟
    double phi = _pressureAngle * M_PI / 180.0; // 杞崲涓哄姬搴?
    
    // 鍙樹綅绯绘暟
    double x1 = _x1;  // 绗竴涓娇杞殑鍙樹綅绯绘暟
    double x2 = _x2;  // 绗簩涓娇杞殑鍙樹綅绯绘暟锛堢敤浜庤绠椾腑蹇冭窛绛夛級
    double centerDistance;
    double y_delt = 0.0;  // 榻块《楂樺彉鍔ㄧ郴鏁?
    double alphaPrime = phi;  // 鍟悎瑙掞紝榛樿涓哄帇鍔涜

    // 璁＄畻涓績璺濆拰鍙樹綅鐩稿叧鍙傛暟
    if (x1 + x2 == 0)
    {
        qDebug() << "绗竴涓娇杞細涓嶇敤鍙樹綅锛岀洿鎺ヨ绠椾腑蹇冭窛";
        centerDistance = (Z + Z2) * m / 2;
    }
    else
    {
        qDebug() << "绗竴涓娇杞細浣跨敤鍙樹綅璁＄畻涓績璺濆強鐩稿叧鍙傛暟";
        
        // 1. 璁＄畻鏈彉浣嶆椂鐨勪腑蹇冭窛
        double a = (Z + Z2) * m / 2;
        
        // 2. 璁＄畻鎬诲彉浣嶇郴鏁?
        double x_sig = x1 + x2;
        
        // 3. 璁＄畻鍟悎瑙?伪'
        double invAlpha = std::tan(phi) - phi; // 娓愬紑绾垮嚱鏁?inv伪
        double invAlphaPrime = invAlpha + 2 * x_sig * std::tan(phi) / (Z + Z2);
        
        // 姹傝В鍟悎瑙?伪'锛堢墰椤胯凯浠ｆ硶锛?
        alphaPrime = phi; // 鍒濆鍊艰涓哄帇鍔涜
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
        
        // 4. 璁＄畻瀹為檯涓績璺?a'
        centerDistance = a * std::cos(phi) / std::cos(alphaPrime);
        
        // 5. 璁＄畻涓績璺濆彉鍔ㄧ郴鏁?y
        double y = (centerDistance - a) / m;
        
        // 6. 璁＄畻榻块《楂樺彉鍔ㄧ郴鏁?y_delt
        y_delt = x_sig - y;
        
        qDebug() << "绗竴涓娇杞彉浣嶅弬鏁? x_sig =" << x_sig 
                 << ", y =" << y 
                 << ", y_delt =" << y_delt;
        qDebug() << "alphaPrime =" << (alphaPrime * 180.0 / M_PI) << " deg";
        qDebug() << "瀹為檯涓績璺?a' =" << centerDistance;
    }

    // 鍚勫渾鍗婂緞璁＄畻锛堝尯鍒嗗彉浣嶅拰闈炲彉浣嶆儏鍐碉級
    double Rref = Z * m / 2.0; // 鍒嗗害鍦嗗崐寰?
    
    // 瀵逛簬鍙樹綅榻胯疆锛岄娇椤堕珮闇€瑕佸噺鍘婚娇椤堕珮鍙樺姩绯绘暟
    double ha = _addendumCoeff * m;
    if (x1 + x2 != 0)
    {
        ha = (_addendumCoeff + x1 - y_delt) * m; // 榻?鐨勯娇椤堕珮
    }
    
    double Rb = Rref * std::cos(phi); // 鍩哄渾鍗婂緞
    double Ra = Rref + ha;            // 榻块《鍦嗗崐寰?
    
    // 榻挎牴楂樿绠?
    double hf = _dedendumCoeff * m;
    if (x1 + x2 != 0)
    {
        hf = (_dedendumCoeff - x1) * m; // 榻?鐨勯娇鏍归珮
    }
    
    double Rf = Rref - hf; // 榻挎牴鍦嗗崐寰?

    // 纭繚榻挎牴鍦嗕笉灏忎簬涓€涓悎鐞嗗€?
    if (Rf < 0)
        Rf = 0.1 * m;

    // 瑙掑害璁＄畻
    double angularPitch = 2.0 * M_PI / Z; // 榻胯窛瑙?
    
    // 榻垮帤鍗婅 (鍦ㄥ垎搴﹀渾涓?
    // 瀵逛簬鍙樹綅榻胯疆锛岄娇鍘氫細鍙樺寲
    double toothThicknessHalfAngle;
    if (x1 + x2 == 0)
    {
        toothThicknessHalfAngle = angularPitch / 4.0;
    }
    else
    {
        // 鍙樹綅榻胯疆鐨勯娇鍘氬崐瑙掞細s = m(蟺/2 + 2x tan蠁)
        double s = m * (M_PI / 2.0 + 2 * x1 * std::tan(phi));
        toothThicknessHalfAngle = s / (2.0 * Rref);
    }

    // 娓愬紑绾垮弬鏁拌寖鍥?
    double thetaStart = 0.0;
    if (Rf > Rb)
    {
        // 榻挎牴鍦嗗湪鍩哄渾澶栵紝娓愬紑绾夸粠榻挎牴鍦嗗紑濮?
        thetaStart = std::sqrt((Rf * Rf - Rb * Rb)) / Rb;
    }
    double thetaEnd = std::sqrt((Ra * Ra - Rb * Rb)) / Rb;

    // 鐢熸垚鍗曚釜榻跨殑娓愬紑绾跨偣
    const int numPoints = 20;
    std::vector<gp_Pnt> involuteLeft;
    std::vector<gp_Pnt> involuteRight;

    // 淇瀷璧风偣鍗婂緞锛堜粠榻块《鍚戜笅 _tipReliefLength 璺濈锛?
    double R_relief_start = Ra - _tipReliefLength;

    // ===== 璋冭瘯杈撳嚭锛氶娇杞弬鏁?=====
    qDebug() << "========== 绗竴涓娇杞弬鏁?==========";
    qDebug() << "榻挎暟 Z =" << Z;
    qDebug() << "鍙樹綅绯绘暟 x1 =" << x1;
    qDebug() << "榻块《鍦嗗崐寰?Ra =" << Ra << "mm";
    qDebug() << "榻挎牴鍦嗗崐寰?Rf =" << Rf << "mm";
    qDebug() << "鍩哄渾鍗婂緞 Rb =" << Rb << "mm";
    qDebug() << "鍒嗗害鍦嗗崐寰?Rref =" << Rref << "mm";
    qDebug() << "toothThicknessHalfAngle =" << (toothThicknessHalfAngle * 180.0 / M_PI) << " deg";
    qDebug() << "娓愬紑绾垮弬鏁拌寖鍥? thetaStart =" << thetaStart 
             << ", thetaEnd =" << thetaEnd;
    qDebug() << "===================================";

    // ===== 璋冭瘯杈撳嚭锛氫慨鍨嬪弬鏁?=====
    qDebug() << "========== 榻胯疆淇瀷鍙傛暟 ==========";
    qDebug() << "榻块《鍦嗗崐寰?Ra =" << Ra << "mm";
    qDebug() << "鍩哄渾鍗婂緞 Rb =" << Rb << "mm";
    qDebug() << "淇瀷閲?Ca =" << _tipReliefAmount << "mm";
    qDebug() << "淇瀷闀垮害 Lca =" << _tipReliefLength << "mm";
    qDebug() << "淇瀷璧风偣鍗婂緞 R_start =" << R_relief_start << "mm";
    qDebug() << "===================================";

		int reliefPointCount = 0; // 缁熻琚慨鍨嬬殑鐐规暟

		for(int i = 0; i <= numPoints; ++i) {
			double t		 = (double)i / numPoints;
			double theta	 = thetaStart + t * (thetaEnd - thetaStart);
			gp_Pnt pt		 = involutePoint(Rb, theta);

			// 璁＄畻褰撳墠鐐圭殑鍗婂緞
			double R_current = std::sqrt(pt.X() * pt.X() + pt.Y() * pt.Y());

			// ===== 鎶涚墿绾夸慨鍨?=====
			// 濡傛灉鍚敤淇瀷涓斿綋鍓嶇偣鍦ㄤ慨鍨嬪尯鍩熷唴
			if(_tipReliefAmount > 0 && _tipReliefLength > 0 && R_current > R_relief_start) {
				// 鍒颁慨鍨嬭捣鐐圭殑璺濈
				double y	   = R_current - R_relief_start;
				// 鎶涚墿绾夸慨鍨嬮噺: 未 = Ca * (y/Lca)虏
				double delta   = _tipReliefAmount * (y / _tipReliefLength) * (y / _tipReliefLength);

				// 璁＄畻璇ョ偣鐨勫帇鍔涜
				double alpha_y = std::acos(Rb / R_current);

				// 绠€鍖栧鐞嗭細娌垮緞鍚戝悜鍐呭亸绉?
				double nx	   = pt.X() / R_current; // 寰勫悜鍗曚綅鍚戦噺
				double ny	   = pt.Y() / R_current;

				// 璋冭瘯杈撳嚭锛氭瘡涓淇瀷鐨勭偣
				if(reliefPointCount < 5) { // 鍙緭鍑哄墠5涓偣閬垮厤鍒峰睆
                    qDebug() << "point" << i << ": R=" << R_current << "mm, y=" << y
							 << "mm, delta=" << delta << "mm";
					qDebug() << "  鍘熷潗鏍?(" << pt.X() << "," << pt.Y() << ")";
				}

				// 鍚戝唴鍋忕Щ delta 璺濈
				pt.SetX(pt.X() - delta * nx);
				pt.SetY(pt.Y() - delta * ny);

				if(reliefPointCount < 5) {
					qDebug() << "  淇瀷鍚?(" << pt.X() << "," << pt.Y() << ")";
				}

				reliefPointCount++;
			}

			// 璁＄畻娓愬紑绾垮湪鍒嗗害鍦嗗鐨勮搴﹀亸绉?
			double angleAtRef = involuteAngle(Rb, Rref);

			// 鏃嬭浆浣块娇瀵圭О浜嶺杞?
			gp_Pnt ptRotated  = rotatePoint(pt, -angleAtRef - toothThicknessHalfAngle);
			involuteLeft.push_back(ptRotated);

			// 闀滃儚寰楀埌鍙︿竴渚ф笎寮€绾?
			involuteRight.push_back(mirrorPoint(ptRotated));
		}

		// 璋冭瘯杈撳嚭锛氫慨鍨嬬粺璁?
		qDebug() << "淇瀷鐐规暟:" << reliefPointCount << "/" << (numPoints + 1);
		if(reliefPointCount == 0 && _tipReliefAmount > 0) {
            qDebug() << "Warning: no relief points were modified.";
		}

		BRepBuilderAPI_MakeWire wireBuilder;

		// 涓烘瘡涓娇鍒涘缓杞粨
		for(int tooth = 0; tooth < Z; ++tooth) {
			double toothAngle = tooth * angularPitch;

			// 娓愬紑绾垮乏渚?(浠庨娇鏍瑰埌榻块《)
			for(size_t i = 0; i < involuteLeft.size() - 1; ++i) {
				gp_Pnt p1 = rotatePoint(involuteLeft[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteLeft[i + 1], toothAngle);
				if(p1.Distance(p2) > 1e-6) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
					wireBuilder.Add(edge);
				}
			}

			// 榻块《鍦嗗姬
			gp_Pnt tipLeft	= rotatePoint(involuteLeft.back(), toothAngle);
			gp_Pnt tipRight = rotatePoint(involuteRight.back(), toothAngle);

			if(tipLeft.Distance(tipRight) > 1e-6) {
				// 浣跨敤鍦嗗姬杩炴帴榻块《
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
						// 濡傛灉鍦嗗姬澶辫触锛屼娇鐢ㄧ洿绾?
						TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(tipLeft, tipRight);
						wireBuilder.Add(edge);
					}
				} catch(...) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(tipLeft, tipRight);
					wireBuilder.Add(edge);
				}
			}

			// 娓愬紑绾垮彸渚?(浠庨娇椤跺埌榻挎牴)
			for(int i = (int)involuteRight.size() - 1; i > 0; --i) {
				gp_Pnt p1 = rotatePoint(involuteRight[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteRight[i - 1], toothAngle);
				if(p1.Distance(p2) > 1e-6) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
					wireBuilder.Add(edge);
				}
			}

			// 榻挎牴鍦嗗姬 (杩炴帴鍒颁笅涓€涓娇)
			gp_Pnt rootRight	= rotatePoint(involuteRight.front(), toothAngle);
			gp_Pnt nextRootLeft = rotatePoint(involuteLeft.front(), toothAngle + angularPitch);

			if(rootRight.Distance(nextRootLeft) > 1e-6) {
				// 璁＄畻榻挎牴鍦嗗姬鐨勮捣濮嬪拰缁撴潫瑙掑害
				double rootRightAngle	 = std::atan2(rootRight.Y(), rootRight.X());
				double nextRootLeftAngle = std::atan2(nextRootLeft.Y(), nextRootLeft.X());
				qDebug() << " 绗竴涓娇杞?cya 0312 ========== 榻挎牴鐐硅皟璇曚俊鎭?==========";
qDebug() << "toothAngle:" << toothAngle << "rad (" << toothAngle * 180/M_PI << "掳)";
qDebug() << "angularPitch:" << angularPitch << "rad (" << angularPitch * 180/M_PI << "掳)";
qDebug() << "next tooth angle:" << toothAngle + angularPitch << "rad (" 
         << (toothAngle + angularPitch) * 180/M_PI << "掳)";

qDebug() << "\n--- 褰撳墠榻垮彸渚ф笎寮€绾胯捣鐐?---";
qDebug() << "鍘熷鐐?(involuteRight.front()):";
qDebug() << "  X:" << involuteRight.front().X();
qDebug() << "  Y:" << involuteRight.front().Y();
qDebug() << "  Z:" << involuteRight.front().Z();

				// 纭繚瑙掑害杩炵画锛堝鐞嗚法瓒?搴︾殑鎯呭喌锛?
				if(nextRootLeftAngle < rootRightAngle) {
					nextRootLeftAngle += 2.0 * M_PI;
					qDebug() << "cya 0312璋冩暣鍚巒extRootLeft瑙掑害:" << nextRootLeftAngle * 180/M_PI << "掳";
				}

				// 璁＄畻榻挎牴鍦嗗姬鐨勮搴﹁法搴?
				double rootArcSpan = nextRootLeftAngle - rootRightAngle;
                qDebug() << "cya 0312 root arc span:" << rootArcSpan;
				// 濡傛灉鍦嗗姬瑙掑害瓒呰繃180搴︼紝闄愬埗涓?80搴?
				if(rootArcSpan > M_PI) {
                    qDebug() << "root arc span > 180 deg, using 180 deg arc";
					// 璁＄畻180搴﹀渾寮х殑缁堢偣瑙掑害
					double limitedEndAngle = rootRightAngle + M_PI;
					// 鍦嗗姬涓偣锛堟伆濂藉湪璧风偣鍋忕Щ90搴﹀锛?
					double rootMidAngle	   = rootRightAngle + M_PI / 2.0;
					gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
					// 180搴﹀渾寮х殑缁堢偣
					gp_Pnt arcEnd(Rf * std::cos(limitedEndAngle), Rf * std::sin(limitedEndAngle),
								  0);

					try {
						// 鍒涘缓180搴﹀渾寮?
						GC_MakeArcOfCircle arcMaker(rootRight, rootMid, arcEnd);
						if(arcMaker.IsDone()) {
							wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
						}
					} catch(...) {
					}

					// 鐢ㄧ洿绾胯繛鎺ュ墿浣欓儴鍒嗭紙浠庡渾寮х粓鐐瑰埌涓嬩竴涓娇鐨勬笎寮€绾胯捣鐐癸級
					if(arcEnd.Distance(nextRootLeft) > 1e-6) {
						TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(arcEnd, nextRootLeft);
						wireBuilder.Add(edge);
					}
				} else {
					qDebug() <<"榻挎牴鍦嗗皬浜?80";
					// 瑙掑害璺ㄥ害杩囧皬鍒欑敤鐩寸嚎锛岄伩鍏?GC_MakeArcOfCircle 鎶涘嚭 StdFail_NotDone
					const double minRootArcSpan = 1e-6;
					if(rootArcSpan < minRootArcSpan) {
						TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft);
						wireBuilder.Add(edge);
                        qDebug() << "root arc span too small, using line segment";
					} else {
					// 鍦嗗姬瑙掑害涓嶈秴杩?80搴︼紝姝ｅ父澶勭悊
					double rootMidAngle = (rootRightAngle + nextRootLeftAngle) / 2.0;
					gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
					qDebug() << "涓偣瑙掑害:" << rootMidAngle * 180/M_PI << "掳";
					qDebug() << "涓偣鍧愭爣: (" << rootMid.X() << "," << rootMid.Y() << ")";
					// 楠岃瘉涓夌偣鏄惁鍏辩嚎
					double det = rootRight.X() * (rootMid.Y() - nextRootLeft.Y()) +
					rootMid.X() * (nextRootLeft.Y() - rootRight.Y()) +
					nextRootLeft.X() * (rootRight.Y() - rootMid.Y());
	   qDebug() << "涓夌偣鍏辩嚎妫€娴?(det):" << det;
	   if(qAbs(det) < 1e-10) {
           qDebug() << "Warning: three points are nearly collinear.";
	   }
	   // 褰撲笁鐐规帴杩戝叡绾?|det|杈冨皬)鏃讹紝涓夌偣娉曚細寰楀埌閫€鍖栧姬(鏄剧ず涓虹洿绾?锛屾敼鐢ㄥ渾蹇冩硶淇濊瘉榻挎牴涓哄渾寮?
	   const double detThreshold = 0.1;
	   bool useCircleMethod = (qAbs(det) < detThreshold);
	   if(useCircleMethod) {
		   qDebug() << "det 杩囧皬锛屾敼鐢ㄥ渾蹇冩硶鍒涘缓榻挎牴鍦嗗姬";
	   }
	   try {
		if(useCircleMethod) {
			// 鍦嗗績娉曪細鍦ㄩ娇鏍瑰渾涓婃寜瑙掑害鍒涘缓鍦嗗姬锛屾暟鍊肩ǔ瀹?
			gp_Circ circle(gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), Rf);
			GC_MakeArcOfCircle arcMaker2(circle, rootRightAngle, nextRootLeftAngle, true);
			if(arcMaker2.IsDone()) {
				TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker2.Value());
				wireBuilder.Add(arcEdge);
                qDebug() << "circle-center method created root arc successfully";
			} else {
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft);
				wireBuilder.Add(edge);
				qDebug() << "鍦嗗績娉曞け璐ワ紝浣跨敤鐩寸嚎鏇夸唬";
			}
		} else {
			GC_MakeArcOfCircle arcMaker(rootRight, rootMid, nextRootLeft);
			if(arcMaker.IsDone()) {
                qDebug() << "arc created successfully";
				TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker.Value());
				wireBuilder.Add(arcEdge);
			} else {
				qDebug() << "鍦嗗姬鍒涘缓澶辫触锛屼娇鐢ㄥ渾蹇冩硶";
				gp_Circ circle(gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), Rf);
				GC_MakeArcOfCircle arcMaker2(circle, rootRightAngle, nextRootLeftAngle, true);
				if(arcMaker2.IsDone()) {
					TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker2.Value());
					wireBuilder.Add(arcEdge);
                    qDebug() << "circle-center method created arc successfully";
				} else {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft);
					wireBuilder.Add(edge);
					qDebug() << "浣跨敤鐩寸嚎鏇夸唬";
				}
			}
		}
	   } catch(...) {
		qDebug() << "寮傚父锛屼娇鐢ㄥ渾蹇冩硶鍒涘缓鍦嗗姬";
		try {
			gp_Circ circle(gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), Rf);
			GC_MakeArcOfCircle arcMaker2(circle, rootRightAngle, nextRootLeftAngle, true);
			if(arcMaker2.IsDone()) {
				TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker2.Value());
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
					} // rootArcSpan >= minRootArcSpan
				}
			}
		}

		if(!wireBuilder.IsDone()) {
			qDebug() << "绗竴涓娇杞?Wire 鏋勫缓鏈畬鎴?(IsDone 涓?false)";
			return TopoDS_Wire();
		}
		try {
			return wireBuilder.Wire();
		} catch(Standard_Failure& e) {
			qDebug() << "绗竴涓娇杞?Wire() 寮傚父:" << e.GetMessageString();
			return TopoDS_Wire();
		}
	}

	TopoDS_Wire GeoCommandCreateGear::createSecondGearProfile()
	{
		// 榻胯疆鍩烘湰鍙傛暟璁＄畻
		double m	= _module;
		int	   Z	= _numberOfSecondTeeth;
		double Z1   = _numberOfTeeth;
		double phi	= _pressureAngle * M_PI / 180.0; // 杞崲涓哄姬搴?
		double centerDistance;

		double x1 = _x1;
		double x2 = _x2;
		double _y_delt;
		qDebug()<<"鍙樹綅绯绘暟"<<x1<<","<<x2;
		if (x1 + x2 == 0)
		{
            qDebug() << "No profile shift, using direct center distance calculation";
			centerDistance = (Z1 + Z)*m/2;
			
		}
		
	  else
	  {
		qDebug() << "浣跨敤鍙樹綅璁＄畻涓績璺濆強鐩稿叧鍙傛暟";
        
        // 1. 璁＄畻鏈彉浣嶆椂鐨勪腑蹇冭窛
        double a = (Z1 + Z) * m / 2;
        
        // 2. 璁＄畻鎬诲彉浣嶇郴鏁?
        double x_sig = x1 + x2;
        
        // 3. 璁＄畻鍟悎瑙?伪'
        // 鏍规嵁鍏紡锛歩nv伪' = inv伪 + 2 * (x1 + x2) * tan(蠁) / (Z1 + Z)
        double invAlpha = std::tan(phi) - phi; // 娓愬紑绾垮嚱鏁?inv伪
        double invAlphaPrime = invAlpha + 2 * x_sig * std::tan(phi) / (Z1 + Z);
        
        // 姹傝В鍟悎瑙?伪'锛堥渶瑕佽凯浠ｆ眰瑙ｏ紝杩欓噷浣跨敤鐗涢】娉曪級
        double alphaPrime = phi; // 鍒濆鍊艰涓哄帇鍔涜
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
        
        // 4. 璁＄畻瀹為檯涓績璺?a'
        centerDistance = a * std::cos(phi) / std::cos(alphaPrime);
        
        // 5. 璁＄畻涓績璺濆彉鍔ㄧ郴鏁?y
        double y = (centerDistance - a) / m;
        
        // 6. 璁＄畻榻块《楂樺彉鍔ㄧ郴鏁?y_delt
        double y_delt = x_sig - y;
        
        // 瀛樺偍璁＄畻寰楀埌鐨勫彉浣嶅弬鏁帮紙濡傛灉闇€瑕侊級
        double _y = y;
        _y_delt = y_delt;
        double _alphaPrime = alphaPrime * 180.0 / M_PI; // 杞崲涓鸿搴?
        
        qDebug() << "鍙樹綅鍙傛暟: x_sig =" << x_sig << ", y =" << y << ", y_delt =" << y_delt;
        qDebug() << "alphaPrime =" << _alphaPrime << " deg";
        qDebug() << "瀹為檯涓績璺?a' =" << centerDistance;
    }

    // 鍚勫渾鍗婂緞璁＄畻锛堥渶瑕佸尯鍒嗗彉浣嶅拰闈炲彉浣嶆儏鍐碉級
    double Rref = Z * m / 2.0; // 鍒嗗害鍦嗗崐寰?
    
    // 瀵逛簬鍙樹綅榻胯疆锛岄娇椤堕珮闇€瑕佸噺鍘婚娇椤堕珮鍙樺姩绯绘暟
    double ha = _addendumCoeff * m;
    if (x1 + x2 != 0)
    {
        qDebug() << "Using profile shift to calculate addendum";
        ha = (_addendumCoeff + x2 - (_y_delt)) * m; // 榻?鐨勯娇椤堕珮
    }
    
    double Rb = Rref * std::cos(phi); // 鍩哄渾鍗婂緞
    double Ra = Rref + ha;            // 榻块《鍦嗗崐寰?
    
    // 榻挎牴楂樿绠?
    double hf = _dedendumCoeff * m;
    if (x1 + x2 != 0)
    {
        hf = (_dedendumCoeff - x2) * m; // 榻?鐨勯娇鏍归珮
    }
    
    double Rf = Rref - hf; // 榻挎牴鍦嗗崐寰?

    // 纭繚榻挎牴鍦嗕笉灏忎簬涓€涓悎鐞嗗€?
    if (Rf < 0)
        Rf = 0.1 * m;

    // 瑙掑害璁＄畻
    double angularPitch = 2.0 * M_PI / Z; // 榻胯窛瑙?
    
    // 榻垮帤鍗婅 (鍦ㄥ垎搴﹀渾涓?
    // 瀵逛簬鍙樹綅榻胯疆锛岄娇鍘氫細鍙樺寲
    double toothThicknessHalfAngle;
    if (x1 + x2 == 0)
    {
        toothThicknessHalfAngle = angularPitch / 4.0;
    }
    else
    {
        // 鍙樹綅榻胯疆鐨勯娇鍘氬崐瑙掞細s = m(蟺/2 + 2x tan蠁)
        double s = m * (M_PI / 2.0 + 2 * x2 * std::tan(phi));
        toothThicknessHalfAngle = s / (2.0 * Rref);
    }

    // 娓愬紑绾垮弬鏁拌寖鍥?
    double thetaStart = 0.0;
    if (Rf > Rb)
    {
        // 榻挎牴鍦嗗湪鍩哄渾澶栵紝娓愬紑绾夸粠榻挎牴鍦嗗紑濮?
        thetaStart = std::sqrt((Rf * Rf - Rb * Rb)) / Rb;
    }
    double thetaEnd = std::sqrt((Ra * Ra - Rb * Rb)) / Rb;

		// 鐢熸垚鍗曚釜榻跨殑娓愬紑绾跨偣
		const int			numPoints = 20;
		std::vector<gp_Pnt> involuteLeft;
		std::vector<gp_Pnt> involuteRight;

		// 淇瀷璧风偣鍗婂緞锛堜粠榻块《鍚戜笅 _tipReliefLength2 璺濈锛?
		double				R_relief_start = Ra - _tipReliefLength2;

		// ===== 璋冭瘯杈撳嚭锛氫慨鍨嬪弬鏁?=====
		qDebug() << "========== 绗簩涓娇杞慨鍨嬪弬鏁?==========";
		qDebug() << "榻块《鍦嗗崐寰?Ra =" << Ra << "mm";
		qDebug() << "鍩哄渾鍗婂緞 Rb =" << Rb << "mm";
		qDebug() << "淇瀷閲?Ca =" << _tipReliefAmount2 << "mm";
		qDebug() << "淇瀷闀垮害 Lca =" << _tipReliefLength2 << "mm";
		qDebug() << "淇瀷璧风偣鍗婂緞 R_start =" << R_relief_start << "mm";
		qDebug() << "涓績璺?a =" << centerDistance << "mm";
		qDebug() << "===================================";

		int reliefPointCount = 0; // 缁熻琚慨鍨嬬殑鐐规暟

		for(int i = 0; i <= numPoints; ++i) {
			double t		 = (double)i / numPoints;
			double theta	 = thetaStart + t * (thetaEnd - thetaStart);
			gp_Pnt pt		 = involutePoint(Rb, theta);

			// 璁＄畻褰撳墠鐐圭殑鍗婂緞
			double R_current = std::sqrt(pt.X() * pt.X() + pt.Y() * pt.Y());

			// ===== 鎶涚墿绾夸慨鍨?=====
			// 濡傛灉鍚敤淇瀷涓斿綋鍓嶇偣鍦ㄤ慨鍨嬪尯鍩熷唴
			if(_tipReliefAmount2 > 0 && _tipReliefLength2 > 0 && R_current > R_relief_start) {
				// 鍒颁慨鍨嬭捣鐐圭殑璺濈
				double y	   = R_current - R_relief_start;
				// 鎶涚墿绾夸慨鍨嬮噺: 未 = Ca * (y/Lca)虏
				double delta   = _tipReliefAmount2 * (y / _tipReliefLength2) * (y / _tipReliefLength2);

				// 璁＄畻璇ョ偣鐨勫帇鍔涜
				double alpha_y = std::acos(Rb / R_current);

				// 绠€鍖栧鐞嗭細娌垮緞鍚戝悜鍐呭亸绉?
				double nx	   = pt.X() / R_current; // 寰勫悜鍗曚綅鍚戦噺
				double ny	   = pt.Y() / R_current;

				// 璋冭瘯杈撳嚭锛氭瘡涓淇瀷鐨勭偣
				if(reliefPointCount < 5) { // 鍙緭鍑哄墠5涓偣閬垮厤鍒峰睆
                    qDebug() << "point" << i << ": R=" << R_current << "mm, y=" << y
							 << "mm, delta=" << delta << "mm";
					qDebug() << "  鍘熷潗鏍?(" << pt.X() << "," << pt.Y() << ")";
				}

				// 鍚戝唴鍋忕Щ delta 璺濈
				pt.SetX(pt.X() - delta * nx);
				pt.SetY(pt.Y() - delta * ny);

				if(reliefPointCount < 5) {
					qDebug() << "  淇瀷鍚?(" << pt.X() << "," << pt.Y() << ")";
				}

				reliefPointCount++;
			}

			// 璁＄畻娓愬紑绾垮湪鍒嗗害鍦嗗鐨勮搴﹀亸绉?
			double angleAtRef = involuteAngle(Rb, Rref);

			// 鏃嬭浆浣块娇瀵圭О浜嶺杞?
			gp_Pnt ptRotated  = rotatePoint(pt, -angleAtRef - toothThicknessHalfAngle);
			involuteLeft.push_back(ptRotated);

			// 闀滃儚寰楀埌鍙︿竴渚ф笎寮€绾?
			involuteRight.push_back(mirrorPoint(ptRotated));
		}

		// 璋冭瘯杈撳嚭锛氫慨鍨嬬粺璁?
		qDebug() << "淇瀷鐐规暟:" << reliefPointCount << "/" << (numPoints + 1);
		if(reliefPointCount == 0 && _tipReliefAmount2 > 0) {
            qDebug() << "Warning: no relief points were modified.";
		}

		BRepBuilderAPI_MakeWire wireBuilder;

		// 涓烘瘡涓娇鍒涘缓杞粨锛堝厛鍦?(0,0,0) 鍒涘缓锛?
		for(int tooth = 0; tooth < Z; ++tooth) {
			double toothAngle = tooth * angularPitch;

			// 娓愬紑绾垮乏渚?(浠庨娇鏍瑰埌榻块《)
			for(size_t i = 0; i < involuteLeft.size() - 1; ++i) {
				gp_Pnt p1 = rotatePoint(involuteLeft[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteLeft[i + 1], toothAngle);
				if(p1.Distance(p2) > 1e-6) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
					wireBuilder.Add(edge);
				}
			}

			// 榻块《鍦嗗姬
			gp_Pnt tipLeft	= rotatePoint(involuteLeft.back(), toothAngle);
			gp_Pnt tipRight = rotatePoint(involuteRight.back(), toothAngle);

			if(tipLeft.Distance(tipRight) > 1e-6) {
				// 浣跨敤鍦嗗姬杩炴帴榻块《
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
						// 濡傛灉鍦嗗姬澶辫触锛屼娇鐢ㄧ洿绾?
						TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(tipLeft, tipRight);
						wireBuilder.Add(edge);
					}
				} catch(...) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(tipLeft, tipRight);
					wireBuilder.Add(edge);
				}
			}

			// 娓愬紑绾垮彸渚?(浠庨娇椤跺埌榻挎牴)
			for(int i = (int)involuteRight.size() - 1; i > 0; --i) {
				gp_Pnt p1 = rotatePoint(involuteRight[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteRight[i - 1], toothAngle);
				if(p1.Distance(p2) > 1e-6) {
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(p1, p2);
					wireBuilder.Add(edge);
				}
			}

			// 榻挎牴鍦嗗姬 (杩炴帴鍒颁笅涓€涓娇)
			gp_Pnt rootRight	= rotatePoint(involuteRight.front(), toothAngle);
			gp_Pnt nextRootLeft = rotatePoint(involuteLeft.front(), toothAngle + angularPitch);

			if(rootRight.Distance(nextRootLeft) > 1e-6) {
				// 璁＄畻榻挎牴鍦嗗姬鐨勮捣濮嬪拰缁撴潫瑙掑害
				double rootRightAngle	 = std::atan2(rootRight.Y(), rootRight.X());
				double nextRootLeftAngle = std::atan2(nextRootLeft.Y(), nextRootLeft.X());
				qDebug() << " 绗簩涓娇杞?cya 0312 ========== 榻挎牴鐐硅皟璇曚俊鎭?==========";
				qDebug() << "toothAngle:" << toothAngle << "rad (" << toothAngle * 180/M_PI << "掳)";
				qDebug() << "angularPitch:" << angularPitch << "rad (" << angularPitch * 180/M_PI << "掳)";
				qDebug() << "next tooth angle:" << toothAngle + angularPitch << "rad ("
				         << (toothAngle + angularPitch) * 180/M_PI << "掳)";
				qDebug() << "\n--- 褰撳墠榻垮彸渚ф笎寮€绾胯捣鐐?---";
				qDebug() << "鍘熷鐐?(involuteRight.front()):";
				qDebug() << "  X:" << involuteRight.front().X();
				qDebug() << "  Y:" << involuteRight.front().Y();
				qDebug() << "  Z:" << involuteRight.front().Z();

				// 纭繚瑙掑害杩炵画锛堝鐞嗚法瓒?搴︾殑鎯呭喌锛?
				if(nextRootLeftAngle < rootRightAngle) {
					nextRootLeftAngle += 2.0 * M_PI;
					qDebug() << "cya 0312璋冩暣鍚巒extRootLeft瑙掑害:" << nextRootLeftAngle * 180/M_PI << "掳";
				}

				// 璁＄畻榻挎牴鍦嗗姬鐨勮搴﹁法搴?
				double rootArcSpan = nextRootLeftAngle - rootRightAngle;
                qDebug() << "cya 0312 root arc span:" << rootArcSpan;
				// 濡傛灉鍦嗗姬瑙掑害瓒呰繃180搴︼紝闄愬埗涓?80搴?
				if(rootArcSpan > M_PI) {
                    qDebug() << "root arc span > 180 deg, using 180 deg arc";
					// 璁＄畻180搴﹀渾寮х殑缁堢偣瑙掑害
					double limitedEndAngle = rootRightAngle + M_PI;
					// 鍦嗗姬涓偣锛堟伆濂藉湪璧风偣鍋忕Щ90搴﹀锛?
					double rootMidAngle	   = rootRightAngle + M_PI / 2.0;
					gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
					// 180搴﹀渾寮х殑缁堢偣
					gp_Pnt arcEnd(Rf * std::cos(limitedEndAngle), Rf * std::sin(limitedEndAngle), 0);

					try {
						// 鍒涘缓180搴﹀渾寮?
						GC_MakeArcOfCircle arcMaker(rootRight, rootMid, arcEnd);
						if(arcMaker.IsDone()) {
							wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
						}
					} catch(...) {
					}

					// 鐢ㄧ洿绾胯繛鎺ュ墿浣欓儴鍒嗭紙浠庡渾寮х粓鐐瑰埌涓嬩竴涓娇鐨勬笎寮€绾胯捣鐐癸級
					if(arcEnd.Distance(nextRootLeft) > 1e-6) {
						TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(arcEnd, nextRootLeft);
						wireBuilder.Add(edge);
					}
				} else {
					qDebug() << "榻挎牴鍦嗗皬浜?80";
					// 鍦嗗姬瑙掑害涓嶈秴杩?80搴︼紝姝ｅ父澶勭悊
					double rootMidAngle = (rootRightAngle + nextRootLeftAngle) / 2.0;
					gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
					qDebug() << "涓偣瑙掑害:" << rootMidAngle * 180/M_PI << "掳";
					qDebug() << "涓偣鍧愭爣: (" << rootMid.X() << "," << rootMid.Y() << ")";
					// 楠岃瘉涓夌偣鏄惁鍏辩嚎
					double det = rootRight.X() * (rootMid.Y() - nextRootLeft.Y()) +
						rootMid.X() * (nextRootLeft.Y() - rootRight.Y()) +
						nextRootLeft.X() * (rootRight.Y() - rootMid.Y());
					qDebug() << "涓夌偣鍏辩嚎妫€娴?(det):" << det;
					if(qAbs(det) < 1e-10) {
                        qDebug() << "Warning: three points are nearly collinear.";
					}
					try {
						GC_MakeArcOfCircle arcMaker(rootRight, rootMid, nextRootLeft);
						if(arcMaker.IsDone()) {
                            qDebug() << "arc created successfully";
							TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker.Value());
							wireBuilder.Add(arcEdge);
						} else {
							qDebug() << "鍦嗗姬鍒涘缓澶辫触锛屼娇鐢ㄤ笁鐐瑰渾寮х殑鏇夸唬鏂规硶";
							gp_Circ circle(gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), Rf);
							GC_MakeArcOfCircle arcMaker2(circle, rootRightAngle, nextRootLeftAngle, true);
							if(arcMaker2.IsDone()) {
								TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker2.Value());
								wireBuilder.Add(arcEdge);
                                qDebug() << "circle-center method created arc successfully";
							} else {
								TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft);
								wireBuilder.Add(edge);
								qDebug() << "浣跨敤鐩寸嚎鏇夸唬";
							}
						}
					} catch(...) {
						qDebug() << "寮傚父锛屼娇鐢ㄥ渾蹇冩硶鍒涘缓鍦嗗姬";
						try {
							gp_Circ circle(gp_Ax2(gp_Pnt(0,0,0), gp_Dir(0,0,1)), Rf);
							GC_MakeArcOfCircle arcMaker2(circle, rootRightAngle, nextRootLeftAngle, true);
							if(arcMaker2.IsDone()) {
								TopoDS_Edge arcEdge = BRepBuilderAPI_MakeEdge(arcMaker2.Value());
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
		}

		// 鍏堝湪 (0,0,0) 鍒涘缓 Wire
		TopoDS_Wire wire = wireBuilder.Wire();

		// 鏈€鍚庣粺涓€骞崇Щ鍒?(0, centerDistance, 0)
		gp_Trsf transform;
		transform.SetTranslation(gp_Vec(0, centerDistance, 0));
		BRepBuilderAPI_Transform transformMaker(wire, transform);
		TopoDS_Wire transformedWire = TopoDS::Wire(transformMaker.Shape());

		return transformedWire;
	}

	double GeoCommandCreateGear::centerDistanceBetweenGears() const
	{
		const double m	 = _module;
		const int	   Z1	= _numberOfTeeth;
		const int	   Z2	= _numberOfSecondTeeth;
		const double x1	 = _x1;
		const double x2	 = _x2;
		const double phi = _pressureAngle * M_PI / 180.0;

		if(x1 + x2 == 0) {
			return (Z1 + Z2) * m / 2.0;
		}

		const double a			  = (Z1 + Z2) * m / 2.0;
		const double x_sig			  = x1 + x2;
		const double invAlpha		  = std::tan(phi) - phi;
		const double invAlphaPrime = invAlpha + 2 * x_sig * std::tan(phi) / (Z1 + Z2);

		double alphaPrime = phi;
		const double tolerance	 = 1e-10;
		const int	 maxIterations = 100;
		for(int i = 0; i < maxIterations; i++) {
			const double f	   = std::tan(alphaPrime) - alphaPrime - invAlphaPrime;
			const double fPrime = 1.0 / (std::cos(alphaPrime) * std::cos(alphaPrime)) - 1.0;
			const double delta  = f / fPrime;
			alphaPrime -= delta;
			if(std::abs(delta) < tolerance)
				break;
		}
		return a * std::cos(phi) / std::cos(alphaPrime);
	}

	TopoDS_Wire GeoCommandCreateGear::createCenterHoleWire(double holeRadius,
															const gp_Pnt& centerOnXYPlane) const
	{
		if(holeRadius <= 1e-9) {
			return TopoDS_Wire();
		}
		gp_Ax2	 ax(centerOnXYPlane, gp_Dir(0, 0, 1));
		gp_Circ	 circ(ax, holeRadius);
		TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(circ);
		BRepBuilderAPI_MakeWire wireMk(edge);
		if(!wireMk.IsDone()) {
			return TopoDS_Wire();
		}
		// 鍐呭瓟杈圭晫涓庨娇寤撳鐜柟鍚戠浉鍙嶏紝鏂瑰彲鍦ㄥ悓涓€骞抽潰闈㈠煙涓綔涓哄瓟
		return TopoDS::Wire(wireMk.Wire().Reversed());
	}

	TopoDS_Shape GeoCommandCreateGear::extrudeProfile(const TopoDS_Wire& outerProfile,
													  const TopoDS_Wire& innerHoleWire)
	{
		// 鍦╔Y骞抽潰涓婂垱寤洪潰锛堝鐜?+ 涓績瀛旈棴鐜級
		gp_Pln					plane(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
		BRepBuilderAPI_MakeFace faceMaker(plane, outerProfile,
										  Standard_True); // Standard_True 琛ㄧず妫€鏌ュ苟淇 wire
		if(!innerHoleWire.IsNull()) {
			faceMaker.Add(innerHoleWire);
		}
		if(!faceMaker.IsDone()) {
			return TopoDS_Shape();
		}

		TopoDS_Face			  face = faceMaker.Face();

		// 娌縕杞存媺浼?
		// 浣跨敤涓?GeoCommandMakeExtrusion 鐩稿悓鐨勫弬鏁帮細Copy=true, Canonize=false
		// 杩欐牱鍙互纭繚鐢熸垚 Solid 鑰屼笉鏄?Shell
		gp_Vec				  extrusionDir(0, 0, _thickness);
		BRepPrimAPI_MakePrism prismMaker(face, extrusionDir, true, false);

		if(!prismMaker.IsDone()) {
			return TopoDS_Shape();
		}

		return prismMaker.Shape();
	}

	bool GeoCommandCreateGear::execute()
	{
		// 鍒涘缓绗竴涓娇杞?D杞粨
		TopoDS_Wire profile = createGearProfile();
		if(profile.IsNull()) {
			return false;
		}

		// 鍒涘缓绗簩涓娇杞?D杞粨
		TopoDS_Wire secondProfile = createSecondGearProfile();
		if(secondProfile.IsNull()) {
			return false;
		}

		const double centerDistance = centerDistanceBetweenGears();

		// 榻胯疆1锛氫腑蹇冨唴瀛旓紙鍒嗗害鍦嗙洿寰?= m * Z锛屽瓟寰?= 0.4 脳 鍒嗗害鍦嗙洿寰勶級
		const double pitchDiameter	= _module * static_cast<double>(_numberOfTeeth);
		const double holeDiameter	= 0.4 * pitchDiameter;
		const double holeRadius		= holeDiameter / 2.0;
		TopoDS_Wire				   holeWire1 = createCenterHoleWire(holeRadius, gp_Pnt(0, 0, 0));

		// 鎷変几鐢熸垚3D榻胯疆
		TopoDS_Shape gearShape = extrudeProfile(profile, holeWire1);
		if(gearShape.IsNull()) {
			return false;
		}

		// 榻胯疆2锛氬瓟涓績涓庡钩绉诲悗榻垮澂涓績 (0, centerDistance, 0) 閲嶅悎
		const double pitchDiameter2  = _module * static_cast<double>(_numberOfSecondTeeth);
		const double holeDiameter2   = 0.4 * pitchDiameter2;
		const double holeRadius2	 = holeDiameter2 / 2.0;
		TopoDS_Wire				   holeWire2 =
			createCenterHoleWire(holeRadius2, gp_Pnt(0, centerDistance, 0));

		// 鎷変几鐢熸垚绗簩涓?D榻胯疆
		TopoDS_Shape secondGearShape = extrudeProfile(secondProfile, holeWire2);
		if(secondGearShape.IsNull()) {
			return false;
		}

    // 璁＄畻鏃嬭浆瑙掑害
    double rotationAngle = 0.0;
    
    // 鏍规嵁榻挎暟璁＄畻鏃嬭浆瑙掑害
    // 鏃嬭浆鍗婁釜榻跨殑瑙掑害锛堝浜庣浜屼釜榻胯疆锛?
    rotationAngle = M_PI / static_cast<double>(_numberOfSecondTeeth);
	// 瀵圭浜屼釜榻胯疆杩涜鏃嬭浆
    if(std::abs(rotationAngle) > 1e-6) {
        gp_Trsf rotateTransform;
        gp_Ax1 rotationAxis(gp_Pnt(0, centerDistance, 0), gp_Dir(0, 0, 1));
        rotateTransform.SetRotation(gp_Ax1(gp_Pnt(0, centerDistance, 0), gp_Dir(0, 0, 1)), rotationAngle);
        BRepBuilderAPI_Transform rotateMaker(secondGearShape, rotateTransform);
        if(rotateMaker.IsDone()) {
            secondGearShape = rotateMaker.Shape();
        }
    }
		// 鍚堝苟涓や釜榻胯疆
		BRepAlgoAPI_Fuse fuseMaker(gearShape, secondGearShape);
		if(!fuseMaker.IsDone()) {
			return false;
		}
		TopoDS_Shape combinedShape = fuseMaker.Shape();

		// 鍒涘缓褰㈢姸鎸囬拡
		TopoDS_Shape* shape		   = new TopoDS_Shape;
		*shape					   = combinedShape;

		// 鍒涘缓鍑犱綍闆嗗璞?
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

		// 鍒涘缓鍙傛暟瀵硅薄
		Geometry::GeometryParaGear* para = new Geometry::GeometryParaGear;
		para->setName(_name);
		para->setNumberOfTeeth(_numberOfTeeth);
		para->setNumberOfSecondTeeth(_numberOfSecondTeeth);
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


