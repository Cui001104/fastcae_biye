#include "GeoCommandCreateGear.h"
#include "Geometry/geometryData.h"
#include "Geometry/geometrySet.h"
#include "Geometry/geometryParaGear.h"

#include <cmath>
#include <limits>
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

#include <TopExp_Explorer.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

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

	void GeoCommandCreateGear::setThickness2(double t)
	{
		_thickness2 = t;
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
		// thin wrapper, real logic in buildGearProfileWire
		return buildGearProfileWire(_numberOfTeeth, _numberOfSecondTeeth,
		                            _x1, _x1 + _x2,
		                            _tipReliefAmount, _tipReliefLength,
		                            gp_Pnt(0, 0, 0));
	}

	TopoDS_Wire GeoCommandCreateGear::buildGearProfileWire(int Z, int Zmate,
	                                                       double xOwn, double xSum,
	                                                       double tipReliefAmount,
	                                                       double tipReliefLength,
	                                                       const gp_Pnt& center)
	{
		// Generic gear profile wire builder (parameterized version of original gear-1 path).
		// Z      : own gear tooth count
		// Zmate  : mating gear tooth count (used in working center distance)
		// xOwn   : this gear's profile shift coefficient
		// xSum   : x1 + x2 (shared between both gears)
		// tipReliefAmount, tipReliefLength : own gear's parabolic tip relief
		// center : final placement of the wire (origin for gear 1, (0, a', 0) for gear 2)
		const double m   = _module;
		const double phi = _pressureAngle * M_PI / 180.0;
		const int    Z2  = Zmate;            // alias: legacy expressions reference Z2
		const double x1  = xOwn;             // alias
		const double x2  = xSum - xOwn;      // alias (x1 + x2 == xSum)

		double centerDistance;
		double y_delt = 0.0;
		double alphaPrime = phi;

		if (xSum == 0.0) {
			centerDistance = (Z + Z2) * m / 2.0;
		} else {
			const double a            = (Z + Z2) * m / 2.0;
			const double x_sig        = xSum;
			const double invAlpha     = std::tan(phi) - phi;
			const double invAlphaPrime = invAlpha + 2 * x_sig * std::tan(phi) / (Z + Z2);

			alphaPrime = phi;
			const double tolerance = 1e-10;
			const int    maxIterations = 100;
			for (int i = 0; i < maxIterations; ++i) {
				const double f      = std::tan(alphaPrime) - alphaPrime - invAlphaPrime;
				const double fPrime = 1.0 / (std::cos(alphaPrime) * std::cos(alphaPrime)) - 1.0;
				const double delta  = f / fPrime;
				alphaPrime -= delta;
				if (std::abs(delta) < tolerance) break;
			}

			centerDistance = a * std::cos(phi) / std::cos(alphaPrime);
			const double y = (centerDistance - a) / m;
			y_delt = x_sig - y;
		}
		(void)centerDistance;  // not used directly inside helper (gear 2 wrapper computes its own)
		(void)alphaPrime;

		const double Rref = Z * m / 2.0;
		double ha = _addendumCoeff * m;
		if (xSum != 0.0) ha = (_addendumCoeff + x1 - y_delt) * m;
		const double Rb = Rref * std::cos(phi);
		const double Ra = Rref + ha;

		double hf = _dedendumCoeff * m;
		if (xSum != 0.0) hf = (_dedendumCoeff - x1) * m;
		double Rf = Rref - hf;
		if (Rf < 0) Rf = 0.1 * m;

		const double angularPitch = 2.0 * M_PI / Z;
		double toothThicknessHalfAngle;
		if (xSum == 0.0) {
			toothThicknessHalfAngle = angularPitch / 4.0;
		} else {
			const double s = m * (M_PI / 2.0 + 2 * x1 * std::tan(phi));
			toothThicknessHalfAngle = s / (2.0 * Rref);
		}

		double thetaStart = 0.0;
		if (Rf > Rb) thetaStart = std::sqrt(Rf * Rf - Rb * Rb) / Rb;
		const double thetaEnd = std::sqrt(Ra * Ra - Rb * Rb) / Rb;

		const int numPoints = 50;  
		std::vector<gp_Pnt> involuteLeft;
		std::vector<gp_Pnt> involuteRight;
		involuteLeft.reserve(numPoints + 1);
		involuteRight.reserve(numPoints + 1);

		const double R_relief_start = Ra - tipReliefLength;

		for (int i = 0; i <= numPoints; ++i) {
			const double t     = (double)i / numPoints;
			const double theta = thetaStart + t * (thetaEnd - thetaStart);
			gp_Pnt pt          = involutePoint(Rb, theta);

			const double R_current = std::sqrt(pt.X() * pt.X() + pt.Y() * pt.Y());

			// Parabolic tip relief: shift inward by Ca * (y/Lca)^2 along the radial direction.
			if (tipReliefAmount > 0.0 && tipReliefLength > 0.0 && R_current > R_relief_start) {
				const double yRel  = R_current - R_relief_start;
				const double delta = tipReliefAmount * (yRel / tipReliefLength) * (yRel / tipReliefLength);
				const double nx    = pt.X() / R_current;
				const double ny    = pt.Y() / R_current;
				pt.SetX(pt.X() - delta * nx);
				pt.SetY(pt.Y() - delta * ny);
			}

			const double angleAtRef = involuteAngle(Rb, Rref);
			gp_Pnt ptRotated = rotatePoint(pt, -angleAtRef - toothThicknessHalfAngle);
			involuteLeft.push_back(ptRotated);
			involuteRight.push_back(mirrorPoint(ptRotated));
		}

		BRepBuilderAPI_MakeWire wireBuilder;

		for (int tooth = 0; tooth < Z; ++tooth) {
			const double toothAngle = tooth * angularPitch;

			// Left flank: root -> tip
			for (size_t i = 0; i + 1 < involuteLeft.size(); ++i) {
				gp_Pnt p1 = rotatePoint(involuteLeft[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteLeft[i + 1], toothAngle);
				if (p1.Distance(p2) > 1e-6)
					wireBuilder.Add(BRepBuilderAPI_MakeEdge(p1, p2));
			}

			// Tip arc through tipLeft, tipMid (on Ra circle), tipRight.
			gp_Pnt tipLeft  = rotatePoint(involuteLeft.back(),  toothAngle);
			gp_Pnt tipRight = rotatePoint(involuteRight.back(), toothAngle);
			if (tipLeft.Distance(tipRight) > 1e-6) {
				gp_Pnt tipMid((tipLeft.X() + tipRight.X()) / 2.0 * Ra
				                  / std::sqrt(std::pow((tipLeft.X() + tipRight.X()) / 2.0, 2)
				                              + std::pow((tipLeft.Y() + tipRight.Y()) / 2.0, 2)),
				              (tipLeft.Y() + tipRight.Y()) / 2.0 * Ra
				                  / std::sqrt(std::pow((tipLeft.X() + tipRight.X()) / 2.0, 2)
				                              + std::pow((tipLeft.Y() + tipRight.Y()) / 2.0, 2)),
				              0);
				try {
					GC_MakeArcOfCircle arcMaker(tipLeft, tipMid, tipRight);
					if (arcMaker.IsDone())
						wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
					else
						wireBuilder.Add(BRepBuilderAPI_MakeEdge(tipLeft, tipRight));
				} catch (...) {
					wireBuilder.Add(BRepBuilderAPI_MakeEdge(tipLeft, tipRight));
				}
			}

			// Right flank: tip -> root
			for (int i = (int)involuteRight.size() - 1; i > 0; --i) {
				gp_Pnt p1 = rotatePoint(involuteRight[i], toothAngle);
				gp_Pnt p2 = rotatePoint(involuteRight[i - 1], toothAngle);
				if (p1.Distance(p2) > 1e-6)
					wireBuilder.Add(BRepBuilderAPI_MakeEdge(p1, p2));
			}

			// Root arc bridging current right-root to next-tooth left-root.
			gp_Pnt rootRight    = rotatePoint(involuteRight.front(), toothAngle);
			gp_Pnt nextRootLeft = rotatePoint(involuteLeft.front(),  toothAngle + angularPitch);
			if (rootRight.Distance(nextRootLeft) > 1e-6) {
				double rootRightAngle    = std::atan2(rootRight.Y(),    rootRight.X());
				double nextRootLeftAngle = std::atan2(nextRootLeft.Y(), nextRootLeft.X());
				if (nextRootLeftAngle < rootRightAngle) nextRootLeftAngle += 2.0 * M_PI;
				const double rootArcSpan = nextRootLeftAngle - rootRightAngle;

				if (rootArcSpan > M_PI) {
					// > 180 degrees: cap at 180 deg arc and bridge the rest with a line.
					const double limitedEndAngle = rootRightAngle + M_PI;
					const double rootMidAngle    = rootRightAngle + M_PI / 2.0;
					gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
					gp_Pnt arcEnd(Rf * std::cos(limitedEndAngle), Rf * std::sin(limitedEndAngle), 0);
					try {
						GC_MakeArcOfCircle arcMaker(rootRight, rootMid, arcEnd);
						if (arcMaker.IsDone())
							wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
					} catch (...) {}
					if (arcEnd.Distance(nextRootLeft) > 1e-6)
						wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcEnd, nextRootLeft));
				} else {
					const double minRootArcSpan = 1e-6;
					if (rootArcSpan < minRootArcSpan) {
						wireBuilder.Add(BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft));
					} else {
						const double rootMidAngle = (rootRightAngle + nextRootLeftAngle) / 2.0;
						gp_Pnt rootMid(Rf * std::cos(rootMidAngle), Rf * std::sin(rootMidAngle), 0);
						const double det = rootRight.X() * (rootMid.Y() - nextRootLeft.Y()) +
						                   rootMid.X() * (nextRootLeft.Y() - rootRight.Y()) +
						                   nextRootLeft.X() * (rootRight.Y() - rootMid.Y());
						const bool useCircleMethod = (qAbs(det) < 0.1);
						bool added = false;
						try {
							if (useCircleMethod) {
								gp_Circ circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), Rf);
								GC_MakeArcOfCircle arcMaker(circle, rootRightAngle, nextRootLeftAngle, true);
								if (arcMaker.IsDone()) {
									wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
									added = true;
								}
							} else {
								GC_MakeArcOfCircle arcMaker(rootRight, rootMid, nextRootLeft);
								if (arcMaker.IsDone()) {
									wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
									added = true;
								} else {
									gp_Circ circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), Rf);
									GC_MakeArcOfCircle arcMaker2(circle, rootRightAngle, nextRootLeftAngle, true);
									if (arcMaker2.IsDone()) {
										wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker2.Value()));
										added = true;
									}
								}
							}
						} catch (...) {}
						if (!added) {
							try {
								gp_Circ circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), Rf);
								GC_MakeArcOfCircle arcMaker(circle, rootRightAngle, nextRootLeftAngle, true);
								if (arcMaker.IsDone())
									wireBuilder.Add(BRepBuilderAPI_MakeEdge(arcMaker.Value()));
								else
									wireBuilder.Add(BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft));
							} catch (...) {
								wireBuilder.Add(BRepBuilderAPI_MakeEdge(rootRight, nextRootLeft));
							}
						}
					}
				}
			}
		}

		if (!wireBuilder.IsDone()) {
			qWarning() << "buildGearProfileWire: wire not done, Z =" << Z;
			return TopoDS_Wire();
		}

		TopoDS_Wire wire;
		try {
			wire = wireBuilder.Wire();
		} catch (Standard_Failure& e) {
			qWarning() << "buildGearProfileWire: wire exception:" << e.GetMessageString();
			return TopoDS_Wire();
		}

		if (center.X() != 0.0 || center.Y() != 0.0 || center.Z() != 0.0) {
			gp_Trsf trsf;
			trsf.SetTranslation(gp_Vec(center.X(), center.Y(), center.Z()));
			BRepBuilderAPI_Transform tf(wire, trsf);
			wire = TopoDS::Wire(tf.Shape());
		}
		return wire;
	}

	TopoDS_Wire GeoCommandCreateGear::createSecondGearProfile()
	{
		// Thin wrapper. Geometry built by buildGearProfileWire then translated to (0, a', 0).
		const double cd = centerDistanceBetweenGears();
		return buildGearProfileWire(_numberOfSecondTeeth, _numberOfTeeth,
		                            _x2, _x1 + _x2,
		                            _tipReliefAmount2, _tipReliefLength2,
		                            gp_Pnt(0, cd, 0));
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
													  const TopoDS_Wire& innerHoleWire,
													  double thickness)
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
		gp_Vec				  extrusionDir(0, 0, thickness);
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
		TopoDS_Shape gearShape = extrudeProfile(profile, holeWire1, _thickness);
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
		TopoDS_Shape secondGearShape = extrudeProfile(secondProfile, holeWire2, _thickness2);
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
		TopoDS_Shape* shape1 = new TopoDS_Shape;
		*shape1 = gearShape;
		TopoDS_Shape* shape2 = new TopoDS_Shape;
		*shape2 = secondGearShape;

		Geometry::GeometrySet* set1 = new Geometry::GeometrySet(Geometry::STEP);
		set1->setShape(shape1);
		Geometry::GeometrySet* set2 = new Geometry::GeometrySet(Geometry::STEP);
		set2->setShape(shape2);

		_res  = set1;
		_res2 = set2;

		const double Rf1 = _module * _numberOfTeeth        / 2.0 - _dedendumCoeff * _module;
		const double Rf2 = _module * _numberOfSecondTeeth  / 2.0 - _dedendumCoeff * _module;
		tagGearFaces(set1, holeRadius,  Rf1 > 0 ? Rf1 : 0.1 * _module);
		tagGearFaces(set2, holeRadius2, Rf2 > 0 ? Rf2 : 0.1 * _module);

		const QString baseName = _name.isEmpty() ? QStringLiteral("Gear") : _name;
		const QString name1    = baseName + QStringLiteral("_1");
		const QString name2    = baseName + QStringLiteral("_2");

		if(_isEdit) {
			set1->setName(_editSet->getName());
			_geoData->replaceSet(set1, _editSet);
			emit removeDisplayActor(_editSet);
			set2->setName(name2);
			_geoData->appendGeometrySet(set2);
		} else {
			set1->setName(name1);
			set2->setName(name2);
			_geoData->appendGeometrySet(set1);
			_geoData->appendGeometrySet(set2);
		}

		// Build parameter object, attach to primary gear Set.
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
		para->setThickness2(_thickness2);
		para->setExternalGear(_externalGear);
		para->setTipReliefAmount(_tipReliefAmount);
		para->setTipReliefLength(_tipReliefLength);
		para->setTipReliefAmount2(_tipReliefAmount2);
		para->setTipReliefLength2(_tipReliefLength2);
		para->setProfileShiftCoefficient1(_x1);
		para->setProfileShiftCoefficient2(_x2);
		_res->setParameter(para);

		GeoCommandBase::execute();
		emit updateGeoTree();
		emit showSet(set1);
		emit showSet(set2);

		return true;
	}

	void GeoCommandCreateGear::undo()
	{
		emit removeDisplayActor(_res);
		if(_res2) emit removeDisplayActor(_res2);
		if(_isEdit) {
			_geoData->replaceSet(_editSet, _res);
			emit showSet(_editSet);
			if(_res2) _geoData->removeTopGeometrySet(_res2);
		} else {
			_geoData->removeTopGeometrySet(_res);
			if(_res2) _geoData->removeTopGeometrySet(_res2);
		}
		GeoCommandBase::undo();
		emit updateGeoTree();
	}

	void GeoCommandCreateGear::redo()
	{
		if(_isEdit) {
			_geoData->replaceSet(_res, _editSet);
			emit removeDisplayActor(_editSet);
			if(_res2) _geoData->appendGeometrySet(_res2);
		} else {
			_geoData->appendGeometrySet(_res);
			if(_res2) _geoData->appendGeometrySet(_res2);
		}
		emit updateGeoTree();
		emit showSet(_res);
		if(_res2) emit showSet(_res2);
	}

	void GeoCommandCreateGear::releaseResult()
	{
		if(_res != nullptr)  delete _res;
		_res = nullptr;
		if(_res2 != nullptr) delete _res2;
		_res2 = nullptr;
	}

	// ==================== Semantic face tags  ====================
	//
	// 几何观察：齿廓 wire 是用 50+ 段折线 + 弧近似的渐开线，extrude 出 3D 后
	// **每段折线变成一个 BSpline / 平面 ruled face**——不是单一圆柱面。所以
	// 仅按 GeomAbs_Cylinder 半径分类只能识别 hub_hole，root_fillet 实际是 fillet
	//
	// 修复策略：统一用**包围盒中心到齿轮轴的径向距离 r** 分类（不依赖面类型）：
	//   * Z 法向平面（front/back 端面）→ 单独按 Z 极值定 front_face / back_face
	//   * 其它面：以 r 与 holeRadius / rootRadius / pitchRadius 比较定 tag
	//       r ≤ holeRadius * (1 + 5%)        → hub_hole
	//       r ≤ rootRadius * (1 - 1%)        → 介于 hub 和 root 之间的过渡面
	//                                          (实际不应有，落入 tooth_flank 兜底)
	//       r ∈ [rootRadius*(1-1%), rootRadius*(1+5%)] → root_fillet
	//       r > rootRadius * (1 + 5%)        → tooth_flank
	//
	// 齿轮轴：取 GeometrySet 的 shape bbox 的 XY 中心。GearCommand 把每个齿轮
	// 单独建在自己的 set 里，所以 bbox 中心 ≈ 该齿轮的轴心。
	void GeoCommandCreateGear::tagGearFaces(Geometry::GeometrySet* set,
	                                        double holeRadius,
	                                        double rootRadius)
	{
		if(!set) return;
		TopoDS_Shape* shapePtr = set->getShape();
		if(!shapePtr || shapePtr->IsNull()) return;
		const TopoDS_Shape& shape = *shapePtr;

		// 整体包围盒 → 推断齿轮轴 (xc, yc)
		Bnd_Box gearBox;
		BRepBndLib::Add(shape, gearBox);
		if(gearBox.IsVoid()) return;
		double gxmin, gymin, gzmin, gxmax, gymax, gzmax;
		gearBox.Get(gxmin, gymin, gzmin, gxmax, gymax, gzmax);
		const double axisX = 0.5 * (gxmin + gxmax);
		const double axisY = 0.5 * (gymin + gymax);

		// 半径阈值
		const double rHubMax  = holeRadius  * 1.05;
		const double rRootMin = rootRadius  * 0.99;
		const double rRootMax = rootRadius  * 1.05;

		double zMaxPlane = -std::numeric_limits<double>::infinity();
		double zMinPlane =  std::numeric_limits<double>::infinity();
		int    frontId = -1, backId = -1;

		int idx = 0;
		for(TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++idx) {
			TopoDS_Face f = TopoDS::Face(exp.Current());
			BRepAdaptor_Surface surf(f, Standard_True);
			GeomAbs_SurfaceType st = surf.GetType();

			Bnd_Box bb;
			BRepBndLib::Add(f, bb);
			if(bb.IsVoid()) continue;
			double fxmin, fymin, fzmin, fxmax, fymax, fzmax;
			bb.Get(fxmin, fymin, fzmin, fxmax, fymax, fzmax);

			// 端面（Z 法向平面）单独处理
			if(st == GeomAbs_Plane) {
				gp_Dir n = surf.Plane().Axis().Direction();
				if(std::abs(n.Z()) > 0.99) {
					const double zMid = 0.5 * (fzmin + fzmax);
					if(zMid > zMaxPlane) { zMaxPlane = zMid; frontId = idx; }
					if(zMid < zMinPlane) { zMinPlane = zMid; backId  = idx; }
					continue;
				}
			}

			// 径向距离：以 face bbox 中心到齿轮轴
			const double fcx = 0.5 * (fxmin + fxmax);
			const double fcy = 0.5 * (fymin + fymax);
			const double dx  = fcx - axisX;
			const double dy  = fcy - axisY;
			const double r   = std::sqrt(dx * dx + dy * dy);

			if(r <= rHubMax) {
				set->setSemanticTag(idx, QStringLiteral("hub_hole"));
			} else if(r >= rRootMin && r <= rRootMax) {
				set->setSemanticTag(idx, QStringLiteral("root_fillet"));
			} else {
				set->setSemanticTag(idx, QStringLiteral("tooth_flank"));
			}
		}
		if(frontId >= 0) set->setSemanticTag(frontId, QStringLiteral("front_face"));
		if(backId  >= 0 && backId != frontId)
			set->setSemanticTag(backId, QStringLiteral("back_face"));

#ifdef GEAR_DEBUG
		qDebug() << "tagGearFaces: faces =" << idx
		         << ", hub_hole =" << set->getFacesByTag("hub_hole").size()
		         << ", root_fillet =" << set->getFacesByTag("root_fillet").size()
		         << ", tooth_flank =" << set->getFacesByTag("tooth_flank").size()
		         << ", front/back =" << (frontId >= 0) << "/" << (backId >= 0);
#endif
	}
} // namespace Command


