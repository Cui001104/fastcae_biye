#ifndef _GEOCOMMANDCREATEGEAR_H_
#define _GEOCOMMANDCREATEGEAR_H_

#include "geometryCommandAPI.h"
#include "GeoCommandBase.h"
#include <QString>
#include <vector>

#include <gp_Pnt.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Shape.hxx>

namespace Geometry {
	class GeometrySet;
}

namespace Command {
	/**
	 * @brief 创建渐开线齿轮的命令类
	 */
	class GEOMETRYCOMMANDAPI GeoCommandCreateGear : public GeoCommandBase {
		Q_OBJECT
	public:
		GeoCommandCreateGear(GUI::MainWindow* m, MainWidget::PreWindow* p);
		~GeoCommandCreateGear() = default;

		bool execute() override;
		void undo() override;
		void redo() override;
		void releaseResult() override;

		/// 设置名称
		void setName(QString name);
		/// 设置齿数
		void setNumberOfTeeth(int n);
		void setNumberOfSecondTeeth(int n);
		/// 设置模数 (mm)
		void setModule(double m);
		/// 设置压力角 (度)
		void setPressureAngle(double angle);
		/// 设置齿顶高系数
		void setAddendumCoefficient(double coeff);
		/// 设置齿根高系数
		void setDedendumCoefficient(double coeff);
		/// 设置齿根圆角系数
		void setFilletCoefficient(double coeff);
		/// 设置齿轮厚度 (mm)
		void setThickness(double t);
		/// 设置是否为外齿轮
		void setExternalGear(bool external);
		/// 设置齿顶修型量 (mm)
		void setTipReliefAmount(double amount);
		/// 设置齿顶修型长度 (mm)
		void setTipReliefLength(double length);
		void setTipReliefAmount2(double amount);
		void setTipReliefLength2(double length);
		//变位系数
		void setprofileShiftCoefficient1(double coefficient1);
		void setprofileShiftCoefficient2(double coefficient2);

	private:
		/// 生成渐开线齿廓点
		void		 generateInvolutePoints(std::vector<gp_Pnt>& points);
		/// 创建齿轮2D轮廓线
		TopoDS_Wire	 createGearProfile();
		TopoDS_Wire createSecondGearProfile();
		/// 分度圆直径对应的中心圆柱孔闭合线（反向 orientation，用作面内孔）
		TopoDS_Wire	 createCenterHoleWire(double holeRadius, const gp_Pnt& centerOnXYPlane) const;
		/// 两齿轮轴线方向上的标准/变位中心距（与 createSecondGearProfile 平移一致）
		double		 centerDistanceBetweenGears() const;
		/// 拉伸生成3D齿轮（外轮廓 + 中心内孔闭环）
		TopoDS_Shape extrudeProfile(const TopoDS_Wire& outerProfile, const TopoDS_Wire& innerHoleWire);

	private:
		QString				   _name{};
		int					   _numberOfTeeth{ 26 };
		int                    _numberOfSecondTeeth{ 26 };
		double				   _module{ 2.5 };
		double				   _pressureAngle{ 20.0 };
		double				   _addendumCoeff{ 1.0 };
		double				   _dedendumCoeff{ 1.25 };
		double				   _filletCoeff{ 0.38 };
		double				   _thickness{ 10.0 };
		bool				   _externalGear{ true };
		double				   _tipReliefAmount{ 0.0 }; ///< 齿顶修型量 (mm)
		double				   _tipReliefLength{ 0.0 }; ///< 齿顶修型长度 (mm)
		double				   _tipReliefAmount2{ 0.0 };
		double                 _tipReliefLength2{ 0.0 };
		double				   _x1 { 0.0 };
		double				   _x2{ 0.0 };//第二个齿轮的变位系数

		Geometry::GeometrySet* _res{};
	};
} // namespace Command

#endif
