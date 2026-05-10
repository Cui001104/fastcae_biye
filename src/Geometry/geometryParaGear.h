#ifndef _GEOMETRYPARAGEAR_H_
#define _GEOMETRYPARAGEAR_H_

#include "geometryModelParaBase.h"
#include <QString>

namespace Geometry
{
	class GEOMETRYAPI GeometryParaGear : public GeometryModelParaBase
	{
	public:
		GeometryParaGear();
		~GeometryParaGear() = default;

		void setName(QString name);
		QString getName();

		void setNumberOfTeeth(int n);
		int getNumberOfTeeth();

		void setNumberOfSecondTeeth(int n);
		int getNumberOfSecondTeeth();

		void setModule(double m);
		double getModule();

		void setPressureAngle(double angle);
		double getPressureAngle();

		void setAddendumCoefficient(double coeff);
		double getAddendumCoefficient();

		void setDedendumCoefficient(double coeff);
		double getDedendumCoefficient();

		void setFilletCoefficient(double coeff);
		double getFilletCoefficient();

		void setThickness(double t);
		double getThickness();

		void setThickness2(double t);
		double getThickness2();

		void setExternalGear(bool external);
		bool isExternalGear();

		void setTipReliefAmount(double amount);
		double getTipReliefAmount();

		void setTipReliefLength(double length);
		double getTipReliefLength();

		void setTipReliefAmount2(double amount);
		double getTipReliefAmount2();

		void setTipReliefLength2(double length);
		double getTipReliefLength2();

		void setProfileShiftCoefficient1(double x1);
		double getProfileShiftCoefficient1();

		void setProfileShiftCoefficient2(double x2);
		double getProfileShiftCoefficient2();

		QDomElement &writeToProjectFile(QDomDocument *doc, QDomElement *parent) override;
		void readDataFromProjectFile(QDomElement *e) override;

	private:
		QString _name{};
		int _numberOfTeeth{26};
		int _numberOfSecondTeeth{0};
		double _module{2.5};
		double _pressureAngle{20.0};
		double _addendumCoeff{1.0};
		double _dedendumCoeff{1.25};
		double _filletCoeff{0.38};
		double _thickness{10.0};
		double _thickness2{10.0};
		bool _externalGear{true};
		double _tipReliefAmount{0.0};
		double _tipReliefLength{0.0};
		double _tipReliefAmount2{0.0};
		double _tipReliefLength2{0.0};
		double _x1{0.0};
		double _x2{0.0};
	};
}

#endif

