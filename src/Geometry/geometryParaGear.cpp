#include "geometryParaGear.h"
#include <QDomDocument>
#include <QDomElement>

namespace Geometry
{
	GeometryParaGear::GeometryParaGear()
	{
		_type = GeometryParaCreateGear;
	}

	void GeometryParaGear::setName(QString name)
	{
		_name = name;
	}

	QString GeometryParaGear::getName()
	{
		return _name;
	}

	void GeometryParaGear::setNumberOfTeeth(int n)
	{
		_numberOfTeeth = n;
	}

	int GeometryParaGear::getNumberOfTeeth()
	{
		return _numberOfTeeth;
	}

	void GeometryParaGear::setNumberOfSecondTeeth(int n)
	{
		_numberOfSecondTeeth = n;
	}

	int GeometryParaGear::getNumberOfSecondTeeth()
	{
		return _numberOfSecondTeeth;
	}

	void GeometryParaGear::setModule(double m)
	{
		_module = m;
	}

	double GeometryParaGear::getModule()
	{
		return _module;
	}

	void GeometryParaGear::setPressureAngle(double angle)
	{
		_pressureAngle = angle;
	}

	double GeometryParaGear::getPressureAngle()
	{
		return _pressureAngle;
	}

	void GeometryParaGear::setAddendumCoefficient(double coeff)
	{
		_addendumCoeff = coeff;
	}

	double GeometryParaGear::getAddendumCoefficient()
	{
		return _addendumCoeff;
	}

	void GeometryParaGear::setDedendumCoefficient(double coeff)
	{
		_dedendumCoeff = coeff;
	}

	double GeometryParaGear::getDedendumCoefficient()
	{
		return _dedendumCoeff;
	}

	void GeometryParaGear::setFilletCoefficient(double coeff)
	{
		_filletCoeff = coeff;
	}

	double GeometryParaGear::getFilletCoefficient()
	{
		return _filletCoeff;
	}

	void GeometryParaGear::setThickness(double t)
	{
		_thickness = t;
	}

	double GeometryParaGear::getThickness()
	{
		return _thickness;
	}

	void GeometryParaGear::setThickness2(double t)
	{
		_thickness2 = t;
	}

	double GeometryParaGear::getThickness2()
	{
		return _thickness2;
	}

	void GeometryParaGear::setExternalGear(bool external)
	{
		_externalGear = external;
	}

	bool GeometryParaGear::isExternalGear()
	{
		return _externalGear;
	}

	void GeometryParaGear::setTipReliefAmount(double amount)
	{
		_tipReliefAmount = amount;
	}

	double GeometryParaGear::getTipReliefAmount()
	{
		return _tipReliefAmount;
	}

	void GeometryParaGear::setTipReliefLength(double length)
	{
		_tipReliefLength = length;
	}

	double GeometryParaGear::getTipReliefLength()
	{
		return _tipReliefLength;
	}

	void GeometryParaGear::setTipReliefAmount2(double amount)
	{
		_tipReliefAmount2 = amount;
	}

	double GeometryParaGear::getTipReliefAmount2()
	{
		return _tipReliefAmount2;
	}

	void GeometryParaGear::setTipReliefLength2(double length)
	{
		_tipReliefLength2 = length;
	}

	double GeometryParaGear::getTipReliefLength2()
	{
		return _tipReliefLength2;
	}

	void GeometryParaGear::setProfileShiftCoefficient1(double x1)
	{
		_x1 = x1;
	}

	double GeometryParaGear::getProfileShiftCoefficient1()
	{
		return _x1;
	}

	void GeometryParaGear::setProfileShiftCoefficient2(double x2)
	{
		_x2 = x2;
	}

	double GeometryParaGear::getProfileShiftCoefficient2()
	{
		return _x2;
	}

	QDomElement &GeometryParaGear::writeToProjectFile(QDomDocument *doc, QDomElement *parent)
	{
		QDomElement element = doc->createElement("Parameter");
		element.setAttribute("Type", this->typeToString());

		QDomElement nameEle = doc->createElement("Name");
		nameEle.appendChild(doc->createTextNode(_name));
		element.appendChild(nameEle);

		QDomElement teethEle = doc->createElement("NumberOfTeeth");
		teethEle.appendChild(doc->createTextNode(QString::number(_numberOfTeeth)));
		element.appendChild(teethEle);

		QDomElement secondTeethEle = doc->createElement("NumberOfSecondTeeth");
		secondTeethEle.appendChild(doc->createTextNode(QString::number(_numberOfSecondTeeth)));
		element.appendChild(secondTeethEle);

		QDomElement moduleEle = doc->createElement("Module");
		moduleEle.appendChild(doc->createTextNode(QString::number(_module)));
		element.appendChild(moduleEle);

		QDomElement angleEle = doc->createElement("PressureAngle");
		angleEle.appendChild(doc->createTextNode(QString::number(_pressureAngle)));
		element.appendChild(angleEle);

		QDomElement addendumEle = doc->createElement("AddendumCoefficient");
		addendumEle.appendChild(doc->createTextNode(QString::number(_addendumCoeff)));
		element.appendChild(addendumEle);

		QDomElement dedendumEle = doc->createElement("DedendumCoefficient");
		dedendumEle.appendChild(doc->createTextNode(QString::number(_dedendumCoeff)));
		element.appendChild(dedendumEle);

		QDomElement filletEle = doc->createElement("FilletCoefficient");
		filletEle.appendChild(doc->createTextNode(QString::number(_filletCoeff)));
		element.appendChild(filletEle);

		QDomElement thicknessEle = doc->createElement("Thickness");
		thicknessEle.appendChild(doc->createTextNode(QString::number(_thickness)));
		element.appendChild(thicknessEle);

		QDomElement thickness2Ele = doc->createElement("Thickness2");
		thickness2Ele.appendChild(doc->createTextNode(QString::number(_thickness2)));
		element.appendChild(thickness2Ele);

		QDomElement externalEle = doc->createElement("ExternalGear");
		externalEle.appendChild(doc->createTextNode(_externalGear ? "true" : "false"));
		element.appendChild(externalEle);

		QDomElement tipReliefAmountEle = doc->createElement("TipReliefAmount");
		tipReliefAmountEle.appendChild(doc->createTextNode(QString::number(_tipReliefAmount)));
		element.appendChild(tipReliefAmountEle);

		QDomElement tipReliefLengthEle = doc->createElement("TipReliefLength");
		tipReliefLengthEle.appendChild(doc->createTextNode(QString::number(_tipReliefLength)));
		element.appendChild(tipReliefLengthEle);

		QDomElement tipReliefAmount2Ele = doc->createElement("TipReliefAmount2");
		tipReliefAmount2Ele.appendChild(doc->createTextNode(QString::number(_tipReliefAmount2)));
		element.appendChild(tipReliefAmount2Ele);

		QDomElement tipReliefLength2Ele = doc->createElement("TipReliefLength2");
		tipReliefLength2Ele.appendChild(doc->createTextNode(QString::number(_tipReliefLength2)));
		element.appendChild(tipReliefLength2Ele);

		QDomElement x1Ele = doc->createElement("ProfileShiftCoefficient1");
		x1Ele.appendChild(doc->createTextNode(QString::number(_x1)));
		element.appendChild(x1Ele);

		QDomElement x2Ele = doc->createElement("ProfileShiftCoefficient2");
		x2Ele.appendChild(doc->createTextNode(QString::number(_x2)));
		element.appendChild(x2Ele);

		parent->appendChild(element);
		return element;
	}

	void GeometryParaGear::readDataFromProjectFile(QDomElement *e)
	{
		QDomNodeList nodeList = e->childNodes();
		for (int i = 0; i < nodeList.size(); ++i)
		{
			QDomElement ele = nodeList.at(i).toElement();
			const QString name = ele.nodeName();
			const QString value = ele.text();

			if (name == "Name")
				_name = value;
			else if (name == "NumberOfTeeth")
				_numberOfTeeth = value.toInt();
			else if (name == "NumberOfSecondTeeth")
				_numberOfSecondTeeth = value.toInt();
			else if (name == "Module")
				_module = value.toDouble();
			else if (name == "PressureAngle")
				_pressureAngle = value.toDouble();
			else if (name == "AddendumCoefficient")
				_addendumCoeff = value.toDouble();
			else if (name == "DedendumCoefficient")
				_dedendumCoeff = value.toDouble();
			else if (name == "FilletCoefficient")
				_filletCoeff = value.toDouble();
			else if (name == "Thickness")
				_thickness = value.toDouble();
			else if (name == "Thickness2")
				_thickness2 = value.toDouble();
			else if (name == "ExternalGear")
				_externalGear = (value == "true");
			else if (name == "TipReliefAmount")
				_tipReliefAmount = value.toDouble();
			else if (name == "TipReliefLength")
				_tipReliefLength = value.toDouble();
			else if (name == "TipReliefAmount2")
				_tipReliefAmount2 = value.toDouble();
			else if (name == "TipReliefLength2")
				_tipReliefLength2 = value.toDouble();
			else if (name == "ProfileShiftCoefficient1")
				_x1 = value.toDouble();
			else if (name == "ProfileShiftCoefficient2")
				_x2 = value.toDouble();
		}
	}
}
