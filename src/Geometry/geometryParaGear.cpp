/**
 * @file geometryParaGear.cpp
 * @brief 渐开线齿轮参数类源文件
 * @author FastCAE研发小组(fastcae@diso.cn)
 * @version 2.5.0
 * @date 2024-12-27
 * @copyright Copyright (c) Since 2020 青岛数智船海科技有限公司  All rights reserved.
 */
#include "geometryParaGear.h"
#include <QDomElement>
#include <QDomDocument>

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

	void GeometryParaGear::setExternalGear(bool external)
	{
		_externalGear = external;
	}

	bool GeometryParaGear::isExternalGear()
	{
		return _externalGear;
	}

	QDomElement& GeometryParaGear::writeToProjectFile(QDomDocument* doc, QDomElement* parent)
	{
		QDomElement element = doc->createElement("Parameter");
		element.setAttribute("Type", this->typeToString());

		QDomElement nameEle = doc->createElement("Name");
		QDomText nameText = doc->createTextNode(_name);
		nameEle.appendChild(nameText);
		element.appendChild(nameEle);

		QDomElement teethEle = doc->createElement("NumberOfTeeth");
		QDomText teethText = doc->createTextNode(QString::number(_numberOfTeeth));
		teethEle.appendChild(teethText);
		element.appendChild(teethEle);

		QDomElement moduleEle = doc->createElement("Module");
		QDomText moduleText = doc->createTextNode(QString::number(_module));
		moduleEle.appendChild(moduleText);
		element.appendChild(moduleEle);

		QDomElement angleEle = doc->createElement("PressureAngle");
		QDomText angleText = doc->createTextNode(QString::number(_pressureAngle));
		angleEle.appendChild(angleText);
		element.appendChild(angleEle);

		QDomElement addendumEle = doc->createElement("AddendumCoefficient");
		QDomText addendumText = doc->createTextNode(QString::number(_addendumCoeff));
		addendumEle.appendChild(addendumText);
		element.appendChild(addendumEle);

		QDomElement dedendumEle = doc->createElement("DedendumCoefficient");
		QDomText dedendumText = doc->createTextNode(QString::number(_dedendumCoeff));
		dedendumEle.appendChild(dedendumText);
		element.appendChild(dedendumEle);

		QDomElement filletEle = doc->createElement("FilletCoefficient");
		QDomText filletText = doc->createTextNode(QString::number(_filletCoeff));
		filletEle.appendChild(filletText);
		element.appendChild(filletEle);

		QDomElement thicknessEle = doc->createElement("Thickness");
		QDomText thicknessText = doc->createTextNode(QString::number(_thickness));
		thicknessEle.appendChild(thicknessText);
		element.appendChild(thicknessEle);

		QDomElement externalEle = doc->createElement("ExternalGear");
		QDomText externalText = doc->createTextNode(_externalGear ? "true" : "false");
		externalEle.appendChild(externalText);
		element.appendChild(externalEle);

		parent->appendChild(element);
		return element;
	}

	void GeometryParaGear::readDataFromProjectFile(QDomElement* e)
	{
		QDomNodeList nodeList = e->childNodes();
		for (int i = 0; i < nodeList.size(); ++i)
		{
			QDomElement ele = nodeList.at(i).toElement();
			QString name = ele.nodeName();
			QString value = ele.text();

			if (name == "Name")
				_name = value;
			else if (name == "NumberOfTeeth")
				_numberOfTeeth = value.toInt();
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
			else if (name == "ExternalGear")
				_externalGear = (value == "true");
		}
	}
}

