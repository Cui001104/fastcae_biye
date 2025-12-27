/**
 * @file geometryParaGear.h
 * @brief 渐开线齿轮参数类头文件
 * @author FastCAE研发小组(fastcae@diso.cn)
 * @version 2.5.0
 * @date 2024-12-27
 * @copyright Copyright (c) Since 2020 青岛数智船海科技有限公司  All rights reserved.
 *
 * ============================================================================
 * Program:   FastCAE
 *
 * Copyright (c) Since 2020 青岛数智船海科技有限公司  All rights reserved.
 * See License or http://www.fastcae.com/ for details.
 *
 * BSD 3-Clause License
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 * ==================================================================================
 */
#ifndef _GEOMETRYPARAGEAR_H_
#define _GEOMETRYPARAGEAR_H_

#include "geometryModelParaBase.h"
#include <QString>

namespace Geometry
{
	/**
	 * @brief 渐开线齿轮参数类
	 * @since 2.5.0
	 */
	class GEOMETRYAPI GeometryParaGear : public GeometryModelParaBase
	{
	public:
		GeometryParaGear();
		~GeometryParaGear() = default;

		void setName(QString name);
		QString getName();

		/// 设置/获取齿数
		void setNumberOfTeeth(int n);
		int getNumberOfTeeth();

		/// 设置/获取模数 (mm)
		void setModule(double m);
		double getModule();

		/// 设置/获取压力角 (度)
		void setPressureAngle(double angle);
		double getPressureAngle();

		/// 设置/获取齿顶高系数
		void setAddendumCoefficient(double coeff);
		double getAddendumCoefficient();

		/// 设置/获取齿根高系数
		void setDedendumCoefficient(double coeff);
		double getDedendumCoefficient();

		/// 设置/获取齿根圆角系数
		void setFilletCoefficient(double coeff);
		double getFilletCoefficient();

		/// 设置/获取齿轮厚度 (mm)
		void setThickness(double t);
		double getThickness();

		/// 设置/获取是否为外齿轮
		void setExternalGear(bool external);
		bool isExternalGear();

		/// 数据写入工程文件
		QDomElement& writeToProjectFile(QDomDocument* doc, QDomElement* parent) override;
		/// 从工程文件读入数据
		virtual void readDataFromProjectFile(QDomElement* e) override;

	private:
		QString _name{};
		int _numberOfTeeth{ 26 };       ///< 齿数
		double _module{ 2.5 };          ///< 模数 (mm)
		double _pressureAngle{ 20.0 };  ///< 压力角 (度)
		double _addendumCoeff{ 1.0 };   ///< 齿顶高系数
		double _dedendumCoeff{ 1.25 };  ///< 齿根高系数
		double _filletCoeff{ 0.38 };    ///< 齿根圆角系数
		double _thickness{ 10.0 };      ///< 齿轮厚度 (mm)
		bool _externalGear{ true };     ///< 是否为外齿轮
	};
}

#endif

