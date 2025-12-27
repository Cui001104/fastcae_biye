#ifndef DIALOGCREATEGEAR_H_
#define DIALOGCREATEGEAR_H_

#include "geometryWidgetsAPI.h"
#include "geoDialogBase.h"

namespace Ui {
	class CreateGear;
}

namespace GeometryWidget {
	/**
	 * @brief 创建渐开线齿轮对话框
	 */
	class GEOMETRYWIDGETSAPI CreateGearDialog : public GeoDialogBase {
		Q_OBJECT
	public:
		CreateGearDialog(GUI::MainWindow* m, MainWidget::PreWindow* p);
		CreateGearDialog(GUI::MainWindow* m, MainWidget::PreWindow* p, Geometry::GeometrySet* set);
		~CreateGearDialog();

	private:
		void init();
		void reject() override;
		void accept() override;

	private slots:
		void onGearTypeChanged(int index);

	private:
		Ui::CreateGear* _ui{};
	};
} // namespace GeometryWidget

#endif
