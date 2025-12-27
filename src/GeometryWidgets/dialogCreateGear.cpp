#include "dialogCreateGear.h"
#include "ui_dialogCreateGear.h"
#include "Geometry/geometrySet.h"
#include "Geometry/geometryData.h"
#include "Geometry/geometryParaGear.h"
#include "Geometry/geometryModelParaBase.h"
#include "GeometryCommand/GeoCommandCreateGear.h"
#include "GeometryCommand/GeoCommandList.h"
#include "PythonModule/PyAgent.h"
#include <QMessageBox>

namespace GeometryWidget {
	CreateGearDialog::CreateGearDialog(GUI::MainWindow* m, MainWidget::PreWindow* p)
		: GeoDialogBase(m, p)
	{
		_ui = new Ui::CreateGear;
		_ui->setupUi(this);
		init();
	}

	CreateGearDialog::CreateGearDialog(GUI::MainWindow* m, MainWidget::PreWindow* p,
									   Geometry::GeometrySet* set)
		: GeoDialogBase(m, p)
	{
		_ui = new Ui::CreateGear;
		_ui->setupUi(this);
		_isEdit	 = true;
		_editSet = set;
		this->setWindowTitle(tr("Edit Gear"));
		init();
	}

	CreateGearDialog::~CreateGearDialog()
	{
		if(_ui != nullptr)
			delete _ui;
	}

	void CreateGearDialog::init()
	{
		this->translateButtonBox(_ui->buttonBox);
		connect(_ui->comboBoxGearType, SIGNAL(currentIndexChanged(int)), this,
				SLOT(onGearTypeChanged(int)));

		if(!_isEdit) {
			int id = Geometry::GeometrySet::getMaxID() + 1;
			_ui->nameLineEdit->setPlaceholderText(QString("Gear_%1").arg(id));
		} else {
			if(_editSet == nullptr)
				return;
			_ui->nameLineEdit->setText(_editSet->getName());
			_ui->nameLineEdit->setEnabled(false);

			Geometry::GeometryModelParaBase* pb = _editSet->getParameter();
			Geometry::GeometryParaGear*		 p	= dynamic_cast<Geometry::GeometryParaGear*>(pb);
			if(p == nullptr)
				return;

			_ui->spinBoxTeeth->setValue(p->getNumberOfTeeth());
			_ui->doubleSpinBoxModule->setValue(p->getModule());
			_ui->doubleSpinBoxPressureAngle->setValue(p->getPressureAngle());
			_ui->doubleSpinBoxAddendum->setValue(p->getAddendumCoefficient());
			_ui->doubleSpinBoxDedendum->setValue(p->getDedendumCoefficient());
			_ui->doubleSpinBoxFillet->setValue(p->getFilletCoefficient());
			_ui->doubleSpinBoxThickness->setValue(p->getThickness());
			_ui->comboBoxGearType->setCurrentIndex(p->isExternalGear() ? 0 : 1);
		}
	}

	void CreateGearDialog::reject()
	{
		QDialog::reject();
		this->close();
	}

	void CreateGearDialog::accept()
	{
		int	   numberOfTeeth = _ui->spinBoxTeeth->value();
		double module		 = _ui->doubleSpinBoxModule->value();
		double pressureAngle = _ui->doubleSpinBoxPressureAngle->value();
		double addendumCoeff = _ui->doubleSpinBoxAddendum->value();
		double dedendumCoeff = _ui->doubleSpinBoxDedendum->value();
		double filletCoeff	 = _ui->doubleSpinBoxFillet->value();
		double thickness	 = _ui->doubleSpinBoxThickness->value();
		bool   externalGear	 = (_ui->comboBoxGearType->currentIndex() == 0);

		// 参数验证
		if(numberOfTeeth < 3) {
			QMessageBox::warning(this, tr("Warning"), tr("Number of teeth must be at least 3!"));
			return;
		}
		if(module <= 0) {
			QMessageBox::warning(this, tr("Warning"), tr("Module must be positive!"));
			return;
		}
		if(thickness <= 0) {
			QMessageBox::warning(this, tr("Warning"), tr("Thickness must be positive!"));
			return;
		}

		QString name = _ui->nameLineEdit->text();
		if(name.isEmpty())
			name = _ui->nameLineEdit->placeholderText();

		Command::GeoCommandCreateGear* command =
			new Command::GeoCommandCreateGear(_mainWindow, _preWindow);
		command->setNumberOfTeeth(numberOfTeeth);
		command->setModule(module);
		command->setPressureAngle(pressureAngle);
		command->setAddendumCoefficient(addendumCoeff);
		command->setDedendumCoefficient(dedendumCoeff);
		command->setFilletCoefficient(filletCoeff);
		command->setThickness(thickness);
		command->setExternalGear(externalGear);
		command->setName(name);

		if(_isEdit)
			command->setEditData(_editSet);

		bool success = Command::GeoComandList::getInstance()->executeCommand(command);
		if(!success) {
			QMessageBox::warning(this, tr("Warning"), tr("Create gear failed!"));
			return;
		}

		QDialog::accept();
		this->close();
	}

	void CreateGearDialog::onGearTypeChanged(int index)
	{
		// 当切换齿轮类型时，更新齿顶高系数默认值
		if(index == 0) // 外齿轮
		{
			_ui->doubleSpinBoxAddendum->setValue(1.0);
		} else // 内齿轮
		{
			_ui->doubleSpinBoxAddendum->setValue(0.6);
		}
	}
} // namespace GeometryWidget
