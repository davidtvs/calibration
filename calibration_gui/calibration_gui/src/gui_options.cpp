#include "calibration_gui/gui_options.h"
#include "ui_options.h"

Options::Options(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Options)
{
    ui->setupUi(this);

    ui->ledit_ball_diameter->setValidator( new QDoubleValidator(0, 100, 3, this) );
    ui->ledit_min_distance->setValidator( new QDoubleValidator(0, 100, 3, this) );
    ui->ledit_num_calib_points->setValidator( new QIntValidator(0, 100, this) );

    ui->ledit_ball_diameter->setText("0.99");
    ui->ledit_min_distance->setText("0.15");
    ui->ledit_num_calib_points->setText("20");

    ball_diameter = ui->ledit_ball_diameter->text();
    num_calibration_points = ui->ledit_num_calib_points->text();
    min_distance = ui->ledit_min_distance->text();
}

Options::~Options()
{
    delete ui;
}

void Options::on_buttonBox_accepted()
{
    ball_diameter = ui->ledit_ball_diameter->text();
    num_calibration_points = ui->ledit_num_calib_points->text();
    min_distance = ui->ledit_min_distance->text();
    auto_acquisition = ui->rb_auto->isChecked();
}
