#include "calibration_gui/gui_options.h"
#include "ui_options.h"

Options::Options(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Options)
{
    ui->setupUi(this);
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
}
