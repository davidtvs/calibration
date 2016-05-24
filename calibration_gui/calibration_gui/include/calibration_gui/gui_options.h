#ifndef OPTIONS_H
#define OPTIONS_H

#include <QDialog>

namespace Ui {
class Options;
}

class Options : public QDialog
{
    Q_OBJECT

public:
    explicit Options(QWidget *parent = 0);
    ~Options();

    QString getBallDiameter() { return ball_diameter; }

    QString getNumCalibPoints(){ return num_calibration_points; }

    QString getMinDistance(){ return min_distance; }

    QString getMaxDisplacement(){ return max_displacement; }

    bool getAutoAcquisition(){ return auto_acquisition; }

private slots:
    void on_buttonBox_accepted();

private:
    Ui::Options *ui;

    QString ball_diameter;
    QString num_calibration_points;
    QString min_distance;
    QString max_displacement;
    bool auto_acquisition;
};

#endif // OPTIONS_H
