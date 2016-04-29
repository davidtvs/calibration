#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTreeWidget>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    void AddRoot ();
    void AddChildRoslaunch (QTreeWidgetItem *parent);
    void AddChildTopic (QTreeWidgetItem *parent);

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_bt_add_clicked();

    void on_bt_remove_clicked();

    void on_toolButton_clicked();

private:
    Ui::MainWindow *ui;
};



#endif // MAINWINDOW_H
