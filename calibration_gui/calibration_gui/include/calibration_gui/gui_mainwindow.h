#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTreeWidget>
#include <QStyledItemDelegate>
#include <QItemDelegate>
#include <QPainter>
#include <QSignalMapper>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    void AddRoot (int rowNUmber);
    void AddChildIP (QTreeWidgetItem *parent, int rowNUmber);
    //void AddChildTopic (QTreeWidgetItem *parent);

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_bt_add_clicked();

    void on_bt_remove_clicked();

    //void toolButton_clicked(int rowNumber);

private:
    Ui::MainWindow *ui;
};


#endif // MAINWINDOW_H
