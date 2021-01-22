#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "my_math.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void PlotData(double value_x, double value_y, double value_z, double t);
    void PrintResult(double x, double y, double z, double roll, double pitch, double yaw, double t);
private slots:
    void on_button_move_clicked();
//    QTimer timer;
private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
