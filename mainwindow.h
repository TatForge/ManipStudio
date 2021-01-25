#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "my_math.h"
#include "robo.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void PlotCoordinates(double value_x, double value_y, double value_z, double yaw, double pitch, double roll, double t);
    void PlotSpeeds(Robo::Angles speeds, double t);
    void PrintResult(double x, double y, double z, double yaw, double pitch, double roll, double t);
private slots:
//    QTimer timer;
private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
