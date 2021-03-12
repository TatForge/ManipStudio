#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "my_math.h"
#include "robo.h"
//#include "emulator.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void PlotCoordinates(double value_x, double value_y, double value_z, double yaw, double pitch, double roll, double dt, bool is_last = false);
    void PlotSpeeds(rm::Angles speeds, bool is_last = false);
    void PrintResult(double x, double y, double z, double yaw, double pitch, double roll);
private slots:
//    QTimer timer;
    void on_pushButton_clicked();

private:
    double m_time;
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
