#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QPen pen;
    pen.setColor(QColor("blue"));
    pen.setWidth(3);

    ui->plot_x->addGraph();
    ui->plot_y->addGraph();
    ui->plot_z->addGraph();
    ui->plot_yaw->addGraph();
    ui->plot_pit->addGraph();
    ui->plot_rol->addGraph();
    ui->plot_x->graph(0)->setPen(pen);
    ui->plot_y->graph(0)->setPen(pen);
    ui->plot_z->graph(0)->setPen(pen);
    ui->plot_yaw->graph(0)->setPen(pen);
    ui->plot_pit->graph(0)->setPen(pen);
    ui->plot_rol->graph(0)->setPen(pen);
    m_time = 0;

    for (int i = 0; i < 6; i ++)
    {
        ui->plot_coord->addGraph();
        ui->plot_speed->addGraph();
        ui->plot_accel->addGraph();
        pen.setColor(QColor(i * 35,i * 1, i));
        ui->plot_coord->graph(i)->setPen(pen);
        ui->plot_speed->graph(i)->setPen(pen);
        ui->plot_accel->graph(i)->setPen(pen);
    }
    ui->plot_coord->yAxis->setRange(-1.5, 1.5);
    ui->plot_coord->yAxis->setLabel("coordinate");
    ui->plot_speed->yAxis->setRange(-1.5, 1.5);
    ui->plot_speed->yAxis->setLabel("speed");
    ui->plot_accel->yAxis->setRange(-1.5, 1.5);
    ui->plot_accel->yAxis->setLabel("acceleration");
//    ui->plot_x->addGraph();
    ui->plot_x->yAxis->setRange(-1, 1);
    ui->plot_x->yAxis->setLabel("tcp_x");
//    ui->plot_y->addGraph();
    ui->plot_y->yAxis->setRange(-1, 1);
    ui->plot_y->yAxis->setLabel("tcp_y");
//    ui->plot_z->addGraph();
    ui->plot_z->yAxis->setRange(-1, 1);
    ui->plot_z->yAxis->setLabel("tcp_z");

    ui->plot_yaw->yAxis->setRange(-1.6, 1.6);
    ui->plot_yaw->yAxis->setLabel("tcp_vx");
    ui->plot_pit->yAxis->setRange(-1.6, 1.6);
    ui->plot_pit->yAxis->setLabel("tcp_vy");
    ui->plot_rol->yAxis->setRange(-1.6, 1.6);
    ui->plot_rol->yAxis->setLabel("tcp_vz");

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::PlotCoordinates(double value_x, double value_y, double value_z, double yaw, double pitch, double roll, double dt, bool is_last)
{
    double t = m_time;
    ui->plot_x->graph(0)->addData(t/1000.0, value_x);
    ui->plot_y->graph(0)->addData(t/1000.0, value_y);
    ui->plot_z->graph(0)->addData(t/1000.0, value_z);
    ui->plot_yaw->graph(0)->addData(t/1000.0, yaw);
    ui->plot_pit->graph(0)->addData(t/1000.0, pitch);
    ui->plot_rol->graph(0)->addData(t/1000.0, roll);
//    ui->plot_x->graph(1)->setPen(QPen(QColor("red")));
//    ui->plot_y->graph(1)->setPen(QPen(QColor("red")));
//    ui->plot_x->graph(1)->setPen(QPen(QColor("red")));
//    ui->plot_x->graph(1)->addData(t, value_vx);
//    ui->plot_y->graph(1)->addData(t, value_vy);
//    ui->plot_z->graph(1)->addData(t, value_vz);
//    ui->plot_x->xAxis->setRange(0, t + 1);
//    ui->plot_y->xAxis->setRange(0, t + 1);
//    ui->plot_z->xAxis->setRange(0, t + 1);
//    ui->plot_yaw->xAxis->setRange(0, t + 1);
//    ui->plot_pit->xAxis->setRange(0, t + 1);
//    ui->plot_rol->xAxis->setRange(0, t + 1);
    if (is_last)
    {
        ui->plot_x->yAxis->rescale();
        ui->plot_y->yAxis->rescale();
        ui->plot_z->yAxis->rescale();
//        ui->plot_yaw->yAxis->setRange(QCPRange(0.01, 0.01));
//        ui->plot_pit->yAxis->setRange(QCPRange(0.01, 0.01));
//        ui->plot_rol->yAxis->setRange(QCPRange(0.01, 0.01));
        ui->plot_yaw->xAxis->setRange(QCPRange(0, t/1000.0 + 0.2));
        ui->plot_pit->xAxis->setRange(QCPRange(0, t/1000.0 + 0.2));
        ui->plot_rol->xAxis->setRange(QCPRange(0, t/1000.0 + 0.2));
        ui->plot_yaw->yAxis->rescale();
        ui->plot_pit->yAxis->rescale();
        ui->plot_rol->yAxis->rescale();
        ui->plot_x->yAxis->rescale();
        ui->plot_y->yAxis->rescale();
        ui->plot_z->yAxis->rescale();
        ui->plot_x->xAxis->rescale();
        ui->plot_y->xAxis->rescale();
        ui->plot_z->xAxis->rescale();
        ui->plot_x->replot();
        ui->plot_y->replot();
        ui->plot_z->replot();
        ui->plot_yaw->replot();
        ui->plot_pit->replot();
        ui->plot_rol->replot();
    }
    m_time += dt;
}

void MainWindow::PlotSpeeds(rm::Angles speeds, bool is_last)
{
    double t = m_time;
//    if (speeds.q_p[1] > ui->plot_rol->yAxis->range().maxRange)
//        ui->plot_rol->yAxis->setRange(-1.5, speeds.q_p[1] + 1);
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        ui->plot_coord->graph(i)->addData(t/1000.0, speeds.q[i]);
        ui->plot_speed->graph(i)->addData(t/1000.0, speeds.q_p[i]);
        ui->plot_accel->graph(i)->addData(t/1000.0, speeds.q_pp[i]);
    }
    if (is_last)
    {
        ui->plot_coord->xAxis->setRange(QCPRange(0, t/1000.0 + 0.2));
        ui->plot_speed->xAxis->setRange(QCPRange(0, t/1000.0 + 0.2));
        ui->plot_accel->xAxis->setRange(QCPRange(0, t/1000.0 + 0.2));

        ui->plot_coord->yAxis->rescale();
        ui->plot_speed->yAxis->rescale();
        ui->plot_accel->yAxis->rescale();
        ui->plot_coord->replot();
        ui->plot_speed->replot();
        ui->plot_accel->replot();
    }
}

void MainWindow::PrintResult(double x, double y, double z, double yaw, double pitch, double roll)
{
    ui->label_x->setText(QString("x = ") + QString::number(x));
    ui->label_y->setText(QString("y = ") + QString::number(y));
    ui->label_z->setText(QString("z = ") + QString::number(z));
    ui->label_roll->setText(QString("roll = ")   + QString::number(ToDeg(roll)));
    ui->label_pitch->setText(QString("pitch = ") + QString::number(ToDeg(pitch)));
    ui->label_yaw->setText(QString("yaw = ")     + QString::number(ToDeg(yaw)));
}


void MainWindow::on_pushButton_clicked()
{
    Robo manipulator;
    rm::Angles start, stop;
//    Polynom pol;
    stop.q[2] = -50;
    manipulator.LoadManip();
//    manipulator.CalcPolynomLine(&start, &stop, 3, pol);
//    Emulator emul(manipulator);
//    emul.MoveManip(pol, 3);
}
