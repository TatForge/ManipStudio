#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
//    ui->plot_x->addGraph();
    ui->plot_x->yAxis->setRange(-1.6, 1.6);
    ui->plot_x->yAxis->setLabel("x");
//    ui->plot_y->addGraph();
    ui->plot_y->yAxis->setRange(-1.6, 1.6);
    ui->plot_y->yAxis->setLabel("y");
//    ui->plot_z->addGraph();
    ui->plot_z->yAxis->setRange(-1.6, 1.6);
    ui->plot_z->yAxis->setLabel("z");

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::PlotData(double value_x, double value_y, double value_z, double t)
{
    ui->plot_x->addGraph();
    ui->plot_y->addGraph();
    ui->plot_z->addGraph();
//    ui->plot_x->addGraph();
//    ui->plot_y->addGraph();
//    ui->plot_z->addGraph();
    ui->plot_x->graph(0)->addData(t, value_x);
    ui->plot_y->graph(0)->addData(t, value_y);
    ui->plot_z->graph(0)->addData(t, value_z);
//    ui->plot_x->graph(1)->setPen(QPen(QColor("red")));
//    ui->plot_y->graph(1)->setPen(QPen(QColor("red")));
//    ui->plot_x->graph(1)->setPen(QPen(QColor("red")));
//    ui->plot_x->graph(1)->addData(t, value_vx);
//    ui->plot_y->graph(1)->addData(t, value_vy);
//    ui->plot_z->graph(1)->addData(t, value_vz);
    ui->plot_x->xAxis->setRange(0, t + 1);
    ui->plot_y->xAxis->setRange(0, t + 1);
    ui->plot_z->xAxis->setRange(0, t + 1);
//    ui->plot_x->yAxis->set
    ui->plot_x->replot();
    ui->plot_y->replot();
    ui->plot_z->replot();

}

void MainWindow::PrintResult(double x, double y, double z, double roll, double pitch, double yaw, double t)
{
    ui->label_x->setText(QString("x = ") + QString::number(x));
    ui->label_y->setText(QString("y = ") + QString::number(y));
    ui->label_z->setText(QString("z = ") + QString::number(z));
    ui->label_roll->setText(QString("roll = ")   + QString::number(ToDeg(roll)));
    ui->label_pitch->setText(QString("pitch = ") + QString::number(ToDeg(pitch)));
    ui->label_yaw->setText(QString("yaw = ")     + QString::number(ToDeg(yaw)));
}


void MainWindow::on_button_move_clicked()
{

}
