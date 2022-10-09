#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <getopt.h>
#include <rplidar.h>
#include <rptypes.h>
#include <thread>
#include <mutex>
#include <painterlidar.h>
#include <fstream>
#include <iostream>

using namespace rp::standalone::rplidar;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButtonRefresh_clicked();

    void on_pushButtonConnect_clicked();

    void on_pushButtonStop_clicked();

    void on_pushButtonChange_clicked();

    void on_pushButtonZoomIn_clicked();

    void on_pushButtonZoomOut_clicked();

    void on_pushButtonSave_clicked();

    void on_pushButtonStopSaving_clicked();

private:
    Ui::MainWindow *ui;
    RPlidarDriver * drv;
    std::thread lidarThread;
    PainterLidar* pLidar;
    std::ofstream currentFile;
    std::mutex fileSavingMutex;

    void showEvent(QShowEvent* event);
    void closeEvent(QCloseEvent* event);
    void resizeEvent(QResizeEvent* event);
    void refreshPortList();

    bool stopThread = false;
    std::mutex threadMutex;

};
#endif // MAINWINDOW_H
