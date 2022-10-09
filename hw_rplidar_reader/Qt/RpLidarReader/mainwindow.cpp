#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QSerialPortInfo>
#include <iostream>
#include <QFileDialog>
#include <math.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define VECTOR_SIZE 360*2
#define YAW_OFFSET 0

void empty_double_dataset(double * dataset, const int &size){
    for(int i=0; i<size; i++){
        dataset[i]=0.0;
    }
}

void insertInDataset(double * dataset, const double &degree, const double &val){
    int index = VECTOR_SIZE-(int)(degree*2.0);//*10.0);
    dataset[index] = val;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    drv = nullptr;

    pLidar = new PainterLidar(this);
    pLidar->show();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showEvent(QShowEvent *event)
{
    refreshPortList();
    ui->label_connetion->setText("Device not connected");
    ui->labelFilePath->setText("");
    ui->pushButtonSave->setEnabled(false);
    ui->label_samples->setText("Number of samples: N.A.");

    ui->pushButtonZoomIn->setEnabled(false);
    ui->pushButtonZoomOut->setEnabled(false);
    ui->pushButtonChange->setEnabled(false);
    ui->pushButtonStop->setEnabled(false);
    ui->pushButtonStopSaving->setEnabled(false);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if(drv!=nullptr){
        on_pushButtonStop_clicked();
        delete drv;
    }
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    ui->verticalWidget->setGeometry(0,0,this->width()/4,this->height());
    double spacing = this->width()/40;
    double zommHeight = this->height()/20;
    pLidar->setGeometry(ui->verticalWidget->width()+spacing,spacing,this->width()-ui->verticalWidget->width()-2*spacing,this->height()-3*spacing-zommHeight);
    ui->horizontalWidget->setGeometry(pLidar->x(),this->height()-spacing-zommHeight,pLidar->width(),zommHeight);
}

void MainWindow::refreshPortList()
{
    QList<QSerialPortInfo> list;
    list = QSerialPortInfo::availablePorts();
    ui->comboBox_portName->clear();
    for (const QSerialPortInfo &portInfo : list) {
        ui->comboBox_portName->addItem(portInfo.portName(),portInfo.portName());
    }
}


void MainWindow::on_pushButtonRefresh_clicked()
{
    refreshPortList();
}

void MainWindow::on_pushButtonConnect_clicked()
{
    QString portName = "/dev/";
    portName+=ui->comboBox_portName->itemData(ui->comboBox_portName->currentIndex()).toString();
    if(drv==nullptr){
        drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    }

    rplidar_response_device_info_t devinfo;
    const char * opt_com_path = "/dev/";
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;
    bool connectSuccess = false;
    // make connection...
    size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));

    std::cout << portName.toStdString() << std::endl;
    if(IS_OK(drv->connect(portName.toStdString().c_str(), 256000)))
    {
        op_result = drv->getDeviceInfo(devinfo);

        if (IS_OK(op_result))
        {
            connectSuccess = true;
        }
        else
        {
            delete drv;
            drv = nullptr;
        }
    }


    if (!connectSuccess) {
        ui->label_connetion->setText("Impossible to establish a connection");        
        ui->pushButtonZoomIn->setEnabled(false);
        ui->pushButtonZoomOut->setEnabled(false);
        ui->pushButtonChange->setEnabled(false);
        ui->pushButtonStop->setEnabled(false);
        ui->comboBox_portName->setEnabled(true);
        ui->pushButtonConnect->setEnabled(true);
        ui->pushButtonRefresh->setEnabled(true);
        return;
    }else{
        ui->label_connetion->setText("Driver connected");
        std::unique_lock<std::mutex> lck(threadMutex);
        stopThread = false;
        ui->pushButtonZoomIn->setEnabled(true);
        ui->pushButtonZoomOut->setEnabled(true);
        ui->pushButtonChange->setEnabled(true);
        ui->pushButtonStop->setEnabled(true);
        ui->comboBox_portName->setEnabled(false);
        ui->pushButtonConnect->setEnabled(false);
        ui->pushButtonRefresh->setEnabled(false);
    }

    drv->startMotor();
    // start scan...
    drv->startScan(0,1);

    if (IS_OK(op_result)) {
        lidarThread = std::thread([&](){
            double scanned_values[VECTOR_SIZE];
            while (1) {

                std::vector<std::pair<double,double>> frontLidar;
                empty_double_dataset(scanned_values,VECTOR_SIZE);
                rplidar_response_measurement_node_hq_t nodes[8192];
                size_t   count = _countof(nodes);

                u_result op_result = drv->grabScanDataHq(nodes, count);

                std::unique_lock<std::mutex> lock(fileSavingMutex);
                if(currentFile.is_open()){
                    currentFile << "new dataset collected" << "\n";
                }

                for (int pos = 0; pos < (int)count ; ++pos) {
                    float angle_in_degrees = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                    float distance_in_meters = nodes[pos].dist_mm_q2 / 1000.f / (1 << 2);
                    double correct_angle_degrees = (angle_in_degrees-90.0);
                    if(correct_angle_degrees < 0.0) correct_angle_degrees = correct_angle_degrees + 360.0;

                    insertInDataset(scanned_values, correct_angle_degrees, distance_in_meters);

                    //std::cout << correct_angle_degrees << " | " << distance_in_meters << std::endl;
                    if(distance_in_meters<0.02) continue;
                    //Compute lidar cartesian point
                    std::pair<double,double> cartesianPair;
                    cartesianPair.first = distance_in_meters*cos(correct_angle_degrees*M_PI/180.0);
                    cartesianPair.second = distance_in_meters*sin(correct_angle_degrees*M_PI/180.0);
                    frontLidar.push_back(cartesianPair);

                    if(currentFile.is_open()){
                        currentFile << std::to_string(correct_angle_degrees) << " | " << std::to_string(distance_in_meters) << "\n";
                    }
                }

                lock.unlock();

                pLidar->modifyLidarPoints(frontLidar);
                ui->label_samples->setText("Number of samples: " + QString::number(frontLidar.size()));

                std::unique_lock<std::mutex> lck(threadMutex);
                if(stopThread){
                    lck.unlock();
                    break;
                }
            }
        });
    }
}

void MainWindow::on_pushButtonStop_clicked()
{
    ui->pushButtonZoomIn->setEnabled(false);
    ui->pushButtonZoomOut->setEnabled(false);
    ui->pushButtonChange->setEnabled(false);
    ui->pushButtonStop->setEnabled(false);
    ui->comboBox_portName->setEnabled(true);
    ui->pushButtonConnect->setEnabled(true);
    ui->pushButtonRefresh->setEnabled(true);
    ui->label_connetion->setText("Driver disconnected");

    std::unique_lock<std::mutex> lck(threadMutex);
    stopThread = true;
    lck.unlock();
    lidarThread.join();
    drv->stop();
    drv->stopMotor();
    delete drv;
    drv = nullptr;

    std::unique_lock<std::mutex> lock(fileSavingMutex);
    if(currentFile.is_open()){
        currentFile.close();
    }
}


void MainWindow::on_pushButtonChange_clicked()
{
    QDir dir = QDir::current();
    if(ui->labelFilePath->text()!=""){
        dir = QFileInfo(ui->labelFilePath->text()).absoluteDir();
    }

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                               dir.absolutePath(),
                               tr("Text files (*.txt)"));


    if(fileName!=""){
        if(fileName.split(".").last()!="txt"){
            fileName.append(".txt");
        }
        ui->labelFilePath->setText(fileName);
        ui->pushButtonSave->setEnabled(true);
    }
}

void MainWindow::on_pushButtonZoomIn_clicked()
{
    pLidar->changeScale(true);
}

void MainWindow::on_pushButtonZoomOut_clicked()
{
    pLidar->changeScale(false);
}

void MainWindow::on_pushButtonSave_clicked()
{
    std::unique_lock<std::mutex> lock(fileSavingMutex);
    currentFile.open(ui->labelFilePath->text().toStdString());

    ui->pushButtonStopSaving->setEnabled(true);
    ui->pushButtonSave->setEnabled(false);
    ui->pushButtonChange->setEnabled(false);
    ui->pushButtonStop->setEnabled(false);

}

void MainWindow::on_pushButtonStopSaving_clicked()
{
    std::unique_lock<std::mutex> lock(fileSavingMutex);
    if(currentFile.is_open()){
        currentFile.close();
    }

    ui->pushButtonStopSaving->setEnabled(false);
    ui->pushButtonSave->setEnabled(true);
    ui->pushButtonChange->setEnabled(true);
    ui->pushButtonStop->setEnabled(true);
}
