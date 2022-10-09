#include "painterlidar.h"

#include <QPainter>
#include <math.h>

PainterLidar::PainterLidar(QWidget *parent) : QWidget(parent)
{
    if(this->height()<this->width()){
        scale = (2*range)/(this->height()); //m/pix
    }else{
        scale = (2*range)/(this->width()); //m/pix
    }

    timerRefresh = new QTimer();
    connect(timerRefresh,&QTimer::timeout,[this](){
        this->update();
    });

    timerRefresh->start(50);
}

void PainterLidar::modifyLidarPoints(std::vector<std::pair<double, double>> scans)
{
    std::unique_lock<std::mutex> lock(plotMutex);
    lidarPoints.clear();
    QPointF lidarPoint;
    for (std::vector<std::pair<double,double>>::iterator it = scans.begin() ; it != scans.end(); ++it){
        lidarPoint = fromCoordsToPix((*it).first,(*it).second);
        lidarPoints.push_back(lidarPoint);
    }
}

void PainterLidar::changeScale(bool increment)
{
    std::unique_lock<std::mutex> lock(plotMutex);
    if(increment){
        range = range-1;
        if(range<1) range = 1;
    }else {
        range = range+1;
        if(range>30) range = 30;
    }
    if(this->height()<this->width()){
        scale = (2*range)/(this->height()); //m/pix
    }else{
        scale = (2*range)/(this->width()); //m/pix
    }
}

void PainterLidar::resizeEvent(QResizeEvent* event){
    //I want to see 6 meters ahead and 6 meters back the device
    if(this->height()<this->width()){
        scale = (2*range)/(this->height()); //m/pix
    }else{
        scale = (2*range)/(this->width()); //m/pix
    }
}

QPointF PainterLidar::fromCoordsToPix(const double &x, const double &y){
    QPoint out;
    double xp = y;
    double yp = -x;
    //I've to rotate the point if the origin is translated
    double xy = sqrt(pow(xp,2)+pow(yp,2));
    double angle = atan2(yp,xp);
    out.setX(this->width()/2+xy*cos(angle)/scale);
    out.setY(this->height()/2+xy*sin(angle)/scale);
    return out;
}

void PainterLidar::fromPixelToCoords(QPoint pt, double &x, double &y){
    //Build the vector that connects the point to the origin in px. In fact I need to find out the orientation and the module of this vector
    double lp = sqrt(pow(pt.x()-this->width(),2)+pow(pt.y()-this->height()/2,2));
    double alpha = atan2((this->height()/2-pt.y()),(pt.x()-this->width())); //angle between the vector and the pixel reference frame. Remember that the pixel frame y grown downwards
    //Angle between the vector and the origin frame
    double beta = alpha - 0;
    //Transform from pixel position to mt with the scale
    x = round(lp*cos(beta)*scale*100)/100;
    y = round(lp*sin(beta)*scale*100)/100;
}

void PainterLidar::paintEvent(QPaintEvent *event)
{
    std::unique_lock<std::mutex> lock(plotMutex);

    QPainter painter(this);
    //Define the bound of the map
    QPen linepen(Qt::black);
    linepen.setCapStyle(Qt::RoundCap);
    linepen.setWidth(3);
    painter.setRenderHint(QPainter::Antialiasing,true);
    painter.setPen(linepen);

    painter.drawRect(0,0,this->width(),this->height());
    painter.setBrush(Qt::red);
    painter.drawEllipse(QPointF(this->width()/2,this->height()/2),10,10);

    painter.setBrush(QColor("#F7F7E2"));
    for (std::vector<QPointF>::iterator it = lidarPoints.begin() ; it != lidarPoints.end(); ++it){
        painter.drawEllipse((*it),3,3);
    }
}
