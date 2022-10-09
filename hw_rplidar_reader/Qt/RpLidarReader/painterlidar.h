#ifndef PAINTERLIDAR_H
#define PAINTERLIDAR_H

#include <QObject>
#include <QWidget>

#include <QTimer>
#include <mutex>

class PainterLidar : public QWidget
{
    Q_OBJECT
public:
    explicit PainterLidar(QWidget *parent = nullptr);
    void modifyLidarPoints(std::vector<std::pair<double,double>> scans);
    void changeScale(bool increment);
signals:

public slots:

private:
    QTimer* timerRefresh;
    std::mutex plotMutex;
    double scale;
    double range=10;
    std::vector<QPointF> lidarPoints;
    void resizeEvent(QResizeEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

    QPointF fromCoordsToPix(const double &x, const double &y);
    void fromPixelToCoords(QPoint pt, double &x, double &y);

private slots:

};

#endif // PAINTERLIDAR_H
