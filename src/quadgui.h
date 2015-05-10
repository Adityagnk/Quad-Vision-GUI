#ifndef QUADGUI_H
#define QUADGUI_H

#include <QMainWindow>
#include <QList>
#include <QListWidgetItem>
#include <QPixmap>
#include <QMutex>
#include <QTime>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "cameraworker.h"

namespace Ui {
class quadGUI;
}
class CameraWorker;

class quadGUI : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit quadGUI(QWidget *parent = 0);
    ~quadGUI();
public slots:
    void onCamImageReady(QPixmap *pm);
    void onTimeout();
signals:
    void colorChanged(int row);
    void stopCamThread();
private slots:
    void on_takeoffButton_clicked();

    void on_landButton_clicked();

    void on_togglecameraButton_clicked();
    void on_keyboardButton_windowTitleChanged(const QString &title);
    
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_keyboardButton_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

private:
    Ui::quadGUI *ui;
    QMutex *camMutex;
    QThread *cameraThread;
    CameraWorker *cw;

    ros::NodeHandle nh;
    std_msgs::Empty msgE;
    geometry_msgs::Twist msg2;
    geometry_msgs::Twist msg1;
    ros::Publisher takeOff;
    ros::Publisher land;
    ros::Publisher twist;
    ros::Publisher front;
    ros::Subscriber sub;


};

#endif // QUADGUI_H
