#include "quadgui.h"
#include "ui_quadgui.h"
#include "camlabel.h"

quadGUI::quadGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::quadGUI)
{
    ui->setupUi(this);

    ///Camera Thread Setup
    cameraThread = new QThread;
    camMutex = new QMutex;
    cw = new CameraWorker;
    cw->setup(cameraThread, camMutex);
    connect(this, SIGNAL(stopCamThread()), cw, SLOT(onStop()));
    connect(ui->imgLabel, SIGNAL(mousePressed(int,int)), cw, SLOT(onMouseClicked(int,int)));
    connect(ui->imgLabel, SIGNAL(mouseClicked(int,int)), cw, SLOT(onMouseClicked(int,int)));
    connect(ui->imgLabel, SIGNAL(mouseRightPressed(int,int)), cw, SLOT(onRightMouseDragged(int,int)));
    connect(cw, SIGNAL(imageReady(QPixmap*)), this, SLOT(onCamImageReady(QPixmap*)));
    cw->moveToThread(cameraThread);
    cameraThread->start();
}

quadGUI::~quadGUI()
{
    emit stopCamThread();
    cameraThread->wait();
    delete cw;
    delete ui;
}

void quadGUI::onTimeout()
{

}

void quadGUI::onCamImageReady(QPixmap *pm)
{
    ros::spinOnce();
    camMutex->lock();
    ui->imgLabel->setPixmap(*pm);
    camMutex->unlock();
}

void quadGUI::on_takeoffButton_clicked()
{
    ROS_INFO("Flying ARdrone");
    takeOff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Rate loop_rate(30);

    int cnt = 0;
    while( ros::ok()) {
        if(cnt < 5){
            cnt+=1;
            takeOff.publish(msgE);
            ros::spinOnce();
        }
        else
            break;
        loop_rate.sleep();
    }
}

void quadGUI::on_landButton_clicked()
{
    ROS_INFO("Landing ARdrone");
    land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Rate loop_rate(30);
    int cnt = 0;
    while( ros::ok()) {
        if(cnt < 5){
            cnt+=1;
            land.publish(msgE);
            ros::spinOnce();
        }
        else
            break;
        loop_rate.sleep();
    }
}

void quadGUI::on_togglecameraButton_clicked()
{
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/quad/ardrone/togglecam");
    std_srvs::Empty service;
    if(client.call(service))
    {
        ROS_INFO("Camera toggled");
    }
}
void quadGUI::on_keyboardButton_windowTitleChanged(const QString &title)
{

}

void quadGUI::on_pushButton_clicked()
{
    ROS_INFO("Moving forward");
    front=nh.advertise<geometry_msgs::Twist>("/quad/cmd_vel", 1000);
    ros::Rate loop_rate(30);
    int cnt=0;
    msg2.linear.x=1.0;
    msg2.linear.y=0.0;
    msg2.linear.z=0.0;
    msg2.angular.x=0.0;
    msg2.angular.y=0.0;
    msg2.angular.z=0.0;
    while(ros::ok())
    {
     if(cnt<5)
     {
         cnt+=1;
         front.publish(msg2);
         ros::spinOnce();
     }
     else
     {
         break;
     }
     loop_rate.sleep();
    }
}

void quadGUI::on_pushButton_2_clicked()
{
    ROS_INFO("Moving backward");
    front=nh.advertise<geometry_msgs::Twist>("/quad/cmd_vel", 1000);
    ros::Rate loop_rate(30);
    int cnt=0;
    msg2.linear.x=-1.0;
    msg2.linear.y=0.0;
    msg2.linear.z=0.0;
    msg2.angular.x=0.0;
    msg2.angular.y=0.0;
    msg2.angular.z=0.0;
    while(ros::ok())
    {
     if(cnt<5)
     {
         cnt+=1;
         front.publish(msg2);
         ros::spinOnce();
     }
     else
     {
         break;
     }
     loop_rate.sleep();
    }
}

void quadGUI::on_pushButton_3_clicked()
{
    ROS_INFO("Moving right");
    front=nh.advertise<geometry_msgs::Twist>("/quad/cmd_vel", 1000);
    ros::Rate loop_rate(30);
    int cnt=0;
    msg2.linear.x=0.0;
    msg2.linear.y=-1.0;
    msg2.linear.z=0.0;
    msg2.angular.x=0.0;
    msg2.angular.y=0.0;
    msg2.angular.z=0.0;
    while(ros::ok())
    {
     if(cnt<5)
     {
         cnt+=1;
         front.publish(msg2);
         ros::spinOnce();
     }
     else
     {
         break;
     }
     loop_rate.sleep();
    }
}

void quadGUI::on_keyboardButton_clicked()
{

}

void quadGUI::on_pushButton_4_clicked()
{
    ROS_INFO("Moving right");
    front=nh.advertise<geometry_msgs::Twist>("/quad/cmd_vel", 1000);
    ros::Rate loop_rate(30);
    int cnt=0;
    msg2.linear.x=0.0;
    msg2.linear.y=1.0;
    msg2.linear.z=0.0;
    msg2.angular.x=0.0;
    msg2.angular.y=0.0;
    msg2.angular.z=0.0;
    while(ros::ok())
    {
     if(cnt<5)
     {
         cnt+=1;
         front.publish(msg2);
         ros::spinOnce();
     }
     else
     {
         break;
     }
     loop_rate.sleep();
    }
}
geometry_msgs::Twist msg1;
geometry_msgs::Twist msg3;
void subCallback(geometry_msgs::Twist msg)
    {
       msg1=msg;
       msg3=msg;
    }
void quadGUI::on_pushButton_5_clicked()
{
    ROS_INFO("killing switch");
    front=nh.advertise<geometry_msgs::Twist>("/quad/cmd_vel", 1000);
    sub = nh.subscribe<geometry_msgs::Twist>("/quad/odom", 10, subCallback);
    ros::Rate loop_rate(30);
    int cnt=0;
    while(ros::ok())
    {
     if(cnt<5)
     {
         msg2.linear.x=msg1.linear.x-(cnt+1)*(msg3.linear.x)/5;
         msg2.linear.y=msg1.linear.y-(cnt+1)*(msg3.linear.y)/5;
         msg2.linear.z=msg1.linear.z-(cnt+1)*(msg3.linear.z)/5;
         msg2.angular.x=msg1.angular.x-(cnt+1)*(msg3.angular.x)/5;
         msg2.angular.y=msg1.angular.y-(cnt+1)*(msg3.angular.y)/5;
         msg2.angular.z=msg1.angular.z-(cnt+1)*(msg3.angular.z)/5;
         cnt+=1;
         front.publish(msg2);
         ros::spinOnce();
     }
     if(cnt<15&&cnt>=5)
     {
         msg2.linear.x=0.0;
         msg2.linear.y=0.0;
         msg2.linear.z=0.0;
         msg2.angular.x=0.0;
         msg2.angular.y=0.0;
         msg2.angular.z=0.0;
         cnt+=1;
         front.publish(msg2);
         ros::spinOnce();
     }
     else
     {
         break;
     }
     loop_rate.sleep();
    }
}
