#ifndef gui_test_H
#define gui_test_H

#include "common.h"

#include <QThread>
#include <QtCore>

class Cloud_Generator : public QObject
{
    Q_OBJECT
    
public:
    Cloud_Generator(QObject* parent = NULL);    
    ~Cloud_Generator(){}
    
    void stop(){running = false;}
public slots:
    void random_cloud_color();
    
signals:
    void cloud_update(CloudType cloud);
    
private:
    CloudType cloud;
    bool running;
};

class Img_Grabber : public QObject
{
    Q_OBJECT
    
public:
    Img_Grabber(QObject* parent = NULL):QObject(parent){
        running = false;
        cap = new cv::VideoCapture(0);
    }
    ~Img_Grabber(){if(cap->isOpened())cap->release();}
    
    void stop(){running = false;}
    
public slots:
    void cap_image();
signals:
    void img_update(cv::Mat img);
    
private:
    cv::VideoCapture* cap;
    cv::Mat img;
    bool running;
};

class ResultGenerator : public QObject
{
    Q_OBJECT
    
public:
    ResultGenerator(QObject* parent = NULL):QObject(parent){
        running = false;
    }
    ~ResultGenerator(){}
    
    void stop(){running = false;}
    
public slots:
    void generate_result();
signals:
    void result_update(ResultDef result);
    void imu_update(IMUDef imu);
    
private:
    ResultDef result;
    IMUDef imu;
    bool running;
};

class Controller : public QObject
{
    Q_OBJECT
    
    QThread cloudthread,imgthread,res_thread;
    Cloud_Generator *cloud_generator;
    Img_Grabber *img_grabber;
    ResultGenerator *result_generator;
public:
    Controller() {
        cloud_generator = new Cloud_Generator;
        cloud_generator->moveToThread(&cloudthread);
        connect(&cloudthread, &QThread::finished, cloud_generator, &QObject::deleteLater);
        connect(this, &Controller::start_change_cloud, cloud_generator, &Cloud_Generator::random_cloud_color);
        connect(cloud_generator, &Cloud_Generator::cloud_update, this, &Controller::cloud_update_from_worker);
        cloudthread.start();
        
        img_grabber = new Img_Grabber;
        img_grabber->moveToThread(&imgthread);
        connect(&imgthread,&QThread::finished, img_grabber,&QObject::deleteLater);
        connect(this,&Controller::start_grab_img,img_grabber,&Img_Grabber::cap_image);
        connect(img_grabber,&Img_Grabber::img_update,this,&Controller::img_update_from_grabber);
        imgthread.start(); 
        
        result_generator = new ResultGenerator;
        result_generator->moveToThread(&res_thread);
        connect(&res_thread,&QThread::finished, result_generator,&QObject::deleteLater);
        connect(this,&Controller::start_generate_result,result_generator,&ResultGenerator::generate_result);
        connect(result_generator,&ResultGenerator::result_update,this,&Controller::result_update_from_generator);
        connect(result_generator,&ResultGenerator::imu_update,this,&Controller::imu_update_from_generator);
        res_thread.start();   
    }
    ~Controller() {
        cloud_generator->stop();
        cloudthread.quit();
        cloudthread.wait();
        
        img_grabber->stop();
        imgthread.quit();
        imgthread.wait();
        
        result_generator->stop();
        res_thread.quit();
        res_thread.wait();
    }
    
public:
    void start_test();
public slots:
    void cloud_update_from_worker(CloudType cloud);
    void img_update_from_grabber(cv::Mat img);
    void result_update_from_generator(ResultDef result);
    void imu_update_from_generator(IMUDef imu);
signals:
    void start_change_cloud();
    void start_grab_img();
    void start_generate_result();
    void cloud_update_to_window(CloudType cloud);
    void img_update_to_window(cv::Mat img);
    void result_update_to_window(ResultDef result);
    void imu_update_to_window(IMUDef imu);
};

#endif