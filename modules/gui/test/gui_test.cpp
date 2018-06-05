#include "gui.h"
#include <QApplication>
#include <QMainWindow>
#include "gui_test.h"

Cloud_Generator::Cloud_Generator(QObject *parent):QObject(parent)
{
    running = false;
    // The number of points in the cloud
    cloud.points.resize (200);


    // Fill the cloud with some points
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

        cloud.points[i].r = 128;
        cloud.points[i].g = 128;
        cloud.points[i].b = 128;
    }
}

void Cloud_Generator::random_cloud_color()
{
    running = true;
    while(running)
    {
        // Set the new color
        for (size_t i = 0; i < cloud.size(); i++)
        {
            cloud.points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
            cloud.points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
            cloud.points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
        }
        
        emit cloud_update(cloud);
        
        sleep(1);
    }
}

void Img_Grabber::cap_image()
{
    running = true;    
    if(cap->isOpened())
    {
        while(running)
        {
            *cap >> img;
            emit img_update(img);
            QThread::msleep(30);
        }
    }
    else
        std::cout<<RED<<"No camera found!"<<RESET<<std::endl;
}

void ResultGenerator::generate_result()
{
    running = true;
    while(running)
    {
        if(result.has_stair)
            result.has_stair = false;
        else
            result.has_stair = true;
        
        result.height = (1024 * rand () / (RAND_MAX + 1.0f));
        result.width = (1024 * rand () / (RAND_MAX + 1.0f));
        result.v_depth = (1024 * rand () / (RAND_MAX + 1.0f));
        result.v_height = (1024 * rand () / (RAND_MAX + 1.0f));
        result.time = abs(100*(1024 * rand () / (RAND_MAX + 1.0f)));

        imu.yaw = 180*(1024 * rand () / (RAND_MAX + 1.0f));
        imu.pitch = 180*(1024 * rand () / (RAND_MAX + 1.0f));
        imu.roll = 180*(1024 * rand () / (RAND_MAX + 1.0f));
        
        emit result_update(result);
        emit imu_update(imu);
        
        sleep(1);
    }
}

void Controller::start_test()
{
    emit start_change_cloud();
    emit start_grab_img();
    emit start_generate_result();
}

void Controller::cloud_update_from_worker(CloudType cloud)
{
    emit cloud_update_to_window(cloud);
}

void Controller::img_update_from_grabber(cv::Mat img)
{
    emit img_update_to_window(img);
}

void Controller::result_update_from_generator(ResultDef result)
{
    emit result_update_to_window(result);
}

void Controller::imu_update_from_generator(IMUDef imu)
{
    emit imu_update_to_window(imu);
}


int main (int argc, char *argv[])
{
    QApplication a (argc, argv);
    
    Controller controller;    
    gui w;
    QObject::connect(&controller,&Controller::cloud_update_to_window, &w, &gui::update_cloud);
    QObject::connect(&controller,&Controller::img_update_to_window,&w,&gui::update_depth_img);
    QObject::connect(&controller,&Controller::img_update_to_window,&w,&gui::update_color_img);
    QObject::connect(&controller,&Controller::result_update_to_window,&w,&gui::update_result);
    QObject::connect(&controller,&Controller::imu_update_to_window,&w,&gui::update_imu);
    controller.start_test();
    w.show ();
    
    return a.exec ();
}
