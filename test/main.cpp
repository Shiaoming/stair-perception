#include <QtCore>
#include <QApplication>

#include "controller.h"
#include "gui.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    Controller controller;
    gui w;

    QObject::connect(&w, &gui::run_mode_update, &controller, &Controller::run_mode_update);
    QObject::connect(&w, &gui::start_signal, &controller, &Controller::start_bt_signal);
    QObject::connect(&w, &gui::stop_signal, &controller, &Controller::stop_bt_signal);

    // TODO: next line is just a test
    QObject::connect(controller.reader, &Reader::cloud_update, &w, &gui::update_original_cloud);

    QObject::connect(controller.reader, &Reader::color_image_update, &w, &gui::update_color_img);
    QObject::connect(controller.reader, &Reader::depth_image_update, &w, &gui::update_depth_img);
    QObject::connect(controller.reader, &Reader::imu_update, &w, &gui::update_imu);
    QObject::connect(controller.reader, &Reader::imu_status_update, &w, &gui::update_imu_serial_status);

    QObject::connect(&w, &gui::imu_serial_update, controller.reader, &Reader::update_serial_name);
    QObject::connect(&w, &gui::img_path_update, controller.reader, &Reader::update_file_path);
    QObject::connect(&w, &gui::save_img_signal, controller.reader, &Reader::save_images);
    QObject::connect(&w, &gui::run_mode_update, controller.reader, &Reader::update_run_mode);

    QObject::connect(controller.reader, &Reader::cloud_update, controller.processor, &DProcessor::update_cloud_in);

    QObject::connect(controller.processor, &DProcessor::cloud_update, &w, &gui::update_cloud);
    QObject::connect(controller.processor, &DProcessor::normal_update, &w, &gui::update_normal);
    QObject::connect(controller.processor, &DProcessor::planes_update, &w, &gui::update_planes);
    QObject::connect(controller.processor, &DProcessor::result_update, &w, &gui::update_result);
    QObject::connect(controller.processor, &DProcessor::robot_serial_update, &w, &gui::update_robot_serial_status);

    QObject::connect(&w, &gui::parameters_update, controller.processor, &DProcessor::update_parameters);
    QObject::connect(&w, &gui::show_mode_update, controller.processor, &DProcessor::update_show_mode);
    QObject::connect(&w, &gui::robot_serial_update, controller.processor, &DProcessor::update_robot_name);

    w.init();
    w.show();

    return a.exec();
}