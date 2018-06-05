#ifndef READER_H
#define READER_H

#include "common.h"
#include "k2g.h"
#include "serial.h"
#include <omp.h>

#include <sys/time.h>

#include <QThread>
#include <QtCore>
#include <QMutex>

class Reader : public QThread
{
Q_OBJECT

public:
    explicit Reader(QObject *parent = nullptr);

    ~Reader() override;

    void stop();

    bool misRunning() { return running; }

public slots:

    void update_serial_name(std::string imu);

    void update_file_path(std::string path);

    void save_images();

    void update_run_mode(RunModeDef);

    void read_file_prev();

    void read_file_next();

    void update_pause(bool p);

signals:

    void cloud_update(const CloudType::Ptr cloud);

    void color_image_update(const cv::Mat* img);

    void depth_image_update(const cv::Mat* img);

    void imu_update(IMUDef);

    void imu_status_update(bool);

protected:
    void run() override;

private:
    void open_imu_serial();

    bool read_file_numbers();

private:
    bool running;//mutex
    bool pause;

    QMutex mutex_running;

    RunModeDef run_mode;//mutex
    QMutex run_mode_mutex;

    std::string file_path;//mutex

    std::string imu_serial_name;//mutex
    int16_t Acc[3];
    int16_t Gyo[3];
    int16_t Mag[3];
    float Euler[3];
    float Quat[4];
    IMUDef pose;
    Eigen::Matrix3f rotation_matrix;

    K2G *pk2g;//mutex
    bool kinect_ok, imu_ok;
    QMutex kinect_mutex;
    cv::Mat color_img;//mutex
    cv::Mat depth_img;//mutex
    CloudType::Ptr cloud_in, cloud;    //mutex
    QMutex cloud_mutex;

    size_t savecount;
    int read_file_count, count;
    bool prev, next, valid_file_path;
    QMutex prev_mutex;

};

#endif