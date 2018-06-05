#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "common.h"
#include "serial_class.h"
#include "Stair.hpp"
#include "StairPerception.hpp"

#include <QThread>
#include <QtCore>
#include <QMutex>

class DProcessor : public QThread
{
    Q_OBJECT
    
public:
    explicit DProcessor();
    ~DProcessor() override;
    
    bool misRunning(){return running;}
    void stop();
    
public slots:
    void update_robot_name(std::string robot);
    void update_parameters(ParameterDef);
    void update_show_mode(ShowModeDef);    
    void update_pause(bool p);
    void update_cloud_in(const CloudType::Ptr cloud);
    
signals:
    void cloud_update(const CloudType::Ptr cloud);
    void normal_update(const pcl::PointCloud<pcl::Normal> &normal);
    void planes_update(const std::vector<Plane> *vector_plane_sorted);
    void result_update(ResultDef);
    void robot_serial_update(bool);
    
private:    
    void open_robot_serial();
    void resolve_result(Stair);
    void pack_and_send_robot();
    
protected:
    void run() override;
    
private:
    bool running;
    bool pause;
    
    std::string robot_serial_name;
    ShowModeDef show_mode;
    bool first_result;
    int result_count;
    ResultDef result;
    ResultDef result_buff[20];
    
    ParameterDef parameters;
    
    bool new_cloud_come;
    QMutex new_cloud_mutex;

    CloudType::Ptr cloud_in;
    CloudType cloud_in_copy;
    QMutex cloud_in_mutex;
    
    CloudType cloud_out;
    pcl::PointCloud<pcl::Normal> normal_out;
    std::vector<Plane> vector_plane_sorted;
    bool has_stair;
    
    Cserial sender;
    bool serial_ok;
    
    //取消字节对齐
    #pragma pack (1)
     typedef union
     {
         struct
         {
             uint8_t head;
             uint8_t flag;
//              float height;
//              float depth;
//              float v_height;
//              float v_depth;
             
             float v_depth;
             float v_height;
             float depth;
             float height;
             uint8_t tail;
         };
         uint8_t data[19];
     } package_;
    #pragma pack ()
     
};

#endif