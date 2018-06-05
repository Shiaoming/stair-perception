#ifndef gui_H
#define gui_H

#include "common.h"

// Qt
#include <QMainWindow>
#include <QImage>
#include <QPixmap>
#include <QFileDialog> 
#include <QMessageBox>  
#include <QDebug> 
#include <QPainter>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <QMetaType>
Q_DECLARE_METATYPE(CloudType::Ptr);
Q_DECLARE_METATYPE(pcl::PointCloud<pcl::Normal>);
Q_DECLARE_METATYPE(std::vector<Plane>*);
Q_DECLARE_METATYPE(cv::Mat*);
Q_DECLARE_METATYPE(std::string);
Q_DECLARE_METATYPE(ParameterDef);
Q_DECLARE_METATYPE(ResultDef);
Q_DECLARE_METATYPE(IMUDef);
Q_DECLARE_METATYPE(RunModeDef);
Q_DECLARE_METATYPE(ShowModeDef);

namespace Ui
{
    class gui;
}

class gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit gui (QWidget *parent = 0);
    virtual ~gui ();
    void init();
    
    // void update_cloud(std::vector<Plane> planes);

public Q_SLOTS:
    void update_original_cloud(const CloudType::Ptr cloud);
    void update_cloud(const CloudType::Ptr cloud);
    void update_normal(const pcl::PointCloud<pcl::Normal> &normal);
    void update_planes(const std::vector<Plane>* planes);
    void update_depth_img(const cv::Mat* pdepth);
    void update_color_img(const cv::Mat* pcolor);
    void update_result(ResultDef result);
    void update_imu(IMUDef imu);
    void update_robot_serial_status(bool status);
    void update_imu_serial_status(bool status);
    
private Q_SLOTS:
   
#define DOUBLE_SPIN_SLIDER_SLOTS(obj) void obj##_horizontalSlider_changed(int v); \
                                      void obj##_doubleSpinBox_set();
#define INT_SPIN_SLIDER_SLOTS(obj)    void obj##_horizontalSlider_changed(int v); \
                                      void obj##_spinBox_set();
    // gui slots
    DOUBLE_SPIN_SLIDER_SLOTS(x_min)
    DOUBLE_SPIN_SLIDER_SLOTS(x_max)
    DOUBLE_SPIN_SLIDER_SLOTS(y_min)
    DOUBLE_SPIN_SLIDER_SLOTS(y_max)
    DOUBLE_SPIN_SLIDER_SLOTS(z_min)
    DOUBLE_SPIN_SLIDER_SLOTS(z_max)
    
    INT_SPIN_SLIDER_SLOTS(normal_compute_points)
    
    DOUBLE_SPIN_SLIDER_SLOTS(voxel_x)
    DOUBLE_SPIN_SLIDER_SLOTS(voxel_y)
    DOUBLE_SPIN_SLIDER_SLOTS(voxel_z)
    
    INT_SPIN_SLIDER_SLOTS(parallel_angle_diff)
    INT_SPIN_SLIDER_SLOTS(perpendicular_angle_diff)
    
    DOUBLE_SPIN_SLIDER_SLOTS(cluster_tolerance)
    INT_SPIN_SLIDER_SLOTS(min_cluster_size)    
    
    DOUBLE_SPIN_SLIDER_SLOTS(seg_threshold)
    INT_SPIN_SLIDER_SLOTS(seg_rest_point)
    DOUBLE_SPIN_SLIDER_SLOTS(seg_plane_angle_diff)    

    DOUBLE_SPIN_SLIDER_SLOTS(merge_threshold)
    DOUBLE_SPIN_SLIDER_SLOTS(merge_angle_diff)
    
    INT_SPIN_SLIDER_SLOTS(min_num_points)
    
    DOUBLE_SPIN_SLIDER_SLOTS(min_length)
    DOUBLE_SPIN_SLIDER_SLOTS(max_length)
    DOUBLE_SPIN_SLIDER_SLOTS(min_width)
    DOUBLE_SPIN_SLIDER_SLOTS(max_width)
    
    DOUBLE_SPIN_SLIDER_SLOTS(max_g_height)
    DOUBLE_SPIN_SLIDER_SLOTS(counter_max_distance)
    DOUBLE_SPIN_SLIDER_SLOTS(min_height)
    DOUBLE_SPIN_SLIDER_SLOTS(max_height)
    
    DOUBLE_SPIN_SLIDER_SLOTS(vertical_angle_diff)
    DOUBLE_SPIN_SLIDER_SLOTS(mcv_angle_diff)
    DOUBLE_SPIN_SLIDER_SLOTS(center_vector_angle_diff)
    DOUBLE_SPIN_SLIDER_SLOTS(vertical_plane_angle_diff)
    DOUBLE_SPIN_SLIDER_SLOTS(noleg_distance)
    
    void bt_clicked_read_parameters();
    void bt_clicked_save_parameters();
    
    void edit_finished_imu_serial();
    void edit_finished_robot_serial();
    
    void bt_group_runmode_clicked(int buttonId);
    void bt_group_showmode_clicked(int buttonId);
    
    void bt_start_clicked();
    void bt_stop_clicked();
    
    void bt_img_path_clicked();
    void bt_img_save_clicked();
    
signals:
    void close_window();
    void imu_serial_update(std::string imu);
    void img_path_update(std::string path);
    void robot_serial_update(std::string robot);
    void parameters_update(ParameterDef);
    void run_mode_update(RunModeDef);
    void show_mode_update(ShowModeDef);
    void start_signal();
    void stop_signal();
    void save_img_signal();
    
private:
    void read_default_parameters();
    void read_parameters();

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private:
    Ui::gui *ui;
    
    QString param_file;
    ParameterDef paramters;
    
    std::string imu_serial,robot_serial;
    std::string img_path;
    
    RunModeDef run_mode;
    ShowModeDef show_mode;
};

#endif // gui_H
