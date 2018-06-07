#include "gui.h"
#include "ui_gui.h"
#include "common.h"
// boost::shared_ptr<Config> Config::config_ = nullptr;

gui::gui(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::gui)
{
    img_path = "../samples";
    param_file = tr("../param.yaml");
    run_mode = files_mode;
    show_mode = original;

    ui->setupUi(this);

    // register custom    type
    qRegisterMetaType<CloudType::Ptr>("CloudType::Ptr");
    qRegisterMetaType<pcl::PointCloud<pcl::Normal>>("pcl::PointCloud<pcl::Normal>");
    qRegisterMetaType<std::vector<Plane>>("std::vector<Plane>");
    qRegisterMetaType<cv::Mat*>("cv::Mat*");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<ParameterDef>("ParameterDef");
    qRegisterMetaType<ResultDef>("ResultDef");
    qRegisterMetaType<IMUDef>("IMUDef");
    qRegisterMetaType<RunModeDef>("RunModeDef");
    qRegisterMetaType<ShowModeDef>("ShowModeDef");

    // Set up the QVTK window
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkwidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkwidget->GetInteractor(), ui->qvtkwidget->GetRenderWindow());
    ui->qvtkwidget->update();
    viewer->setPosition(0, 0);
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.3);
    viewer->setCameraPosition(-0.426281, -1.44271, 2.79198, -0.968207, 0.201592, -0.148106);

#define DOUBLE_SPIN_SLIDER_CONNECT(obj) connect(ui->obj##_horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(obj##_horizontalSlider_changed(int)));\
                                        connect(ui->obj##_doubleSpinBox, SIGNAL(editingFinished()), this, SLOT(obj##_doubleSpinBox_set()));
#define INT_SPIN_SLIDER_CONNECT(obj)    connect(ui->obj##_horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(obj##_horizontalSlider_changed(int)));\
                                        connect(ui->obj##_spinBox, SIGNAL(editingFinished()), this, SLOT(obj##_spinBox_set()));

    // setup ui_gui
    DOUBLE_SPIN_SLIDER_CONNECT(x_min)
    DOUBLE_SPIN_SLIDER_CONNECT(x_max)
    DOUBLE_SPIN_SLIDER_CONNECT(y_min)
    DOUBLE_SPIN_SLIDER_CONNECT(y_max)
    DOUBLE_SPIN_SLIDER_CONNECT(z_min)
    DOUBLE_SPIN_SLIDER_CONNECT(z_max)

    INT_SPIN_SLIDER_CONNECT(normal_compute_points)

    DOUBLE_SPIN_SLIDER_CONNECT(voxel_x)
    DOUBLE_SPIN_SLIDER_CONNECT(voxel_y)
    DOUBLE_SPIN_SLIDER_CONNECT(voxel_z)

    INT_SPIN_SLIDER_CONNECT(parallel_angle_diff)
    INT_SPIN_SLIDER_CONNECT(perpendicular_angle_diff)

    DOUBLE_SPIN_SLIDER_CONNECT(cluster_tolerance)
    INT_SPIN_SLIDER_CONNECT(min_cluster_size)

    DOUBLE_SPIN_SLIDER_CONNECT(seg_threshold)
    INT_SPIN_SLIDER_CONNECT(seg_rest_point)
    DOUBLE_SPIN_SLIDER_CONNECT(seg_plane_angle_diff)

    DOUBLE_SPIN_SLIDER_CONNECT(merge_threshold)
    DOUBLE_SPIN_SLIDER_CONNECT(merge_angle_diff)

    INT_SPIN_SLIDER_CONNECT(min_num_points)

    DOUBLE_SPIN_SLIDER_CONNECT(min_length)
    DOUBLE_SPIN_SLIDER_CONNECT(max_length)
    DOUBLE_SPIN_SLIDER_CONNECT(min_width)
    DOUBLE_SPIN_SLIDER_CONNECT(max_width)

    DOUBLE_SPIN_SLIDER_CONNECT(max_g_height)
    DOUBLE_SPIN_SLIDER_CONNECT(counter_max_distance)
    DOUBLE_SPIN_SLIDER_CONNECT(min_height)
    DOUBLE_SPIN_SLIDER_CONNECT(max_height)

    DOUBLE_SPIN_SLIDER_CONNECT(vertical_angle_diff)
    DOUBLE_SPIN_SLIDER_CONNECT(mcv_angle_diff)
    DOUBLE_SPIN_SLIDER_CONNECT(center_vector_angle_diff)
    DOUBLE_SPIN_SLIDER_CONNECT(vertical_plane_angle_diff)
    DOUBLE_SPIN_SLIDER_CONNECT(noleg_distance)

    connect(this, SIGNAL(close_window()), this, SLOT(close()));
    connect(ui->open_param_pushButton, SIGNAL(clicked()), this, SLOT(bt_clicked_read_parameters()));
    connect(ui->save_param_pushButton, SIGNAL(clicked()), this, SLOT(bt_clicked_save_parameters()));

    connect(ui->IMU_Serial_lineEdit, SIGNAL(editingFinished()), this, SLOT(edit_finished_imu_serial()));
    connect(ui->robot_serial_lineEdit, SIGNAL(editingFinished()), this, SLOT(edit_finished_robot_serial()));

    ui->mode_buttonGroup->setId(ui->mode_files_radioButton, int(files_mode));
    ui->mode_buttonGroup->setId(ui->mode_kinect_radioButton, int(kinect_mode));
    connect(ui->mode_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(bt_group_runmode_clicked(int)));

    ui->show_buttonGroup->setId(ui->original_cloud_checkBox, int(original));
    ui->show_buttonGroup->setId(ui->range_checkBox, int(range_limited));
    ui->show_buttonGroup->setId(ui->voxel_checkBox, int(voxel));
    ui->show_buttonGroup->setId(ui->noleg_checkBox, int(noleg));
    ui->show_buttonGroup->setId(ui->horizontal_cloud_checkBox, int(horizontal_cloud));
    ui->show_buttonGroup->setId(ui->vertical_cloud_checkBox, int(vertical_cloud));
    ui->show_buttonGroup->setId(ui->seg_h_p_checkBox, int(seg_horizontal_plane));
    ui->show_buttonGroup->setId(ui->seg_v_pcheckBox, int(seg_vertical_plane));
    ui->show_buttonGroup->setId(ui->merge_h_p_checkBox, int(merge_horizontal_plane));
    ui->show_buttonGroup->setId(ui->merge_v_pcheckBox, int(merge_vertical_plane));
    ui->show_buttonGroup->setId(ui->stair_checkBox, int(stair));
    connect(ui->show_buttonGroup, SIGNAL(buttonClicked(int)), this, SLOT(bt_group_showmode_clicked(int)));

    connect(ui->start_pushButton, SIGNAL(clicked()), this, SLOT(bt_start_clicked()));
    connect(ui->stop_pushButton, SIGNAL(clicked()), this, SLOT(bt_stop_clicked()));
    connect(ui->img_path_pushButton, SIGNAL(clicked()), this, SLOT(bt_img_path_clicked()));
    connect(ui->save_img_pushButton, SIGNAL(clicked()), this, SLOT(bt_img_save_clicked()));
}

void gui::init()
{
    read_default_parameters();
    edit_finished_imu_serial();
    edit_finished_robot_serial();
    bt_group_runmode_clicked(0);
    bt_group_showmode_clicked(0);
    emit img_path_update(img_path);
}

void gui::update_original_cloud(const CloudType::Ptr cloud)
{
    if (show_mode == original)
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->addPointCloud(cloud, "cloud");
        ui->qvtkwidget->update();
    }
}

void gui::update_cloud(const CloudType::Ptr cloud)
{
    if (show_mode != original)
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->addPointCloud(cloud, "cloud");
        ui->qvtkwidget->update();
    }
}

cv::Mat NormIntensity(const cv::Mat &mat, int minvalue, int maxvalue)
{
    if (mat.channels() != 1)
    {
        std::cout << "Waring: NormIntensity only handle 1 channel float image!" << std::endl;
        return mat;
    }

    cv::Mat normmat = cv::Mat(mat.size(), CV_8UC1);

    if (mat.step.buf[1] == 4)
        for (int i = 0; i < mat.rows; i++)
        {
            const float *mat_ptr = mat.ptr<float>(i);
            uchar *normmat_ptr = normmat.ptr<uchar>(i);
            for (int j = 0; j < mat.cols; j++)
            {
                normmat_ptr[j] = cv::saturate_cast<uchar>((mat_ptr[j] - minvalue) * 255 / (maxvalue - minvalue));
            }
        }
    else if (mat.step.buf[1] == 2)
        for (int i = 0; i < mat.rows; i++)
        {
            const uchar *mat_ptr = mat.ptr<uchar>(i);
            uchar *normmat_ptr = normmat.ptr<uchar>(i);
            for (int j = 0; j < mat.cols; j++)
            {
                int intgray = mat_ptr[j * 2] + (mat_ptr[j * 2 + 1] << 8);
                normmat_ptr[j] = cv::saturate_cast<uchar>((intgray - minvalue) * 255 / (maxvalue - minvalue));
            }
        }

    return normmat;
}

void gui::update_depth_img(const cv::Mat* pmat_img)
{
    if (pmat_img->cols != 0)
    {
        if (pmat_img->channels() != 1)
        {
            ui->image_depth_label->setText("Depth image must be 1 channel");
            return;
        }

        cv::Mat mat_img_rgb = NormIntensity(*pmat_img, 0, 4500);
//        cv::cvtColor(NormIntensity(*pmat_img, 0, 4500), mat_img_rgb, cv::COLOR_GRAY2RGB);

        int w_h, i_h, i_w;
        w_h = ui->widget_image->size().height();
        i_h = w_h;
        i_w = 512 * w_h / 424;

        QImage qimg = QImage((const uchar *) (mat_img_rgb.data),
                             mat_img_rgb.cols, mat_img_rgb.rows,
                             mat_img_rgb.cols * mat_img_rgb.channels(),
                             QImage::Format_Grayscale8);

        QPixmap qpixmap = QPixmap::fromImage(qimg);

        ui->image_depth_label->setGeometry(0, 0, i_w, i_h);
        ui->image_depth_label->setPixmap(qpixmap);
        ui->image_depth_label->show();
    }
}

void gui::update_color_img(const cv::Mat* pmat_img)
{
    if (pmat_img->cols != 0)
    {
        cv::Mat mat_img_rgb;
        cv::cvtColor(*pmat_img, mat_img_rgb, cv::COLOR_BGRA2RGBA);

        int w_h, i_h, i_w, pos;
        w_h = ui->widget_image->size().height();
        i_h = w_h;
        i_w = 1920 * w_h / 1080;
        pos = 512 * w_h / 424;

        cv::resize(mat_img_rgb,mat_img_rgb,cv::Size(i_w,i_h));

        QImage qimg;
        qimg = QImage((const uchar *) (mat_img_rgb.data),
                      mat_img_rgb.cols, mat_img_rgb.rows,
                      mat_img_rgb.cols * mat_img_rgb.channels(),
                      QImage::Format_RGBA8888);

        QPixmap qpixmap = QPixmap::fromImage(qimg);

        ui->image_color_label->setGeometry(pos, 0, i_w, i_h);
        ui->image_color_label->setPixmap(qpixmap);
        ui->image_color_label->show();
    }
}

void gui::update_imu(IMUDef imu)
{
    ui->yaw_label->setText(tr("yaw: %1").arg(imu.yaw));
    ui->pitch_label->setText(tr("pitch: %1").arg(imu.pitch));
    ui->roll_label->setText(tr("roll: %1").arg(imu.roll));
}

void gui::update_result(ResultDef result)
{
    if (result.has_stair)
        ui->find_stair_label->setText(tr("Find stair!"));
    else
        ui->find_stair_label->setText(tr("No stair!"));

    ui->height_label->setText(tr("height: %1").arg(result.height));
    ui->width_label->setText(tr("width: %1").arg(result.width));
    ui->v_height_label->setText(tr("v_height: %1").arg(result.v_height));
    ui->v_depth_label->setText(tr("v_depth: %1").arg(result.v_depth));
    ui->time_label->setText(tr("time: %1 ms").arg(result.time));
}

void gui::update_robot_serial_status(bool status)
{
    QPalette palette = ui->robot_serial_lineEdit->palette();
    if (status)
        palette.setColor(QPalette::Base, QColor(100, 230, 100));
    else
        palette.setColor(QPalette::Base, QColor(230, 50, 50));

    ui->robot_serial_lineEdit->setPalette(palette);
}

void gui::update_imu_serial_status(bool status)
{
    QPalette palette = ui->IMU_Serial_lineEdit->palette();
    if (status)
        palette.setColor(QPalette::Base, QColor(100, 230, 100));
    else
        palette.setColor(QPalette::Base, QColor(230, 50, 50));

    ui->IMU_Serial_lineEdit->setPalette(palette);
}

void gui::update_normal(const pcl::PointCloud<pcl::Normal> &normal)
{

}

void gui::update_planes(const std::vector<Plane> *vector_plane_sorted)
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    std::string viewr_cloud_name = "rcloud_";
    if (vector_plane_sorted->size() > 0)
    {
        srand(0);
        for (unsigned int i = 0; i < vector_plane_sorted->size(); i++)
        {
            if ((*vector_plane_sorted)[i].ptype == Plane::Ptype::stair_component)
            {
                double r, g, b;
                r = int(255.0 * rand() / (RAND_MAX + 1.0));
                g = int(255.0 * rand() / (RAND_MAX + 1.0));
                b = int(255.0 * rand() / (RAND_MAX + 1.0));
                pcl::visualization::PointCloudColorHandlerCustom<PointType>
                        single_color((*vector_plane_sorted)[i].cloud.makeShared(), r, g, b);

                std::stringstream ss;

                // add cloud
                ss << viewr_cloud_name << i << "c";
                viewer->addPointCloud<PointType>((*vector_plane_sorted)[i].cloud.makeShared(), single_color, ss.str());
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());

                // add center
                ss << "c";
                // viewer->addSphere(vector_plane_sorted[i].center, 0.02, r / 255.0, g / 255.0, b / 255.0, ss.str());

                // add counter
                ss << "c";
                viewer->addPolygon<pcl::PointXYZ>((*vector_plane_sorted)[i].counter.makeShared(), r / 255.0, g / 255.0,
                                                  b / 255.0, ss.str());

            } else if ((*vector_plane_sorted)[i].ptype == Plane::Ptype::others)
            {
                pcl::visualization::PointCloudColorHandlerCustom<PointType>
                        single_color((*vector_plane_sorted)[i].cloud.makeShared(), 255, 0, 0);

                std::stringstream ss;

                // add cloud
                ss << viewr_cloud_name << i << "c";
                viewer->addPointCloud<PointType>((*vector_plane_sorted)[i].cloud.makeShared(), single_color, ss.str());
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
            } else if ((*vector_plane_sorted)[i].ptype == Plane::Ptype::ground)
            {
                pcl::visualization::PointCloudColorHandlerCustom<PointType>
                        single_color((*vector_plane_sorted)[i].cloud.makeShared(), 0, 255, 0);

                std::stringstream ss;

                // add cloud
                ss << viewr_cloud_name << i << "c";
                viewer->addPointCloud<PointType>((*vector_plane_sorted)[i].cloud.makeShared(), single_color, ss.str());
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
            }
        }
    }

    ui->qvtkwidget->update();
}


#define DOUBLE_SPIN_SLIDER_FUNC(obj) void gui::obj##_horizontalSlider_changed(int v)\
                                    {\
                                        double max_slider,min_slider,max_spin,min_spin;\
                                        max_slider = ui->obj##_horizontalSlider->maximum();\
                                        min_slider = ui->obj##_horizontalSlider->minimum();\
                                        max_spin = ui->obj##_doubleSpinBox->maximum();\
                                        min_spin = ui->obj##_doubleSpinBox->minimum();\
                                        \
                                        double value = (v - min_slider) * (max_spin - min_spin)\
                                            / (max_slider - min_slider) + min_spin;\
                                        \
                                        ui->obj##_doubleSpinBox->setValue(value);\
                                        paramters.obj##_=value;\
                                        emit parameters_update(paramters);\
                                        }\
                                    \
                                    void gui::obj##_doubleSpinBox_set()\
                                    {\
                                        double v = ui->obj##_doubleSpinBox->value();\
                                        double max_slider,min_slider,max_spin,min_spin;\
                                        max_slider = ui->obj##_horizontalSlider->maximum();\
                                        min_slider = ui->obj##_horizontalSlider->minimum();\
                                        max_spin = ui->obj##_doubleSpinBox->maximum();\
                                        min_spin = ui->obj##_doubleSpinBox->minimum();\
                                        \
                                        double value = (v - min_spin) * (max_slider - min_slider)\
                                            / (max_spin - min_spin) + min_slider;\
                                        \
                                        ui->obj##_horizontalSlider->setValue(int(value)); \
                                        }
//std::cout<<#obj<<"_doubleSpinBox changed to " << value << std::endl;
//std::cout<<#obj<<"_horizontalSlider changed to " << value << std::endl;

#define INT_SPIN_SLIDER_FUNC(obj)    void gui::obj##_horizontalSlider_changed(int v)\
                                    {\
                                        double max_slider,min_slider,max_spin,min_spin;\
                                        max_slider = ui->obj##_horizontalSlider->maximum();\
                                        min_slider = ui->obj##_horizontalSlider->minimum();\
                                        max_spin = ui->obj##_spinBox->maximum();\
                                        min_spin = ui->obj##_spinBox->minimum();\
                                        \
                                        double value = (v - min_slider) * (max_spin - min_spin)\
                                            / (max_slider - min_slider) + min_spin;\
                                            \
                                        ui->obj##_spinBox->setValue(value);\
                                        paramters.obj##_=value;\
                                        emit parameters_update(paramters);\
                                        }\
                                    \
                                    void gui::obj##_spinBox_set()\
                                    {\
                                        double v = ui->obj##_spinBox->value();\
                                        double max_slider,min_slider,max_spin,min_spin;\
                                        max_slider = ui->obj##_horizontalSlider->maximum();\
                                        min_slider = ui->obj##_horizontalSlider->minimum();\
                                        max_spin = ui->obj##_spinBox->maximum();\
                                        min_spin = ui->obj##_spinBox->minimum();\
                                        \
                                        double value = (v - min_spin) * (max_slider - min_slider)\
                                            / (max_spin - min_spin) + min_slider;\
                                        \
                                        ui->obj##_horizontalSlider->setValue(int(value));\
                                    }
//std::cout<<#obj<<"_spinBox changed to " << value << std::endl;
//std::cout<<#obj<<"_horizontalSlider changed to " << value << std::endl;

DOUBLE_SPIN_SLIDER_FUNC(x_min)

DOUBLE_SPIN_SLIDER_FUNC(x_max)

DOUBLE_SPIN_SLIDER_FUNC(y_min)

DOUBLE_SPIN_SLIDER_FUNC(y_max)

DOUBLE_SPIN_SLIDER_FUNC(z_min)

DOUBLE_SPIN_SLIDER_FUNC(z_max)

INT_SPIN_SLIDER_FUNC(normal_compute_points)

DOUBLE_SPIN_SLIDER_FUNC(voxel_x)

DOUBLE_SPIN_SLIDER_FUNC(voxel_y)

DOUBLE_SPIN_SLIDER_FUNC(voxel_z)

INT_SPIN_SLIDER_FUNC(parallel_angle_diff)

INT_SPIN_SLIDER_FUNC(perpendicular_angle_diff)

DOUBLE_SPIN_SLIDER_FUNC(cluster_tolerance)

INT_SPIN_SLIDER_FUNC(min_cluster_size)

DOUBLE_SPIN_SLIDER_FUNC(seg_threshold)

INT_SPIN_SLIDER_FUNC(seg_rest_point)

DOUBLE_SPIN_SLIDER_FUNC(seg_plane_angle_diff)

DOUBLE_SPIN_SLIDER_FUNC(merge_threshold)

DOUBLE_SPIN_SLIDER_FUNC(merge_angle_diff)

INT_SPIN_SLIDER_FUNC(min_num_points)

DOUBLE_SPIN_SLIDER_FUNC(min_length)

DOUBLE_SPIN_SLIDER_FUNC(max_length)

DOUBLE_SPIN_SLIDER_FUNC(min_width)

DOUBLE_SPIN_SLIDER_FUNC(max_width)

DOUBLE_SPIN_SLIDER_FUNC(max_g_height)

DOUBLE_SPIN_SLIDER_FUNC(counter_max_distance)

DOUBLE_SPIN_SLIDER_FUNC(min_height)

DOUBLE_SPIN_SLIDER_FUNC(max_height)

DOUBLE_SPIN_SLIDER_FUNC(vertical_angle_diff)

DOUBLE_SPIN_SLIDER_FUNC(mcv_angle_diff)

DOUBLE_SPIN_SLIDER_FUNC(center_vector_angle_diff)

DOUBLE_SPIN_SLIDER_FUNC(vertical_plane_angle_diff)

DOUBLE_SPIN_SLIDER_FUNC(noleg_distance)

void gui::bt_clicked_read_parameters()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open parameters file"),
                                                    "./",
                                                    tr("yaml file(*.yaml)"));
    if (fileName.isEmpty())
    {
        QMessageBox mesg;
        mesg.warning(this, "Warning", "Open parameters file failed!, using default file ./param.yaml.");
    } else
        param_file = fileName;
    qDebug() << "open param_file: " << param_file;

    cv::FileStorage fsSettings(param_file.toStdString().c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        QMessageBox mesg;
        mesg.critical(this, "Error", tr("Can not open :") + param_file);
//        emit close_window();
        return;
    }

    read_parameters();
}

void gui::read_default_parameters()
{
    cv::FileStorage fsSettings(param_file.toStdString().c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        QMessageBox mesg;
        mesg.warning(this, "Warn", tr("Can not open default parameters file :") + param_file);
//        emit close_window();
        return;
    }
    read_parameters();
}

void gui::read_parameters()
{
    cv::FileStorage fsSettings(param_file.toStdString().c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        QMessageBox mesg;
        mesg.critical(this, "Error", tr("Can not open :") + param_file);
//        emit close_window();
        return;
    }

#define READ_DOUBLE_PARAM(obj)  paramters.obj##_ = fsSettings[#obj];\
                                ui->obj##_doubleSpinBox->setValue(paramters.obj##_);\
                                obj##_doubleSpinBox_set();\
                                std::cout<<"fsSettings["<<#obj<<"]:"<<float(fsSettings[#obj])<<std::endl;
#define READ_INT_PARAM(obj)     paramters.obj##_ = fsSettings[#obj];\
                                ui->obj##_spinBox->setValue(paramters.obj##_);\
                                obj##_spinBox_set();\
                                std::cout<<"fsSettings["<<#obj<<"]:"<<float(fsSettings[#obj])<<std::endl;
    READ_DOUBLE_PARAM(x_min)
    READ_DOUBLE_PARAM(x_max)
    READ_DOUBLE_PARAM(y_min)
    READ_DOUBLE_PARAM(y_max)
    READ_DOUBLE_PARAM(z_min)
    READ_DOUBLE_PARAM(z_max)

    READ_INT_PARAM(normal_compute_points)

    READ_DOUBLE_PARAM(voxel_x)
    READ_DOUBLE_PARAM(voxel_y)
    READ_DOUBLE_PARAM(voxel_z)

    READ_INT_PARAM(parallel_angle_diff)
    READ_INT_PARAM(perpendicular_angle_diff)

    READ_DOUBLE_PARAM(cluster_tolerance)
    READ_INT_PARAM(min_cluster_size)
    paramters.max_cluster_size_ = fsSettings["max_cluster_size"];

    READ_DOUBLE_PARAM(seg_threshold)
    paramters.seg_max_iters_ = fsSettings["seg_max_iters"];
    READ_INT_PARAM(seg_rest_point)
    READ_DOUBLE_PARAM(seg_plane_angle_diff)

    READ_DOUBLE_PARAM(merge_threshold)
    READ_DOUBLE_PARAM(merge_angle_diff)

    READ_INT_PARAM(min_num_points)

    READ_DOUBLE_PARAM(min_length)
    READ_DOUBLE_PARAM(max_length)
    READ_DOUBLE_PARAM(min_width)
    READ_DOUBLE_PARAM(max_width)
    READ_DOUBLE_PARAM(max_g_height)
    READ_DOUBLE_PARAM(counter_max_distance)
    READ_DOUBLE_PARAM(min_height)
    READ_DOUBLE_PARAM(max_height)
    READ_DOUBLE_PARAM(vertical_angle_diff)
    READ_DOUBLE_PARAM(mcv_angle_diff)
    READ_DOUBLE_PARAM(center_vector_angle_diff)
    READ_DOUBLE_PARAM(vertical_plane_angle_diff)
    READ_DOUBLE_PARAM(noleg_distance)
}

void gui::bt_clicked_save_parameters()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save parameters file"),
                                                    "./",
                                                    tr("yaml file(*.yaml)"));

    cv::FileStorage fs(fileName.toStdString().c_str(), cv::FileStorage::WRITE);

    if (!fs.isOpened())
    {
        QMessageBox mesg;
        mesg.critical(this, "Error", tr("Can not save :") + fileName);
        return;
    }

#define SAVE_PARAM(obj) fs<<#obj<<paramters.obj##_;
    SAVE_PARAM(x_min)
    SAVE_PARAM(x_max)
    SAVE_PARAM(y_min)
    SAVE_PARAM(y_max)
    SAVE_PARAM(z_min)
    SAVE_PARAM(z_max)
    SAVE_PARAM(normal_compute_points)
    SAVE_PARAM(voxel_x)
    SAVE_PARAM(voxel_y)
    SAVE_PARAM(voxel_z)
    SAVE_PARAM(parallel_angle_diff)
    SAVE_PARAM(perpendicular_angle_diff)
    SAVE_PARAM(cluster_tolerance)
    SAVE_PARAM(min_cluster_size)
    SAVE_PARAM(max_cluster_size)
    SAVE_PARAM(seg_threshold)
    SAVE_PARAM(seg_max_iters)
    SAVE_PARAM(seg_rest_point)
    SAVE_PARAM(seg_plane_angle_diff)
    SAVE_PARAM(merge_threshold)
    SAVE_PARAM(merge_angle_diff)
    SAVE_PARAM(min_num_points)
    SAVE_PARAM(min_length)
    SAVE_PARAM(max_length)
    SAVE_PARAM(min_width)
    SAVE_PARAM(max_width)
    SAVE_PARAM(max_g_height)
    SAVE_PARAM(counter_max_distance)
    SAVE_PARAM(min_height)
    SAVE_PARAM(max_height)
    SAVE_PARAM(vertical_angle_diff)
    SAVE_PARAM(mcv_angle_diff)
    SAVE_PARAM(center_vector_angle_diff)
    SAVE_PARAM(vertical_plane_angle_diff)
    SAVE_PARAM(noleg_distance)

    fs.release();
}

void gui::edit_finished_imu_serial()
{
    imu_serial = ui->IMU_Serial_lineEdit->text().toStdString();
    std::cout << "imu_serial: " << imu_serial << std::endl;
    emit imu_serial_update(imu_serial);
}

void gui::edit_finished_robot_serial()
{
    robot_serial = ui->robot_serial_lineEdit->text().toStdString();
    std::cout << "robot_serial: " << robot_serial << std::endl;
    emit robot_serial_update(robot_serial);
}

void gui::bt_group_runmode_clicked(int buttonId)
{
    run_mode = RunModeDef(buttonId);
    emit run_mode_update(run_mode);
    std::cout << "run_mode: " << buttonId << std::endl;

    if (run_mode == files_mode)
    {
        ui->start_pushButton->setText(tr("Prev"));
        ui->stop_pushButton->setText(tr("Next"));
        ui->save_img_pushButton->setEnabled(false);
    } else
    {
        ui->start_pushButton->setText(tr("Continue"));
        ui->stop_pushButton->setText(tr("Pause"));
        ui->save_img_pushButton->setEnabled(true);
    }
}

void gui::bt_group_showmode_clicked(int buttonId)
{
    show_mode = ShowModeDef(buttonId);
    emit show_mode_update(show_mode);
    std::cout << "show_mode: " << buttonId << std::endl;
}

void gui::bt_start_clicked()
{
    std::cout << "bt_start_clicked" << std::endl;
    emit start_signal();
}

void gui::bt_stop_clicked()
{
    std::cout << "bt_stop_clicked" << std::endl;
    emit stop_signal();
}

void gui::bt_img_path_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this,
                                                    tr("Image and cloud saving path"),
                                                    "./",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    img_path = dir.toStdString();
    if (img_path != "")
    {
        emit img_path_update(img_path);
        std::cout << "set image and cloud save path: " << img_path << std::endl;
    }
}

void gui::bt_img_save_clicked()
{
    emit save_img_signal();
    std::cout << "save_img_signal" << std::endl;
}

gui::~gui()
{
    delete ui;
}
