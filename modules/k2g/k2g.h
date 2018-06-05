/*
Copyright 2015, Giacomo Dabisias"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author 
Giacomo Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#define WITH_PCL

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#ifdef WITH_PCL

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#endif

#include <opencv2/opencv.hpp>
#include <signal.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <chrono>
#include <Eigen/Core>

#ifdef WITH_SERIALIZATION
#include "serialization.h"
#endif
#ifdef WITH_ROS
#include "ros_impl.h"
#endif

enum Processor
{
    CPU, OPENCL, OPENGL, CUDA
};


class K2G
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

    K2G(bool mirror = false);

    bool start(Processor p = CPU, std::string serial = std::string());

    bool stop();

    libfreenect2::Freenect2Device::IrCameraParams getIrParameters()
    {
        libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
        return ir;
    }

    libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters()
    {
        libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
        return rgb;
    }

    void disableLog()
    {
        logger_ = libfreenect2::getGlobalLogger();
        libfreenect2::setGlobalLogger(nullptr);
    }

    void enableLog()
    {
        libfreenect2::setGlobalLogger(logger_);
    }

    void printParameters();

    void storeParameters();

#ifdef WITH_PCL

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud(const libfreenect2::Frame *rgb, const libfreenect2::Frame *depth,
                                                     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    updateCloud(const libfreenect2::Frame *rgb, const libfreenect2::Frame *depth,
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

#endif

    void shutDown()
    {
        dev_->stop();
        dev_->close();
    }

    void mirror()
    {
        mirror_ != mirror_;
    }

    libfreenect2::SyncMultiFrameListener *getListener()
    {
        return &listener_;
    }

    // Use only if you want only depth, else use get(cv::Mat, cv::Mat) to have the images aligned
    void getDepth(cv::Mat depth_mat);

    void getIr(cv::Mat ir_mat);

    // Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
    void getColor(cv::Mat &color_mat);

    // Depth and color are aligned and registered 
    void get(cv::Mat &color_mat, cv::Mat &depth_mat, const bool full_hd = true, const bool remove_points = false);


    // Depth and color are aligned and registered 
    void get(cv::Mat &color_mat, cv::Mat &depth_mat, cv::Mat &ir_mat, const bool full_hd = true,
             const bool remove_points = false);

#ifdef WITH_PCL

    // All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
    void get(cv::Mat &color_mat, cv::Mat &depth_mat, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
             const bool full_hd = true, const bool remove_points = true);

#endif

#ifdef WITH_SERIALIZATION
    void enableSerialization(){
        serialize_ = true;
    }

    void disableSerialization(){
        serialize_ = false;
    }

    bool serialize_status(){
        return serialize_;
    }
#endif

private:

#ifdef WITH_SERIALIZATION
    void serializeFrames(const cv::Mat & depth, const cv::Mat & color);
#ifdef WITH_PCL

    void serializeCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
#endif
#endif

    void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams &depth_p);

    libfreenect2::Freenect2 freenect2_;
    libfreenect2::Freenect2Device *dev_ = 0;
    libfreenect2::PacketPipeline *pipeline_ = 0;
    libfreenect2::Registration *registration_ = 0;

    libfreenect2::SyncMultiFrameListener listener_;

    libfreenect2::Logger *logger_ = nullptr;
    libfreenect2::FrameMap frames_;
    libfreenect2::Frame undistorted_, registered_, big_mat_;
    Eigen::Matrix<float, 512, 1> colmap;
    Eigen::Matrix<float, 424, 1> rowmap;
    std::string serial_;
    int map_[512 * 424];
    float qnan_;
    bool mirror_;
#ifdef WITH_SERIALIZATION
    bool serialize_;
    std::ofstream * file_streamer_;
    boost::archive::binary_oarchive * oa_;
#endif
};
