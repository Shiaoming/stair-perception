/*
 * common.h
 *
 *  Created on: 2017年10月6日
 *      Author: zxm
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <omp.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

// #include <config.h>

#include <pcl/common/time.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#define ABS(x) ((x)>0?(x):-1*(x))
typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> CloudType;

enum RunModeDef
{
    files_mode = 0,
    kinect_mode,
};

enum ShowModeDef
{
    original = 0,
    range_limited,
    voxel,
    noleg,
    horizontal_cloud,
    vertical_cloud,
    seg_horizontal_plane,
    seg_vertical_plane,
    merge_horizontal_plane,
    merge_vertical_plane,
    stair
};

typedef struct
{
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    double z_min_;
    double z_max_;
    int normal_compute_points_;
    double voxel_x_;
    double voxel_y_;
    double voxel_z_;
    double parallel_angle_diff_;
    double perpendicular_angle_diff_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double seg_threshold_;
    int seg_max_iters_;
    int seg_rest_point_;
    double seg_plane_angle_diff_;
    double merge_threshold_;
    double merge_angle_diff_;
    int min_num_points_;
    double min_length_;
    double max_length_;
    double min_width_;
    double max_width_;
    double max_g_height_;
    double counter_max_distance_;
    double min_height_;
    double max_height_;
    double vertical_angle_diff_;
    double mcv_angle_diff_;
    double center_vector_angle_diff_;
    double vertical_plane_angle_diff_;
    double noleg_distance_;
} ParameterDef;

typedef struct
{
    bool has_stair;
    float height;
    float width;
    float v_height;
    float v_depth;
    double time;
} ResultDef;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} IMUDef;

typedef struct
{
    pcl::PointCloud<PointType> cloud;
    pcl::ModelCoefficients coefficients;
    pcl::PointXYZ center;
    pcl::PointXYZ pcenter;

    pcl::PointXYZ eigen_vectors[3];
    float eigen_values[3];

    // min and max distance in the direction of eigen vector
    float min[3], max[3];
    // and associated points
    pcl::PointXYZRGBA points_min[3], points_max[3];

    // plane counter
    pcl::PointCloud<pcl::PointXYZ> counter;

    typedef enum
    {
        vertical = 0,
        horizontal
    } Type;

    typedef enum
    {
        stair_component = 0,
        pstair_component,
        ground,
        others
    } Ptype;

    Ptype ptype;

    Type type;
} Plane;

typedef struct
{
    Eigen::Vector3f normal;
    int count;
    Plane::Type type;
} normalcount;

#define ptv pcl::PointCloud<PointType>::VectorType

inline ptv operator+(ptv &a, ptv &b)
{
    ptv ret;
    ret.insert(ret.end(), a.begin(), a.end());
    ret.insert(ret.end(), b.begin(), b.end());
    return ret;
}

#endif /* COMMON_H_ */
