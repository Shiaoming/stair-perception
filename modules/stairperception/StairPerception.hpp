/*
 * StairDetection.hpp
 *
 *  Created on: 2017年10月6日
 *      Author: zxm
 */

#ifndef STAIRDETECTION_HPP_
#define STAIRDETECTION_HPP_

//#define DEBUG_CONSOLE

//#define TIME_RECORD

#include "common.h"
#include <time.h>
#include "Stair.hpp"
#include "voxel_grid_indices.h"

typedef struct
{
    int type;// default:-1  horizontal:1  vertical:0
    int index;// index of cloud plane vector
    pcl::PointXYZ center;// planes' center
} IndexMap;

class StairDetection
{
private:
    // preprocess parameters
    // range limit of point cloud after rotation
    float x_min;// = 0.55;
    float x_max;// = 1.5;
    float y_min;// = 0;
    float y_max;// = 1;
    float z_min;// = -0.5;
    float z_max;// = 0.5;

    // the number of points normals computed based on
    // Note: this parameters should be set based on cloud grid size.
    //       in planes, the base radius to compute normals is
    //       sqrt(normal_compute_points/\pi)*normal_compute_points
    int normal_compute_points;// = 70;

    // voxel gird
    float voxel_x,voxel_y,voxel_z;

    // extract perpendicular plane
    float parallel_angle_diff;
    float perpendicular_angle_diff;
    
    // remove the points too close to origin
    double noleg_distance;

    // plane segmentation
    float seg_threshold,seg_plane_angle_diff;
    int seg_rest_point;
    // segmentation max iteration numbers
    int seg_max_iters;// = 100;
    // points with distance to plane smaller than this parameter (units:m)
    // will be segment into this plane

    // merge
    float merge_threshold;
    float merge_angle_diff;

    // find main cluster
    float cluster_tolerance;
    int min_cluster_size,max_cluster_size;

    // min number of points of final planes
    int min_num_points;

    // plane length and width constraint
    float min_length,max_length;
    float min_width,max_width;

    // min distance of horizontal plane counter points
    float counter_max_distance;

    // step height constraint
    float min_height,max_height;

    // direction threshold
    float center_vector_angle_diff,mcv_angle_diff,vertical_angle_diff;

    float max_g_height,vertical_plane_angle_diff;

    bool two_main_vertical_normal;

    Eigen::Matrix3f rotation_matrix;
    pcl::PointCloud<PointType> cloud_rotation;
    pcl::PointCloud<PointType> cloud_ps,cloud_ps_nonan,cloud_no_leg;
    pcl::PointCloud<PointType> cloud_down_sample;
    pcl::PointCloud<PointType> cloud_down_sample_valid;
    pcl::PointCloud<PointType> cloud_h;
    pcl::PointCloud<PointType> cloud_r;
    pcl::PointCloud<PointType> cloud_v;
    pcl::PointCloud<pcl::Normal> normal;
    pcl::PointCloud<pcl::Normal> normal_test;
    pcl::PointCloud<pcl::Normal> normal_ps,normal_ps_nonan,normal_no_leg;
    pcl::PointCloud<pcl::Normal> normal_down_sample;
    pcl::PointCloud<pcl::Normal> normal_down_sample_valid;
    pcl::PointCloud<pcl::Normal> normal_h;
    pcl::PointCloud<pcl::Normal> normal_v;
    std::vector<pcl::PointCloud<PointType> > vector_cloud_h;
    std::vector<pcl::ModelCoefficients> vector_coefficients_h;
    std::vector<pcl::PointCloud<PointType> > vector_cloud_v;
    std::vector<pcl::ModelCoefficients> vector_coefficients_v;
    std::vector<Plane> vector_plane_h,vector_plane_v;
    std::vector<Plane> vector_plane_sorted;
    ConcaveLine *pconcave_line;
    Step *pstep;
    Stair stair;

    Eigen::Vector3f stair_snormal,stair_vnormal;
    pcl::ModelCoefficients slide_plane_left,slide_plane_right;

    pcl::console::TicToc tictoc;

    std::string file_name;
    ofstream ofile;

    typedef struct {Eigen::Vector3f dir;int count;} dircount;

public:
    bool process(const pcl::PointCloud<PointType> &cloud_in,
                    const ShowModeDef show_mode,
                    Stair &stair,
                    std::vector<Plane> &vector_plane_sorted,
                    pcl::PointCloud<PointType> &cloud_show,
                    pcl::PointCloud<pcl::Normal> &normal_show);

    /** \brief print stair model
     * \param[in] stair: stair model
     */
    void printStairModel(Stair &stair);

    void printStairEstParam(Stair &stair);

    // StairDetection():x_min(0.55),x_max(1.5),y_limit_min(0),y_max(1),
    //      z_min(-0.5),z_max(0.5),normal_compute_points(70),seg_rest_point(0.1),
    //      file_name("./time.txt")
    StairDetection(ParameterDef parameters):file_name("./time.txt")
    {
        x_min                   = parameters.x_min_;
        x_max                   = parameters.x_max_;
        y_min                   = parameters.y_min_;
        y_max                   = parameters.y_max_;
        z_min                   = parameters.z_min_;
        z_max                   = parameters.z_max_;
        normal_compute_points   = parameters.normal_compute_points_;
        voxel_x                 = parameters.voxel_x_;
        voxel_y                 = parameters.voxel_y_;
        voxel_z                 = parameters.voxel_z_;
        parallel_angle_diff     = parameters.parallel_angle_diff_;
        perpendicular_angle_diff= parameters.perpendicular_angle_diff_;
        seg_threshold           = parameters.seg_threshold_;
        seg_max_iters           = parameters.seg_max_iters_;
        seg_rest_point          = parameters.seg_rest_point_;
        seg_plane_angle_diff    = parameters.seg_plane_angle_diff_;
        cluster_tolerance       = parameters.cluster_tolerance_;
        min_cluster_size        = parameters.min_cluster_size_;
        max_cluster_size        = parameters.max_cluster_size_;
        min_num_points          = parameters.min_num_points_;
        merge_threshold         = parameters.merge_threshold_;
        merge_angle_diff        = parameters.merge_angle_diff_;
        min_length              = parameters.min_length_;
        max_length              = parameters.max_length_;
        min_width               = parameters.min_width_;
        max_width               = parameters.max_width_;
        counter_max_distance    = parameters.counter_max_distance_;
        min_height              = parameters.min_height_;
        max_height              = parameters.max_height_;
        center_vector_angle_diff= parameters.center_vector_angle_diff_;
        mcv_angle_diff          = parameters.mcv_angle_diff_;
        max_g_height            = parameters.max_g_height_;
        vertical_plane_angle_diff= parameters.vertical_plane_angle_diff_;
        vertical_angle_diff     = parameters.vertical_angle_diff_;
        noleg_distance     = parameters.noleg_distance_;
        #ifdef TIME_RECORD
        ofile.open(file_name.c_str(),std::ios::app);
        #endif
    }

    ~StairDetection()
    {
        #ifdef TIME_RECORD
        ofile.close();
        #endif
    }
private:
    /** \brief rotation clouds
     * \param[in] cloud_in: reference to input cloud
     * \param[in] rotation_matrix: rotation matrix
     * \param[out] cloud_rotation: reference to output cloud
     * \param[in] downsamplerate: downsample rate
     */
    void rotationCloudPoints(const pcl::PointCloud<PointType> &cloud_in,
            const Eigen::Matrix3f rotation_matrix,
            pcl::PointCloud<PointType> &cloud_rotation,
            int downsamplerate);
    
    void remove_nan_points(CloudType &cloud);

    /** \brief PassThrough passes points in a cloud in x,y,z dimensions.
     * \param[in] cloud_in: reference to input cloud
     * \param[out] cloud_out: reference to output cloud
     */
    void passThrough(const pcl::PointCloud<PointType> &cloud_in,
            pcl::PointCloud<PointType> &cloud_out);

    /** \brief Benefited from normals have already estimated, this pass
     * through both points cloud and normals based on there coordinate
     */
    void passThroughCloudANDNormal(const pcl::PointCloud<PointType> &cloud_in,
            const pcl::PointCloud<pcl::Normal> &normal_in,
            pcl::PointCloud<PointType> &cloud_out,
            pcl::PointCloud<pcl::Normal> &normal_out);

    /** \brief downsample the cloud using VoxelGrid filter
     * \param[in] cloud_in: reference to input cloud
     * \param[out] cloud_out: reference to output cloud
     */
    void voxelGrid(const pcl::PointCloud<PointType> &cloud_in,
            float leaf_size_x,float leaf_size_y,float leaf_size_z,
            pcl::PointCloud<PointType> &cloud_out);
    
    void removeClosetPoints(const CloudType &cloud_in,const pcl::PointCloud<pcl::Normal> &normal_in,
        CloudType &cloud_out,pcl::PointCloud<pcl::Normal> &normal_out,float th);

    /** \brief A modified VoxelGrid who returns indices of points
     * in input cloud which closet to there centriod
     */
    void voxelGridCloudANDNormal(const pcl::PointCloud<PointType> &cloud_in,
                const pcl::PointCloud<pcl::Normal> &normal_in,
                float leaf_size_x,float leaf_size_y,float leaf_size_z,
                pcl::PointCloud<PointType> &cloud_out,
                pcl::PointCloud<pcl::Normal> &normal_out);

    /** \brief estimation normals of points
     * \param[in] cloud_in: reference to input cloud
     * \param[in] cloud_in: search surface
     * \param[out] cloud_out: reference to output cloud
     */
    void normalEstimationOMP(const pcl::PointCloud<PointType> &cloud_in,
            const pcl::PointCloud<PointType> &cloud_sufrace,
            pcl::PointCloud<pcl::Normal> &cloud_out);

    void normalEstimationTest(const pcl::PointCloud<PointType> &cloud_in,
                    pcl::PointCloud<pcl::Normal> &normal_out);

    /** \brief Normal estimation using method in paper
     * "Real-Time Plane Segmentation using RGB-D Cameras " by Dirk.
     * This method takes advantages of organized image and is
     * much faster than normal normal estimation method based on
     * unorganized point cloud
     */
    void normalEstimationIntegral(const pcl::PointCloud<PointType> &cloud_in,
                    pcl::PointCloud<pcl::Normal> &normal_out);

    /** \brief After pass through and voxel grid, all the points is invalid,
     * but there still has some normals still invalid because of normals are
     * computed using "IntegralImageNormalEstimation"
     */
    void removeNANNormalPoints(const pcl::PointCloud<PointType> &cloud_in,
            const pcl::PointCloud<pcl::Normal> &normal_in,
            pcl::PointCloud<PointType> &cloud_out,
            pcl::PointCloud<pcl::Normal> &normal_out);

    void generateNormalSphere(const pcl::PointCloud<pcl::Normal> &normal_in,
            pcl::PointCloud<PointType> &cloud_out);

    /** \brief extract cloud points and normals in parallel plane which perpendicular to specific axis
     * \param[in] cloud_in: input point cloud
     * \param[in] normal_in: input normal cloud
     * \param[in] ax: axis planes perpendicular to
     * \param[in] angle_threshold: angle(rad) threshold point normal with ax
     * \param[out] cloud_parallel: output points cloud
     * \param[out] normal_parallel: output normal cloud
     */
    void extractPerpendicularPlanePoints(const pcl::PointCloud<PointType> &cloud_in,
            const pcl::PointCloud<pcl::Normal> &normal_in,
            const Eigen::Vector3f &ax,
            const double angle_threshold_1,
            const double angle_threshold_2,
            pcl::PointCloud<PointType> &cloud_parallel,
            pcl::PointCloud<pcl::Normal> &normal_parallel,
            pcl::PointCloud<PointType> &cloud_perpendicular,
            pcl::PointCloud<pcl::Normal> &normal_perpendicular
            );

    /** \brief segment cloud points plane which perpendicular to specific axis
     * \param[in] cloud_in: input point cloud
     * \param[in] normal_in: input normal cloud
     * \param[out] vector_cloud: output vector of points cloud
     * \param[out] vector_coefficients: output vector of plane coefficients
     */
    void segmentHorizontalPlanes(const pcl::PointCloud<PointType> &cloud_in,
            const pcl::PointCloud<pcl::Normal> &normal_in,
            const int seg_max_iters,
            const float seg_threshold,
            const float angle_threshold,
            const int min_plane_points,
            std::vector<pcl::PointCloud<PointType> > &vector_cloud,
            std::vector<pcl::ModelCoefficients> &vector_coefficients,
            std::vector<Plane> &vector_plane_sorted
            );

    /** \brief segment cloud points plane which perpendicular to specific axis
     * \param[in] cloud_in: input point cloud
     * \param[in] normal_in: input normal cloud
     * \param[out] vector_cloud: output vector of points cloud
     * \param[out] vector_coefficients: output vector of plane coefficients
     */
    void segmentVerticalPlanes(const pcl::PointCloud<PointType> &cloud_in,
            const int seg_max_iters,
            const float seg_threshold,
            const float angle_threshold,
            const int min_plane_points,
            std::vector<pcl::PointCloud<PointType> > &vector_cloud,
            std::vector<pcl::ModelCoefficients> &vector_coefficients,
            std::vector<Plane> &vector_plane_sorted
            );

    void pointCluster(const pcl::PointCloud<PointType> &cloud_in,
            std::vector<pcl::PointCloud<PointType> > &vec_cluster);

    void mergeSeperatedPlanes(std::vector<pcl::PointCloud<PointType> > &vector_cloud,
            std::vector<pcl::ModelCoefficients> &vector_coefficients, 
            std::vector<Plane> &merge_planes,
            const double angle_threshold,const float seg_threshold);
    
    /** \brief compute cross line of two planes
     * \param[in] coefficients_plane1: input plane coefficients
     * \param[in] coefficients_plane2: input plane coefficients
     * \param[out] coefficients_line: output line coefficients
     */
    void computeLineFrom2Planes(
            const pcl::ModelCoefficients &coefficients_plane1,
            const pcl::ModelCoefficients &coefficients_plane2,
            Line &line);

    /** \brief compute rotation matrix based on euler angle
     * \detail the rotation order is ZXY, and the IMU is right
     * hand coordinate system, so there is a negative before pitch.
     * the XYZ of kinect V2 is correspond with ZXY of IMU
     * \param[in] pitch: pitch
     * \param[in] roll: roll
     * \param[in] yaw: yaw
     * \param[out] rotation_matrix: rotation matrix
     */
    void computeRotationMatrix(const float pitch, const float roll, const float yaw,
            Eigen::Matrix3f &rotation_matrix);

    /** \brief rotation point with rotation matrix
     * \param[in] point_in: input point
     * \param[in] rotation_matrix: rotation matrix
     * \param[out] point_out: point after rotation
     */
    inline void pointRotation(const PointType &point_in,PointType &point_out,
            const Eigen::Matrix3f &rotation_matrix);

    /** \brief gather and compute all the information of a plane into Plane struct
     * \param[in] vector_cloud_in: a vector of planes cloud point
     * \param[in] vector_plane_coefficients_in: a vector of planes coefficients
     * \param[in] type: a enum indicate the type of plane (horizontal or vertical)
     * \param[out] vector_plane: a vector of planes
     */
    void computeVectorPlaneInfo(const std::vector<pcl::PointCloud<PointType> > &vector_cloud_in,
            const std::vector<pcl::ModelCoefficients> &vector_plane_coefficients_in,
            const Plane::Type &type,
            std::vector<Plane> &vector_plane);

    void countPlanesNormal(const pcl::ModelCoefficients &coefficients,
            Plane::Type type,
            std::vector<normalcount> &plane_normal_count);

    /** \brief sort planes using X value (indicate the height to the ground)
     * \param[in] vector_plane_h_in: a vector of horizontal planes
     * \param[in] vector_plane_v_in: a vector of vertical planes
     * \param[out] vector_plane_sorted: a vector of sorted planes
     */
    void sortPlanesByXValues(const std::vector<Plane> &vector_plane_h_in,
            const std::vector<Plane> &vector_plane_v_in,
            std::vector<Plane> &vector_plane_sorted);

    float minDistaceOfTwoCloud(const Plane &plane1,const Plane &plane2);

    static bool sortByDirCount(dircount i,dircount j)
    {
        return i.count > j.count;
    }

    /** \brief model the stair using vector_plane_sorted
     * \param[in] vector_plane_sorted: a vector of all the planes sorted by height to ground
     * \param[out] stair: a link list of stair which indicate the stair model
     * \param[out] return: there is a stair or not
     */
    bool stairModel(std::vector<Plane> &vector_plane_sorted,
            Stair &stair);

    /** \brief find the point whose projection in on vector is max and min
     * \param[in] plane: include input cloud
     * \param[in] vector: (vx,vy,vz),the projection direction
     * \param[out] point: max_point and min_point
     */
    inline void findMinMaxProjPoint(const Plane &plane,
            const float &vx,const float &vy,const float &vz,
            pcl::PointXYZRGBA &max_point,pcl::PointXYZRGBA &min_point);

    /** \brief compute cross point of plane and line
     * \param[in] line: (x-x0)/z=(y-y0)/b=(z-z0)/c
     * \param[in] plnae: A(x-x1)+B(y-y1)+C(z-z1)=0
     * \param[out] point: pc(xc,yc,zc)
     */
    inline void crossPointOfLineAndPlane(
            const float &x0,const float &y0,const float &z0,const float &a,const float &b,const float &c,
            const float &x1,const float &y1,const float &z1,const float &A,const float &B,const float &C,
            pcl::PointXYZ &pc);

    inline void crossPointOfLineAndPlane(
            const float &x0,const float &y0,const float &z0,const float &a,const float &b,const float &c,
            const pcl::ModelCoefficients plane_coefficients,
            pcl::PointXYZ &pc);

    /** \brief cross product of two normal vectors
     * \param[in] normal vector: n0 = (n01,n02,n03)
     * \param[in] normal vector: n1 = (n11,n12,n13)
     * \param[out] normal vector: nc = n0*n1
     */
    inline void crossProduct(
            const float &n01,const float &n02,const float &n03,
            const float &n11,const float &n12,const float &n13,
            float &nc1,float &nc2,float &nc3);

    void computePlaneCounter(Stair &stair);
};



#endif /* STAIRDETECTION_HPP_ */
