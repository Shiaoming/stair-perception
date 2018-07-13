#include "StairPerception.hpp"

// #define DEBUG_STAIR_MODEL

bool StairDetection::process(const pcl::PointCloud<PointType> &cloud_in,
            const ShowModeDef show_mode,
            Stair &stair,
            std::vector<Plane> &vector_plane_sorted,
            pcl::PointCloud<PointType> &cloud_show,
            pcl::PointCloud<pcl::Normal> &normal_show)
{
    bool has_satir = false;

    struct timeval tv;
    gettimeofday(&tv,NULL);
    long long time1 = tv.tv_sec*1000 + tv.tv_usec/1000;

//      #ifdef TIME_RECORD
//        gettimeofday(&tv,NULL);
//      long long t1 = tv.tv_sec*1000 + tv.tv_usec/1000;
//      #endif
//      computeRotationMatrix(pitch,roll,yaw,rotation_matrix);
    
    if(show_mode == original)
        return false;

    if(cloud_in.points.size() > 0)
    {
//          rotationCloudPoints(cloud_in,rotation_matrix,cloud_rotation,4);
        cloud_rotation= cloud_in;
        #ifdef TIME_RECORD
        gettimeofday(&tv,NULL);
        long long t2 = tv.tv_sec*1000 + tv.tv_usec/1000;
        ofile << t2 - t1 << "\t";
        t1 = t2;
        #endif
        
        remove_nan_points(cloud_rotation);
        
        normalEstimationIntegral(cloud_rotation,normal);
        #ifdef TIME_RECORD
        gettimeofday(&tv,NULL);
        t2 = tv.tv_sec*1000 + tv.tv_usec/1000;
        ofile << t2 - t1 << "\t";
        t1 = t2;
        #endif
        passThroughCloudANDNormal(cloud_rotation,normal,cloud_ps,normal_ps);
        
        if(show_mode == range_limited)
        {
            cloud_show = cloud_ps;
            normal_show = normal_ps;
            return false;
        }

        if(cloud_ps.points.size() < 100)
            return has_satir;

        voxelGridCloudANDNormal(cloud_ps,normal_ps,voxel_x,voxel_y,voxel_z,cloud_down_sample,normal_down_sample);                

        removeNANNormalPoints(cloud_down_sample,normal_down_sample,cloud_ps_nonan,normal_ps_nonan);
        
        if(show_mode == voxel)
        {
            cloud_show = cloud_ps_nonan;
            normal_show = normal_ps_nonan;
            return false;
        }
        
        removeClosetPoints(cloud_ps_nonan,normal_ps_nonan,cloud_no_leg,normal_no_leg,noleg_distance);
        if(show_mode == noleg)
        {
            cloud_show = cloud_no_leg;
            normal_show = normal_no_leg;
            return false;
        }

        // extract horizontal cloud and rest
        extractPerpendicularPlanePoints(cloud_no_leg, normal_no_leg, Eigen::Vector3f(1,0,0),
                pcl::deg2rad(parallel_angle_diff),pcl::deg2rad(perpendicular_angle_diff),
                cloud_h,normal_h,cloud_v,normal_v);
        #ifdef TIME_RECORD
        gettimeofday(&tv,NULL);
        t2 = tv.tv_sec*1000 + tv.tv_usec/1000;
        ofile << t2 - t1 << "\t";
        t1 = t2;
        #endif
        
        vector_plane_sorted.clear();
        if(show_mode == horizontal_cloud)
        {
            cloud_show = cloud_h;
            normal_show = normal_h;
            
            Plane plane;
            plane.cloud = cloud_h;
            plane.ptype = Plane::Ptype::stair_component;
            vector_plane_sorted.push_back(plane);
            
            return false;
        }
        
        vector_plane_sorted.clear();
        if(show_mode == vertical_cloud)
        {
            cloud_show = cloud_v;
            normal_show = normal_v;
            
            Plane plane;
            plane.cloud = cloud_v;
            plane.ptype = Plane::Ptype::stair_component;
            vector_plane_sorted.push_back(plane);
            return false;
        }
        
        // segment horizontal Planes
        segmentHorizontalPlanes(cloud_h,normal_h,seg_max_iters,
                seg_threshold,pcl::deg2rad(seg_plane_angle_diff),
                seg_rest_point,vector_cloud_h,vector_coefficients_h
                 ,vector_plane_h
                );
        #ifdef TIME_RECORD
        gettimeofday(&tv,NULL);
        t2 = tv.tv_sec*1000 + tv.tv_usec/1000;
        ofile << t2 - t1 << "\t";
        t1 = t2;
        #endif
        
        vector_plane_sorted.clear();
        if(show_mode == seg_horizontal_plane)
        {
            vector_plane_sorted = vector_plane_h;
            return false;
        }
        
        // segment vertical Planes
        segmentVerticalPlanes(cloud_v,seg_max_iters,seg_threshold,
                pcl::deg2rad(seg_plane_angle_diff),seg_rest_point,
                vector_cloud_v,vector_coefficients_v
                ,vector_plane_v
                );
        #ifdef TIME_RECORD
        gettimeofday(&tv,NULL);
        t2 = tv.tv_sec*1000 + tv.tv_usec/1000;
        ofile << t2 - t1 << "\t";
        t1 = t2;
        #endif
        
        vector_plane_sorted.clear();
        if(show_mode == seg_vertical_plane)
        {
            vector_plane_sorted = vector_plane_v;
            return false;
        }

        std::vector<Plane> merge_plane_h;
        mergeSeperatedPlanes(vector_cloud_h,vector_coefficients_h,merge_plane_h,
                pcl::deg2rad(merge_angle_diff),merge_threshold);
        
        vector_plane_sorted.clear();
        if(show_mode == merge_horizontal_plane)
        {
            vector_plane_sorted = merge_plane_h;
            return false;
        }

        std::vector<Plane> merge_plane_v;
        mergeSeperatedPlanes(vector_cloud_v,vector_coefficients_v,merge_plane_v,
                pcl::deg2rad(merge_angle_diff),merge_threshold);
        
        vector_plane_sorted.clear();
        if(show_mode == merge_vertical_plane)
        {
            vector_plane_sorted = merge_plane_v;
            return false;
        }
        

        if((vector_cloud_v.size() != 0 ) || (vector_cloud_h.size() != 0))
        {
            vector_plane_h.clear();
            vector_plane_v.clear();
            vector_plane_sorted.clear();
            // compute plane center, MAX and MIN y,z, gather all the info to one struct Plane
            computeVectorPlaneInfo(vector_cloud_h,vector_coefficients_h,Plane::horizontal,vector_plane_h);
            computeVectorPlaneInfo(vector_cloud_v,vector_coefficients_v,Plane::vertical,vector_plane_v);

            sortPlanesByXValues(vector_plane_h,vector_plane_v,vector_plane_sorted);

            has_satir = stairModel(vector_plane_sorted,stair);

            if(has_satir)
                computePlaneCounter(stair);
            #ifdef TIME_RECORD
            gettimeofday(&tv,NULL);
            t2 = tv.tv_sec*1000 + tv.tv_usec/1000;
            ofile << t2 - t1 << "\t";
            #endif

            #ifdef TIME_RECORD
            ofile << time2-time1 << std::endl;
            #endif
        }

        gettimeofday(&tv,NULL);
        long long time2 = tv.tv_sec*1000 + tv.tv_usec/1000;
        if(has_satir)
            std::cout << YELLOW << "Total process time:" << time2-time1 << "ms\t" << GREEN << "Find stair!" << RESET << std::endl;
        else
            std::cout << YELLOW << "Total process time:" << time2-time1 << "ms\t" << RED << "No stair!" << RESET << std::endl;
    }

    return has_satir;
}

/** \brief print stair model
    * \param[in] stair: stair model
    */
void StairDetection::printStairModel(Stair &stair)
{
    Node* pnext;
    pnext = stair.getHead();

    std::cout << GREEN << "Stair Model:" << RESET << std::endl;
    while(true)
    {
        if(pnext->pnext_type == step_point)
        {
            Step *pstep = (Step*)(pnext->pnext);
            std::cout << "[step" << pstep->count << "]:" << std::endl;
            std::cout << "point cloud vertical" << std::endl;
            std::cout << "\tpoints number: " << pstep->plane_v->cloud.points.size()<< std::endl;
            std::cout << "\theight: " << pstep->height << std::endl;
            std::cout << "point cloud horizontal" << std::endl;
            std::cout << "\tpoints number: " << pstep->plane_h->cloud.points.size()<< std::endl;
            std::cout << "\tdepth:  " << pstep->depth << std::endl;
            std::cout << "convex line: " << std::endl;
            std::cout << "\t" << "coeff: ";
            for(size_t i = 0; i < pstep->line.coeff.values.size(); i++)
                std::cout << pstep->line.coeff.values[i] << " ";
            std::cout << std::endl << "\th:" << pstep->line.h ;
            std::cout << std::endl << "\td:" << pstep->line.d ;
            std::cout << std::endl;
        }
        else if(pnext->pnext_type == concaveline_point)
        {
            ConcaveLine *pline = (ConcaveLine*)(pnext->pnext);
            std::cout << "[concave line]: " << std::endl;
            std::cout << "\t" << "coeff: ";
            for(size_t i = 0; i < pline->line.coeff.values.size(); i++)
                std::cout << pline->line.coeff.values[i] << " ";
            std::cout << std::endl << "\th: " << pline->line.h;
            std::cout << std::endl << "\td: " << pline->line.d;
            std::cout << std::endl;
        }
        else
        {
            break;
        }
        pnext = pnext->pnext;
    }
}


void StairDetection::printStairEstParam(Stair &stair)
{
    Node* pnext;
    pnext = stair.getHead();

    std::cout << GREEN << "Stair Model:" << RESET << std::endl;
    std::cout << "Height\t\tDepth\tV_Height\tV_Depth" << std::endl;
    while(true)
    {
        if(pnext->pnext_type == step_point)
        {
            Step *pstep = (Step*)(pnext->pnext);
            std::cout << pstep->height << "\t" << pstep->depth <<"\t" << pstep->line.h << "\t" << pstep->line.d << std::endl;
        }
        else if(pnext->pnext_type == concaveline_point)
        {
            ConcaveLine *pline = (ConcaveLine*)(pnext->pnext);
//            std::cout << pstep->line.h << "\t" << pstep->line.d << std::endl;
        }
        else
        {
            break;
        }
        pnext = pnext->pnext;
    }
}

/** \brief rotation clouds
    * \param[in] cloud_in: reference to input cloud
    * \param[in] rotation_matrix: rotation matrix
    * \param[out] cloud_rotation: reference to output cloud
    * \param[in] downsamplerate: downsample rate
    */
void StairDetection::rotationCloudPoints(const pcl::PointCloud<PointType> &cloud_in,
        const Eigen::Matrix3f rotation_matrix,
        pcl::PointCloud<PointType> &cloud_rotation,
        int downsamplerate)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Rotation Cloud Points");
    #endif

    size_t height = cloud_in.height;
    size_t width = cloud_in.width;
    size_t width_out = width/downsamplerate;
    size_t height_out = height/downsamplerate;
    cloud_rotation.resize(width_out*height_out);

    for(size_t i = 0; i < height; i = i + downsamplerate)
    {
        for(size_t j = 0; j < width; j = j + downsamplerate)
        {
            // same as function "pointRotation", but this is for cheaper compute time
            cloud_rotation.points[i*width/downsamplerate/downsamplerate + j/downsamplerate] =
                    cloud_in.points[i*width + j];
            cloud_rotation.points[i*width/downsamplerate/downsamplerate + j/downsamplerate].x =
                    rotation_matrix(0,0)*cloud_in.points[i*width + j].x+
                    rotation_matrix(0,1)*cloud_in.points[i*width + j].y+
                    rotation_matrix(0,2)*cloud_in.points[i*width + j].z;
            cloud_rotation.points[i*width/downsamplerate/downsamplerate + j/downsamplerate].y =
                    rotation_matrix(1,0)*cloud_in.points[i*width + j].x+
                    rotation_matrix(1,1)*cloud_in.points[i*width + j].y+
                    rotation_matrix(1,2)*cloud_in.points[i*width + j].z;
            cloud_rotation.points[i*width/downsamplerate/downsamplerate + j/downsamplerate].z =
                    rotation_matrix(2,0)*cloud_in.points[i*width + j].x+
                    rotation_matrix(2,1)*cloud_in.points[i*width + j].y+
                    rotation_matrix(2,2)*cloud_in.points[i*width + j].z;
        }
    }
    cloud_rotation.width = width_out;
    cloud_rotation.height = height_out;
}

void StairDetection::remove_nan_points(CloudType& cloud)
{
    size_t height = cloud.height;
    size_t width = cloud.width;
    for(size_t i = 0; i < height; i++)
    {
        for(size_t j = 0; j < width; j++)
        {
            if(cloud.points[i*width + j].r == 0 ||
                cloud.points[i*width + j].g == 0 ||
                cloud.points[i*width + j].b == 0)
            {
                cloud.points[i*width + j].x = std::numeric_limits<float>::quiet_NaN();
                cloud.points[i*width + j].y = std::numeric_limits<float>::quiet_NaN();
                cloud.points[i*width + j].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
}


/** \brief PassThrough passes points in a cloud in x,y,z dimensions.
    * \param[in] cloud_in: reference to input cloud
    * \param[out] cloud_out: reference to output cloud
    */
void StairDetection::passThrough(const pcl::PointCloud<PointType> &cloud_in,
        pcl::PointCloud<PointType> &cloud_out)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Pass Through");
    #endif

    pcl::PointCloud<PointType>::Ptr tmp_cloud_ptr(
            new pcl::PointCloud<PointType>);

    // passthrough filter
    pcl::PassThrough<PointType> pass_through;
    pass_through.setInputCloud(cloud_in.makeShared());
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(x_min, x_max);
    pass_through.filter(*tmp_cloud_ptr);

    pass_through.setInputCloud(tmp_cloud_ptr);
    pass_through.setFilterFieldName("y");
    pass_through.setFilterLimits(y_min, y_max);
    pass_through.filter(*tmp_cloud_ptr);

    pass_through.setInputCloud(tmp_cloud_ptr);
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(z_min, z_max);
    pass_through.filter(cloud_out);
}

/** \brief Benefited from normals have already estimated, this pass
    * through both points cloud and normals based on there coordinate
    */
void StairDetection::passThroughCloudANDNormal(const pcl::PointCloud<PointType> &cloud_in,
        const pcl::PointCloud<pcl::Normal> &normal_in,
        pcl::PointCloud<PointType> &cloud_out,
        pcl::PointCloud<pcl::Normal> &normal_out)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Pass Through cloud and normal");
    #endif

    pcl::PointCloud<PointType>::Ptr tmp_cloud_ptr(
            new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::Normal>::Ptr tmp_normal_ptr(
                    new pcl::PointCloud<pcl::Normal>);

    pcl::ExtractIndices<PointType> extract_points;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    // passthrough filter x
    indices->indices.clear();
    pcl::PassThrough<PointType> pass_through;
    pass_through.setInputCloud(cloud_in.makeShared());
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(x_min, x_max);
    pass_through.filter(indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(cloud_in.makeShared());
    extract_points.setNegative(false);
    extract_points.filter(*tmp_cloud_ptr);

    extract_normals.setIndices(indices);
    extract_normals.setInputCloud(normal_in.makeShared());
    extract_normals.setNegative(false);
    extract_normals.filter(*tmp_normal_ptr);

    // passthrough filter y
    indices->indices.clear();
    pass_through.setInputCloud(tmp_cloud_ptr);
    pass_through.setFilterFieldName("y");
    pass_through.setFilterLimits(y_min, y_max);
    pass_through.filter(indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(tmp_cloud_ptr);
    extract_points.setNegative(false);
    extract_points.filter(*tmp_cloud_ptr);

    extract_normals.setIndices(indices);
    extract_normals.setInputCloud(tmp_normal_ptr);
    extract_normals.setNegative(false);
    extract_normals.filter(*tmp_normal_ptr);

    // passthrough filter z
    indices->indices.clear();
    pass_through.setInputCloud(tmp_cloud_ptr);
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(z_min, z_max);
    pass_through.filter(indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(tmp_cloud_ptr);
    extract_points.setNegative(false);
    extract_points.filter(cloud_out);

    extract_normals.setIndices(indices);
    extract_normals.setInputCloud(tmp_normal_ptr);
    extract_normals.setNegative(false);
    extract_normals.filter(normal_out);

    cloud_out.is_dense = false;
}

/** \brief downsample the cloud using VoxelGrid filter
    * \param[in] cloud_in: reference to input cloud
    * \param[out] cloud_out: reference to output cloud
    */
void StairDetection::voxelGrid(const pcl::PointCloud<PointType> &cloud_in,
        float leaf_size_x,float leaf_size_y,float leaf_size_z,
        pcl::PointCloud<PointType> &cloud_out)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Voxel Grid");
    #endif

    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud_in.makeShared());
    voxel_grid.setLeafSize(leaf_size_x, leaf_size_y,leaf_size_z);
    voxel_grid.filter(cloud_out);
}

/** \brief A modified VoxelGrid who returns indices of points
    * in input cloud which closet to there centriod
    */
void StairDetection::voxelGridCloudANDNormal(const pcl::PointCloud<PointType> &cloud_in,
            const pcl::PointCloud<pcl::Normal> &normal_in,
            float leaf_size_x,float leaf_size_y,float leaf_size_z,
            pcl::PointCloud<PointType> &cloud_out,
            pcl::PointCloud<pcl::Normal> &normal_out)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Voxel Grid Cloud And Normals");
    #endif

    pcl::ExtractIndices<PointType> extract_points;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    pcl::VoxelGridIndices<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud_in.makeShared());
    voxel_grid.setLeafSize(leaf_size_x, leaf_size_y,leaf_size_z);
    voxel_grid.filter(indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(cloud_in.makeShared());
    extract_points.setNegative(false);
    extract_points.filter(cloud_out);

    extract_normals.setIndices(indices);
    extract_normals.setInputCloud(normal_in.makeShared());
    extract_normals.setNegative(false);
    extract_normals.filter(normal_out);
}

/** \brief estimation normals of points
    * \param[in] cloud_in: reference to input cloud
    * \param[in] cloud_in: search surface
    * \param[out] cloud_out: reference to output cloud
    */
void StairDetection::normalEstimationOMP(const pcl::PointCloud<PointType> &cloud_in,
        const pcl::PointCloud<PointType> &cloud_sufrace,
        pcl::PointCloud<pcl::Normal> &cloud_out)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Normal Estimation OMP");
    #endif

    pcl::search::KdTree<PointType>::Ptr tree(
            new pcl::search::KdTree<PointType>());
    // Estimate point normals
    pcl::NormalEstimationOMP<PointType, pcl::Normal> ne(4);//4 threads
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in.makeShared());
    ne.setSearchSurface(cloud_sufrace.makeShared());
    ne.setKSearch(normal_compute_points);
    ne.compute(cloud_out);
}

void StairDetection::normalEstimationTest(const pcl::PointCloud<PointType> &cloud_in,
                pcl::PointCloud<pcl::Normal> &normal_out)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Normal Estimation Test");
    #endif

#define POINT(i,j) cloud_in.points[h*i+j].x,cloud_in.points[h*i+j].y,cloud_in.points[h*i+j].z

    size_t h = cloud_in.height;
    size_t w = cloud_in.width;

    normal_out.header = cloud_in.header;
    normal_out.height = cloud_in.height;
    normal_out.width = cloud_in.width;
    normal_out.is_dense = true;
    normal_out.points.resize(cloud_in.height*cloud_in.width);

    if(cloud_in.isOrganized())
    {
        for(size_t i = 0; i < h; i++)
        {
            for(size_t j = 0; j < w; j++)
            {
                //Eigen::Vector3f v1(POINT(i,j));
            }
        }
    }
}

/** \brief Normal estimation using method in paper
    * "Real-Time Plane Segmentation using RGB-D Cameras " by Dirk.
    * This method takes advantages of organized image and is
    * much faster than normal normal estimation method based on
    * unorganized point cloud
    */
void StairDetection::normalEstimationIntegral(const pcl::PointCloud<PointType> &cloud_in,
                pcl::PointCloud<pcl::Normal> &normal_out)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Normal Estimation Integral");
    #endif

    pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(5.0f);
    ne.setInputCloud(cloud_in.makeShared());
    ne.compute(normal_out);
}

/** \brief After pass through and voxel grid, all the points is invalid,
    * but there still has some normals still invalid because of normals are
    * computed using "IntegralImageNormalEstimation"
    */
void StairDetection::removeNANNormalPoints(const pcl::PointCloud<PointType> &cloud_in,
        const pcl::PointCloud<pcl::Normal> &normal_in,
        pcl::PointCloud<PointType> &cloud_out,
        pcl::PointCloud<pcl::Normal> &normal_out)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Remove NAN Normal Points");
    #endif

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::ExtractIndices<PointType> extract_points;

    removeNaNNormalsFromPointCloud(normal_in,normal_out,indices->indices);

    extract_points.setIndices(indices);
    extract_points.setInputCloud(cloud_in.makeShared());
    extract_points.setNegative(false);
    extract_points.filter(cloud_out);
}

// remove points too close
void StairDetection::removeClosetPoints(const CloudType& cloud_in, const pcl::PointCloud<pcl::Normal>& normal_in, 
                                        CloudType& cloud_out, pcl::PointCloud<pcl::Normal>& normal_out, float th)
{
    PointType point;
    pcl::Normal normal;
    for(size_t i = 0; i < cloud_in.points.size(); i++)
    {
        point = cloud_in.points[i];
        if( point.x * point.x + point.y * point.y + point.y * point.y > th * th)
        {
            normal = normal_in.points[i];
            cloud_out.points.push_back(point);
            normal_out.points.push_back(normal);
        }
    }
    
    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
    normal_out.width = normal_out.points.size();
    normal_out.height = 1;
}


void StairDetection::generateNormalSphere(const pcl::PointCloud<pcl::Normal> &normal_in,
            pcl::PointCloud<PointType> &cloud_out)
{
    PointType point;
    pcl::Normal normal;
    cloud_out.points.clear();
    for (size_t i = 0; i < normal_in.points.size(); i++)
    {
        normal = normal_in.points[i];
        point.x = normal.normal_x;
        point.y = normal.normal_y;
        point.z = normal.normal_z;
        point.rgba=0xFFFFFFFF;

        cloud_out.points.push_back(point);
    }
    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
}

/** \brief extract cloud points and normals in parallel plane which perpendicular to specific axis
    * \param[in] cloud_in: input point cloud
    * \param[in] normal_in: input normal cloud
    * \param[in] ax: axis planes perpendicular to
    * \param[in] angle_threshold: angle(rad) threshold point normal with ax
    * \param[out] cloud_parallel: output points cloud
    * \param[out] normal_parallel: output normal cloud
    */
void StairDetection::extractPerpendicularPlanePoints(const pcl::PointCloud<PointType> &cloud_in,
        const pcl::PointCloud<pcl::Normal> &normal_in,
        const Eigen::Vector3f &ax,
        const double angle_threshold_1,
        const double angle_threshold_2,
        pcl::PointCloud<PointType> &cloud_parallel,
        pcl::PointCloud<pcl::Normal> &normal_parallel,
        pcl::PointCloud<PointType> &cloud_perpendicular,
        pcl::PointCloud<pcl::Normal> &normal_perpendicular
        )
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Extract Perpendicular PlanePoints");
    #endif

    Eigen::Vector3f point_normal;
    double angle;
    for (size_t i = 0; i < normal_in.points.size(); i++)
    {
        point_normal = Eigen::Vector3f(normal_in.points[i].normal_x,
                normal_in.points[i].normal_y, normal_in.points[i].normal_z);

        angle = acos(point_normal.dot(ax));

        if ((angle < angle_threshold_1) || (M_PI - angle < angle_threshold_1))
        {
            cloud_parallel.points.push_back(cloud_in.points[i]);
            normal_parallel.points.push_back(normal_in.points[i]);
        }
        else if(fabs(angle - M_PI/2) < angle_threshold_2)
        {
            cloud_perpendicular.points.push_back(cloud_in.points[i]);
            normal_perpendicular.points.push_back(normal_in.points[i]);
        }
    }
    cloud_parallel.width = cloud_parallel.points.size();
    cloud_parallel.height = 1;
    normal_parallel.width = normal_parallel.points.size();
    normal_parallel.height = 1;
    cloud_perpendicular.width = cloud_perpendicular.points.size();
    cloud_perpendicular.height = 1;
    normal_perpendicular.width = normal_perpendicular.points.size();
    normal_perpendicular.height = 1;
}

/** \brief segment cloud points plane which perpendicular to specific axis
    * \param[in] cloud_in: input point cloud
    * \param[in] normal_in: input normal cloud
    * \param[out] vector_cloud: output vector of points cloud
    * \param[out] vector_coefficients: output vector of plane coefficients
    */
void StairDetection::segmentHorizontalPlanes(const pcl::PointCloud<PointType> &cloud_in,
        const pcl::PointCloud<pcl::Normal> &normal_in,
        const int seg_max_iters,
        const float seg_threshold,
        const float angle_threshold,
        const int min_plane_points,
        std::vector<pcl::PointCloud<PointType> > &vector_cloud,
        std::vector<pcl::ModelCoefficients> &vector_coefficients
         ,std::vector<Plane> &vector_plane_sorted
        )
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Segment Horizontal Planes");
    #endif

    pcl::PointCloud<PointType> rest_points = cloud_in;
//      pcl::PointCloud<pcl::Normal> rest_normals = normal_in;

    if(rest_points.points.size() == 0)
        return;

    std::vector<pcl::ModelCoefficients> vector_plane_coefficients;
    std::vector<pcl::PointIndices> vector_plane_inliers;
    // segmentation object
//      pcl::SACSegmentationFromNormals<PointType,pcl::Normal> seg;
//      seg.setOptimizeCoefficients(true);
//      seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//      seg.setMaxIterations (seg_max_iters);
//      seg.setNormalDistanceWeight (pcl::deg2rad(10.0));
//      seg.setMethodType (pcl::SAC_RANSAC);
//      seg.setDistanceThreshold (seg_threshold);
//      seg.setAxis(Eigen::Vector3f(1,0,0));
//      seg.setEpsAngle(angle_threshold);

    // Create the segmentation object
    pcl::SACSegmentation<PointType> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(seg_max_iters);
    seg.setDistanceThreshold (seg_threshold);
    seg.setAxis(Eigen::Vector3f(1,0,0));
    seg.setEpsAngle(angle_threshold);

    // extract indices object
    pcl::ExtractIndices<PointType> extract_points;
//      pcl::ExtractIndices<pcl::Normal> extract_normal;

    std::vector<pcl::PointCloud<PointType> > vector_cluster;
    pointCluster(rest_points,vector_cluster);
    for(int i=0;i<vector_cluster.size();i++)
    {
        pcl::PointCloud<PointType> iter_points = vector_cluster[i];
        for(;;)
        {
            if(iter_points.points.size() < min_plane_points)
                break;

            pcl::PointCloud<PointType> plane_inliers_points;
            pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
            pcl::ModelCoefficients plane_coefficients;
            seg.setInputCloud(iter_points.makeShared());
            seg.segment(*plane_inliers, plane_coefficients);

            if(plane_inliers->indices.size() < min_plane_points)
                break;

            extract_points.setInputCloud(iter_points.makeShared());
            extract_points.setIndices(plane_inliers);
            extract_points.setNegative(false);
            extract_points.filter(plane_inliers_points);
            extract_points.setNegative(true);
            extract_points.filter(iter_points);

            vector_cloud.push_back(plane_inliers_points);
            vector_coefficients.push_back(plane_coefficients);

            Plane plane;
            plane.cloud = plane_inliers_points;
            plane.ptype = Plane::Ptype::stair_component;
            vector_plane_sorted.push_back(plane);

            if(iter_points.points.size() < min_plane_points)
                break;
        }
    }
}

/** \brief segment cloud points plane which perpendicular to specific axis
    * \param[in] cloud_in: input point cloud
    * \param[in] normal_in: input normal cloud
    * \param[out] vector_cloud: output vector of points cloud
    * \param[out] vector_coefficients: output vector of plane coefficients
    */
void StairDetection::segmentVerticalPlanes(const pcl::PointCloud<PointType> &cloud_in,
        const int seg_max_iters,
        const float seg_threshold,
        const float angle_threshold,
        const int min_plane_points,
        std::vector<pcl::PointCloud<PointType> > &vector_cloud,
        std::vector<pcl::ModelCoefficients> &vector_coefficients,
        std::vector<Plane> &vector_plane_sorted
        )
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Segment Vertical Planes");
    #endif

    pcl::PointCloud<PointType> rest_points = cloud_in;

    if(rest_points.points.size() == 0)
        return;

    std::vector<pcl::ModelCoefficients> vector_plane_coefficients;
    std::vector<pcl::PointIndices> vector_plane_inliers;
    // segmentation object
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setAxis(Eigen::Vector3f(1,0,0));
    seg.setEpsAngle(angle_threshold);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(seg_max_iters);
    seg.setDistanceThreshold(seg_threshold);

    // extract indices object
    pcl::ExtractIndices<PointType> extract_points;

    std::vector<pcl::PointCloud<PointType> > vector_cluster;
    pointCluster(rest_points,vector_cluster);
    for(int i=0;i<vector_cluster.size();i++)
    {
        pcl::PointCloud<PointType> iter_points = vector_cluster[i];
        for(;;)
        {
            pcl::PointCloud<PointType> plane_inliers_points;
            pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
            pcl::ModelCoefficients plane_coefficients;
            seg.setInputCloud(iter_points.makeShared());
            seg.segment(*plane_inliers, plane_coefficients);

            if(plane_inliers->indices.size() < min_plane_points)
                    break;

            extract_points.setInputCloud(iter_points.makeShared());
            extract_points.setIndices(plane_inliers);
            extract_points.setNegative(false);
            extract_points.filter(plane_inliers_points);
            extract_points.setNegative(true);
            extract_points.filter(iter_points);

            vector_cloud.push_back(plane_inliers_points);
            vector_coefficients.push_back(plane_coefficients);

            Plane plane;
            plane.cloud = plane_inliers_points;
            plane.ptype = Plane::Ptype::stair_component;
            vector_plane_sorted.push_back(plane);

            if(iter_points.points.size() < min_plane_points)
                break;
        }
    }
//      // some condition
//      for (;;)
//      {
//          pcl::PointCloud<PointType> plane_inliers_points;
//          pcl::ModelCoefficients plane_coefficients;
//          pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
//          pcl::PointIndices::Ptr main_cluster(new pcl::PointIndices);
//          pcl::PointCloud<PointType> main_cluster_points;
//          pcl::PointCloud<PointType> main_cluster_rest_points;
//
////            findMainCluster(rest_points,*main_cluster);
//          extract_points.setInputCloud(rest_points.makeShared());
//          extract_points.setIndices(main_cluster);
//          extract_points.setNegative(false);
//          extract_points.filter(main_cluster_points);
//          if(main_cluster_points.points.size() < min_plane_points)
//              break;
//          extract_points.setNegative(true);
//          extract_points.filter(main_cluster_rest_points);
//
//          seg.setInputCloud(main_cluster_points.makeShared());
//          seg.segment(*plane_inliers, plane_coefficients);
//
//          if(plane_inliers->indices.size() < min_plane_points)
//              break;
//
//          extract_points.setInputCloud(main_cluster_points.makeShared());
//          extract_points.setIndices(plane_inliers);
//          extract_points.setNegative(false);
//          extract_points.filter(plane_inliers_points);
//          extract_points.setNegative(true);
//          extract_points.filter(rest_points);
//
//          rest_points.points = rest_points.points + main_cluster_rest_points.points;
//          rest_points.width = rest_points.points.size();
//          rest_points.height = 1;
//
//          if(rest_points.points.size() < min_plane_points)
//              break;
//
//          Plane plane;
//          plane.cloud = plane_inliers_points;
//          vector_plane_sorted.push_back(plane);
//
//          vector_cloud.push_back(plane_inliers_points);
//          vector_coefficients.push_back(plane_coefficients);
//      }
}

void StairDetection::pointCluster(const pcl::PointCloud<PointType> &cloud_in,
        std::vector<pcl::PointCloud<PointType> > &vec_cluster)
{
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (cloud_in.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (cluster_tolerance); // 7cm
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in.makeShared());
    ec.extract (cluster_indices);

    pcl::ExtractIndices<PointType> extract_points;
    for(int i=0; i < cluster_indices.size();i++)
    {
        pcl::PointCloud<PointType> point_cloud;
        extract_points.setInputCloud(cloud_in.makeShared());
        extract_points.setNegative(false);
        pcl::PointIndicesPtr indices(new pcl::PointIndices);
        indices->indices=cluster_indices[i].indices;
        extract_points.setIndices(indices);
        extract_points.filter(point_cloud);

        vec_cluster.push_back(point_cloud);
    }
}

void StairDetection::mergeSeperatedPlanes(std::vector<pcl::PointCloud<PointType> > &vector_cloud,
        std::vector<pcl::ModelCoefficients> &vector_coefficients, std::vector<Plane> &merge_planes,
        const double angle_threshold,const float seg_threshold)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Merge Seperated Planes");
    #endif
    
    int sample_point_num = 20;
    
    std::vector<std::vector<PointType>> vec_plane_sample_points;
    
    // random sample sample_point_num points from each point cloud
    for(int i=0;i<vector_cloud.size();i++)
    {
        std::vector<PointType> vec_sample_points;
        
        for(size_t j = 0; j < sample_point_num; j++)
        {
            PointType random_sample_points;
            random_sample_points = vector_cloud[i].points[ rand() % vector_cloud.size() ];
            vec_sample_points.push_back(random_sample_points);
        }

        vec_plane_sample_points.push_back(vec_sample_points);
    }

    for(int i=0;i<vector_coefficients.size();i++)
    {
        pcl::PointCloud<PointType> cloud1;
        pcl::ModelCoefficients coeff1;
        std::vector<PointType> random_sample_points1;
        cloud1 = vector_cloud[i];
        coeff1 = vector_coefficients[i];
        random_sample_points1 = vec_plane_sample_points[i];
        Eigen::Vector3f normal1(coeff1.values[0],coeff1.values[1],coeff1.values[2]);
        for(int j=i+1;j<vector_coefficients.size();j++)
        {
            pcl::PointCloud<PointType> cloud2;
            pcl::ModelCoefficients coeff2;
            std::vector<PointType> random_sample_points2;
            cloud2 = vector_cloud[j];
            coeff2 = vector_coefficients[j];
            random_sample_points2 = vec_plane_sample_points[j];
            Eigen::Vector3f normal2(coeff2.values[0],coeff2.values[1],coeff2.values[2]);

            float dot_product = normal1.dot(normal2);
            float angle = acos(dot_product);
            if((angle < angle_threshold) || (M_PI - angle < angle_threshold))
            {
                float a1,b1,c1,d1,px,py,pz;
                float a2,b2,c2,d2,distance;
                int index;
                a1 = coeff1.values[0];b1 = coeff1.values[1];c1 = coeff1.values[2];d1 = coeff1.values[3];
                a2 = coeff2.values[0];b2 = coeff2.values[1];c2 = coeff2.values[2];d2 = coeff2.values[3];

                //compute distance to cloud2 form 20 points sampled from cloud1
                size_t count = 0;
                for(size_t k = 0; k < sample_point_num; k++)
                {
                    // sampled points from cloud1
                    px = random_sample_points1[k].x;py = random_sample_points1[k].y;pz = random_sample_points1[k].z;
                    // compute the distance to cloud2
                    distance = fabs(a2*px+b2*py+c2*pz+d2);
                    if(distance <= seg_threshold)
                        count++;
                }
                
                for(size_t k = 0; k < sample_point_num; k++)
                {
                    // sampled points from cloud2
                    px = random_sample_points2[k].x;py = random_sample_points2[k].y;pz = random_sample_points2[k].z;
                    // compute the distance to cloud1
                    distance = fabs(a1*px+b1*py+c1*pz+d1);
                    if(distance <= seg_threshold)
                        count++;
                }

                // judge should merge or not
                if(count > sample_point_num)
                {
                    // satisfy the above conditions, then merge the two plane.
                    pcl::ModelCoefficients coeff;pcl::PointCloud<PointType> cloud;
                    coeff.values.resize(4);
                    float &a = coeff.values[0],&b = coeff.values[1];
                    float &c = coeff.values[2],&d = coeff.values[3];
                    int s1=cloud1.points.size(),s2=cloud2.points.size();
                    int s=s1+s2;
                    if(dot_product > 0)
                    {   a=(a1*s1+a2*s2)/s;b=(b1*s1+b2*s2)/s;c=(c1*s1+c2*s2)/s;d=(d1*s1+d2*s2)/s;    }
                    else
                    {   a=(a1*s1-a2*s2)/s;b=(b1*s1-b2*s2)/s;c=(c1*s1-c2*s2)/s;d=(d1*s1-d2*s2)/s;    }

                    cloud.points = cloud1.points + cloud2.points;
                    cloud.width = cloud.points.size();
                    cloud.height = 1;

                    vector_cloud[i] = cloud;vector_coefficients[i]=coeff;
                    vector_cloud.erase(vector_cloud.begin()+j);
                    vector_coefficients.erase(vector_coefficients.begin()+j);

                    // resample 20 points from mereged cloud
                    std::vector<PointType> resample_points;        
                    for(size_t k = 0; k < sample_point_num; k++)
                    {
                        PointType random_sample_points;
                        random_sample_points = vector_cloud[i].points[ rand() % vector_cloud.size() ];
                        resample_points.push_back(random_sample_points);
                    }
                    vec_plane_sample_points[i] = resample_points;
                    vec_plane_sample_points.erase(vec_plane_sample_points.begin()+j);
                    // finish one combination,then restart this process
                    break;
                }
            }
        }
    }
    
    for(size_t i = 0; i < vector_cloud.size(); i++)
    {
        Plane plane;
        plane.cloud = vector_cloud[i];
        plane.ptype = Plane::Ptype::stair_component;
        merge_planes.push_back(plane);
    }
}
/** \brief compute cross line of two planes
    * \param[in] coefficients_plane1: input plane coefficients
    * \param[in] coefficients_plane2: input plane coefficients
    * \param[out] coefficients_line: output line coefficients
    */
void StairDetection::computeLineFrom2Planes(
        const pcl::ModelCoefficients &coefficients_plane1,
        const pcl::ModelCoefficients &coefficients_plane2,
        Line &line)
{
    float a1, b1, c1, d1, a2, b2, c2, d2;
    a1 = coefficients_plane1.values[0];
    a2 = coefficients_plane2.values[0];
    b1 = coefficients_plane1.values[1];
    b2 = coefficients_plane2.values[1];
    c1 = coefficients_plane1.values[2];
    c2 = coefficients_plane2.values[2];
    d1 = coefficients_plane1.values[3];
    d2 = coefficients_plane2.values[3];

    // compute line normal
    Eigen::Vector3f normal1, normal2, normal_line;
    normal1 = Eigen::Vector3f(a1, b1, c1);
    normal2 = Eigen::Vector3f(a2, b2, c2);

    normal_line = normal1.cross(normal2);

    // compute line point (in xoy plane)
    float x0 = (b2 * d1 - b1 * d2) / (b1 * a2 - b2 * a1);
    float y0 = (a2 * d1 - a1 * d2) / (a1 * b2 - a2 * b1);
    float z0 = 0;

    // line : (x-x0)/n1=(y-y0)/n2=(z-z0)/n3=t
    // plane: x0*n1+y0*n2+z0*n3=0
    // compute cross point between line and plane perpendicular to this line with in origin
    float t = -1*(normal_line[0]*x0+normal_line[1]*y0+normal_line[2]*z0)
            / (normal_line[0]*normal_line[0]+normal_line[1]*normal_line[1]+normal_line[2]*normal_line[2]);

    float x = normal_line[0]*t+x0;
    float y = normal_line[1]*t+y0;
    float z = normal_line[2]*t+z0;

    // set line coefficients
    line.coeff.values.push_back(x0);
    line.coeff.values.push_back(y0);
    line.coeff.values.push_back(z0);
    line.coeff.values.push_back(normal_line[0]);
    line.coeff.values.push_back(normal_line[1]);
    line.coeff.values.push_back(normal_line[2]);
    line.h = x;
    line.d = sqrt(y*y+z*z);
}

/** \brief compute rotation matrix based on euler angle
    * \detail the rotation order is ZXY, and the IMU is right
    * hand coordinate system, so there is a negative before pitch.
    * the XYZ of kinect V2 is correspond with ZXY of IMU
    * \param[in] pitch: pitch
    * \param[in] roll: roll
    * \param[in] yaw: yaw
    * \param[out] rotation_matrix: rotation matrix
    */
void StairDetection::computeRotationMatrix(const float pitch, const float roll, const float yaw,
        Eigen::Matrix3f &rotation_matrix)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Compute Rotation Matrix");
    #endif

    Eigen::Matrix3f m1;
    m1 = Eigen::AngleAxisf(yaw / 180.0 * M_PI, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(-pitch / 180.0 * M_PI, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(roll / 180.0 * M_PI, Eigen::Vector3f::UnitZ());

    rotation_matrix = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f m2;
    m2 << 0, 1, 0, 0, 0, 1, 1, 0, 0;

    rotation_matrix = m2 * m1;
}

/** \brief rotation point with rotation matrix
    * \param[in] point_in: input point
    * \param[in] rotation_matrix: rotation matrix
    * \param[out] point_out: point after rotation
    */
inline void StairDetection::pointRotation(const PointType &point_in,PointType &point_out,
        const Eigen::Matrix3f &rotation_matrix)
{
    Eigen::Vector3f point_in_xyz(point_in.x,point_in.y,point_in.z);
    Eigen::Vector3f point_out_xyz;
    point_out_xyz = rotation_matrix*point_in_xyz;
    point_out = point_in;
    point_out.x = point_out_xyz[0];
    point_out.y = point_out_xyz[1];
    point_out.z = point_out_xyz[2];
}

/** \brief gather and compute all the information of a plane into Plane struct
    * \param[in] vector_cloud_in: a vector of planes cloud point
    * \param[in] vector_plane_coefficients_in: a vector of planes coefficients
    * \param[in] type: a enum indicate the type of plane (horizontal or vertical)
    * \param[out] vector_plane: a vector of planes
    */
void StairDetection::computeVectorPlaneInfo(const std::vector<pcl::PointCloud<PointType> > &vector_cloud_in,
        const std::vector<pcl::ModelCoefficients> &vector_plane_coefficients_in,
        const Plane::Type &type,
        std::vector<Plane> &vector_plane)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Compute Vector Plane Information");
    #endif

    for(size_t i = 0; i < vector_cloud_in.size(); i++)
    {
        pcl::PointXYZ center;
        float x_min,x_max,y_min,y_max,z_min,z_max;
        x_min =  FLT_MAX;y_min =  FLT_MAX;z_min =  FLT_MAX;
        x_max = -FLT_MAX;y_max = -FLT_MAX;z_max = -FLT_MAX;

        Plane plane;
        plane.cloud = vector_cloud_in[i];
        plane.coefficients = vector_plane_coefficients_in[i];
        plane.type = type;

        // 1.compute center
        float xc=0,yc=0,zc=0;
        for(size_t index = 0; index < vector_cloud_in[i].points.size(); index++)
        {
            xc+=vector_cloud_in[i].points[index].x;
            yc+=vector_cloud_in[i].points[index].y;
            zc+=vector_cloud_in[i].points[index].z;
        }
        center.x = xc/vector_cloud_in[i].points.size();
        center.y = yc/vector_cloud_in[i].points.size();
        center.z = zc/vector_cloud_in[i].points.size();
        plane.center = center;

        // 2.compute eigen value and eigen vector
        Eigen::MatrixXf points(3,vector_cloud_in[i].points.size());
        for(size_t index = 0; index < vector_cloud_in[i].points.size(); index++)
        {
            points(0,index) = vector_cloud_in[i].points[index].x - center.x;
            points(1,index) = vector_cloud_in[i].points[index].y - center.y;
            points(2,index) = vector_cloud_in[i].points[index].z - center.z;
        }
        Eigen::MatrixXf convariance(3,3);
        convariance = points*points.transpose()/vector_cloud_in[i].points.size();
        //Singular values are always sorted in decreasing order.
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(convariance, Eigen::ComputeThinU);

        plane.eigen_values[0] = svd.singularValues()[0];
        plane.eigen_values[1] = svd.singularValues()[1];
        plane.eigen_values[2] = svd.singularValues()[2];

        plane.eigen_vectors[0].x = svd.matrixU()(0,0);
        plane.eigen_vectors[0].y = svd.matrixU()(1,0);
        plane.eigen_vectors[0].z = svd.matrixU()(2,0);

        plane.eigen_vectors[1].x = svd.matrixU()(0,1);
        plane.eigen_vectors[1].y = svd.matrixU()(1,1);
        plane.eigen_vectors[1].z = svd.matrixU()(2,1);

        plane.eigen_vectors[2].x = svd.matrixU()(0,2);
        plane.eigen_vectors[2].y = svd.matrixU()(1,2);
        plane.eigen_vectors[2].z = svd.matrixU()(2,2);

        // 3.compute max distance in the direction of eigen vector
        float min[3]={FLT_MAX,FLT_MAX,FLT_MAX},max[3]={-FLT_MAX,-FLT_MAX,-FLT_MAX};
        pcl::PointXYZRGBA points_min[3],points_max[3];
        for(size_t index = 0; index < vector_cloud_in[i].points.size(); index++)
        {
            // vector form center to point
            Eigen::Vector3f cp;
            cp[0] = vector_cloud_in[i].points[index].x - center.x;
            cp[1] = vector_cloud_in[i].points[index].y - center.y;
            cp[2] = vector_cloud_in[i].points[index].z - center.z;
            // eigen vectors
            Eigen::Vector3f ev0 = svd.matrixU().col(0);
            Eigen::Vector3f ev1 = svd.matrixU().col(1);
            Eigen::Vector3f ev2 = svd.matrixU().col(2);
            // cp projection to eigen vectors
            float pj0 = cp.dot(ev0);
            float pj1 = cp.dot(ev1);
            float pj2 = cp.dot(ev2);

            // compare and save
            if(pj0 > max[0])
            {
                max[0] = pj0;
                points_max[0] = vector_cloud_in[i].points[index];
            }
            else if(pj0 < min[0])
            {
                min[0] = pj0;
                points_min[0] = vector_cloud_in[i].points[index];
            }

            if(pj1 > max[1])
            {
                max[1] = pj1;
                points_max[1] = vector_cloud_in[i].points[index];
            }
            else if(pj1 < min[1])
            {
                min[1] = pj1;
                points_min[1] = vector_cloud_in[i].points[index];
            }

            if(pj2 > max[2])
            {
                max[2] = pj2;
                points_max[2] = vector_cloud_in[i].points[index];
            }
            else if(pj2 < min[2])
            {
                min[2] = pj2;
                points_min[2] = vector_cloud_in[i].points[index];
            }
        }
        
        // 4.middle point of principle vector
        plane.pcenter.x = (points_max[0].x + points_max[0].x)/2;
        plane.pcenter.y = (points_max[0].y + points_max[0].y)/2;
        plane.pcenter.z = (points_max[0].z + points_max[0].z)/2;

        memcpy(plane.min,min,sizeof(min));
        memcpy(plane.max,max,sizeof(max));
        memcpy(plane.points_min,points_min,sizeof(points_min));
        memcpy(plane.points_max,points_max,sizeof(points_max));

        vector_plane.push_back(plane);
    }
}

void StairDetection::countPlanesNormal(const pcl::ModelCoefficients &coefficients,
        Plane::Type type,
        std::vector<normalcount> &plane_normal_count)
{
    Eigen::Vector3f normal;
    normal[0] = coefficients.values[0];
    normal[1] = coefficients.values[1];
    normal[2] = coefficients.values[2];
    normal.normalize();

    size_t i = 0;
    for(;i < plane_normal_count.size(); i++)
    {
        // find the closet normal in vector plane_normal_count
        // and average plane_normal_count.normal with normal
        float dot_product = normal.dot(plane_normal_count[i].normal);
        if(dot_product > 0)
        {
            float angle_diff = acos(dot_product);
            if(angle_diff < pcl::deg2rad(20.0))
            {
                plane_normal_count[i].normal = plane_normal_count[i].normal + normal;
                plane_normal_count[i].normal.normalize();
                plane_normal_count[i].count++;
                break;
            }
        }
        else
        {
            float angle_diff = acos(-dot_product);
            if(angle_diff < pcl::deg2rad(20.0))
            {
                plane_normal_count[i].normal = plane_normal_count[i].normal - normal;
                plane_normal_count[i].normal.normalize();
                plane_normal_count[i].count++;
                break;
            }
        }
    }
    // didn't find any plane_normal_count.normal close to normal
    // push back a new one
    if(i == plane_normal_count.size())
    {
        normalcount newnormal;
        newnormal.normal = normal;
        newnormal.count = 1;
        newnormal.type = type;
        plane_normal_count.push_back(newnormal);
    }
}

/** \brief sort planes using X value (indicate the height to the ground)
    * \param[in] vector_plane_h_in: a vector of horizontal planes
    * \param[in] vector_plane_v_in: a vector of vertical planes
    * \param[out] vector_plane_sorted: a vector of sorted planes
    */
void StairDetection::sortPlanesByXValues(const std::vector<Plane> &vector_plane_h_in,
        const std::vector<Plane> &vector_plane_v_in,
        std::vector<Plane> &vector_plane_sorted)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Sort Planes By X Values");
    #endif

    std::vector<pcl::PointXYZ> vector_center_h,vector_center_v;
    for(size_t i = 0; i < vector_plane_h_in.size(); i++ )
        vector_center_h.push_back(vector_plane_h_in[i].center);
    for(size_t i = 0; i < vector_plane_v_in.size(); i++ )
        vector_center_v.push_back(vector_plane_v_in[i].center);

    std::vector<IndexMap> vector_index_map;
    vector_index_map.resize(vector_center_h.size() + vector_center_v.size());

    for(size_t i = 0; i < vector_center_h.size() + vector_center_v.size(); i++)
    {
        float x_max_value = -FLT_MAX;
        size_t h,v,h_index,v_index;
        bool in_h_not_in_v = false;
        for(h = 0;h < vector_center_h.size();h++)
        {
            if(vector_center_h[h].x > x_max_value)
            {
                x_max_value = vector_center_h[h].x;
                h_index = h;
                in_h_not_in_v = true;
            }
        }
        for(v = 0;v < vector_center_v.size();v++)
        {
            if(vector_center_v[v].x > x_max_value)
            {
                x_max_value = vector_center_v[v].x;
                v_index = v;
                in_h_not_in_v = false;
            }
        }

        IndexMap index_map;
        if(in_h_not_in_v)
        {
            index_map.center = vector_center_h[h_index];
            vector_center_h[h_index].x = -FLT_MAX;
            index_map.index = h_index;
            index_map.type = Plane::horizontal;
        }
        else if(!in_h_not_in_v)
        {
            index_map.center = vector_center_v[v_index];
            vector_center_v[v_index].x = -FLT_MAX;
            index_map.index = v_index;
            index_map.type = Plane::vertical;
        }

        vector_index_map[i]= index_map;
    }

//      std::vector<normalcount> plane_normal_count;
    int count = 0;
    for(size_t i = 0; i < vector_index_map.size(); i++)
    {
        Plane plane;
        int index;

        if(vector_index_map[i].type == Plane::horizontal)
        {
            index = vector_index_map[i].index;
            plane = vector_plane_h_in[index];
            plane.counter.points.resize(4);
            plane.counter.height = 1;
            plane.ptype = Plane::Ptype::others;
            vector_plane_sorted.push_back(plane);
//              countPlanesNormal(plane.coefficients,Plane::horizontal,plane_normal_count);
        }
        else if(vector_index_map[i].type == Plane::vertical)
        {
            index = vector_index_map[i].index;
            plane = vector_plane_v_in[index];
            plane.counter.points.resize(4);
            plane.counter.height = 1;
            plane.ptype = Plane::Ptype::others;
            vector_plane_sorted.push_back(plane);
//              countPlanesNormal(plane.coefficients,Plane::vertical,plane_normal_count);
        }
        else
        {
            std::cerr << "error plane type" << std::endl;
        }
    }

//      std::sort(plane_normal_count.begin(),plane_normal_count.end(),sortByNormalCount);
//
//      for(size_t i = 0; i < vector_plane_sorted.size(); i++)
//      {
//          vector_plane_sorted[i].ptype = Plane::Ptype::others;
//
//          // min number of point constraint
//          if(vector_plane_sorted[i].cloud.points.size()<min_num_points)
//              continue;
//
//          // length constraint: l>0.15m
//          if(vector_plane_sorted[i].max[0]-vector_plane_sorted[i].min[0] < min_length)
//              continue;
//          // width constraint: w>0.05m
//          if(vector_plane_sorted[i].max[1]-vector_plane_sorted[i].min[1] < min_width)
//              continue;
//
//
//          // set width threshold of vertical planes: w<0.2m
//          if(vector_plane_sorted[i].type == Plane::vertical)
//              if(vector_plane_sorted[i].max[1]-vector_plane_sorted[i].min[1] > max_width)
//                  continue;
//
//          vector_plane_sorted[i].ptype = Plane::Ptype::pstair_component;
//      }
}

float StairDetection::minDistaceOfTwoCloud(const Plane &plane1,const Plane &plane2)
{
    float min_distance = FLT_MAX;

    for(size_t i=0;i<3;i++)
    {
        for(size_t j=0;j<3;j++)
        {
            float x1,x2,y1,y2,z1,z2,d;

            x1 = plane1.points_min[i].x;
            y1 = plane1.points_min[i].y;
            z1 = plane1.points_min[i].z;

            x2 = plane2.points_min[j].x;
            y2 = plane2.points_min[j].y;
            z2 = plane2.points_min[j].z;

            d = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);

            if(d < min_distance)
                min_distance = d;
        }
    }

    for(size_t i=0;i<3;i++)
    {
        for(size_t j=0;j<3;j++)
        {
            float x1,x2,y1,y2,z1,z2,d;

            x1 = plane1.points_min[i].x;
            y1 = plane1.points_min[i].y;
            z1 = plane1.points_min[i].z;

            x2 = plane2.points_max[j].x;
            y2 = plane2.points_max[j].y;
            z2 = plane2.points_max[j].z;

            d = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);

            if(d < min_distance)
                min_distance = d;
        }
    }

    for(size_t i=0;i<3;i++)
    {
        for(size_t j=0;j<3;j++)
        {
            float x1,x2,y1,y2,z1,z2,d;

            x1 = plane1.points_max[i].x;
            y1 = plane1.points_max[i].y;
            z1 = plane1.points_max[i].z;

            x2 = plane2.points_min[j].x;
            y2 = plane2.points_min[j].y;
            z2 = plane2.points_min[j].z;

            d = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);

            if(d < min_distance)
                min_distance = d;
        }
    }

    for(size_t i=0;i<3;i++)
    {
        for(size_t j=0;j<3;j++)
        {
            float x1,x2,y1,y2,z1,z2,d;

            x1 = plane1.points_max[i].x;
            y1 = plane1.points_max[i].y;
            z1 = plane1.points_max[i].z;

            x2 = plane2.points_max[j].x;
            y2 = plane2.points_max[j].y;
            z2 = plane2.points_max[j].z;

            d = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);

            if(d < min_distance)
                min_distance = d;
        }
    }

    for(size_t i=0;i<3;i++)
    {
        for(size_t j=0;j<3;j++)
        {
            float x1,x2,y1,y2,z1,z2,d;

            x1 = plane1.points_min[i].x;
            y1 = plane1.points_min[i].y;
            z1 = plane1.points_min[i].z;

            x2 = plane2.points_min[j].x;
            y2 = plane2.points_min[j].y;
            z2 = plane2.points_min[j].z;

            d = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);

            if(d < min_distance)
                min_distance = d;
        }
    }

    return min_distance;
}


/** \brief model the stair using vector_plane_sorted
    * \param[in] vector_plane_sorted: a vector of all the planes sorted by height to ground
    * \param[out] stair: a link list of stair which indicate the stair model
    * \param[out] return: there is a stair or not
    */
bool StairDetection::stairModel(std::vector<Plane> &vector_plane_sorted,
        Stair &stair)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Stair Model");
    #endif

    typedef struct
    {        
        Eigen::Vector3f center_direction;
        Eigen::Vector3f pcenter_direction;
        float distance;
        float delta_h;
        float delta_d;
    } relation;
    
#ifdef DEBUG_STAIR_MODEL
    std::cout.flags(ios::fixed);  
    std::cout.precision(3); 

    std::cout  << YELLOW<< "[0] vector_plane_sorted before mark ground" <<RESET<< std::endl;
    for(size_t i = 0; i < vector_plane_sorted.size(); i++)
    {
        std::cout << "Plane["<<i<<"]: " << vector_plane_sorted[i].coefficients.values[0] <<","
                                << vector_plane_sorted[i].coefficients.values[1] <<","
                                << vector_plane_sorted[i].coefficients.values[2] <<","
                                << vector_plane_sorted[i].coefficients.values[3];
        if(vector_plane_sorted[i].type == Plane::horizontal)
            std::cout<<"\thorizontal";
        else
            std::cout<<"\tvertical";
        
        if(vector_plane_sorted[i].ptype == Plane::Ptype::ground)
            std::cout<<"\tground";
        else if (vector_plane_sorted[i].ptype == Plane::Ptype::others)
            std::cout<<"\tothers";
        
        std::cout << "\tcenter:"<<"(" << vector_plane_sorted[i].center.x 
                        <<"," << vector_plane_sorted[i].center.y
                        <<"," << vector_plane_sorted[i].center.z << ")";
                        
        std::cout << "\tpcenter:"<<"(" << vector_plane_sorted[i].pcenter.x 
                        <<"," << vector_plane_sorted[i].pcenter.y
                        <<"," << vector_plane_sorted[i].pcenter.z << ")";
                        
        std::cout << "\tpoints:"<<vector_plane_sorted[i].cloud.points.size()<<std::endl;;
    }
    
#endif

    //####################### 1.mark ground       
    int max_horizontal_plane_points = 0, max_horizontal_plane_points_index = -1;
    
    struct plane_height
    {
        float height;
        int index;
        int point_num;
    };
    std::vector<plane_height> vec_plane_height;
    // all the horizontal planes and its height
    bool first = true;
    float lowest_x;
    for(size_t i = 0; i<vector_plane_sorted.size(); i++)
    {
        if(vector_plane_sorted[i].type == Plane::horizontal)
        {
            plane_height ph;
            int plane_points_number = vector_plane_sorted[i].cloud.points.size();
            
            // find lowest horizontal plane
            if(first)
            {
                first = false;
                lowest_x = vector_plane_sorted[i].center.x;
                ph.height = 0;
                ph.index = i;
                ph.point_num = plane_points_number;
            }
            else
            {
                ph.height = lowest_x - vector_plane_sorted[i].center.x;
                ph.index = i;
                ph.point_num = plane_points_number;
            }
            vec_plane_height.push_back(ph);
        }
    }

    // find first ground planes index
    std::vector<int> vec_g_index;
    int g_index;int p_ground_count = 0;
    for(size_t i = 0 ; i < vec_plane_height.size(); i++)
    {
        if(vec_plane_height[i].height < 2*max_g_height)
        {
            if(vec_plane_height[i].point_num > 3000)
            {
                // length and with constraint
                if((vector_plane_sorted[i].max[0]-vector_plane_sorted[i].min[0] > max_length)
                        ||(vector_plane_sorted[i].max[1]-vector_plane_sorted[i].min[1] > max_width))
                {
                    // if it's erea bigger 3500 cm^2, it's ground
                    vector_plane_sorted[vec_plane_height[i].index].ptype = Plane::Ptype::ground;
                    g_index = i;
                    vec_g_index.push_back(i);
                    p_ground_count ++;
                    break;
                }
            }
        }
    }
    
    // find others ground planes
    if(p_ground_count > 0)
    {
        for(size_t i = 0 ; i < vec_plane_height.size(); i++)
        {
            if(i != g_index)
            if(fabs(vec_plane_height[i].height - vec_plane_height[g_index].height) < max_g_height)
            {
                vec_g_index.push_back(vec_plane_height[i].index);
                vector_plane_sorted[vec_plane_height[i].index].ptype = Plane::Ptype::ground;
                p_ground_count ++;
            }
        }
        
        if(p_ground_count > 1)
        {
            // merge ground plane
            CloudType g_cloud;
            pcl::ModelCoefficients g_coeff;
            g_coeff.values.push_back(0);g_coeff.values.push_back(0);
            g_coeff.values.push_back(0);g_coeff.values.push_back(0);
            Eigen::Vector3f g_normal(0,0,0);
            int total_g_points_number = 0;
            for(size_t i = 0; i<vector_plane_sorted.size(); i++)
            {
                if(vector_plane_sorted[i].ptype == Plane::Ptype::ground)
                {
                    g_cloud += vector_plane_sorted[i].cloud;
                    Eigen::Vector3f coeff_normal(vector_plane_sorted[i].coefficients.values[0],
                                                vector_plane_sorted[i].coefficients.values[1],
                                                vector_plane_sorted[i].coefficients.values[2]);
                    
                    g_normal += coeff_normal * vector_plane_sorted[i].cloud.points.size();
                    g_coeff.values[3] += vector_plane_sorted[i].coefficients.values[3] * vector_plane_sorted[i].cloud.points.size();
                    total_g_points_number += vector_plane_sorted[i].cloud.points.size();
                }
            }
            g_normal.normalize();
            g_coeff.values[0] = g_normal[0];
            g_coeff.values[1] = g_normal[1];
            g_coeff.values[2] = g_normal[2];
            g_coeff.values[3] = g_coeff.values[3]/total_g_points_number;
            
            std::vector<CloudType> vec_g_cloud;
            std::vector<pcl::ModelCoefficients> vec_g_coeff;
            std::vector<Plane> vec_g_plane;
            vec_g_cloud.push_back(g_cloud);vec_g_coeff.push_back(g_coeff);
            computeVectorPlaneInfo(vec_g_cloud,vec_g_coeff,Plane::horizontal,vec_g_plane);
            
            std::sort(vec_g_index.begin(),vec_g_index.end());
            
            // remove all previous ground plane
            for(int i = vec_g_index.size() - 1; i >= 0; i--)
                vector_plane_sorted.erase(vector_plane_sorted.begin()+vec_g_index[i]);
            
            // insert new merged ground plane 
            vec_g_plane[0].ptype = Plane::Ptype::ground;
            vec_g_plane[0].counter.points.resize(4);
            vec_g_plane[0].counter.height = 1;
            vector_plane_sorted.insert(vector_plane_sorted.begin(), vec_g_plane[0]);
        }
    }
    
    // mark all potential stair plane
    for(size_t i = 0; i < vector_plane_sorted.size(); i ++)
    {
        if(vector_plane_sorted[i].ptype != Plane::Ptype::ground)
        if((vector_plane_sorted[i].max[0]-vector_plane_sorted[i].min[0] > max_length)
            ||(vector_plane_sorted[i].max[1]-vector_plane_sorted[i].min[1] > max_width))
        {
            vector_plane_sorted[i].ptype = Plane::Ptype::others;
        }
        else
            vector_plane_sorted[i].ptype = Plane::Ptype::pstair_component;
    }        
    
    //####################### 1.mark ground
    
#ifdef DEBUG_STAIR_MODEL
    std::cout << std::endl<<YELLOW<<"[1] mark ground"<<RESET<<std::endl;    
    
    std::cout << "vector_plane_sorted:" << std::endl;
    for(size_t i = 0; i < vector_plane_sorted.size(); i++)
    {
        std::cout << "Plane["<<i<<"]: " << vector_plane_sorted[i].coefficients.values[0] <<","
                                << vector_plane_sorted[i].coefficients.values[1] <<","
                                << vector_plane_sorted[i].coefficients.values[2] <<","
                                << vector_plane_sorted[i].coefficients.values[3];
        if(vector_plane_sorted[i].type == Plane::horizontal)
            std::cout<<"\thorizontal";
        else
            std::cout<<"\tvertical";
        if(vector_plane_sorted[i].ptype == Plane::Ptype::ground)
            std::cout<<"\tground";
        else if (vector_plane_sorted[i].ptype == Plane::Ptype::others)
            std::cout<<"\tothers";
        else
            std::cout<<"\tpstair";
        
        std::cout << "\tcenter:"<<"(" << vector_plane_sorted[i].center.x 
                        <<"," << vector_plane_sorted[i].center.y
                        <<"," << vector_plane_sorted[i].center.z << ")";
                        
        std::cout << "\tpcenter:"<<"(" << vector_plane_sorted[i].pcenter.x 
                        <<"," << vector_plane_sorted[i].pcenter.y
                        <<"," << vector_plane_sorted[i].pcenter.z << ")";
                        
        std::cout << "\tpoints:"<<vector_plane_sorted[i].cloud.points.size()<<std::endl;;
    }
    
#endif    

//####################### 2.compute key relations between planes
    int d = vector_plane_sorted.size();

    std::vector<std::vector<relation> > conjact_array;
    conjact_array.resize(d);
    for(size_t i=0;i<d;i++)
        conjact_array[i].resize(d);
    
    // for all planes
    for(size_t i=0;i<d;i++)
    {
        for(size_t j=i+1;j<d;j++)
        {
            // compute matrix
            conjact_array[i][j].distance = minDistaceOfTwoCloud(vector_plane_sorted[i],vector_plane_sorted[j]);
            conjact_array[i][j].delta_h = fabs(vector_plane_sorted[i].center.x -
                    vector_plane_sorted[j].center.x);
            conjact_array[i][j].center_direction = Eigen::Vector3f(
                    vector_plane_sorted[j].center.x - vector_plane_sorted[i].center.x,
                    vector_plane_sorted[j].center.y - vector_plane_sorted[i].center.y,
                    vector_plane_sorted[j].center.z - vector_plane_sorted[i].center.z);
            conjact_array[i][j].pcenter_direction = Eigen::Vector3f(
                    vector_plane_sorted[j].pcenter.x - vector_plane_sorted[i].pcenter.x,
                    vector_plane_sorted[j].pcenter.y - vector_plane_sorted[i].pcenter.y,
                    vector_plane_sorted[j].pcenter.z - vector_plane_sorted[i].pcenter.z);
        }
    }   
//####################### 2.compute key relations between planes

#ifdef DEBUG_STAIR_MODEL
    std::cout <<YELLOW<< "[2] compute key relations between planes" <<RESET<< std::endl;
    std::cout<<"conjact_array:"<<std::endl;
    for(size_t i = 0; i < d; i++)
    {
        for(size_t j=0;j<d;j++)
        {
            std::cout << "D:  " << conjact_array[i][j].distance <<"\t\t";
        }
        std::cout << std::endl;
        
        for(size_t j=0;j<d;j++)
        {
            std::cout << "dh: " << conjact_array[i][j].delta_h <<"\t\t";
        }
        std::cout << std::endl;
        
        for(size_t j=0;j<d;j++)
        {
            std::cout << "cd: " << conjact_array[i][j].center_direction[0] 
                        <<"," << conjact_array[i][j].center_direction[1]
                        <<"," << conjact_array[i][j].center_direction[2] << "\t";
        }        
        std::cout << std::endl;
        
        for(size_t j=0;j<d;j++)
        {
            std::cout << "pcd: " << conjact_array[i][j].pcenter_direction[0] 
                        <<"," << conjact_array[i][j].pcenter_direction[1]
                        <<"," << conjact_array[i][j].pcenter_direction[2] << "\t";
        }
        std::cout << std::endl;
        std::cout << std::endl;
    }
#endif


//####################### 3.find main vertical plane direction
#ifdef DEBUG_STAIR_MODEL
    std::cout <<YELLOW<< "[3] find main vertical plane direction" <<RESET<< std::endl;
#endif
    std::vector<Eigen::Vector3f> vertical_normals;
    for(size_t i=0;i<d;i++)
    {
        if(vector_plane_sorted[i].type == Plane::Type::vertical
            && vector_plane_sorted[i].ptype == Plane::Ptype::pstair_component)
        {
            Eigen::Vector3f normal = Eigen::Vector3f(
                    vector_plane_sorted[i].coefficients.values[0],
                    vector_plane_sorted[i].coefficients.values[1],
                    vector_plane_sorted[i].coefficients.values[2]);
            vertical_normals.push_back(normal);
        }
    }
    
#ifdef DEBUG_STAIR_MODEL
    std::cout << "vertical_normals:" << std::endl;
    for(size_t i = 0; i < vertical_normals.size(); i++)
        std::cout << vertical_normals[i] <<";" << std::endl;
#endif

    two_main_vertical_normal = false;
    std::vector<dircount> main_vertical_normals;
    if(vertical_normals.size() > 0)
    {
        // for all vertical normals
        for(size_t i = 0; i < vertical_normals.size(); i++)
        {
            Eigen::Vector3f dir1 = vertical_normals[i];
            dir1.normalize();

            // for all normals in clusters
            bool exist = false;
            for(size_t j = 0; j < main_vertical_normals.size(); j++)
            {
                Eigen::Vector3f dir2 = main_vertical_normals[j].dir;
                dir2.normalize();
                float dot_product = dir1.dot(dir2);
                float angle = acos(dot_product-0.0001);

                // add and count exist one
                if(angle < pcl::deg2rad(vertical_angle_diff))
                {
                    main_vertical_normals[j].dir += dir1;
                    main_vertical_normals[j].count += 1;
                    exist = true;
                    break;
                }
                else if(M_PI-angle < pcl::deg2rad(vertical_angle_diff))
                {
                    main_vertical_normals[j].dir -= dir1;
                    main_vertical_normals[j].count += 1;
                    exist = true;
                    break;
                }
            }

            // not exist in cluster, add a new one
            if(exist == false)
            {
                dircount dir_count;
                dir_count.dir = vertical_normals[i];
                dir_count.count = 1;
                main_vertical_normals.push_back(dir_count);
            }
        }

        for(size_t i = 0; i < main_vertical_normals.size(); i++)
            main_vertical_normals[i].dir.normalize();

        // sort based on the counts
        std::sort(main_vertical_normals.begin(),main_vertical_normals.end(),sortByDirCount);

        // check if there are two main vertical normals
        if(main_vertical_normals.size()>=2)
            if(main_vertical_normals[0].count == main_vertical_normals[1].count)
                two_main_vertical_normal = true;
    }

#ifdef DEBUG_STAIR_MODEL
    std::cout  << std::endl<< "main_vertical_normals:" << std::endl;
    for(size_t i = 0;i < main_vertical_normals.size(); i++)
    {
        std::cout << "count["<<i<<"]: " << main_vertical_normals[i].count << std::endl;
        std::cout << "dir:\n"<< main_vertical_normals[i].dir << std::endl;
    }
#endif
    
    if(main_vertical_normals.size() == 0)
        return false;
//####################### 3.find main vertical plane direction

//####################### 4.find main center_vector direction    
#ifdef DEBUG_STAIR_MODEL
    std::cout <<YELLOW<< "[4] find main center_vector direction " <<RESET<< std::endl;
#endif
    
    // for all planes, compute delta_d
    for(size_t i=0;i<d;i++)
    {
        for(size_t j=i+1;j<d;j++)
        {            
            conjact_array[i][j].delta_d  = fabs(main_vertical_normals[0].dir.dot(conjact_array[i][j].center_direction ));
        }
    }
    
    // mark others plane based on main_vertical_normals
    for(size_t i = 0; i < d; i++)
    {
        if(vector_plane_sorted[i].type == Plane::vertical)
        {
            Eigen::Vector3f normal_i;
            normal_i[0] = vector_plane_sorted[i].coefficients.values[0];
            normal_i[1] = vector_plane_sorted[i].coefficients.values[1];
            normal_i[2] = vector_plane_sorted[i].coefficients.values[2];
            
            float dot_product = main_vertical_normals[0].dir.dot(normal_i);
            float angle = pcl::rad2deg(acos(dot_product-0.0001));
            
#ifdef DEBUG_STAIR_MODEL            
            std::cout << "angle of main_vertical_normals and normal " << i <<": " << angle <<" th:"<< vertical_angle_diff << std::endl;
#endif
            if((angle < vertical_angle_diff)||(180 - angle < vertical_angle_diff)){}
            else
                vector_plane_sorted[i].ptype = Plane::others;
        }
    }
    
#ifdef DEBUG_STAIR_MODEL        
    std::cout << "vector_plane_sorted:" << std::endl;
    for(size_t i = 0; i < vector_plane_sorted.size(); i++)
    {
        std::cout << "Plane["<<i<<"]: " << vector_plane_sorted[i].coefficients.values[0] <<","
                                << vector_plane_sorted[i].coefficients.values[1] <<","
                                << vector_plane_sorted[i].coefficients.values[2] <<","
                                << vector_plane_sorted[i].coefficients.values[3];
        if(vector_plane_sorted[i].type == Plane::horizontal)
            std::cout<<"\thorizontal";
        else
            std::cout<<"\tvertical";
        if(vector_plane_sorted[i].ptype == Plane::Ptype::ground)
            std::cout<<"\tground";
        else if (vector_plane_sorted[i].ptype == Plane::Ptype::others)
            std::cout<<"\tothers";
        else
            std::cout<<"\tpstair";
        
        std::cout << "\tcenter:"<<"(" << vector_plane_sorted[i].center.x 
                        <<"," << vector_plane_sorted[i].center.y
                        <<"," << vector_plane_sorted[i].center.z << ")";
                        
        std::cout << "\tpcenter:"<<"(" << vector_plane_sorted[i].pcenter.x 
                        <<"," << vector_plane_sorted[i].pcenter.y
                        <<"," << vector_plane_sorted[i].pcenter.z << ")";
                        
        std::cout << "\tpoints:"<<vector_plane_sorted[i].cloud.points.size()<<std::endl;;
    }
#endif
    
    std::vector<Eigen::Vector3f> cv_directions;
    cv_directions.resize(2*d);
    for(size_t i = 0; i < cv_directions.size(); i++)
        cv_directions[i]=Eigen::Vector3f(0,0,0);
    // for all pstair planes
    for(size_t i=0;i<d;i++)
    {
        // horizontal -> horizontal/vertical
        if(vector_plane_sorted[i].type == Plane::horizontal
            && vector_plane_sorted[i].ptype == Plane::Ptype::pstair_component)
        {
            float min_dis_hh = FLT_MAX,min_delta_h_hh = FLT_MAX;
            float min_dis_hv = FLT_MAX,min_delta_h_hv = FLT_MAX;
            for(size_t j=i+1;j<d;j++)
            {
                if(vector_plane_sorted[j].type == Plane::horizontal
                    && vector_plane_sorted[j].ptype == Plane::Ptype::pstair_component)
                {
#ifdef DEBUG_STAIR_MODEL
                    std::cout << "horizontal -> horizontal: " << std::endl;
                    
                    std::cout << "find plane satisfy: \t";
                    std::cout << "conjact_array["<<i<<"]["<<j<<"].distance: "<<conjact_array[i][j].distance;
                    std::cout << " < counter_max_distance: "<<counter_max_distance;
                    std::cout << " && < min_dis_hh: "<< min_dis_hh << std::endl;
#endif                      
                    // find plane satisfy: distance < max_distance && min_h<delta_h<max_h
                    if((conjact_array[i][j].distance < counter_max_distance)
                            &&(conjact_array[i][j].distance < min_dis_hh))
                    {
                        min_dis_hh = conjact_array[i][j].distance;
#ifdef DEBUG_STAIR_MODEL
                        std::cout << "condition: \t";
                        std::cout << "conjact_array["<<i<<"]["<<j<<"]\t";
                        
                        std::cout << "min_height("<<min_height<<")<";
                        std::cout<< "delta_h("<<conjact_array[i][j].delta_h<<")";
                        std::cout << " < max_height("<<max_height<<")";
                        
                        std::cout << " && min_width("<<min_width<<")<";
                        std::cout<< "max_width("<<conjact_array[i][j].delta_d<<")";
                        std::cout << " < max_width("<<max_width<<")";
                        
                        std::cout<< "delta_h("<<conjact_array[i][j].delta_h<<")";
                        std::cout << " < min_delta_h_hh("<< min_delta_h_hh<<")";
                        
                        std::cout << std::endl;
#endif                        
                        if((conjact_array[i][j].delta_h > min_height)
                                &&(conjact_array[i][j].delta_h < max_height)
                                &&(conjact_array[i][j].delta_d > min_width)
                                &&(conjact_array[i][j].delta_d < max_width)
                                &&(conjact_array[i][j].delta_h < min_delta_h_hh))
                        {
//                             std::cout << RED << "MCV record!" << RESET <<std::endl;
                            min_delta_h_hh = conjact_array[i][j].delta_h;
                            // save to cv_directions
                            cv_directions[i] = conjact_array[i][j].center_direction;
                        }
                    }
                }
                else if(vector_plane_sorted[j].type == Plane::vertical && 
                     vector_plane_sorted[j].ptype == Plane::Ptype::pstair_component)
                {
#ifdef DEBUG_STAIR_MODEL
                    std::cout << "horizontal -> vertical: " << std::endl;
                    
                    std::cout << "find plane satisfy: \t";
                    std::cout << "conjact_array["<<i<<"]["<<j<<"].distance: "<<conjact_array[i][j].distance;
                    std::cout << " < counter_max_distance: "<<counter_max_distance;
                    std::cout << " && < min_dis_hv: "<< min_dis_hv << std::endl;
#endif                      
                    // find plane satisfy: distance < max_distance && min_h<delta_h<max_h
                    if((conjact_array[i][j].distance < counter_max_distance)
                            &&(conjact_array[i][j].distance < min_dis_hv))
                    {
                        min_dis_hv = conjact_array[i][j].distance;
#ifdef DEBUG_STAIR_MODEL
                        std::cout << "condition: \t";
                        std::cout << "conjact_array["<<i<<"]["<<j<<"]\t";
                        
                        std::cout<< "delta_h("<<conjact_array[i][j].delta_h<<")";
                        std::cout << " < max_height/2("<<max_height/2<<")";
                        
                        std::cout << " && min_width("<<min_width<<")<";
                        std::cout<< "max_width("<<conjact_array[i][j].delta_d<<")";
                        std::cout << " > max_width/2("<<max_width/2<<")";
                        
                        std::cout<< "delta_h("<<conjact_array[i][j].delta_h<<")";
                        std::cout << " < min_delta_h_hv("<< min_delta_h_hv<<")";
                        
                        std::cout << std::endl;
#endif                        
                        if((conjact_array[i][j].delta_h < max_height/2)
                            &&(conjact_array[i][j].delta_d > min_width)
                                &&(conjact_array[i][j].delta_d < max_width/2)
                                &&(conjact_array[i][j].delta_h < min_delta_h_hv))
                        {
//                             std::cout << RED << "MCV record!" << RESET <<std::endl;
                            
                            min_delta_h_hv = conjact_array[i][j].delta_h;
                            // save to cv_directions
                            cv_directions[i+d] = conjact_array[i][j].center_direction;
                        }
                    }
                }
            }
        }        
        // vertical -> vertical/horizontal
        else if(vector_plane_sorted[i].type == Plane::vertical
            && vector_plane_sorted[i].ptype == Plane::Ptype::pstair_component)
        {
            float min_dis_vh = FLT_MAX,min_delta_h_vh = FLT_MAX;
            float min_dis_vv = FLT_MAX,min_delta_h_vv = FLT_MAX;
            for(size_t j=i+1;j<d;j++)
            {
                if(vector_plane_sorted[j].type == Plane::horizontal
                    && vector_plane_sorted[j].ptype == Plane::Ptype::pstair_component)
                {
#ifdef DEBUG_STAIR_MODEL
                    std::cout << "vertical -> horizontal: " << std::endl;
                    
                    std::cout << "find plane satisfy: \t";
                    std::cout << "conjact_array["<<i<<"]["<<j<<"].distance: "<<conjact_array[i][j].distance;
                    std::cout << " < counter_max_distance: "<<counter_max_distance;
                    std::cout << " && < min_dis_vh: "<< min_dis_vh << std::endl;
#endif                      
                    // find plane satisfy: distance < max_distance && min_h<delta_h<max_h
                    if((conjact_array[i][j].distance < counter_max_distance)                        
                            &&(conjact_array[i][j].distance < min_dis_vh))
                    {
                        min_dis_vh = conjact_array[i][j].distance;
#ifdef DEBUG_STAIR_MODEL
                        std::cout << "condition: \t";
                        std::cout << "conjact_array["<<i<<"]["<<j<<"]\t";
                        
                        std::cout<< "delta_h("<<conjact_array[i][j].delta_h<<")";
                        std::cout << " < max_height/2("<<max_height/2<<")";
                        
                        std::cout << " && min_width("<<min_width<<")<";
                        std::cout<< "max_width("<<conjact_array[i][j].delta_d<<")";
                        std::cout << " < max_width/2("<<max_width/2<<")";
                        
                        std::cout<< "delta_h("<<conjact_array[i][j].delta_h<<")";
                        std::cout << " < min_delta_h_vh("<< min_delta_h_vh<<")";
                        
                        std::cout << std::endl;                        
#endif                        
                        if((conjact_array[i][j].delta_h < max_height/2)
                            &&(conjact_array[i][j].delta_d > min_width)
                                &&(conjact_array[i][j].delta_d < max_width/2)
                                &&(conjact_array[i][j].delta_h < min_delta_h_vh))
                        {
//                             std::cout << RED << "MCV record!" << RESET <<std::endl;

                            min_delta_h_vh = conjact_array[i][j].delta_h;
                            // save to cv_directions
                            cv_directions[i] = conjact_array[i][j].center_direction;
                        }
                    }
                }
                else if(vector_plane_sorted[j].type == Plane::vertical && 
                     vector_plane_sorted[j].ptype == Plane::Ptype::pstair_component)
                {
#ifdef DEBUG_STAIR_MODEL
                    std::cout << "vertical -> vertical: " << std::endl;
                    
                    std::cout << "find plane satisfy: \t";
                    std::cout << "conjact_array["<<i<<"]["<<j<<"].distance: "<<conjact_array[i][j].distance;
                    std::cout << " < counter_max_distance: "<<counter_max_distance;
                    std::cout << " && < min_dis_vv: "<< min_dis_vv << std::endl;
#endif                      
                    // find plane satisfy: distance < max_distance && min_h<delta_h<max_h
                    if((conjact_array[i][j].distance < counter_max_distance)
                            &&(conjact_array[i][j].distance < min_dis_vv))
                    {
                        min_dis_vv = conjact_array[i][j].distance;
#ifdef DEBUG_STAIR_MODEL
                        std::cout << "condition: \t";
                        std::cout << "conjact_array["<<i<<"]["<<j<<"]\t";
                        
                        std::cout << "min_height("<<min_height<<")<";
                        std::cout<< "delta_h("<<conjact_array[i][j].delta_h<<")";
                        std::cout << " < max_height("<<max_height<<")";
                        
                        std::cout << " && min_width("<<min_width<<")<";
                        std::cout<< "max_width("<<conjact_array[i][j].delta_d<<")";
                        std::cout << " < max_width("<<max_width<<")";
                        
                        std::cout<< "delta_h("<<conjact_array[i][j].delta_h<<")";
                        std::cout << " < min_delta_h_vv("<< min_delta_h_vv<<")";
                        
                        std::cout << std::endl;                        
#endif                        
                        if((conjact_array[i][j].delta_h > min_height)
                                &&(conjact_array[i][j].delta_h < max_height)
                                &&(conjact_array[i][j].delta_d > min_width)
                                &&(conjact_array[i][j].delta_d < max_width)
                                &&(conjact_array[i][j].delta_h < min_delta_h_vv))
                        {
//                             std::cout << RED << "MCV record!" << RESET <<std::endl;

                            min_delta_h_vv = conjact_array[i][j].delta_h;
                            // save to cv_directions
                            cv_directions[i+d] = conjact_array[i][j].center_direction;
                        }
                    }
                }
            }
        }
    }
    
#ifdef DEBUG_STAIR_MODEL
    std::cout << "cv_directions:" << std::endl;
    for(size_t i = 0;i < cv_directions.size(); i++)
    {
        std::cout << "dir:\n"<< cv_directions[i] << std::endl;
    }
#endif

    // find main direction
    std::vector<dircount> main_cv_directions;
    for(size_t i=0;i<cv_directions.size();i++)
    {
        // except null ones
        if((cv_directions[i](0) != 0)
            &&(cv_directions[i](1) != 0)
            &&(cv_directions[i](2) != 0))
        {
            Eigen::Vector3f dir1 = cv_directions[i];
            dir1.normalize();

            // for all normals in clusters
            bool exist = false;
            for(size_t j = 0; j < main_cv_directions.size(); j++)
            {
                Eigen::Vector3f dir2 = main_cv_directions[j].dir;
                dir2.normalize();
                float dot_product = dir1.dot(dir2);
                float angle = pcl::rad2deg(acos(dot_product-0.0001));

                // add and count exist one
                if(angle < mcv_angle_diff)
                {
                    main_cv_directions[j].dir += dir1;
                    main_cv_directions[j].count += 1;
                    exist = true;
                    break;
                }
            }

            // not exist in cluster, add a new one
            if(exist == false)
            {
                dircount dir_count;
                dir_count.dir = cv_directions[i];
                dir_count.count = 1;
                main_cv_directions.push_back(dir_count);
            }
        }
    }

    for(size_t i = 0; i < main_cv_directions.size(); i++)
        main_cv_directions[i].dir.normalize();

    // sort based on the counts
    std::sort(main_cv_directions.begin(),main_cv_directions.end(),sortByDirCount);
    
#ifdef DEBUG_STAIR_MODEL
    std::cout << "main_cv_directions:" << std::endl;
    for(size_t i = 0;i < main_cv_directions.size(); i++)
    {
        std::cout << "count["<<i<<"]: " << main_cv_directions[i].count;
        std::cout << "\ndir:\n"<< main_cv_directions[i].dir << std::endl;
    }
#endif
    
//####################### 4.find main center_vector direction

//####################### 5.find stair components based on main center_vector and normal_cross_product direction
#ifdef DEBUG_STAIR_MODEL
    std::cout <<YELLOW<< "[5] find stair components based on main center_vector and normal_cross_product direction" <<RESET<< std::endl;
#endif
    
    if(main_cv_directions.size() == 0)
        return false;

    typedef struct{
        int from;int to;
    } int2;
    std::vector<int2> fragment_vector;

    for(size_t i=0;i<d;i++)
    {
//         for(size_t j=i+1;(j<i+3)&&(j<d);j++)
        for(size_t j=i+1;j<d;j++)
        {
            Eigen::Vector3f normal_i,normal_j;
            normal_i[0] = vector_plane_sorted[i].coefficients.values[0];
            normal_i[1] = vector_plane_sorted[i].coefficients.values[1];
            normal_i[2] = vector_plane_sorted[i].coefficients.values[2];
            normal_j[0] = vector_plane_sorted[j].coefficients.values[0];
            normal_j[1] = vector_plane_sorted[j].coefficients.values[1];
            normal_j[2] = vector_plane_sorted[j].coefficients.values[2];
            Eigen::Vector3f dir = conjact_array[i][j].center_direction;
            dir.normalize();
            float dot_product = main_cv_directions[0].dir.dot(dir);
            float angle = pcl::rad2deg(acos(dot_product-0.0001));

#ifdef DEBUG_STAIR_MODEL
            std::cout << "compare: (" ;
            std::cout << i << "," << j << "):" << "angle of main_cv_directions and dir ij: " << angle<<" th:"<<center_vector_angle_diff;
            std::cout << "\tdistance: " << conjact_array[i][j].distance << angle <<" th:"<<counter_max_distance<<std::endl;
#endif
            
            if((angle < center_vector_angle_diff)
                &&(conjact_array[i][j].distance < counter_max_distance)
                && vector_plane_sorted[i].ptype == Plane::Ptype::pstair_component
                && vector_plane_sorted[j].ptype == Plane::Ptype::pstair_component)
            {
                if(vector_plane_sorted[i].type == Plane::vertical)
                {
                    float dot_product = main_vertical_normals[0].dir.dot(normal_i);
                    float angle = pcl::rad2deg(acos(dot_product-0.0001));
                    
#ifdef DEBUG_STAIR_MODEL     
                    std::cout << "angle of main_vertical_normals and normal " <<j <<": " << angle <<" th:"<< vertical_angle_diff << std::endl;
#endif
                    if((angle < vertical_angle_diff)||(180 - angle < vertical_angle_diff)){}
                    else
                    {
                        if(two_main_vertical_normal)
                        {
                            dot_product = main_vertical_normals[1].dir.dot(normal_i);
                            angle = pcl::rad2deg(acos(dot_product-0.0001));
                            
//                             std::cout << "angle of main_vertical_normals and normal j: " << angle << std::endl;
                            
                            if((angle < vertical_angle_diff)||(180 - angle < vertical_angle_diff)){}
                            else
                                vector_plane_sorted[i].ptype = Plane::others;
                        }
                        else
                            vector_plane_sorted[i].ptype = Plane::others;
                    }
                }
                if(vector_plane_sorted[j].type == Plane::vertical)
                {
                    float dot_product = main_vertical_normals[0].dir.dot(normal_j);
                    float angle = pcl::rad2deg(acos(dot_product-0.0001));
                    
//                     std::cout << "angle of main_vertical_normals and normal i: " << angle <<" th:"<< vertical_angle_diff << std::endl;
                    
                    if((angle < vertical_angle_diff)||(180 - angle < vertical_angle_diff)){}
                    else
                    {
                        if(two_main_vertical_normal)
                        {
                            dot_product = main_vertical_normals[1].dir.dot(normal_j);
                            angle = pcl::rad2deg(acos(dot_product-0.0001));
                            if((angle < vertical_angle_diff)||(180 - angle < vertical_angle_diff)){}
                            else
                                vector_plane_sorted[j].ptype = Plane::others;
                        }
                        else
                            vector_plane_sorted[j].ptype = Plane::others;
                    }
                }

                if(vector_plane_sorted[i].ptype == vector_plane_sorted[j].ptype ==
                        Plane::pstair_component)
                {
                    int2 ft;
                    ft.from = i;ft.to = j;
                    fragment_vector.push_back(ft);
                    break;
                }
            }
            else
            {}
        }
    }

#ifdef DEBUG_STAIR_MODEL    
    std::cout<<"fragment_vector:"<<std::endl;
    for(size_t i = 0; i < fragment_vector.size();i++)
    {
        std::cout << fragment_vector[i].from << " -> " << fragment_vector[i].to << " ; ";
    }
    std::cout<<std::endl;
#endif
//####################### 5.find stair components based on main center_vector and normal_cross_product direction

//####################### 6.from connect fragment find a longest stair link
#ifdef DEBUG_STAIR_MODEL
    std::cout <<YELLOW<< "[6] from connect fragment find a longest stair link" <<RESET<< std::endl;
#endif    
    std::vector<int> plane_index_list;
    if(fragment_vector.size() > 1)
    {
        std::vector<std::vector<int>> vector_index_list;
        std::vector<int> index_list;
        for(size_t i = 0; i < fragment_vector.size()-1; i++)
        {
            if(index_list.size()==0)
            {
                index_list.push_back(fragment_vector[i].from);
                index_list.push_back(fragment_vector[i].to);
            }

            if(fragment_vector[i].to == fragment_vector[i+1].from)
            {
                index_list.push_back(fragment_vector[i+1].to);
            }
            else
            {
                vector_index_list.push_back(index_list);
                index_list.clear();
            }
        }
        vector_index_list.push_back(index_list);

        // find the max length in the vector_index_list
        size_t max_vec_length = 0, vec_index = 0;
        for(size_t i = 0; i < vector_index_list.size(); i++)
        {
            if(vector_index_list[i].size() > max_vec_length)
            {
                max_vec_length = vector_index_list[i].size();
                vec_index = i;
            }
        }

        plane_index_list = vector_index_list[vec_index];
    }
    else if(fragment_vector.size() == 1)
    {
        plane_index_list.push_back(fragment_vector[0].from);
        plane_index_list.push_back(fragment_vector[0].to);
    }
    else
        return false;

#ifdef DEBUG_STAIR_MODEL
    std::cout<<"plane_index_list:"<<std::endl;
    for(size_t i = 0; i< plane_index_list.size();i++)
        std::cout<<RED<<"   "<<plane_index_list[i];
    std::cout<<RESET<<std::endl;
#endif
//####################### 6.from connect fragment find a longest stair link

//####################### 7.mark plane and find the main vertical normal
#ifdef DEBUG_STAIR_MODEL
    std::cout <<YELLOW<< "[7] mark plane and find the main vertical normal" <<RESET<< std::endl;
#endif        
    Eigen::Vector3f vnormal(0,0,0);
    for(size_t i = 0; i < d ; i++)
    {
        if(vector_plane_sorted[i].ptype != Plane::ground)
        {
            vector_plane_sorted[i].ptype = Plane::others;
            for(size_t j = 0; j < plane_index_list.size(); j++)
            {
                if(i == plane_index_list[j])
                {
                    if(vector_plane_sorted[i].type == Plane::vertical)
                    {
                        Eigen::Vector3f normal(vector_plane_sorted[i].coefficients.values[0],
                                vector_plane_sorted[i].coefficients.values[1],
                                vector_plane_sorted[i].coefficients.values[2]);

                        float dot_product = normal.dot(vnormal);
                        if(dot_product >= 0)
                            vnormal += normal*vector_plane_sorted[i].cloud.width;
                        else
                            vnormal -= normal*vector_plane_sorted[i].cloud.width;
                    }
                    vector_plane_sorted[i].ptype = Plane::stair_component;
                    break;
                }
            }
        }
    }
    vnormal.normalize();
    if(main_cv_directions[0].dir.dot(vnormal) < 0)
        vnormal = -vnormal;
//####################### 7.mark plane and find the main vertical normal

//####################### 8.compute stair slide plane
#ifdef DEBUG_STAIR_MODEL
    std::cout <<YELLOW<< "[8] compute stair slide plane" <<RESET<< std::endl;
#endif            
    // Section normal vector of stair, (snormal point to right)
    Eigen::Vector3f snormal = vnormal.cross(Eigen::Vector3f(-1,0,0));
    snormal.normalize();
    // compute slide planes point
    float right_max=-FLT_MAX,left_min=FLT_MAX;
    pcl::PointXYZRGBA r_max_point,l_min_point;
    for(size_t i = 0; i < plane_index_list.size(); i++)
    {
        int index = plane_index_list[i];
        Plane plane = vector_plane_sorted[index];
        if(plane.ptype == Plane::ground)
            continue;
        for(size_t i=0; i<plane.cloud.points.size();i++)
        {
            pcl::PointXYZ center = plane.center;
            Eigen::Vector3f vp(plane.cloud.points[i].x-center.x,
                    plane.cloud.points[i].y-center.y,
                    plane.cloud.points[i].z-center.z);
            float proj = snormal.dot(vp);
            if(proj > right_max)
            {
                right_max = proj;
                r_max_point.x = plane.cloud.points[i].x;
                r_max_point.y = plane.cloud.points[i].y;
                r_max_point.z = plane.cloud.points[i].z;
            }
            else if(proj < left_min)
            {
                left_min = proj;
                l_min_point.x = plane.cloud.points[i].x;
                l_min_point.y = plane.cloud.points[i].y;
                l_min_point.z = plane.cloud.points[i].z;
            }
        }
    }
    float D;
    D = -1*(snormal[0]*l_min_point.x + snormal[1]*l_min_point.y + snormal[2]*l_min_point.z);
    slide_plane_left.values.push_back(snormal[0]);
    slide_plane_left.values.push_back(snormal[1]);
    slide_plane_left.values.push_back(snormal[2]);
    slide_plane_left.values.push_back(D);

    D = -1*(snormal[0]*r_max_point.x + snormal[1]*r_max_point.y + snormal[2]*r_max_point.z);
    slide_plane_right.values.push_back(snormal[0]);
    slide_plane_right.values.push_back(snormal[1]);
    slide_plane_right.values.push_back(snormal[2]);
    slide_plane_right.values.push_back(D);

    stair_snormal = snormal;
    stair_vnormal = vnormal;
//####################### 8.compute stair slide plane

//####################### 9.modeling stair
#ifdef DEBUG_STAIR_MODEL
    std::cout <<YELLOW<< "[9] modeling stair" <<RESET<< std::endl;
#endif            
    ConcaveLine* prev_concave_line = NULL;
    Step* prev_step = NULL;

    // modeling
    int count = 0;
    for(size_t i = 0; i < plane_index_list.size()-1; i++)
    {
        int index = plane_index_list[i];
        int index_next = plane_index_list[i+1];
        if((vector_plane_sorted[index].type == Plane::horizontal)
            &&((vector_plane_sorted[index_next].type == Plane::vertical)))
        {
            pconcave_line = new ConcaveLine;

            Line line;
            pcl::ModelCoefficients plane1_coefficients,plane2_coefficients;

            plane1_coefficients = vector_plane_sorted[index].coefficients;
            plane2_coefficients = vector_plane_sorted[index_next].coefficients;

            computeLineFrom2Planes(plane1_coefficients,plane2_coefficients,line);

            pconcave_line->plane_h = &vector_plane_sorted[index];
            pconcave_line->plane_v = &vector_plane_sorted[index_next];
            pconcave_line->line = line;

            if(prev_step != NULL)
            {
                prev_step->depth = fabs(line.d-prev_step->line.d);
            }

            stair.pushBack((Node*)pconcave_line, concaveline_point );

            prev_concave_line = (ConcaveLine*)stair.getCurPoint();
        }
        else if((vector_plane_sorted[index].type == Plane::vertical)
                &&((vector_plane_sorted[index_next].type == Plane::horizontal)))
        {
            pstep = new Step;

            Line line;
            pcl::ModelCoefficients plane1_coefficients,plane2_coefficients;

            plane1_coefficients = vector_plane_sorted[index].coefficients;
            plane2_coefficients = vector_plane_sorted[index_next].coefficients;

            computeLineFrom2Planes(plane1_coefficients,plane2_coefficients,line);

            pstep->plane_v = &vector_plane_sorted[index];
            pstep->plane_h = &vector_plane_sorted[index_next];
            pstep->line = line;
            pstep->count = ++count;

            if(prev_concave_line != NULL)
            {
                pstep->height = fabs(line.h-prev_concave_line->line.h);
            }else{
                pcl::PointXYZRGBA point_max,point_min;
                findMinMaxProjPoint(vector_plane_sorted[index],1,0,0,point_max,point_min);
                pstep->height = fabs(point_max.x - line.h);
            }

            stair.pushBack((Node*)pstep,step_point);

            prev_step = (Step*)stair.getCurPoint();
        }
        else if((vector_plane_sorted[index].type == Plane::horizontal)
                &&((vector_plane_sorted[index_next].type == Plane::horizontal)))
        {
            Plane virtual_virtcal_plane;
            pcl::PointXYZRGBA point_h0,point_h1,tmp;

            // find the back point of the lower plane
            findMinMaxProjPoint(vector_plane_sorted[index],vnormal[0],vnormal[1],vnormal[2],point_h0,tmp);

            // find the front point of the upper plane
            findMinMaxProjPoint(vector_plane_sorted[index_next],vnormal[0],vnormal[1],vnormal[2],tmp,point_h1);

            // compute center point of virtual vertical plane
            float cx,cy,cz;
            cx = (point_h0.x + point_h1.x)/2;
            cy = (point_h0.y + point_h1.y)/2;
            cz = (point_h0.z + point_h1.z)/2;

            virtual_virtcal_plane.center.x = cx;
            virtual_virtcal_plane.center.y = cy;
            virtual_virtcal_plane.center.z = cz;

            // compute the coefficients of virtual vertical plane
            float d = -cx*vnormal(0)-cy*vnormal(1)-cz*vnormal(2);
            virtual_virtcal_plane.coefficients.values.push_back(vnormal(0));
            virtual_virtcal_plane.coefficients.values.push_back(vnormal(1));
            virtual_virtcal_plane.coefficients.values.push_back(vnormal(2));
            virtual_virtcal_plane.coefficients.values.push_back(d);
            // get the min max point
            virtual_virtcal_plane.points_min[1] = point_h0;
            virtual_virtcal_plane.points_max[1] = point_h1;

            virtual_virtcal_plane.counter.points.resize(4);
            virtual_virtcal_plane.counter.width = 0;
            virtual_virtcal_plane.counter.height = 1;

            // push back the virtual_virtcal_plane to vector_plane_sorted
            vector_plane_sorted.push_back(virtual_virtcal_plane);

            /////////////////////ConcaveLine
            pconcave_line = new ConcaveLine;
            Line line1;
            pcl::ModelCoefficients plane1_coefficients,plane2_coefficients;

            plane1_coefficients = vector_plane_sorted[index].coefficients;
            plane2_coefficients = virtual_virtcal_plane.coefficients;
            computeLineFrom2Planes(plane1_coefficients,plane2_coefficients,line1);

            pconcave_line->plane_h = &vector_plane_sorted[index];
            pconcave_line->plane_v = &vector_plane_sorted[vector_plane_sorted.size()-1];
            pconcave_line->line = line1;

            if(prev_step != NULL)
                prev_step->depth = fabs(line1.d-prev_step->line.d);

            stair.pushBack((Node*)pconcave_line, concaveline_point );

            /////////////////////Step
            pstep = new Step;
            Line line2;
            plane1_coefficients = vector_plane_sorted[index_next].coefficients;
            computeLineFrom2Planes(plane1_coefficients,plane2_coefficients,line2);

            pstep->plane_v = &vector_plane_sorted[vector_plane_sorted.size()-1];
            pstep->plane_h = &vector_plane_sorted[index_next];
            pstep->line = line2;
            pstep->count = ++count;

            if(prev_concave_line != NULL)
                pstep->height = fabs(line2.h-prev_concave_line->line.h);
            else
                pstep->height = fabs(vector_plane_sorted[index].center.x - vector_plane_sorted[index_next].center.x);

            stair.pushBack((Node*)pstep,step_point);

            prev_step = (Step*)stair.getCurPoint();
        }
    }
    stair.pushBackEnd();
//####################### 9.modeling stair

    if(count > 0)
        return true;
    else
        return false;
}

/** \brief find the point whose projection in on vector is max and min
    * \param[in] plane: include input cloud
    * \param[in] vector: (vx,vy,vz),the projection direction
    * \param[out] point: max_point and min_point
    */
inline void StairDetection::findMinMaxProjPoint(const Plane &plane,
        const float &vx,const float &vy,const float &vz,
        pcl::PointXYZRGBA &max_point,pcl::PointXYZRGBA &min_point)
{
    float max_proj=-FLT_MAX,min_proj=FLT_MAX;
    Eigen::Vector3f ve(vx,vy,vz);
    for(size_t i=0; i<plane.cloud.points.size();i++)
    {
        pcl::PointXYZ center = plane.center;
        Eigen::Vector3f vp(plane.cloud.points[i].x-center.x,
                plane.cloud.points[i].y-center.y,
                plane.cloud.points[i].z-center.z);
        float proj = ve.dot(vp);
        if(proj > max_proj)
        {
            max_proj = proj;
            max_point.x = plane.cloud.points[i].x;
            max_point.y = plane.cloud.points[i].y;
            max_point.z = plane.cloud.points[i].z;
        }
        if(proj < min_proj)
        {
            min_proj = proj;
            min_point.x = plane.cloud.points[i].x;
            min_point.y = plane.cloud.points[i].y;
            min_point.z = plane.cloud.points[i].z;
        }
    }
}

/** \brief compute cross point of plane and line
    * \param[in] line: (x-x0)/z=(y-y0)/b=(z-z0)/c
    * \param[in] plnae: A(x-x1)+B(y-y1)+C(z-z1)=0
    * \param[out] point: pc(xc,yc,zc)
    */
inline void StairDetection::crossPointOfLineAndPlane(
        const float &x0,const float &y0,const float &z0,const float &a,const float &b,const float &c,
        const float &x1,const float &y1,const float &z1,const float &A,const float &B,const float &C,
        pcl::PointXYZ &pc)
{
    float t = A*(x1-x0)+B*(y1-y0)+C*(z1-z0);
    t = t / (A*a+B*b+C*c);
    pc.x = a*t + x0;
    pc.y = b*t + y0;
    pc.z = c*t + z0;
}

inline void StairDetection::crossPointOfLineAndPlane(
        const float &x0,const float &y0,const float &z0,const float &a,const float &b,const float &c,
        const pcl::ModelCoefficients plane_coefficients,
        pcl::PointXYZ &pc)
{
    float A,B,C,D;
    A = plane_coefficients.values[0];
    B = plane_coefficients.values[1];
    C = plane_coefficients.values[2];
    D = plane_coefficients.values[3];
    float t = -1*(A*x0+B*y0+C*z0+D);
    t = t / (A*a+B*b+C*c);
    pc.x = a*t + x0;
    pc.y = b*t + y0;
    pc.z = c*t + z0;
}

/** \brief cross product of two normal vectors
    * \param[in] normal vector: n0 = (n01,n02,n03)
    * \param[in] normal vector: n1 = (n11,n12,n13)
    * \param[out] normal vector: nc = n0*n1
    */
inline void StairDetection::crossProduct(
        const float &n01,const float &n02,const float &n03,
        const float &n11,const float &n12,const float &n13,
        float &nc1,float &nc2,float &nc3)
{
    Eigen::Vector3f normal1(n01,n02,n03);
    Eigen::Vector3f normal2(n11,n12,n13);
    Eigen::Vector3f normalc = normal1.cross(normal2);
    normalc.normalize();
    nc1 = normalc[0];nc2 = normalc[1];nc3 = normalc[2];
}

void StairDetection::computePlaneCounter(Stair &stair)
{
    #ifdef DEBUG_CONSOLE
    pcl::ScopeTime scope_time ("Compute Plane Counter");
    #endif

    Node* pnext;
    pcl::PointXYZ point;
    pnext = stair.getHead();
    while(true)
    {
        if(pnext->pnext_type == step_point)
        {
            Step *pstep = (Step*)(pnext->pnext);

            if(pstep->plane_v->counter.width == 0)
            {
                // 台阶垂直面上边沿直线参数
                float xu=pstep->line.coeff.values[0];
                float yu=pstep->line.coeff.values[1];
                float zu=pstep->line.coeff.values[2];
                float nux=pstep->line.coeff.values[3];
                float nuy=pstep->line.coeff.values[4];
                float nuz=pstep->line.coeff.values[5];

                // 台阶垂直面下边沿直线参数
                pcl::PointXYZRGBA max_point,min_point;
                findMinMaxProjPoint(*pstep->plane_v,1,0,0,max_point,min_point);
                float xd,yd,zd,ndx,ndy,ndz;
                xd = max_point.x;yd = max_point.y;zd = max_point.z;
                ndx=nux;ndy=nuy;ndz=nuz;

                // 左下侧交点
                crossPointOfLineAndPlane(xd,yd,zd,ndx,ndy,ndz,slide_plane_left,point);
                pstep->plane_v->counter.points[0] = point;
                // 右下侧交点
                crossPointOfLineAndPlane(xd,yd,zd,ndx,ndy,ndz,slide_plane_right,point);
                pstep->plane_v->counter.points[1] = point;
                // 右上侧交点
                crossPointOfLineAndPlane(xu,yu,zu,nux,nuy,nuz,slide_plane_right,point);
                pstep->plane_v->counter.points[2] = point;
                // 左上侧交点
                crossPointOfLineAndPlane(xu,yu,zu,nux,nuy,nuz,slide_plane_left,point);
                pstep->plane_v->counter.points[3] = point;

                pstep->plane_v->counter.width = 4;
            }
            else if(pstep->plane_v->counter.width == 2)
            {
                // 台阶垂直面上边沿直线参数
                float xu=pstep->line.coeff.values[0];
                float yu=pstep->line.coeff.values[1];
                float zu=pstep->line.coeff.values[2];
                float nux=pstep->line.coeff.values[3];
                float nuy=pstep->line.coeff.values[4];
                float nuz=pstep->line.coeff.values[5];

                // 右上侧交点
                crossPointOfLineAndPlane(xu,yu,zu,nux,nuy,nuz,slide_plane_right,point);
                pstep->plane_v->counter.points[2] = point;
                // 左上侧交点
                crossPointOfLineAndPlane(xu,yu,zu,nux,nuy,nuz,slide_plane_left,point);
                pstep->plane_v->counter.points[3] = point;

                pstep->plane_v->counter.width = 4;
            }

            if(pstep->plane_h->counter.width == 0)
            {
                // 台阶水平面前沿直线参数
                float xf=pstep->line.coeff.values[0];
                float yf=pstep->line.coeff.values[1];
                float zf=pstep->line.coeff.values[2];
                float nfx=pstep->line.coeff.values[3];
                float nfy=pstep->line.coeff.values[4];
                float nfz=pstep->line.coeff.values[5];

                // 左前侧交点
                crossPointOfLineAndPlane(xf,yf,zf,nfx,nfy,nfz,slide_plane_left,point);
                pstep->plane_h->counter.points[0] = point;
                // 右前侧交点
                crossPointOfLineAndPlane(xf,yf,zf,nfx,nfy,nfz,slide_plane_right,point);
                pstep->plane_h->counter.points[1] = point;

                pstep->plane_h->counter.width = 2;
                // last one
                if(pnext->pnext->pnext->pnext == NULL)
                {
                    pcl::PointXYZRGBA max_point,min_point;
                    // 找点云在stair_vnormal方向上的最远点和最近点
                    findMinMaxProjPoint(*pstep->plane_h,stair_vnormal(0),stair_vnormal(1),stair_vnormal(2),
                            max_point,min_point);

                    // 后侧交点
                    float xb,yb,zb,nbx,nby,nbz;
                    xb = max_point.x;yb = max_point.y;zb = max_point.z;
                    nbx = nfx;nby = nfy;nbz = nfz;

                    // 右后侧交点
                    crossPointOfLineAndPlane(xb,yb,zb,nbx,nby,nbz,slide_plane_right,point);
                    pstep->plane_h->counter.points[2] = point;
                    // 左后侧交点
                    crossPointOfLineAndPlane(xb,yb,zb,nbx,nby,nbz,slide_plane_left,point);
                    pstep->plane_h->counter.points[3] = point;

                    pstep->plane_h->counter.width = 4;
                }
            }
        }
        else if(pnext->pnext_type == concaveline_point)
        {
            ConcaveLine *pline = (ConcaveLine*)(pnext->pnext);

            if(pline->plane_h->counter.width == 0)
            {
                // 平面后边沿直线参数
                float xb=pline->line.coeff.values[0];
                float yb=pline->line.coeff.values[1];
                float zb=pline->line.coeff.values[2];
                float nbx=pline->line.coeff.values[3];
                float nby=pline->line.coeff.values[4];
                float nbz=pline->line.coeff.values[5];

                pcl::PointXYZRGBA max_point,min_point;
                // 找点云在stair_vnormal方向上的最远点和最近点
                findMinMaxProjPoint(*pline->plane_h,stair_vnormal(0),stair_vnormal(1),stair_vnormal(2),
                        max_point,min_point);

                // 前侧交点
                float xf,yf,zf,nfx,nfy,nfz;
                xf = min_point.x;yf = min_point.y;zf = min_point.z;
                nfx = nbx;nfy = nby;nfz = nbz;

                // 左前侧交点
                crossPointOfLineAndPlane(xf,yf,zf,nfx,nfy,nfz,slide_plane_left,point);
                pline->plane_h->counter.points[0] = point;
                // 右前侧交点
                crossPointOfLineAndPlane(xf,yf,zf,nfx,nfy,nfz,slide_plane_right,point);
                pline->plane_h->counter.points[1] = point;

                // 右后侧交点
                crossPointOfLineAndPlane(xb,yb,zb,nbx,nby,nbz,slide_plane_right,point);
                pline->plane_h->counter.points[2] = point;
                // 左后侧交点
                crossPointOfLineAndPlane(xb,yb,zb,nbx,nby,nbz,slide_plane_left,point);
                pline->plane_h->counter.points[3] = point;

                pline->plane_h->counter.width = 4;
            }
            else if(pline->plane_h->counter.width == 2)
            {
                // 平面后边沿直线参数
                float xb=pline->line.coeff.values[0];
                float yb=pline->line.coeff.values[1];
                float zb=pline->line.coeff.values[2];
                float nbx=pline->line.coeff.values[3];
                float nby=pline->line.coeff.values[4];
                float nbz=pline->line.coeff.values[5];

                // 右后侧交点
                crossPointOfLineAndPlane(xb,yb,zb,nbx,nby,nbz,slide_plane_right,point);
                pline->plane_h->counter.points[2] = point;
                // 左后侧交点
                crossPointOfLineAndPlane(xb,yb,zb,nbx,nby,nbz,slide_plane_left,point);
                pline->plane_h->counter.points[3] = point;

                pline->plane_h->counter.width = 4;
            }

            if(pline->plane_v->counter.width == 0)
            {
                // 平面下边沿直线参数
                float xd=pline->line.coeff.values[0];
                float yd=pline->line.coeff.values[1];
                float zd=pline->line.coeff.values[2];
                float ndx=pline->line.coeff.values[3];
                float ndy=pline->line.coeff.values[4];
                float ndz=pline->line.coeff.values[5];

                // 左下侧交点
                crossPointOfLineAndPlane(xd,yd,zd,ndx,ndy,ndz,slide_plane_left,point);
                pline->plane_v->counter.points[0] = point;
                // 右下侧交点
                crossPointOfLineAndPlane(xd,yd,zd,ndx,ndy,ndz,slide_plane_right,point);
                pline->plane_v->counter.points[1] = point;

                pline->plane_v->counter.width = 2;
                // last one
                if(pnext->pnext->pnext->pnext == NULL)
                {
                    // 上侧面所过的点
                    pcl::PointXYZRGBA max_point,min_point;
                    findMinMaxProjPoint(*pline->plane_v,1,0,0,max_point,min_point);
                    float xu,yu,zu,nux,nuy,nuz;
                    xu = min_point.x;yu = min_point.y;zu = min_point.z;
                    nux = ndx;nuy = ndy;nuz = ndz;

                    // 右上侧交点
                    crossPointOfLineAndPlane(xu,yu,zu,nux,nuy,nuz,slide_plane_right,point);
                    pline->plane_v->counter.points[2] = point;
                    // 左上侧交点
                    crossPointOfLineAndPlane(xu,yu,zu,nux,nuy,nuz,slide_plane_left,point);
                    pline->plane_v->counter.points[3] = point;

                    pline->plane_v->counter.width = 4;
                }
            }
        }
        else
        {
            break;
        }
            pnext = pnext->pnext;
    }
}