#include "reader.h"
#include <utility>

Reader::Reader(QObject *parent)
{
    running = false;
    cloud = CloudType::Ptr(new CloudType(512, 424));
    cloud_in = CloudType::Ptr(new CloudType(512, 424));
    pk2g = new K2G(true);
    prev = false;
    next = false;
    kinect_ok = false;
    pause = false;
    run_mode = files_mode;
}

Reader::~Reader()
{
    if (run_mode == kinect_mode)
    {
        pk2g->shutDown();
        close_imu();
        delete pk2g;
    }
}


void Reader::stop()
{
    mutex_running.lock();
    running = false;
    mutex_running.unlock();
}

void Reader::update_pause(bool p)
{
    pause = p;
}


void Reader::open_imu_serial()
{
    if (open_imu(imu_serial_name.c_str()) == -1)
    {
        imu_ok = false;
        emit imu_status_update(false);
    } else
    {
        imu_ok = true;
        emit imu_status_update(true);
    }
}


void Reader::update_serial_name(std::string imu)
{
    imu_serial_name = std::move(imu);

    mutex_running.lock();
    run_mode_mutex.lock();
    if (running && run_mode == kinect_mode)
        open_imu_serial();
    run_mode_mutex.unlock();
    mutex_running.unlock();
}

bool Reader::read_file_numbers()
{
    if (access(file_path.c_str(), 0) == -1)
    {
        std::cout << RED << "Can't access \"" << file_path << "\"" << RESET << std::endl;
        return false;
    } else
    {
        count = 0;
        // get number of files
        DIR *dir;
        struct dirent *ptr;
        dir = opendir(file_path.c_str());
        while ((ptr = readdir(dir)) != nullptr)
            count++;
        closedir(dir);

        std::cout << GREEN;
        std::cout << "Find " << count - 2 << " files in \"" << file_path << "\"" << std::endl;
        std::cout << RESET;
    }
    read_file_count = 0;

    return true;
}

void Reader::update_file_path(std::string path)
{
    file_path = std::move(path);
    valid_file_path = read_file_numbers();
}

cv::Mat convertTo(const cv::Mat &mat, int depth)
{
    if (mat.depth() == depth)
        return mat;

    cv::Mat result;
    mat.convertTo(result, depth);
    return result;
}

void Reader::save_images()
{
    if (run_mode == kinect_mode)
    {
        std::ostringstream oss;
        pcl::PCDWriter writer;

        // construct name
        oss.str("");
        oss << file_path << "/" << std::setfill('0') << std::setw(4) << savecount;
        const std::string cloudName = oss.str() + "_cloud.pcd";
        const std::string colorName = oss.str() + "_color.png";
        const std::string depthName = oss.str() + "_depth.png";
        const std::string imuName = oss.str() + "_imu.txt";

        // save cloud, color image, and depth image
        cloud_mutex.lock();
        writer.writeBinary(cloudName, *cloud);
        cv::imwrite(colorName, color_img);
        cv::imwrite(depthName, convertTo(depth_img, CV_16UC1));
        cloud_mutex.unlock();

        ofstream file;
        file.open(imuName.c_str(), ios::out);
        if (file.is_open())
        {
            time_t timep;
            struct tm *p;
            std::vector<std::string> wday;
            wday.emplace_back("Sun");
            wday.emplace_back("Mon");
            wday.emplace_back("Tue");
            wday.emplace_back("Wed");
            wday.emplace_back("Thu");
            wday.emplace_back("Fri");
            wday.emplace_back("Sat");
            time(&timep);
            p = localtime(&timep);
            file << "[" << 1900 + p->tm_year << "-" << 1 + p->tm_mon << "-" << p->tm_mday;
            file << "-" << wday[p->tm_wday] << " " << p->tm_hour << ":" << p->tm_min << ":" << p->tm_sec << "]"
                 << std::endl;

            file << "Euler[0-2],Acc[0-2],Gyo[0-2],Mag[0-2]" << std::endl;

            file << Euler[0] << "," << Euler[1] << "," << Euler[2] << ","
                 << Acc[0] << "," << Acc[1] << "," << Acc[2] << ","
                 << Gyo[0] << "," << Gyo[1] << "," << Gyo[2] << ","
                 << Mag[0] << "," << Mag[1] << "," << Mag[2] << "," << std::endl;

            file.close();
        } else
            std::cout << RED << "open file \"" << imuName << "\" FAILED" << RESET << std::endl;

        std::cout << YELLOW << "Saved " << cloudName << " successes!" << RESET << std::endl;

        savecount++;
    }
}

void Reader::update_run_mode(RunModeDef mode)
{
    //std::cout<<"run_mode_mutex lock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
    run_mode_mutex.lock();
    //std::cout<<"run_mode_mutex lock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;

    // didn't change, do nothing
    if (run_mode == mode)
    {
        run_mode_mutex.unlock();
        return;
    }
        // kinect -> files
    else if (run_mode != files_mode && mode == files_mode)
    {
        //std::cout<<"lock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        kinect_mutex.lock();
        //std::cout<<"lock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        //std::cout<<"pk2g->stop();"<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        // stop kinect
        pk2g->stop();
        kinect_ok = false;
        close_imu();

        run_mode = mode;

        //std::cout<<"unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        kinect_mutex.unlock();
        //std::cout<<"unlock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;

        // read file list
        valid_file_path = read_file_numbers();
    } else if (run_mode != kinect_mode && mode == kinect_mode)
    {
        //std::cout<<"lock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        kinect_mutex.lock();
        //std::cout<<"lock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        if (pk2g->start(OPENGL))
        {
            //std::cout<<"pk2g->start(OPENGL)) OK: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
            open_imu_serial();
            pk2g->disableLog();
            kinect_ok = true;
        } else
        {
            //std::cout<<"pk2g->start(OPENGL)) NOT OK: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
            kinect_ok = false;
        }

        run_mode = mode;
        //std::cout<<"unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        kinect_mutex.unlock();
        //std::cout<<"unlock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
    }

    //std::cout<<"run_mode_mutex unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
    run_mode_mutex.unlock();
    //std::cout<<"run_mode_mutex unlock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
}

void computeRotationMatrix(const float pitch,
                           const float roll,
                           const float yaw,
                           Eigen::Matrix3f &rotation_matrix)
{
    Eigen::Matrix3f m1,m2,m3;
    m1 <<   0, 0, 1,
            -1, 0, 0,
            0, -1, 0;
//    m1 = Eigen::AngleAxisf(yaw / 180.0 * M_PI, Eigen::Vector3f::UnitY())
//         * Eigen::AngleAxisf(-pitch / 180.0 * M_PI, Eigen::Vector3f::UnitX())
//         * Eigen::AngleAxisf(roll / 180.0 * M_PI, Eigen::Vector3f::UnitZ());
    m2 = Eigen::AngleAxisf(roll / 180.0 * M_PI, Eigen::Vector3f::UnitX())
         * Eigen::AngleAxisf(pitch / 180.0 * M_PI, Eigen::Vector3f::UnitY())
         * Eigen::AngleAxisf(yaw / 180.0 * M_PI, Eigen::Vector3f::UnitZ());

    m3 <<   0, 0, -1,
            1, 0, 0,
            0,-1, 0;

    rotation_matrix = m3 * m2 * m1;
}

void computeRotationMatrix(const float W,
                           const float X,
                           const float Y,
                           const float Z,
                           Eigen::Matrix3f &rotation_matrix)
{
    Eigen::Quaternionf quaternionf(W, X, Y, Z);
    Eigen::Matrix3f m1,m2,m3;

    m1 <<   0, 0, 1,
            -1, 0, 0,
            0, -1, 0;

    m2 = quaternionf.normalized().toRotationMatrix();

    m3 <<   0, 0, -1,
            1, 0, 0,
            0,-1, 0;

    rotation_matrix = m3 * m2 * m1;
}

void rotationCloudPoints(const CloudType &cloud_in,
                         const Eigen::Matrix3f rotation_matrix,
                         CloudType &cloud_rotation)
{
    cloud_rotation.is_dense = false;
    cloud_rotation.resize(512 * 424);
    float r00, r01, r02, r10, r11, r12, r20, r21, r22;

    r00 = rotation_matrix(0, 0);
    r01 = rotation_matrix(0, 1);
    r02 = rotation_matrix(0, 2);
    r10 = rotation_matrix(1, 0);
    r11 = rotation_matrix(1, 1);
    r12 = rotation_matrix(1, 2);
    r20 = rotation_matrix(2, 0);
    r21 = rotation_matrix(2, 1);
    r22 = rotation_matrix(2, 2);

#ifndef _OPENMP
    std::cerr<< "OpenMP not supported";
#endif

    size_t height = 424, width = 512;
//     #pragma omp parallel for
    for (size_t i = 0; i < height; i = i + 1)
    {
        for (size_t j = 0; j < width; j = j + 1)
        {
            cloud_rotation.points[i * width + j].r = cloud_in.points[i * width + j].r;
            cloud_rotation.points[i * width + j].g = cloud_in.points[i * width + j].g;
            cloud_rotation.points[i * width + j].b = cloud_in.points[i * width + j].b;
            cloud_rotation.points[i * width + j].a = cloud_in.points[i * width + j].a;
            if (!std::isnan(cloud_in.points[i * width + j].x))
            {
                cloud_rotation.points[i * width + j].x =
                        r00 * cloud_in.points[i * width + j].x +
                        r01 * cloud_in.points[i * width + j].y +
                        r02 * cloud_in.points[i * width + j].z;
                cloud_rotation.points[i * width + j].y =
                        r10 * cloud_in.points[i * width + j].x +
                        r11 * cloud_in.points[i * width + j].y +
                        r12 * cloud_in.points[i * width + j].z;
                cloud_rotation.points[i * width + j].z =
                        r20 * cloud_in.points[i * width + j].x +
                        r21 * cloud_in.points[i * width + j].y +
                        r22 * cloud_in.points[i * width + j].z;
            }
        }
    }
    cloud_rotation.width = static_cast<uint32_t>(width);
    cloud_rotation.height = static_cast<uint32_t>(height);
}

void Reader::run()
{
    mutex_running.lock();
    running = true;
    mutex_running.unlock();

    std::ostringstream oss;

    while (running)
    {
        start_point:

        //std::cout<<"run_mode_mutex lock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        run_mode_mutex.lock();
        //std::cout<<"run_mode_mutex lock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        if (run_mode == kinect_mode)
        {
            //std::cout<<"run_mode_mutex unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
            run_mode_mutex.unlock();
            //std::cout<<"run_mode_mutex unlock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;

            while (pause && running)
            {
                //std::cout<<"pause: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
                if (run_mode == kinect_mode)
                    msleep(500);
                else
                    goto start_point;
            }


            if (kinect_ok && imu_ok)
            {
                //std::cout<<"cloud_mutex lock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
                cloud_mutex.lock();
                //std::cout<<"cloud_mutex lock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;

                //std::cout<<"lock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
                kinect_mutex.lock();
                //std::cout<<"lock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
                if (run_mode != kinect_mode)
                {
                    kinect_mutex.unlock();
                    cloud_mutex.unlock();
                    //std::cout<<"goto start_point; "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
                    goto start_point;
                }
                //std::cout<<"pk2g->get(color_img, depth_img, cloud_in);"<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
                pk2g->get(color_img, depth_img, cloud_in);
                read_once(Acc, Gyo, Mag, Euler, Quat);
                //std::cout<<"unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
                kinect_mutex.unlock();
                //std::cout<<"unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;

                pose.pitch = Euler[0];
                pose.roll = Euler[1];
                pose.yaw = Euler[2];

//                computeRotationMatrix(pose.pitch, pose.roll, pose.yaw, rotation_matrix);
                computeRotationMatrix(Quat[0], Quat[1], Quat[2], Quat[3], rotation_matrix);

                rotationCloudPoints(*cloud_in, rotation_matrix, *cloud);

                //std::cout<<"cloud_mutex unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
                cloud_mutex.unlock();
                //std::cout<<"cloud_mutex unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
            }


            if (!kinect_ok)
            {
                msleep(1000);
                std::cout << RED << "KINECT ERROR!" << RESET << std::endl;
            }

            if (!imu_ok)
            {
                msleep(1000);
                std::cout << RED << "IMU ERROR!" << RESET << std::endl;
            }

            msleep(5);
        } else
        {
            //std::cout<<"run_mode_mutex unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
            run_mode_mutex.unlock();
            //std::cout<<"run_mode_mutex unlock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;

            while (!valid_file_path && running)
            {
                if (run_mode == files_mode)
                    msleep(100);
                else
                    goto start_point;
            }

            while (!prev && !next && running)
            {
                if (run_mode == files_mode)
                    msleep(100);
                else
                    goto start_point;
            }

            prev_mutex.lock();
            if (next)
            {
                read_file_count++;
                next = false;
            }

            if (prev)
            {
                prev = false;
                read_file_count--;
                if (read_file_count < 0)
                    read_file_count = (count - 2) / 4 - 1;
            }
            prev_mutex.unlock();

            if (read_file_count >= (count - 2) / 4)
                read_file_count = 0;

            oss.str("");
            oss << file_path << std::setfill('0') << "/" << std::setw(4) << read_file_count;
            const std::string cloudName = oss.str() + "_cloud.pcd";
            const std::string colorName = oss.str() + "_color.png";
            const std::string depthName = oss.str() + "_depth.png";
            const std::string imuName = oss.str() + "_imu.txt";

            std::cout << YELLOW;
            std::cout << "Opening: " << cloudName << std::endl;
            std::cout << "Opening: " << imuName << std::endl;
            std::cout << RESET;

            // read imu info
            std::ifstream file;
            std::vector<std::vector<float> > vector_euler;
            file.open(imuName.c_str());
            if (file.is_open())
            {
                std::string line;
                std::vector<std::string> vector_word;
                std::getline(file, line);
                std::getline(file, line);
                std::getline(file, line);
                std::vector<float> imu;
                boost::split(vector_word, line, boost::is_any_of(","));
                for (size_t i = 0; i < vector_word.size() - 1; i++)
                {
                    imu.push_back(atof(vector_word[i].c_str()));
                }
                file.close();

                pose.pitch = imu[0];
                pose.roll = imu[1];
                pose.yaw = imu[2];
            } else
            {
                std::cout << RED;
                std::cout << "Can not able to open file \"" << imuName << "\".\n";
                std::cout << RESET;
                continue;
            }
            std::cout << "Pitch:" << pose.pitch << "\tRoll:" << pose.roll << "\tYaw:" << pose.yaw << std::endl;

            // read PCD file
            if (pcl::io::loadPCDFile(cloudName, *cloud) == -1)
            {
                std::cout << RED;
                std::cout << "Can not able to open file \"" << cloudName << "\".\n";
                std::cout << RESET;
                continue;
            }
            std::cout << "Cloud points number:" << cloud->points.size() << std::endl;

            // read color and depth file
            color_img = cv::imread(colorName);
            depth_img = cv::imread(depthName, cv::IMREAD_ANYDEPTH);
        }
        //std::cout<<"run_mode_mutex unlock in: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;
        run_mode_mutex.unlock();
        //std::cout<<"run_mode_mutex unlock out: "<<__LINE__<<" in:"<<__FUNCTION__<<std::endl;

//        cv::cvtColor(color_img, color_img, cv::COLOR_BGR2RGB);
        emit color_image_update(&color_img);
        emit depth_image_update(&depth_img);
        emit cloud_update(cloud);
        emit imu_update(pose);
    }
}

void Reader::read_file_prev()
{
    prev_mutex.lock();
    prev = true;
    prev_mutex.unlock();
}

void Reader::read_file_next()
{
    prev_mutex.lock();
    next = true;
    prev_mutex.unlock();
}
