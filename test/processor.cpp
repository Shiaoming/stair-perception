#include "processor.h"
#include <utility>
#include <common.h>

DProcessor::DProcessor()
{
    running = false;
    new_cloud_come = false;
    pause = false;
    show_mode = original;
    first_result = true;
    result.has_stair = false;
    result.height = 0;
    result.time = 999;
    result.v_depth = 0;
    result.v_height = 0;
    result.width = 0;
}

DProcessor::~DProcessor()
= default;

void DProcessor::stop()
{
    running = false;
}

void DProcessor::update_robot_name(std::string robot)
{
    robot_serial_name = std::move(robot);
    open_robot_serial();
}

void DProcessor::open_robot_serial()
{
    if (sender.Open(robot_serial_name.c_str(), 9600, 8, EVEN, 1) == 0)
    {
        serial_ok = false;
    } else
    {
        serial_ok = true;
    }
    emit robot_serial_update(serial_ok);
}

void DProcessor::resolve_result(Stair stair)
{
    if (has_stair)
    {
        ResultDef tmp_result;

        Node *pnext;
        pnext = stair.getHead();

        while (true)
        {
            if (pnext->pnext_type == step_point)
            {
                auto *pstep = (Step *) (pnext->pnext);
                tmp_result.height = pstep->height * 1000;
                tmp_result.width = pstep->depth * 1000;
                tmp_result.v_height = pstep->line.h * 1000;
                tmp_result.v_depth = pstep->line.d * 1000;
                break;
            }

            pnext = pnext->pnext;
        }

        if(tmp_result.height > 80 && tmp_result.height < 150)
            result.height = tmp_result.height;

        if(tmp_result.width > 250 && tmp_result.width < 330)
            result.width = tmp_result.width;

        if(tmp_result.v_height > 700 && tmp_result.v_height < 1150)
            result.v_height = tmp_result.v_height;

        if(tmp_result.v_depth > 0 && tmp_result.v_depth < 600)
            result.v_depth = tmp_result.v_depth;
    }
}


void DProcessor::pack_and_send_robot()
{
    if (serial_ok)
    {
        package_ pack;
        pack.head = 0xFF;
        pack.flag = static_cast<uint8_t>(result.has_stair);
        pack.height = result.height;
        pack.depth = result.width;
        pack.v_height = result.v_height;
        pack.v_depth = result.v_depth;
        pack.tail = 0xFE;

        sender.Write(pack.data, 19);
    }
}

void DProcessor::update_parameters(ParameterDef p)
{
    parameters = p;

    new_cloud_mutex.lock();
    new_cloud_come = true;
    new_cloud_mutex.unlock();
}


void DProcessor::update_show_mode(ShowModeDef mode)
{
    show_mode = mode;

    new_cloud_mutex.lock();
    new_cloud_come = true;
    new_cloud_mutex.unlock();
}

void DProcessor::update_cloud_in(const CloudType::Ptr cloud)
{
    cloud_in_mutex.lock();
    cloud_in = cloud;
    cloud_in_mutex.unlock();

    new_cloud_mutex.lock();
    new_cloud_come = true;
    new_cloud_mutex.unlock();
}

void DProcessor::update_pause(bool p)
{
    pause = p;
}

void DProcessor::run()
{
    struct timeval tv{};
    running = true;
    while (running)
    {
        while (pause && running)
            msleep(500);

        while (!new_cloud_come && running)
            msleep(50);

        if (!running)
            break;

        new_cloud_mutex.lock();
        new_cloud_come = false;
        new_cloud_mutex.unlock();

        StairDetection stair_detection(parameters);
        Stair stair;

        cloud_in_mutex.lock();
        if (cloud_in)
        {
            cloud_in_copy = *cloud_in;
            cloud_in_mutex.unlock();
        } else
        {
            cloud_in_mutex.unlock();
            continue;
        }

        gettimeofday(&tv, nullptr);
        long long time1 = tv.tv_sec * 1000 + tv.tv_usec / 1000;

        has_stair = stair_detection.process(cloud_in_copy, show_mode, stair,
                                            vector_plane_sorted, cloud_out, normal_out);

        gettimeofday(&tv, nullptr);
        long long time2 = tv.tv_sec * 1000 + tv.tv_usec / 1000;

        if(has_stair)
            stair_detection.printStairEstParam(stair);

        result.has_stair = has_stair;
        result.time = time2 - time1;
        resolve_result(stair);
        pack_and_send_robot();
        emit result_update(result);

        if (show_mode == range_limited || show_mode == voxel || show_mode == noleg)
            // || show_mode == horizontal_cloud || show_mode == vertical_cloud )
        {
            if (!cloud_out.points.empty())
                    emit cloud_update(cloud_out.makeShared());
//             emit normal_update(normal_out);
        } else if (show_mode != original)
        {
            emit planes_update(&vector_plane_sorted);
        }

        msleep(2);
    }
}
