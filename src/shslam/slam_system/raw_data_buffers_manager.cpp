#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/raw_data_buffers_manager.hpp>

namespace shslam
{
    shslam::SlamSystem::RawDataBuffersManager::RawDataBuffersManager() :
    raw_data_buffers_ptr{std::make_shared<shslam::RawDataBuffers>()}
    {}

    std::shared_ptr<shslam::RawDataBuffers> shslam::SlamSystem::RawDataBuffersManager::GetBuffersPtr()
    {
        return raw_data_buffers_ptr;
    }

    void shslam::SlamSystem::RawDataBuffersManager::Init(std::shared_ptr<NumSensors> num_sensors_ptr)
    {
        printf("Initialize buffers.\n");
        raw_data_buffers_ptr->mono_imgs.clear();
        raw_data_buffers_ptr->mono_imgs.resize(num_sensors_ptr->mono_cams);
        printf("buffers of Mono cameras : %d\n", raw_data_buffers_ptr->mono_imgs.size());
    }
}