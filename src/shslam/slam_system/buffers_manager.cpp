#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/buffers_manager.hpp>

namespace shslam
{
    shslam::SlamSystem::BuffersManager::BuffersManager() :
    input_buffers_ptr{std::make_shared<shslam::InputBuffers>()},
    output_buffers_ptr{std::make_shared<shslam::OutputBuffers>()}
    {}

    std::shared_ptr<shslam::InputBuffers> shslam::SlamSystem::BuffersManager::GetInputBuffersPtr()
    {
        return input_buffers_ptr;
    }

    std::shared_ptr<shslam::OutputBuffers> shslam::SlamSystem::BuffersManager::GetOutputBuffersPtr()
    {
        return output_buffers_ptr;
    }

    void shslam::SlamSystem::BuffersManager::Init(std::shared_ptr<NumSensors> num_sensors_ptr)
    {
        printf("Initialize buffers.\n");
        input_buffers_ptr->mono_imgs.clear();
        input_buffers_ptr->mono_imgs.resize(num_sensors_ptr->mono_cams);
        output_buffers_ptr->mono_imgs.clear();
        output_buffers_ptr->mono_imgs.resize(num_sensors_ptr->mono_cams);
        printf("buffers of Mono cameras : %ld\n", input_buffers_ptr->mono_imgs.size());
    }
}