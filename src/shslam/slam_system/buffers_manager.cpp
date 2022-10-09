#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/buffers_manager.hpp>

namespace shslam
{
    SlamSystem::BuffersManager::BuffersManager() :
    input_buffers_ptr(std::make_shared<InputBuffers>()),
    output_buffers_ptr(std::make_shared<OutputBuffers>())
    {}

    std::shared_ptr<InputBuffers> SlamSystem::BuffersManager::GetInputBuffersPtr()
    {
        return input_buffers_ptr;
    }

    std::shared_ptr<OutputBuffers> SlamSystem::BuffersManager::GetOutputBuffersPtr()
    {
        return output_buffers_ptr;
    }

    void SlamSystem::BuffersManager::Init(std::shared_ptr<const NumSensors> num_sensors_ptr)
    {
        printf("Initialize buffers.\n");
        input_buffers_ptr->mono_imgs.clear();
        input_buffers_ptr->mono_imgs.resize(num_sensors_ptr->mono_cams);
        output_buffers_ptr->mono_imgs.clear();
        output_buffers_ptr->mono_imgs.resize(num_sensors_ptr->mono_cams);
        output_buffers_ptr->monocam_pcs.clear();
        output_buffers_ptr->monocam_pcs.resize(num_sensors_ptr->mono_cams);
        printf("buffers of Mono cameras : %ld\n", input_buffers_ptr->mono_imgs.size());
    }
}