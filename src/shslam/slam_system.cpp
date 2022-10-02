#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>
#include <shslam/slam_system/common_info_manager.hpp>
#include <shslam/slam_system/buffers_manager.hpp>

namespace shslam
{
    shslam::SlamSystem::SlamSystem() :
    trackers_manager_ptr{std::make_unique<shslam::SlamSystem::TrackersManager>()},
    buffers_manager_ptr{std::make_unique<shslam::SlamSystem::BuffersManager>()},
    common_info_manager_ptr{std::make_unique<shslam::SlamSystem::CommonInfoManager>()}
    {
        cv::setUseOptimized(true);
    }

    std::shared_ptr<shslam::InputBuffers> shslam::SlamSystem::GetInputBuffersPtr()
    {
        return buffers_manager_ptr->GetInputBuffersPtr();
    }

    std::shared_ptr<shslam::OutputBuffers> shslam::SlamSystem::GetOutputBuffersPtr()
    {
        return buffers_manager_ptr->GetOutputBuffersPtr();
    }

    void shslam::SlamSystem::Run()
    {
        printf("Run System.\n");
        trackers_manager_ptr->RunAllTrackingThreads();
    }
    void shslam::SlamSystem::InitBy(const std::string& config_path)
    {
        printf("Start initializing the system.\n");

        auto config = std::move(YAML::LoadFile(config_path));
        
        trackers_manager_ptr->ApplyConfig(config);
        common_info_manager_ptr->ApplyConfig(config);
        buffers_manager_ptr->Init(common_info_manager_ptr->GetNumSensorsPtr());
        auto input_buffers_ptr = buffers_manager_ptr->GetInputBuffersPtr();
        auto output_buffers_ptr = buffers_manager_ptr->GetOutputBuffersPtr();
        trackers_manager_ptr->AssociateBuffers(input_buffers_ptr, output_buffers_ptr);

        printf("Complete initializing the system.\n\n");
    }

    std::shared_ptr<shslam::NumSensors> shslam::SlamSystem::GetNumSensorsPtr() const
    {
        auto num_sensors_ptr = common_info_manager_ptr->GetNumSensorsPtr();
        printf("Mono cameras : %d\n", num_sensors_ptr->mono_cams);
        return num_sensors_ptr;
    }

    bool shslam::SlamSystem::IsRunnable()
    {
        auto nums_sensors_ptr = GetNumSensorsPtr();
        bool is_runnable = false;
        if(nums_sensors_ptr->mono_cams > 0) //.........
            is_runnable = true;
        
        if(is_runnable)
            printf("System is runnable.\n");
        else
            printf("System is not runnable.\n");

        return is_runnable;
    }

}