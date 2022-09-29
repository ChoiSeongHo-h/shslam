#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>
#include <shslam/slam_system/common_info_manager.hpp>
#include <shslam/slam_system/raw_data_buffers_manager.hpp>

namespace shslam
{
    shslam::SlamSystem::SlamSystem() :
    want_visualize{true},
    trackers_manager_ptr{std::make_unique<shslam::SlamSystem::TrackersManager>()},
    raw_data_buffers_manager_ptr{std::make_unique<shslam::SlamSystem::RawDataBuffersManager>()},
    common_info_manager_ptr{std::make_unique<shslam::SlamSystem::CommonInfoManager>()}
    {
        cv::setUseOptimized(true);
    }

    std::shared_ptr<shslam::RawDataBuffers> shslam::SlamSystem::GetRawDataBuffersPtr()
    {
        return raw_data_buffers_manager_ptr->GetBuffersPtr();
    }

    void shslam::SlamSystem::Run()
    {
        printf("Run System.\n");
        trackers_manager_ptr->RunAllTrackingThreads();
    }
    void shslam::SlamSystem::InitBy(const std::string& config_path)
    {
        printf("Start initializing the system.\n");
        auto num_sensors = trackers_manager_ptr->ApplyConfig(config_path);
        common_info_manager_ptr->SetNumSensors(num_sensors);
        raw_data_buffers_manager_ptr->Init(num_sensors);
        auto raw_data_buffers_ptr = raw_data_buffers_manager_ptr->GetBuffersPtr();
        trackers_manager_ptr->AssociateBuffers(raw_data_buffers_ptr);
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