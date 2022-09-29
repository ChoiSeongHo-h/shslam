#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>

namespace shslam
{
    shslam::SlamSystem::TrackersManager::TrackersManager() :
    mono_cams_tracker_ptr{std::make_unique<MonoCamsTracker>()}
    {}
    
    void shslam::SlamSystem::TrackersManager::RunAllTrackingThreads()
    {
        printf("Run tracking threads\n");
        std::vector<std::thread> managers_threads;
        managers_threads.emplace_back(std::thread(&shslam::SlamSystem::TrackersManager::MonoCamsTracker::RunTrackingThreads, mono_cams_tracker_ptr.get()));

        for(auto& thread : managers_threads)
            thread.join();
    }

    std::shared_ptr<shslam::NumSensors> shslam::SlamSystem::TrackersManager::ApplyConfig(const std::string& config_path)
    {
        auto config = YAML::LoadFile(config_path);
        std::shared_ptr<shslam::NumSensors> num_sensors_ptr = std::make_shared<shslam::NumSensors>();

        auto num_mono_cams = config["mono_cameras"].size();
        if(num_mono_cams < 1) // &&&&&&.....
        {
            printf("Config file error. No sensor.\n");
            return num_sensors_ptr;
        }

        if(num_mono_cams > 0)
        {
            std::vector<std::pair<cv::Mat, cv::Mat>> mono_cams_config;
            printf("%d mono cameras will be used.\n", num_mono_cams);
            for(auto i_th = 0; i_th<num_mono_cams; ++i_th)
            {
                auto cam_mat_vec = config["mono_cameras"][i_th]["camera_matrix"].as<std::vector<double>>();
                auto cam_mat_1d = cv::Mat1d(3, 3, cam_mat_vec.data());

                auto dist_coeffs_vec = config["mono_cameras"][i_th]["distortion_coefficient"].as<std::vector<double>>();
                auto dist_coeffs_1d = cv::Mat1d(5, 1, dist_coeffs_vec.data());

                mono_cams_config.emplace_back(std::make_pair(cam_mat_1d.clone(), dist_coeffs_1d.clone()));
            }
            mono_cams_tracker_ptr->ApplyConfig(mono_cams_config);
            num_sensors_ptr->mono_cams = num_mono_cams;
        }

        printf("Config file applying complete.\n\n");
        return num_sensors_ptr;
    }

    void shslam::SlamSystem::TrackersManager::AssociateBuffers(std::shared_ptr<shslam::RawDataBuffers> raw_data_buffers_ptr)
    {
            mono_cams_tracker_ptr->AssociateBuffers(raw_data_buffers_ptr);
    }
}