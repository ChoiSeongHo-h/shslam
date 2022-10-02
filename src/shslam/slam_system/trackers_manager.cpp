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

    void shslam::SlamSystem::TrackersManager::ApplyConfig(const YAML::Node& config)
    {
        auto num_mono_cams = config["mono_cameras"].size();
        if(num_mono_cams < 1) // &&&&&&.....
        {
            printf("Tracker config error. No sensor.\n");
            return;
        }

        if(num_mono_cams > 0)
        {
            std::vector<std::pair<cv::Matx33d, cv::Matx<double, 1, 5>>> mono_cams_params;
            std::vector<bool> want_visualizes;
            printf("%ld mono cameras will be used.\n", num_mono_cams);
            for(auto i_th = 0; i_th<num_mono_cams; ++i_th)
            {
                auto cam_mat_vec = std::move(config["mono_cameras"][i_th]["camera_matrix"].as<std::vector<double>>());
                auto kCamMat = std::move(cv::Matx33d(cam_mat_vec.data()));

                auto dist_coeffs_vec = std::move(config["mono_cameras"][i_th]["distortion_coefficient"].as<std::vector<double>>());
                auto kDistCoeffs = std::move(cv::Matx<double, 1, 5>(dist_coeffs_vec.data()));

                mono_cams_params.emplace_back(std::make_pair(kCamMat, kDistCoeffs));

                auto want_visualize = config["mono_cameras"][i_th]["want_to_visualize"].as<bool>();
                want_visualizes.emplace_back(want_visualize);
            }

            mono_cams_tracker_ptr->ApplyConfig(mono_cams_params, want_visualizes);
        }


        printf("Tracker config complete.\n\n");
    }
    

    void shslam::SlamSystem::TrackersManager::AssociateBuffers(std::shared_ptr<shslam::InputBuffers> input_buffers_ptr, std::shared_ptr<shslam::OutputBuffers> output_buffers_ptr)
    {
            mono_cams_tracker_ptr->AssociateBuffers(input_buffers_ptr, output_buffers_ptr);
    }
}