#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam/ref_info.hpp>

namespace shslam
{
    SlamSystem::TrackersManager::TrackersManager() :
    mono_cams_tracker_ptr(std::make_unique<SlamSystem::TrackersManager::MonoCamsTracker>())
    {}
    
    void SlamSystem::TrackersManager::RunAllTrackingThreads()
    {
        printf("Run tracking threads\n");
        std::vector<std::thread> managers_threads;
        managers_threads.emplace_back(std::thread
        (
            &SlamSystem::TrackersManager::MonoCamsTracker::RunTrackingThreads,
            mono_cams_tracker_ptr.get()
        ));

        for(auto& thread : managers_threads)
            thread.join();
    }

    void SlamSystem::TrackersManager::ApplyConfig(const YAML::Node& config)
    {
        auto num_mono_cams = config["mono_cameras"].size();
        if(num_mono_cams < 1) // &&&&&&.....
        {
            printf("Tracker config error. No sensor.\n");
            return;
        }

        if(num_mono_cams > 0)
        {
            std::vector<int32_t> width_vec;
            std::vector<int32_t> height_vec;
            std::vector<cv::Matx33d> camera_matrix_vec;
            std::vector<cv::Matx<double, 1, 5>> dist_coeffs_vec;
            std::vector<bool> want_visualize_vec;
            std::vector<double> resizing_ratio_vec;
            std::vector<int32_t> max_features_vec;
            std::vector<double> rejection_ratio_vec;
            std::vector<double> min_features_gap_vec;
            printf("%ld mono cameras will be used.\n", num_mono_cams);
            for(auto i_th = 0; i_th<num_mono_cams; ++i_th)
            {
                auto width = config["mono_cameras"][i_th]["width"].as<int32_t>();
                width_vec.emplace_back(width);
                
                auto height = config["mono_cameras"][i_th]["height"].as<int32_t>();
                height_vec.emplace_back(height);

                auto resizing_ratio = config["mono_cameras"][i_th]["resizing_ratio"].as<double>();
                resizing_ratio_vec.emplace_back(resizing_ratio);

                auto cam_mat_1d = std::move(config["mono_cameras"][i_th]["camera_matrix"].as<std::vector<double>>());
                auto cam_mat = std::move(cv::Matx33d(cam_mat_1d.data()));
                cam_mat *= resizing_ratio;
                camera_matrix_vec.emplace_back(cam_mat);

                auto dist_coeffs_1d = std::move(config["mono_cameras"][i_th]["distortion_coefficient"].as<std::vector<double>>());
                auto dist_coeffs = std::move(cv::Matx<double, 1, 5>(dist_coeffs_1d.data()));
                dist_coeffs_vec.emplace_back(dist_coeffs);

                auto want_visualize = config["mono_cameras"][i_th]["want_to_visualize"].as<bool>();
                want_visualize_vec.emplace_back(want_visualize);

                auto max_features = config["mono_cameras"][i_th]["max_finding_features"].as<int32_t>();
                max_features_vec.emplace_back(max_features);

                auto rejection_ratio = config["mono_cameras"][i_th]["bad_features_rejection_ratio"].as<double>();
                rejection_ratio_vec.emplace_back(rejection_ratio);

                auto min_features_gap_width_ratio = config["mono_cameras"][i_th]["min_features_distance-image_width_ratio"].as<double>();
                min_features_gap_vec.emplace_back(resizing_ratio * double(width) * min_features_gap_width_ratio);
            }

            mono_cams_tracker_ptr->ApplyConfig
            (
                width_vec,
                height_vec,
                camera_matrix_vec, 
                dist_coeffs_vec, 
                want_visualize_vec, 
                resizing_ratio_vec,
                max_features_vec,
                rejection_ratio_vec,
                min_features_gap_vec
            );
        }


        printf("Tracker config complete.\n\n");
    }
    

    void SlamSystem::TrackersManager::AssociateBuffers(std::shared_ptr<InputBuffers> input_buffers_ptr, std::shared_ptr<OutputBuffers> output_buffers_ptr)
    {
            mono_cams_tracker_ptr->AssociateBuffers(input_buffers_ptr, output_buffers_ptr);
    }
}