#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>

namespace shslam
{
    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::ApplyConfig(const std::vector<std::pair<cv::Mat, cv::Mat>> &mono_cam_config)
    {
        auto num_mono_cams = mono_cam_config.size();
        for(auto i_th = 0; i_th<num_mono_cams; ++i_th)
        {
            printf("Mono camera %d's config is applied. :\n", i_th);

            mono_cam_ptrs.emplace_back(std::make_unique<shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam>());
            mono_cam_ptrs[i_th]->idx = i_th;

            auto& cam_mat_from = mono_cam_config[i_th].first;
            auto& cam_mat_to = mono_cam_ptrs[i_th]->kCamMat;
            cam_mat_from.copyTo(cam_mat_to);
            printf("camera matrix\n");
            std::cout<<cam_mat_to<<std::endl;

            auto& dist_coeffs_from = mono_cam_config[i_th].second;
            auto& dist_coeffs_to = mono_cam_ptrs[i_th]->kDistCoeffs;
            dist_coeffs_from.copyTo(dist_coeffs_to);
            printf("distortion coefficient\n");
            std::cout<<dist_coeffs_to<<std::endl<<std::endl;
        }
    }

    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::RunTrackingThreads()
    {
        std::vector<std::thread> tracking_threads;
        for(auto i_th = 0; i_th<mono_cam_ptrs.size(); ++i_th)
        {
            printf("Run mono camera %d tracking threads\n", i_th);
            tracking_threads.emplace_back(std::thread(&shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::Track, mono_cam_ptrs[i_th].get()));
        }
        for(auto& thread : tracking_threads)
            thread.join();
    }

    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::AssociateBuffers(std::shared_ptr<shslam::RawDataBuffers> raw_data_buffers_ptr)
    {
        for(auto i_th = 0; i_th<mono_cam_ptrs.size(); ++i_th)
        {
            mono_cam_ptrs[i_th]->img_buf_original_ptr = &raw_data_buffers_ptr->mono_imgs[i_th];
        }
    }
}