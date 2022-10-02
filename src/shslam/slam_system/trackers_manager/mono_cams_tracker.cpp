#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>

namespace shslam
{
    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::ApplyConfig(const std::vector<std::pair<cv::Matx33d, cv::Matx<double, 1, 5>>> &mono_cam_params, const std::vector<bool>& want_visualizes)
    {
        auto num_mono_cams = mono_cam_params.size();
        for(auto i_th = 0; i_th<num_mono_cams; ++i_th)
        {
            printf("Mono camera %d's config is applied. :\n", i_th);
            mono_cam_ptrs.emplace_back
            (
                std::make_unique<shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam>
                (
                    i_th, mono_cam_params[i_th].first, mono_cam_params[i_th].second, want_visualizes[i_th]
                )
            );
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