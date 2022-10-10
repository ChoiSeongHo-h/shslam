#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam/ref_info.hpp>

namespace shslam
{
    void SlamSystem::TrackersManager::MonoCamsTracker::ApplyConfig
    (
        const std::vector<int32_t>& width_vec,
        const std::vector<int32_t>& height_vec,
        const std::vector<cv::Matx33d>& camera_matrix_vec,
        const std::vector<cv::Matx<double, 1, 5>>& dist_coeffs_vec,
        const std::vector<bool>& want_visualize_vec,
        const std::vector<double>& resizing_ratio_vec,
        const std::vector<int32_t>& max_pts2d_vec,
        const std::vector<double>& rejection_ratio_vec,
        const std::vector<int32_t> min_ref_pts2d_vec,
        const std::vector<double>& min_pts2d_gap_vec,
        const std::vector<int32_t>& OF_patch_sz_vec,
        const std::vector<int32_t>& OF_pyr_lv_vec,
        const std::vector<double>& min_disparity_vec,
        const std::vector<double>& LMedS_prob_vec,
        const std::vector<int32_t>& min_pts2d_passed_E_vec
    )
    {
        auto num_mono_cams = camera_matrix_vec.size();
        for(auto i_th = 0; i_th<num_mono_cams; ++i_th)
        {
            printf("Mono camera %d's config is applied. :\n", i_th);
            mono_cam_ptrs.emplace_back
            (
                std::make_unique<SlamSystem::TrackersManager::MonoCamsTracker::MonoCam>
                (
                    i_th,
                    width_vec[i_th],
                    height_vec[i_th],
                    camera_matrix_vec[i_th],
                    dist_coeffs_vec[i_th],
                    want_visualize_vec[i_th],
                    resizing_ratio_vec[i_th],
                    max_pts2d_vec[i_th],
                    rejection_ratio_vec[i_th],
                    min_ref_pts2d_vec[i_th],
                    min_pts2d_gap_vec[i_th],
                    OF_patch_sz_vec[i_th],
                    OF_pyr_lv_vec[i_th],
                    min_disparity_vec[i_th],
                    LMedS_prob_vec[i_th],
                    min_pts2d_passed_E_vec[i_th]
                )
            );
        }
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::RunTrackingThreads()
    {
        std::vector<std::thread> tracking_threads;
        for(auto i_th = 0; i_th<mono_cam_ptrs.size(); ++i_th)
        {
            printf("Run mono camera %d tracking threads\n", i_th);
            tracking_threads.emplace_back(std::thread
            (
                &SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::Track, 
                mono_cam_ptrs[i_th].get()
            ));
        }
        for(auto& thread : tracking_threads)
            thread.join();
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::AssociateBuffers
    (
        std::shared_ptr<InputBuffers> input_buffers_ptr,
        std::shared_ptr<OutputBuffers> output_buffers_ptr
    )
    {
        for(auto i_th = 0; i_th<mono_cam_ptrs.size(); ++i_th)
        {
            mono_cam_ptrs[i_th]->AssociateBuffers
            (
                &input_buffers_ptr->mono_imgs[i_th],
                &output_buffers_ptr->mono_imgs[i_th],
                &output_buffers_ptr->monocam_pcs[i_th]
            );
        }
    }
}