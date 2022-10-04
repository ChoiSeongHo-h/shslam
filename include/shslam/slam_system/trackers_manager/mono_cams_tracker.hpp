namespace shslam
{
    class SlamSystem::TrackersManager::MonoCamsTracker
    {
    public:
        void ApplyConfig
        (
            const std::vector<int32_t>& width_vec,
            const std::vector<int32_t>& height_vec,
            const std::vector<cv::Matx33d>& camera_matrix_vec,
            const std::vector<cv::Matx<double, 1, 5>>& dist_coeffs_vec,
            const std::vector<bool>& want_visualize_vec,
            const std::vector<double>& resizing_ratio_vec,
            const std::vector<int32_t>& max_features_vec,
            const std::vector<double>& rejection_ratio_vec,
            const std::vector<int32_t> min_ref_features_vec,
            const std::vector<double>& min_features_gap_vec,
            const std::vector<int32_t>& OF_patch_sz_vec,
            const std::vector<int32_t>& OF_pyr_lv_vec,
            const std::vector<double>& min_disparity_vec,
            const std::vector<double>& LMedS_prob_vec,
            const std::vector<int32_t>& min_features_passed_E_vec
        );

        void RunTrackingThreads();

        void AssociateBuffers
        (
            std::shared_ptr<InputBuffers> input_buffers_ptr, 
            std::shared_ptr<OutputBuffers> output_buffers_ptr
        );

    private:
        class MonoCam;

        std::vector<std::unique_ptr<MonoCam>> mono_cam_ptrs;
    };
}