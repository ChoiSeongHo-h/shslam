namespace shslam
{
    class SlamSystem::TrackersManager::MonoCamsTracker::MonoCam
    {
    public:
        MonoCam
        (
            const int32_t idx,
            const int32_t width,
            const int32_t height,
            const cv::Matx33d& cam_mat,
            const cv::Matx<double, 1, 5>& dist_coeffs,
            const bool want_visualize,
            const double resizing_ratio,
            const int32_t max_pts2d,
            const double rejection_ratio,
            const int32_t min_ref_pts2d,
            const double min_pts2d_gap,
            const int32_t OF_patch_sz,
            const int32_t OF_pyr_lv,
            const double min_disparity,
            const double min_prop,
            const int32_t min_pts2d_passed_E
        );

        void Track();

        void AssociateBuffers
        (
            std::queue<std::pair<uint64_t, cv::Mat>>* input_img_buffers_ptr,
            std::queue<std::pair<uint64_t, cv::Mat>>* output_img_buffers_ptr,
            std::queue<std::pair<uint64_t, cv::Mat>>* output_pc_buffers_ptr
        );


    private :
        class RefInfo;

        void SendPose();
        
        void DrawInitOF
        (
            uint64_t time_now,
            const std::vector<uchar>& is_pts2d_passed_tests,
            cv::Mat& img_color,
            const std::vector<cv::Point2f>& cur_pts2d_raw
        );

        void CalcInitPose
        (
            bool& is_passed_this_test,
            std::vector<uchar>& is_pts2d_passed_tests,
            cv::Matx33d& E,
            std::vector<cv::Point2f>& cur_pts2d,
            std::vector<cv::Point2f>& cur_pts2d_raw,
            cv::Matx34d &Rt_cur_to_ref
        );

        void CalcE
        (
            bool& is_passed_this_test,
            std::vector<uchar>& is_pts2d_passed_tests,
            cv::Matx33d& E,
            std::vector<cv::Point2f>& cur_pts2d
        );

        void RmOutliersForPts
        (
            const std::vector<uchar>& is_inliers, 
            const std::vector<std::vector<cv::Point2f>*>& vec_pts_ptrs,
            const std::vector<cv::Mat*>& mat_pts_ptrs
        );

        void GetInitPose
        (
            cv::Matx34d& Rt_cur_to_ref, 
            std::vector<cv::Point2f>& cur_pts2d,
            bool& is_received_init_pose
        );

        void TrackCurrnetPts2d
        (
            bool& is_passed_this_test, 
            std::vector<cv::Point2f>& cur_pts2d_raw, 
            std::vector<cv::Point2f>& cur_pts2d, 
            const cv::Mat& img
        );

        void ResizeImg(const std::pair<uint64_t, cv::Mat>& original, cv::Mat &resized, cv::Mat& color);

        void GetQuaternion(const cv::Matx33d& R, double* Q);

        std::unique_ptr<RefInfo> ref_info_ptr;                    
        std::queue<std::pair<uint64_t, cv::Mat>>* input_img_buf_ptr;
        std::queue<std::pair<uint64_t, cv::Mat>>* output_img_buf_ptr;
        std::queue<std::pair<uint64_t, cv::Mat>>* output_pc_buf_ptr;
        bool is_initialized;

        const int32_t kIdx;
        const int32_t kWidth;
        const int32_t kHeight;
        const cv::Matx33d kCamMat;
        const cv::Matx<double, 1, 5> kDistCoeffs;
        const bool kWantVisualize;
        const double kResizingRatio;
        const int32_t kOFPatchSz;
        const int32_t kOFPyrLv;
        const double kMinDisparity;
        const double kLMedSProp;
        const int32_t kMinPts2dPassedE;

        cv::Matx33d R_org_to_cur;
        cv::Matx31d t_org_to_cur_in_org;
        cv::Mat pts4d_in_org;

        // cv::Mat img_prev;
        // uint64_t time_prev;
        // std::vector<cv::Point2f> prev_pts2d_raw;
    
    };

}