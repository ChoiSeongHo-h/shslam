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
            const int32_t max_features,
            const double rejection_ratio,
            const double min_features_gap_vec
        );

        void Track();

        void AssociateBuffers
        (
            std::queue<std::pair<uint64_t, cv::Mat>>* input_buffers_ptr,
            std::queue<std::pair<uint64_t, cv::Mat>>* output_buffers_ptr
        );


    private :
        class RefInfo;

        void GetInitPose(cv::Matx33d& R_ref_to_cur, cv::Matx31d& t_ref_to_cur);

        void GetCurrnetFeaturesRaw(std::vector<cv::Point2f>& current_pts_raw, const cv::Mat& img, double& mean_dist);
        //RefInfo ref_info_raw;

        void Preprocess(const std::pair<uint64_t, cv::Mat>& original, cv::Mat &resized, cv::Mat& color);


        void GetQuaternion(const cv::Matx33d& R, double* Q);
        //void GetRefFeatures();

        std::unique_ptr<RefInfo> ref_info_ptr;                    
        std::queue<std::pair<uint64_t, cv::Mat>>* input_img_buf_ptr;
        std::queue<std::pair<uint64_t, cv::Mat>>* output_img_buf_ptr;
        bool is_initialized;

        const int32_t kIdx;
        const int32_t kWidth;
        const int32_t kHeight;
        const cv::Matx33d kCamMat;
        const cv::Matx<double, 1, 5> kDistCoeffs;
        const bool kWantVisualize;
        const double kResizingRatio;

        cv::Matx33d accR;
        cv::Matx31d act;


    
    };

}