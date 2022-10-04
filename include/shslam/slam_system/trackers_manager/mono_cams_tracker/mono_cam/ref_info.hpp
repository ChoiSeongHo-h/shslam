namespace shslam
{
    class SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo
    {
    public:
        RefInfo
        (
            const int32_t max_features,
            const double rejection_ratio,
            const double min_features_gap,
            const int32_t min_ref_features
        );
        bool IsEmpty();
        void Clear();
        void GetFeatures
        (
            const uint64_t& now, 
            const cv::Mat& img, 
            const cv::Matx33d& kCamMat, 
            const cv::Matx<double, 1, 5>& kDistCoeffs
        );
        
        uint64_t time = -1;
        std::vector<cv::Point2f> features_raw;
        std::vector<cv::Point2f> features;
        cv::Mat img;
        const int32_t kMaxFeatures;
        const double kRejectionRatio;
        const double kMinFeaturesGap;
        const int32_t kMinRefFeatures;
    };
}