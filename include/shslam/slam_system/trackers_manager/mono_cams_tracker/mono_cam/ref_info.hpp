namespace shslam
{
    class SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo
    {
    public:
        RefInfo
        (
            const int32_t max_pts2d,
            const double rejection_ratio,
            const double min_pts2d_gap,
            const int32_t min_ref_pts2d
        );
        bool IsEmpty();
        void Clear();
        void GetPts2d
        (
            const uint64_t& now, 
            const cv::Mat& img, 
            const cv::Matx33d& kCamMat, 
            const cv::Matx<double, 1, 5>& kDistCoeffs
        );
        
        uint64_t time = -1;
        std::vector<cv::Point2f> pts2d_raw;
        std::vector<cv::Point2f> pts2d;
        cv::Mat img;
        const int32_t kMaxPts2d;
        const double kRejectionRatio;
        const double kMinPts2dGap;
        const int32_t kMinRefPts2d;
    };
}