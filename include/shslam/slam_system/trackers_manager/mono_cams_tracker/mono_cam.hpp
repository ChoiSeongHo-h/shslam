namespace shslam
{
    class SlamSystem::TrackersManager::MonoCamsTracker::MonoCam
    {
    public:
        MonoCam
        (
            const int32_t kIdx,
            const cv::Matx33d kCamMat,
            const cv::Matx<double, 1, 5> kDistCoeffs,
            bool kWantVisualize
        );
        void Track();
        std::queue<std::pair<uint64_t, cv::Mat>>* input_img_buf_ptr;
        std::queue<std::pair<uint64_t, cv::Mat>>* output_img_buf_ptr;
        int32_t kIdx;
        cv::Matx33d kCamMat;
        cv::Matx<double, 1, 5> kDistCoeffs;
        bool kWantVisualize;


    private :
        bool is_initialized;

        cv::Matx33d accR;
        cv::Matx31d act;

        struct RefInfo
        {
            uint64_t time = -1;
            std::vector<cv::Point2f> features_raw;
            std::vector<cv::Point2f> features;
            cv::Mat img;
            bool IsEmpty();
            void Clear();
            void Get(const uint64_t& now, const cv::Mat& img, const cv::Matx33d& kCamMat, const cv::Matx<double, 1, 5>& kDistCoeffs);
        };

        void GetInitPose(cv::Matx33d& R_ref_to_cur, cv::Matx31d& t_ref_to_cur);

        void GetCurrnetPtsRaw(std::vector<cv::Point2f>& current_pts_raw, const cv::Mat& img, double& mean_dist);
        //RefInfo ref_info_raw;
        RefInfo ref_info;                    

        void Preprocess(const std::pair<uint64_t, cv::Mat>& original, cv::Mat &resized, cv::Mat& color);


        void GetQuaternion(const cv::Matx33d& R, double* Q);
        //void GetRefFeatures();
    
    };

}