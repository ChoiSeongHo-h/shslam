namespace shslam
{
    class SlamSystem::TrackersManager::MonoCamsTracker::MonoCam
    {
    public:
        MonoCam();
        int32_t idx;
        std::queue<std::pair<uint64_t, cv::Mat>>* img_buf_original_ptr;
        std::map<uint64_t, cv::Mat> img_buf;
        std::map<uint64_t, cv::Mat> img_buf_color;
        cv::Mat kCamMat = cv::Mat(3, 3, CV_64FC1);
        cv::Mat kDistCoeffs = cv::Mat(1, 5, CV_64FC1);

        cv::Matx33d accR;
        cv::Matx31d act;

        struct RefFeatures
        {
            uint64_t time = -1;
            std::vector<cv::Point2f> features;
            bool IsEmpty();
            void Clear();
        };


        RefFeatures ref_features_raw;
        RefFeatures ref_features;                    

        void Preprocess(const std::pair<uint64_t, cv::Mat>& original, cv::Mat &resized, const bool want_to_visualize, cv::Mat& color);

        void Track();

        void getQuaternion(const cv::Matx33d& R, double* Q);


    private :
    
    
    };

}