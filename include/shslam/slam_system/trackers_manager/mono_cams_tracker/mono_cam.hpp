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
        std::queue<std::pair<uint64_t, cv::Mat>>* img_buf_original_ptr;
        int32_t kIdx;
        cv::Matx33d kCamMat;
        cv::Matx<double, 1, 5> kDistCoeffs;
        bool kWantVisualize;


    private :
        bool is_initialized;
        std::map<uint64_t, cv::Mat> img_buf;
        std::map<uint64_t, cv::Mat> img_buf_color;

        cv::Matx33d accR;
        cv::Matx31d act;

        struct RefInfo
        {
            uint64_t time = -1;
            std::vector<cv::Point2f> features;
            bool IsEmpty();
            void Clear();
        };

        RefInfo ref_info_raw;
        RefInfo ref_info;                    

        void Preprocess(const std::pair<uint64_t, cv::Mat>& original, cv::Mat &resized, const bool want_to_visualize, cv::Mat& color);


        void GetQuaternion(const cv::Matx33d& R, double* Q);
        //void GetRefFeatures();
    
    };

}