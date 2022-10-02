namespace shslam
{
    class SlamSystem::TrackersManager::MonoCamsTracker
    {
    public:
        void ApplyConfig(const std::vector<std::pair<cv::Matx33d, cv::Matx<double, 1, 5>>>& mono_cam_config, const std::vector<bool>& want_visualizes);

        void RunTrackingThreads();

        void AssociateBuffers(std::shared_ptr<RawDataBuffers> raw_data_buffers_ptr);

    private:
        class MonoCam;

        std::vector<std::unique_ptr<MonoCam>> mono_cam_ptrs;
    };
}