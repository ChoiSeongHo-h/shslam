namespace shslam
{
    class shslam::SlamSystem::TrackersManager
    {
    public:
        class MonoCamsTracker;

        TrackersManager();

        void RunAllTrackingThreads();

        void ApplyConfig(const YAML::Node& config);

        void AssociateBuffers(std::shared_ptr<RawDataBuffers> raw_data_buffers_ptr);


    private:
        std::unique_ptr<MonoCamsTracker> mono_cams_tracker_ptr;

    };
}