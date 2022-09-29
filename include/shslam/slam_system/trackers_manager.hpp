namespace shslam
{
    class shslam::SlamSystem::TrackersManager
    {
    public:
        TrackersManager();

        class MonoCamsTracker;

        std::unique_ptr<MonoCamsTracker> mono_cams_tracker_ptr;
        void RunAllTrackingThreads();

        std::shared_ptr<NumSensors> ApplyConfig(const std::string& config_path);

        void AssociateBuffers(std::shared_ptr<RawDataBuffers> raw_data_buffers_ptr);


    private:

    };
}