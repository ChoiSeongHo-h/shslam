namespace shslam
{
    class SlamSystem::TrackersManager
    {
    public:
        class MonoCamsTracker;

        TrackersManager();

        void RunAllTrackingThreads();

        void ApplyConfig(const YAML::Node& config);

        void AssociateBuffers(std::shared_ptr<InputBuffers> input_buffers_ptr, std::shared_ptr<OutputBuffers> output_buffers_ptr);


    private:
        std::unique_ptr<MonoCamsTracker> mono_cams_tracker_ptr;

    };
}