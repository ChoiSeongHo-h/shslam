namespace shslam
{
    class SlamSystem
    {
    public:
        SlamSystem();

        std::shared_ptr<InputBuffers> GetInputBuffersPtr();

        std::shared_ptr<OutputBuffers> GetOutputBuffersPtr();

        void Run();

        void InitBy(const std::string& config_path);

        std::shared_ptr<const NumSensors> GetNumSensorsPtr() const;

        bool IsRunnable();
        
    private:
        class TrackersManager;
        class CommonInfoManager;
        class BuffersManager;

        std::unique_ptr<TrackersManager> trackers_manager_ptr;
        std::unique_ptr<CommonInfoManager> common_info_manager_ptr;
        std::unique_ptr<BuffersManager> buffers_manager_ptr;
    };
}