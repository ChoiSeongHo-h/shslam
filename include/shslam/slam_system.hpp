namespace shslam
{
    class SlamSystem
    {
    public:
        SlamSystem();

        std::shared_ptr<RawDataBuffers> GetRawDataBuffersPtr();

        void Run();

        void InitBy(const std::string& config_path);

        std::shared_ptr<NumSensors> GetNumSensorsPtr() const;

        bool IsRunnable();
        
    private:
        class TrackersManager;
        class CommonInfoManager;
        class RawDataBuffersManager;

        std::unique_ptr<TrackersManager> trackers_manager_ptr;
        std::unique_ptr<CommonInfoManager> common_info_manager_ptr;
        std::unique_ptr<RawDataBuffersManager> raw_data_buffers_manager_ptr;
    };
}