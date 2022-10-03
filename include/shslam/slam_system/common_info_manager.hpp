namespace shslam
{
    class SlamSystem::CommonInfoManager
    {
    public:
        CommonInfoManager();
        
        std::shared_ptr<const NumSensors> GetNumSensorsPtr() const;

        void ApplyConfig(const YAML::Node& config);
    
    private:
        std::shared_ptr<NumSensors> num_sensors_ptr;
    };
}