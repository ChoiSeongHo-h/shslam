namespace shslam
{
    class SlamSystem::CommonInfoManager
    {
    public:
        std::shared_ptr<NumSensors> GetNumSensorsPtr() const;
        void SetNumSensors(const std::shared_ptr<NumSensors> num_sensors_ptr);
    private:
        std::shared_ptr<NumSensors> num_sensors_ptr;
    };
}