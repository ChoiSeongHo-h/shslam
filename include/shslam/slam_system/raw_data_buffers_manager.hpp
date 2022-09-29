namespace shslam
{
    class SlamSystem::RawDataBuffersManager
    {
    public:
        RawDataBuffersManager();
        std::shared_ptr<RawDataBuffers> GetBuffersPtr();

        void Init(std::shared_ptr<NumSensors> num_sensors_ptr);

    private:
        std::shared_ptr<RawDataBuffers> raw_data_buffers_ptr;
    };
}