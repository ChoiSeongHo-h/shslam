namespace shslam
{
    class SlamSystem::BuffersManager
    {
    public:
        BuffersManager();
        std::shared_ptr<InputBuffers> GetInputBuffersPtr();
        std::shared_ptr<OutputBuffers> GetOutputBuffersPtr();

        void Init(std::shared_ptr<NumSensors> num_sensors_ptr);

    private:
        std::shared_ptr<InputBuffers> input_buffers_ptr;
        std::shared_ptr<OutputBuffers> output_buffers_ptr;
    };
}