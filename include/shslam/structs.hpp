namespace shslam
{
    struct NumSensors
    {
        int32_t mono_cams = 0;
    };
    
    struct RawDataBuffers
    {
        std::vector<std::queue<std::pair<uint64_t, cv::Mat>>> mono_imgs;
    };
}