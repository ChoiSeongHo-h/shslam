#include <shslam.hpp>
#include <ros_headers.hpp>

std::shared_ptr<shslam::RawDataBuffers> raw_data_buffers_ptr;

void CallbackImg0(const sensor_msgs::CompressedImageConstPtr& msg)
{
    cv::Mat img0 = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    auto now = ros::Time::now().toNSec();
    raw_data_buffers_ptr->mono_imgs[0].emplace(std::make_pair(now, img0));
}
void CallbackImg1(const sensor_msgs::CompressedImageConstPtr& msg)
{
    cv::Mat img1 = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    auto now = ros::Time::now().toNSec();
    raw_data_buffers_ptr->mono_imgs[1].emplace(std::make_pair(now, img1));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shslam");
    ros::NodeHandle node_handle;

    shslam::SlamSystem slam_system;
    slam_system.InitBy(std::string("/home/csh/catkin_ws/src/shslam/config/config.yaml"));
    std::thread system_thread;
    if(slam_system.IsRunnable())
    {
        system_thread = std::thread(&shslam::SlamSystem::Run, &slam_system);
    }
    ::raw_data_buffers_ptr = slam_system.GetRawDataBuffersPtr();

    auto subscriber_cam0 = node_handle.subscribe("/camera_array/cam0/image_raw/compressed", 10, CallbackImg0);
    auto subscriber_cam1 = node_handle.subscribe("/camera_array/cam1/image_raw/compressed", 10, CallbackImg1);
    ros::spin();

    system_thread.join();
}