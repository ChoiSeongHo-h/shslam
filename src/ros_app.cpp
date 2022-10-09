#include <shslam.hpp>
#include <ros_headers.hpp>

std::shared_ptr<shslam::InputBuffers> input_buffers_ptr;
std::shared_ptr<shslam::OutputBuffers> output_buffers_ptr;
image_transport::Publisher img0_pub;
image_transport::Publisher img1_pub;
ros::Publisher pc0_pub;

void CallbackImg0(const sensor_msgs::CompressedImageConstPtr& msg)
{
    cv::Mat img0 = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    auto now = ros::Time::now().toNSec();
    input_buffers_ptr->mono_imgs[0].emplace(std::make_pair(now, img0));
}
void CallbackImg1(const sensor_msgs::CompressedImageConstPtr& msg)
{
    cv::Mat img1 = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    auto now = ros::Time::now().toNSec();
    input_buffers_ptr->mono_imgs[1].emplace(std::make_pair(now, img1));
}

void Visualize()
{  
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if(!output_buffers_ptr->mono_imgs[0].empty())
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_buffers_ptr->mono_imgs[0].front().second).toImageMsg();
            img0_pub.publish(msg);
            output_buffers_ptr->mono_imgs[0].pop();
        }
        if(!output_buffers_ptr->mono_imgs[1].empty())
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_buffers_ptr->mono_imgs[1].front().second).toImageMsg();
            img1_pub.publish(msg);
            output_buffers_ptr->mono_imgs[1].pop();
        }
        if(!output_buffers_ptr->monocam_pcs[0].empty())
        {
            sensor_msgs::PointCloud msg;
            msg.header = std_msgs::Header();
            msg.header.frame_id = std::string("car0");
            msg.points.resize(output_buffers_ptr->monocam_pcs[0].front().second.cols);
            for(auto n = 0; n<msg.points.size(); ++n)
            {
                msg.points[n].x = output_buffers_ptr->monocam_pcs[0].front().second.at<float>(0, n);
                msg.points[n].y = output_buffers_ptr->monocam_pcs[0].front().second.at<float>(1, n);
                msg.points[n].z = output_buffers_ptr->monocam_pcs[0].front().second.at<float>(2, n);
            }
            pc0_pub.publish(msg);

            output_buffers_ptr->monocam_pcs[0].pop();
        }
        

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shslam");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport it(node_handle);
    img0_pub = it.advertise("output_image0", 1);
    img1_pub = it.advertise("output_image1", 1);
    pc0_pub = node_handle.advertise<sensor_msgs::PointCloud>("output_pointcloud0", 1);

    shslam::SlamSystem slam_system;
    slam_system.InitBy(std::string("/home/csh/catkin_ws/src/shslam/config/config.yaml"));
    std::thread system_thread;
    if(slam_system.IsRunnable())
    {
        system_thread = std::thread(&shslam::SlamSystem::Run, &slam_system);
    }

    ::input_buffers_ptr = slam_system.GetInputBuffersPtr();
    ::output_buffers_ptr = slam_system.GetOutputBuffersPtr();
    std::thread visualization_thread(Visualize);

    auto subscriber_cam0 = node_handle.subscribe("/camera_array/cam0/image_raw/compressed", 10, CallbackImg0);
    auto subscriber_cam1 = node_handle.subscribe("/camera_array/cam1/image_raw/compressed", 10, CallbackImg1);
    ros::spin();

    system_thread.join();
}