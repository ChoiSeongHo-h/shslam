#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace shslam
{
    shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::MonoCam
    (
        const int32_t kIdx,
        const cv::Matx33d kCamMat,
        const cv::Matx<double, 1, 5> kDistCoeffs,
        bool kWantVisualize
    ) :
    kIdx{kIdx},
    kCamMat{kCamMat},
    kDistCoeffs{kDistCoeffs},
    kWantVisualize{kWantVisualize},
    is_initialized{false},
    accR
    {
        cv::Matx33d(
        1.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 
        0.0, 0.0, 1.0)
    },
    act
    {
        cv::Matx31d(
        0.0,
        0.0,
        0.0)
    }
    {
        printf("camera matrix : \n");
        std::cout<<kCamMat<<std::endl;
        printf("distortion coefficient : \n");
        std::cout<<kDistCoeffs<<std::endl;
    }

    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::Preprocess(const std::pair<uint64_t, cv::Mat>& original, cv::Mat &resized, cv::Mat &color)
    {
        cv::Mat img_temp;
        auto cp_target_ref = kWantVisualize ? std::ref(color) : std::ref(img_temp);
        cv::resize(original.second, cp_target_ref.get(), cv::Size(original.second.cols * 0.3, original.second.rows * 0.3), 0, 0, cv::INTER_LINEAR);
        cv::cvtColor(cp_target_ref.get(), resized, cv::COLOR_BGR2GRAY);
    }


    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::Track()
    {
        while(true)
        {
            if(input_img_buf_ptr->empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            is_initialized = false;
            if(!is_initialized)
            {
                cv::Matx33d R_ref_to_cur;
                cv::Matx31d t_ref_to_cur;
                GetInitPose(R_ref_to_cur, t_ref_to_cur);

                if(!is_initialized)
                    continue;

                std::cout<<R_ref_to_cur<<std::endl;
                t_ref_to_cur(0,0) = 0;
                t_ref_to_cur(1,0) = 0;
                t_ref_to_cur(2,0) = 1;
                std::cout<<t_ref_to_cur<<std::endl;
                act = act + 0.03*accR*t_ref_to_cur;
                accR = accR*R_ref_to_cur;
                std::cout<<std::endl;
                std::cout<<accR<<std::endl;
                std::cout<<act<<std::endl;

                double qq[4];
                GetQuaternion(accR, qq);
                static tf::TransformBroadcaster pos_pub;
                tf::Transform transform;
                //transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
                transform.setOrigin( tf::Vector3(act(0,0), act(1,0), act(2,0)) );
                tf::Quaternion q;
                q.setW(qq[3]);
                q.setX(qq[0]);
                q.setY(qq[1]);
                q.setZ(qq[2]);
                transform.setRotation(q);

                auto node = std::string("car");
                auto std_idx = std::to_string(kIdx);
                pos_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", node+std_idx));
                ref_info.Clear();

                //is_initialized = true;
                //continue;
            }
        


            
            // if(true)
            // {
            //     std::vector<cv::Point2f> start_pts;
            //     std::vector<cv::Point2f> end_pts;

            //     for(uint32_t idx = 0; idx<mask_E.size(); ++idx)
            //     {
            //         if(mask_E[idx] == 0)
            //             continue;

            //         start_pts.emplace_back(ref_info.features_raw[idx]);
            //         end_pts.emplace_back(pts_raw_from_OF[idx]);
            //     }
            //     printf("test E2 : %ld\n", start_pts.size());

            //     for (int i = 0; i < static_cast<int>(start_pts.size()); ++i)
            //     {
            //         cv::arrowedLine(img_color, start_pts[i], end_pts[i], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            //     }

            //     /*
            //     for(int i=0; i<features.size() ; i++)
            //     {
            //         cv::circle(drawingImg.get(), features[i], 5, cv::Scalar(0, 255, 0));
            //     }
            //     */
            //     cv::imshow("view", img_color);
            //     cv::waitKey(10);
            // }

            
        }
    }

    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::GetInitPose(cv::Matx33d& R_ref_to_cur, cv::Matx31d& t_ref_to_cur)
    {
        if(input_img_buf_ptr->empty())
            return;

        cv::Mat img, img_color;
        uint64_t time_now = input_img_buf_ptr->front().first;
        Preprocess(input_img_buf_ptr->front(), img, img_color);
        input_img_buf_ptr->pop();

        if(ref_info.IsEmpty())
        {
            ref_info.Get(time_now, img, kCamMat, kDistCoeffs);
            return;
        }
    
        std::vector<cv::Point2f> current_features_raw;
        double mean_disparity = 0;
        GetCurrnetPtsRaw(current_features_raw, img, mean_disparity);
        if(mean_disparity < 20)
            return;
        printf("test OF : %ld, dist %f\n", current_features_raw.size(), mean_disparity);
        std::vector<cv::Point2f> current_features;
        cv::undistortPoints(current_features_raw, current_features, kCamMat, kDistCoeffs);
        
        std::vector<uchar> mask_E;
        cv::Matx33d E = cv::findEssentialMat(current_features, ref_info.features, 1.0, cv::Point2f(0, 0), cv::LMEDS, 0.995, 1.0, mask_E);
        int32_t num_good_pts = std::accumulate(mask_E.begin(), mask_E.end(), 0);
        if(num_good_pts < 50)
        {
            ref_info.Clear();
            return;
        }
        printf("test E : %d\n", num_good_pts);

        cv::recoverPose(E, current_features, ref_info.features, R_ref_to_cur, t_ref_to_cur, 1.0, cv::Point2d(0, 0), mask_E);
        if(R_ref_to_cur(0,0) < 0 || R_ref_to_cur(1,1) < 0 || R_ref_to_cur(2,2) < 0)
            return;    
        
        if(kWantVisualize)
        {
            for (int idx = 0; idx < ref_info.features_raw.size(); ++idx)
            {
                if(mask_E[idx] == 0)
                    continue;

                cv::arrowedLine(img_color, ref_info.features_raw[idx], current_features_raw[idx], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                output_img_buf_ptr->emplace(std::make_pair(time_now, img_color));
            }
        }
        is_initialized = true;
    }

    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::GetCurrnetPtsRaw(std::vector<cv::Point2f>& current_features_raw, const cv::Mat& img, double& mean_disparity)
    {
        std::vector<cv::Point2f> pts_tracked;
        std::vector<uchar> mask_OF;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(ref_info.img, img, ref_info.features_raw, pts_tracked, mask_OF, err, cv::Size(21, 21), 3);

        for(auto idx = 0; idx<mask_OF.size(); ++idx)
        {
            if(mask_OF[idx] == 0)
                continue;

            mean_disparity += cv::norm(pts_tracked[idx] - ref_info.features_raw[idx]);
            current_features_raw.emplace_back(pts_tracked[idx]);
            ref_info.features_raw[current_features_raw.size()-1] = ref_info.features_raw[idx];
            ref_info.features[current_features_raw.size()-1] = ref_info.features[idx];
        }
        ref_info.features_raw.resize(current_features_raw.size());
        ref_info.features.resize(current_features_raw.size());
        mean_disparity /= current_features_raw.size();
    }

    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo::Get(const uint64_t& time_now, const cv::Mat& img, const cv::Matx33d& kCamMat, const cv::Matx<double, 1, 5>& kDistCoeffs)
    {
        goodFeaturesToTrack(img, features_raw, 150, 0.05, 20);
        if(features_raw.size() < 100)
            return;
            
        time = time_now;
        this->img = std::move(img);
        cv::undistortPoints(features_raw, features, kCamMat, kDistCoeffs);
    }

    bool shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo::IsEmpty()
    {
        if (time == -1)
            return true;
        else
            return false;
    }
    
    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo::Clear()
    {
        time = -1;
    }

    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::GetQuaternion(const cv::Matx33d& R, double* Q)
    {
        double trace = R(0,0) + R(1,1) + R(2,2);

        if (trace > 0.0) 
        {
            double s = sqrt(trace + 1.0);
            Q[3] = (s * 0.5);
            s = 0.5 / s;
            Q[0] = ((R(2,1) - R(1,2)) * s);
            Q[1] = ((R(0,2) - R(2,0)) * s);
            Q[2] = ((R(1,0) - R(0,1)) * s);
        } 
        
        else 
        {
            int i = R(0,0) < R(1,1) ? (R(1,1) < R(2,2) ? 2 : 1) : (R(0,0) < R(2,2) ? 2 : 0); 
            int j = (i + 1) % 3;  
            int k = (i + 2) % 3;

            double s = sqrt(R(i, i) - R(j,j) - R(k,k) + 1.0);
            Q[i] = s * 0.5;
            s = 0.5 / s;

            Q[3] = (R(k,j) - R(j,k)) * s;
            Q[j] = (R(j,i) + R(i,j)) * s;
            Q[k] = (R(k,i) + R(i,k)) * s;
        }
    }
}