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

    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::Preprocess(const std::pair<uint64_t, cv::Mat>& original, cv::Mat &resized, bool want_to_visualize, cv::Mat &color)
    {
        cv::Mat img_temp;
        auto cp_target = want_to_visualize ? color : img_temp;
        cv::resize(original.second, cp_target, cv::Size(original.second.cols * 0.3, original.second.rows * 0.3), 0, 0, cv::INTER_LINEAR);
        cv::cvtColor(cp_target, resized, cv::COLOR_BGR2GRAY);

        img_buf.emplace(original.first, resized);
        if(want_to_visualize)
            img_buf_color.emplace(original.first, color);
    }


    void shslam::SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::Track()
    {
        while(true)
        {
            if(img_buf_original_ptr->empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            cv::Mat img, img_color;
            uint64_t now = img_buf_original_ptr->front().first;
            Preprocess(img_buf_original_ptr->front(), img, true, img_color);
            img_buf_original_ptr->pop();

            if(ref_info_raw.IsEmpty())
            {
                std::vector<cv::Point2f> features_raw;
                goodFeaturesToTrack(img, features_raw, 150, 0.05, 20);

                ref_info_raw.time = now;
                ref_info_raw.features = std::move(std::vector<cv::Point2f>(features_raw));

                ref_info.time = ref_info_raw.time;
                cv::undistortPoints(ref_info_raw.features, ref_info.features, kCamMat, kDistCoeffs);
                continue;
            }
        
            auto& ref_img = img_buf[ref_info_raw.time];
            std::vector<cv::Point2f> pts_tracked;
            std::vector<uchar> io_mask;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(ref_img, img, ref_info_raw.features, pts_tracked, io_mask, err, cv::Size(25, 25), 3);

            std::vector<cv::Point2f> pts_from_opticalflow_distorted;

            double mean_dist = 0;
            for(uint32_t idx = 0; idx<io_mask.size(); ++idx)
            {
                if(io_mask[idx] == 0)
                    continue;

                mean_dist += cv::norm(pts_tracked[idx] - ref_info_raw.features[idx]);
                pts_from_opticalflow_distorted.emplace_back(pts_tracked[idx]);
                ref_info_raw.features[pts_from_opticalflow_distorted.size()-1] = ref_info_raw.features[idx];
                ref_info.features[pts_from_opticalflow_distorted.size()-1] = ref_info.features[idx];
            }
            ref_info_raw.features.resize(pts_from_opticalflow_distorted.size());
            ref_info.features.resize(pts_from_opticalflow_distorted.size());
            mean_dist /= pts_from_opticalflow_distorted.size();
            if(mean_dist < 20)
                continue;
            printf("test OF : %d, dist %f\n", pts_from_opticalflow_distorted.size(), mean_dist);
            
            std::vector<cv::Point2f> pts_from_opticalflow;
            cv::undistortPoints(pts_from_opticalflow_distorted, pts_from_opticalflow, kCamMat, kDistCoeffs);
            //std::cout<<refFeatures.features<<std::endl;
            //std::cout<<ptsFromOF<<std::endl;
            
            
            std::vector<uchar> result_E;
            cv::Mat E = cv::findEssentialMat(pts_from_opticalflow, ref_info.features, 1.0, cv::Point2f(0, 0), cv::LMEDS, 0.995, 1.0, result_E);
            //std::cout<<cv::norm(E)<<std::endl;
            //Eigen::Matrix3d eigenE;
            //cv::cv2eigen(E, eigenE);
            //Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr(eigenE);
            //qr.setThreshold(0.2);
            //auto rankE = qr.rank();
            //std::cout<<rankE<<std::endl;
            //if(rankE < 2)
            //    break;

            int32_t num_pts_good = 0;
            for(auto is_ok : result_E)
            {
                if(is_ok == 0)
                    continue;

                //printf("%d\n", isOK);
                
                ++num_pts_good;
            }
            if(num_pts_good < 50)
            {
                ref_info_raw.Clear();
                ref_info.Clear();
                continue;
            }

            printf("test E : %d\n", num_pts_good);
            //std::cout<<E<<std::endl;
            cv::Matx33d R;
            cv::Matx31d t;
            cv::recoverPose(E, pts_from_opticalflow, ref_info.features, R, t, 1.0, cv::Point2d(0, 0), result_E);
            if(R(0,0) < 0 || R(1,1) < 0 || R(2,2) < 0)
                continue;
            
            std::cout<<R<<std::endl;
            t(0,0) = 0;
            t(1,0) = 0;
            t(2,0) = 1;
            std::cout<<t<<std::endl;
            act = act + 0.03*accR*t;
            accR = accR*R;
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
            
            if(true)
            {
                std::vector<cv::Point2f> start_pts;
                std::vector<cv::Point2f> end_pts;

                for(uint32_t idx = 0; idx<result_E.size(); ++idx)
                {
                    if(result_E[idx] == 0)
                        continue;

                    start_pts.emplace_back(ref_info_raw.features[idx]);
                    end_pts.emplace_back(pts_from_opticalflow_distorted[idx]);
                }
                printf("test E2 : %d\n", start_pts.size());

                for (int i = 0; i < static_cast<int>(start_pts.size()); ++i)
                {
                    cv::arrowedLine(img_color, start_pts[i], end_pts[i], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                }

                /*
                for(int i=0; i<features.size() ; i++)
                {
                    cv::circle(drawingImg.get(), features[i], 5, cv::Scalar(0, 255, 0));
                }
                */
                //cv::imshow("view", img_color);
                //cv::waitKey(10);
            }

            ref_info_raw.Clear();
            ref_info.Clear();
            continue;
            
        }
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