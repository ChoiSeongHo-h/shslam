#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam/ref_info.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace shslam
{
    SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::MonoCam
    (
        const int32_t idx,
        const int32_t width,
        const int32_t height,
        const cv::Matx33d& cam_mat,
        const cv::Matx<double, 1, 5>& dist_coeffs,
        const bool want_visualize,
        const double resizing_ratio,
        const int32_t max_features,
        const double rejection_ratio,
        const int32_t min_ref_features,
        const double min_features_gap,
        const int32_t OF_patch_sz,
        const int32_t OF_pyr_lv,
        const double min_disparity,
        const double min_prop,
        const int32_t min_features_passed_E
    ) :
    is_initialized(false),
    kIdx(idx),
    kWidth(width),
    kHeight(height),
    kCamMat(cam_mat),
    kDistCoeffs(dist_coeffs),
    kWantVisualize(want_visualize),
    kResizingRatio(resizing_ratio),
    kOFPatchSz(OF_patch_sz),
    kOFPyrLv(OF_pyr_lv),
    kMinDisparity(min_disparity),
    kLMedSProp(min_prop),
    kMinFeaturesPassedE(min_features_passed_E),
    ref_info_ptr(std::make_unique<SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo>
    (
        max_features, 
        rejection_ratio, 
        min_features_gap,
        min_ref_features
    )),
    R_org_to_cur
    {
        cv::Matx33d(
        1.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 
        0.0, 0.0, 1.0)
    },
    t_org_to_cur_in_org
    {
        cv::Matx31d(
        0.0,
        0.0,
        0.0)
    }
    {
        printf("width ( resized ) : %d\n", kWidth);
        printf("height ( resized ) : %d\n", kHeight);
        printf("camera matrix ( resized ) : \n");
        std::cout<<kCamMat<<std::endl;
        printf("distortion coefficient : \n");
        std::cout<<kDistCoeffs<<std::endl;
        printf("want to visualize : %d\n", kWantVisualize);
        printf("resizing ratio : %f\n", kResizingRatio);
        printf("opticalflow patch size : %d\n", kOFPatchSz);
        printf("opticalflow pyramid level : %d\n", kOFPyrLv);
        printf("min disparity : %f\n", kMinDisparity);
        printf("LMedS probablity : %f\n", kLMedSProp);
        printf("min features by essential matrix : %d\n", kMinFeaturesPassedE);

        printf("\n");
    }


    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::AssociateBuffers
    (
        std::queue<std::pair<uint64_t, cv::Mat>>* input_img_buf_ptr,
        std::queue<std::pair<uint64_t, cv::Mat>>* output_img_buf_ptr,
        std::queue<std::pair<uint64_t, cv::Mat>>* output_pc_buf_ptr
    )
    {
        this->input_img_buf_ptr = input_img_buf_ptr;
        this->output_img_buf_ptr = output_img_buf_ptr;
        this->output_pc_buf_ptr = output_pc_buf_ptr;
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::ResizeImg
    (const std::pair<uint64_t, cv::Mat>& original, cv::Mat &resized, cv::Mat &color)
    {
        cv::Mat img_temp;
        auto cp_target_ref = kWantVisualize ? std::ref(color) : std::ref(img_temp);
        cv::resize
        (original.second, cp_target_ref.get(), cv::Size(kWidth, kHeight), 0, 0, cv::INTER_LINEAR);
        if(kWantVisualize)
            cv::cvtColor(color, resized, cv::COLOR_BGR2GRAY);
    }


    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::Track()
    {
        while(true)
        {
            if(input_img_buf_ptr->empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
//
            is_initialized = false;
            if(!is_initialized)
            {
                cv::Matx34d Rt_cur_to_ref;
                std::vector<cv::Point2f> cur_features;
                bool is_received_init_pose = false;
                GetInitPose(Rt_cur_to_ref, cur_features, is_received_init_pose);
                if(!is_received_init_pose)
                    continue;

                is_initialized = true;

                cv::Matx34d Rt_ref_to_ref
                (
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0
                );

                auto R_cur_to_ref = Rt_cur_to_ref.get_minor<3, 3>(0, 0);
                auto t_cur_to_ref_in_cur = Rt_cur_to_ref.get_minor<3, 1>(0, 3);
                auto R_ref_to_cur = R_cur_to_ref.t();
                cv::Matx33d& R_org_to_prev = R_org_to_cur;
                t_org_to_cur_in_org += R_org_to_prev * (-R_ref_to_cur * t_cur_to_ref_in_cur);
                R_org_to_cur = R_org_to_prev * R_ref_to_cur;
                auto R_cur_to_ref = Rt_cur_to_ref.get_minor<3, 3>(0, 0);

                cv::Mat pts_4d_in_ref;

                cv::triangulatePoints(Rt_ref_to_ref, Rt_cur_to_ref, ref_info_ptr->features, cur_features, pts_4d_in_ref);
                cv::Mat pts_scales = pts_4d_in_ref(cv::Rect(0, 3, pts_4d_in_ref.cols, 1));
                for(auto row = 0; row<3; ++row)
                {
                    auto pts_element = pts_4d_in_ref(cv::Rect(0, row, pts_4d_in_ref.cols, 1));
                    cv::divide(pts_element, pts_scales, pts_element);
                }
                pts_scales = cv::Mat::ones(pts_scales.rows, pts_scales.cols, pts_scales.type());

                cv::Mat_<float> T_cur_to_ref;
                cv::Mat Rt_cur_to_ref_float(Rt_cur_to_ref);
                Rt_cur_to_ref_float.convertTo(Rt_cur_to_ref_float, T_cur_to_ref.type());
                cv::Matx14f T_4th_row{0.0, 0.0, 0.0, 1.0};
                cv::vconcat(Rt_cur_to_ref_float, T_4th_row, T_cur_to_ref);
                cv::Mat pts_4d_on_cur = T_cur_to_ref*pts_4d_in_ref;
                cv::Mat pts_3d_on_cur = pts_4d_on_cur(cv::Rect(0, 0, pts_4d_in_ref.cols, 3));

                output_pc_buf_ptr->emplace(std::make_pair(ros::Time::now().toNSec(), pts_3d_on_cur.clone()));

                SendPose();

                ref_info_ptr->Clear();

            }        
        }
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::SendPose()
    {
        double qq[4];
        GetQuaternion(R_org_to_cur, qq);
        static tf::TransformBroadcaster pos_pub;
        tf::Transform transform;
        //transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
        transform.setOrigin( tf::Vector3(t_org_to_cur_in_org(0,0), t_org_to_cur_in_org(1,0), t_org_to_cur_in_org(2,0)) );
        tf::Quaternion q;
        q.setW(qq[3]);
        q.setX(qq[0]);
        q.setY(qq[1]);
        q.setZ(qq[2]);
        transform.setRotation(q);

        auto node = std::string("car");
        auto std_idx = std::to_string(kIdx);
        pos_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", node+std_idx));
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::GetInitPose
    (
        cv::Matx34d& Rt_cur_to_ref, 
        std::vector<cv::Point2f>& cur_features,
        bool& is_received_init_pose
    )
    {
        cv::Mat img, img_color;
        uint64_t time_now = input_img_buf_ptr->front().first;
        ResizeImg(input_img_buf_ptr->front(), img, img_color);
        input_img_buf_ptr->pop();

        if(ref_info_ptr->IsEmpty())
        {
            ref_info_ptr->GetFeatures(time_now, img, kCamMat, kDistCoeffs);
            return;
        }
    
        bool is_passed_this_test = false;
        std::vector<cv::Point2f> cur_features_raw;
        TrackCurrnetFeatures(is_passed_this_test, cur_features_raw, cur_features, img);
        if(!is_passed_this_test)
            return;

        is_passed_this_test = false;
        std::vector<uchar> is_features_passed_tests;
        cv::Matx33d E;
        CalcE(is_passed_this_test, is_features_passed_tests, E, cur_features);
        if(!is_passed_this_test)
            return;

        is_passed_this_test = false;
        CalcInitPose
        (is_passed_this_test, is_features_passed_tests, E, cur_features, Rt_cur_to_ref);
        if(!is_passed_this_test)
            return;
        
        if(kWantVisualize)
            DrawInitOF(time_now, is_features_passed_tests, img_color, cur_features_raw);

        is_received_init_pose = true;
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::TrackCurrnetFeatures
    (
        bool& is_passed_this_test,
        std::vector<cv::Point2f>& cur_features_raw, 
        std::vector<cv::Point2f>& cur_features, 
        const cv::Mat& img
    )
    {
        std::vector<uchar> is_features_passed_OF;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK
        (
            ref_info_ptr->img, 
            img,
            ref_info_ptr->features_raw,
            cur_features_raw,
            is_features_passed_OF,
            err,
            cv::Size(kOFPatchSz, kOFPatchSz),
            kOFPyrLv
        );

        std::vector<std::vector<cv::Point2f>*> features_want_reordering_ptrs
        {&ref_info_ptr->features_raw, &ref_info_ptr->features, &cur_features_raw};
        ReorderFeatures(is_features_passed_OF, features_want_reordering_ptrs);

        double mean_disparity = 0.0;
        auto num_features = ref_info_ptr->features_raw.size();
        for(auto idx = 0; idx<num_features; ++idx)
            mean_disparity += cv::norm(cur_features_raw[idx] - ref_info_ptr->features_raw[idx]);
        mean_disparity /= num_features;

        if(mean_disparity < kMinDisparity)
        {
            is_passed_this_test = false;
            return;
        }
        is_passed_this_test = true;
        cv::undistortPoints(cur_features_raw, cur_features, kCamMat, kDistCoeffs);

        printf("test OF : %ld, dist %f\n", cur_features_raw.size(), mean_disparity);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::ReorderFeatures
    (
        const std::vector<uchar> &is_features_passed_test, 
        const std::vector<std::vector<cv::Point2f>*>& features_ptrs
    )
    {
        auto num_test_passed = 0;
        for(auto idx = 0; idx<is_features_passed_test.size(); ++idx)
        {
            if(is_features_passed_test[idx] == 0)
                continue;

            for(auto& features_ptr : features_ptrs)
                (*features_ptr)[num_test_passed] = (*features_ptr)[idx];
            ++num_test_passed;
        }
        for(auto& features_ptr : features_ptrs)
            (*features_ptr).resize(num_test_passed);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::CalcE
    (
        bool& is_passed_this_test,
        std::vector<uchar>& is_features_passed_tests,
        cv::Matx33d& E,
        std::vector<cv::Point2f>& cur_features
    )
    {
        E = cv::findEssentialMat
        (
            ref_info_ptr->features, 
            cur_features, 
            1.0, 
            cv::Point2f(0, 0), 
            cv::LMEDS, 
            kLMedSProp, 
            1.0, 
            is_features_passed_tests
        );
        auto num_features_passed_E = std::accumulate
        (is_features_passed_tests.begin(), is_features_passed_tests.end(), 0);
        if(num_features_passed_E < kMinFeaturesPassedE)
        {
            ref_info_ptr->Clear();
            is_passed_this_test = false;
            return;
        }

        is_passed_this_test = true;
        printf("test E : %d\n", num_features_passed_E);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::CalcInitPose
    (
        bool& is_passed_this_test, 
        std::vector<uchar>& is_features_passed_tests,
        cv::Matx33d& E,
        std::vector<cv::Point2f>& cur_features,
        cv::Matx34d& Rt_cur_to_ref
    )
    {
        cv::Matx33d R_cur_to_ref;
        cv::Matx31d t_cur_to_ref_in_cur;
        cv::recoverPose
        (
            E,
            ref_info_ptr->features, 
            cur_features,
            R_cur_to_ref, 
            t_cur_to_ref_in_cur,
            1.0, 
            cv::Point2f(0, 0), 
            is_features_passed_tests
        );
        bool has_wrong_R = R_cur_to_ref(0,0) < 0 || R_cur_to_ref(1,1) < 0 || R_cur_to_ref(2,2) < 0;
        if(has_wrong_R)
        {
            is_passed_this_test = false;
            return;    
        }

        is_passed_this_test = true;
        std::vector<std::vector<cv::Point2f>*> features_want_reordering_ptrs
        {&ref_info_ptr->features, &cur_features};
        ReorderFeatures(is_features_passed_tests, features_want_reordering_ptrs);
        printf("test pose : %ld\n", cur_features.size());
        cv::hconcat(R_cur_to_ref, t_cur_to_ref_in_cur, Rt_cur_to_ref);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::DrawInitOF
    (
        uint64_t time_now,
        const std::vector<uchar>& is_features_passed_tests,
        cv::Mat& img_color,
        const std::vector<cv::Point2f>& cur_features_raw
    )
    {
        for (int idx = 0; idx < ref_info_ptr->features_raw.size(); ++idx)
        {
            if(is_features_passed_tests[idx] == 0)
                continue;

            cv::arrowedLine
            (
                img_color, 
                ref_info_ptr->features_raw[idx], 
                cur_features_raw[idx], 
                cv::Scalar(0, 255, 0), 
                1, 
                cv::LINE_AA
            );
            output_img_buf_ptr->emplace(std::make_pair(time_now, img_color));
        }
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::GetQuaternion
    (const cv::Matx33d& R, double* Q)
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