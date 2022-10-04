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
        printf("width ( resizing applied ) : %d\n", kWidth);
        printf("height ( resizing applied ) : %d\n", kHeight);
        printf("camera matrix ( resizing applied ) : \n");
        std::cout<<kCamMat<<std::endl;
        printf("distortion coefficient : \n");
        std::cout<<kDistCoeffs<<std::endl;
        printf("want to visualize : %d\n", kWantVisualize);
        printf("resizing ratio : %f\n", kResizingRatio);
        printf("opticalflow patch size : %d\n", kOFPatchSz);
        printf("opticalflow pyramid level : %d\n", kOFPyrLv);
        printf("min disparity : %f\n", kMinDisparity);
        printf("LMedS probablity : %f\n", kLMedSProp);
        printf("num features by essential matrix : %d\n", kMinFeaturesPassedE);

        printf("\n");
    }


    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::AssociateBuffers
    (
        std::queue<std::pair<uint64_t, cv::Mat>>* input_buffers_ptr,
        std::queue<std::pair<uint64_t, cv::Mat>>* output_buffers_ptr
    )
    {
        this->input_img_buf_ptr = input_buffers_ptr;
        this->output_img_buf_ptr = output_buffers_ptr;
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
                ref_info_ptr->Clear();

                //is_initialized = true;
                //continue;
            }        
        }
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::GetInitPose
    (cv::Matx33d& R_ref_to_cur, cv::Matx31d& t_ref_to_cur)
    {
        if(input_img_buf_ptr->empty())
            return;

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
        std::vector<cv::Point2f> current_features, current_features_raw;
        TrackCurrnetFeaturesRaw(is_passed_this_test, current_features_raw, current_features, img);
        if(!is_passed_this_test)
            return;

        is_passed_this_test = false;
        std::vector<uchar> is_features_passed_tests;
        cv::Matx33d E;
        CalcE(is_passed_this_test, is_features_passed_tests, E, current_features);
        if(!is_passed_this_test)
            return;

        is_passed_this_test = false;
        CalcInitPose(is_passed_this_test, is_features_passed_tests, E, current_features, R_ref_to_cur, t_ref_to_cur);
        if(!is_passed_this_test)
            return;
        
        if(kWantVisualize)
            DrawInitOF(time_now, is_features_passed_tests, img_color, current_features_raw);

        is_initialized = true;
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::TrackCurrnetFeaturesRaw
    (
        bool& is_passed_this_test,
        std::vector<cv::Point2f>& current_features_raw, 
        std::vector<cv::Point2f>& current_features, 
        const cv::Mat& img
    )
    {
        std::vector<uchar> is_features_passed_OF;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK
        (
            ref_info_ptr->img, img,
            ref_info_ptr->features_raw,
            current_features_raw,
            is_features_passed_OF,
            err,
            cv::Size(kOFPatchSz, kOFPatchSz),
            kOFPyrLv
        );

        std::vector<std::vector<cv::Point2f>*> features_want_reordering_ptrs
        {&ref_info_ptr->features_raw, &ref_info_ptr->features, &current_features_raw};
        ReorderFeatures(is_features_passed_OF, features_want_reordering_ptrs);

        double mean_disparity = 0.0;
        auto num_features = ref_info_ptr->features_raw.size();
        for(auto idx = 0; idx<num_features; ++idx)
            mean_disparity += cv::norm(current_features_raw[idx] - ref_info_ptr->features_raw[idx]);
        mean_disparity /= num_features;

        if(mean_disparity < kMinDisparity)
        {
            is_passed_this_test = false;
            return;
        }
        is_passed_this_test = true;
        cv::undistortPoints(current_features_raw, current_features, kCamMat, kDistCoeffs);

        printf("test OF : %ld, dist %f\n", current_features_raw.size(), mean_disparity);
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
        std::vector<cv::Point2f>& current_features
    )
    {
        E = cv::findEssentialMat
        (
            current_features, 
            ref_info_ptr->features, 
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
        std::vector<cv::Point2f>& current_features,
        cv::Matx33d& R_ref_to_cur,
        cv::Matx31d& t_ref_to_cur
    )
    {
        cv::recoverPose
        (
            E,
            current_features,
            ref_info_ptr->features, 
            R_ref_to_cur, 
            t_ref_to_cur,
            1.0, 
            cv::Point2f(0, 0), 
            is_features_passed_tests
        );
        bool has_wrong_R = R_ref_to_cur(0,0) < 0 || R_ref_to_cur(1,1) < 0 || R_ref_to_cur(2,2) < 0;
        if(has_wrong_R)
        {
            is_passed_this_test = false;
            return;    
        }

        is_passed_this_test = true;
        std::vector<std::vector<cv::Point2f>*> features_want_reordering_ptrs
        {&ref_info_ptr->features, &current_features};
        ReorderFeatures(is_features_passed_tests, features_want_reordering_ptrs);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::DrawInitOF
    (
        uint64_t time_now,
        const std::vector<uchar>& is_features_passed_tests,
        cv::Mat& img_color,
        const std::vector<cv::Point2f>& current_features_raw
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
                current_features_raw[idx], 
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