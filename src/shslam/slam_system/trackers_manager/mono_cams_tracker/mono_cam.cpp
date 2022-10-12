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
        const int32_t max_pts2d,
        const double rejection_ratio,
        const int32_t min_ref_pts2d,
        const double min_pts2d_gap,
        const int32_t OF_patch_sz,
        const int32_t OF_pyr_lv,
        const double min_disparity,
        const double min_prop,
        const int32_t min_pts2d_passed_E
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
    kMinPts2dPassedE(min_pts2d_passed_E),
    ref_info_ptr(std::make_unique<SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo>
    (
        max_pts2d, 
        rejection_ratio, 
        min_pts2d_gap,
        min_ref_pts2d
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
        printf("min 2d features by essential matrix : %d\n", kMinPts2dPassedE);

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
            if(!is_initialized)
            {
                cv::Matx34d Rt_cur_to_ref;
                std::vector<cv::Point2f> cur_pts2d;
                bool is_received_init_pose = false;
                GetInitPose(Rt_cur_to_ref, cur_pts2d, is_received_init_pose);
                if(!is_received_init_pose)
                    continue;


                cv::Matx34d Rt_ref_to_ref
                (
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0
                );

                auto R_cur_to_ref = Rt_cur_to_ref.get_minor<3, 3>(0, 0);
                auto t_cur_to_ref_in_cur = Rt_cur_to_ref.get_minor<3, 1>(0, 3);
                auto R_ref_to_cur = R_cur_to_ref.t();
                auto R_org_to_prev = R_org_to_cur;
                auto t_org_to_prev_in_org = t_org_to_cur_in_org;
                t_org_to_cur_in_org = t_org_to_prev_in_org + R_org_to_prev * (-R_ref_to_cur * t_cur_to_ref_in_cur);
                R_org_to_cur = R_org_to_prev * R_ref_to_cur;
                SendPose();

                cv::Mat pts4d_in_ref;

                cv::triangulatePoints(Rt_ref_to_ref, Rt_cur_to_ref, ref_info_ptr->pts2d, cur_pts2d, pts4d_in_ref);
                cv::Mat pts_scales = pts4d_in_ref(cv::Rect(0, 3, pts4d_in_ref.cols, 1));
                for(auto row = 0; row<3; ++row)
                {
                    auto pts_element = pts4d_in_ref(cv::Rect(0, row, pts4d_in_ref.cols, 1));
                    cv::divide(pts_element, pts_scales, pts_element);
                }
                pts_scales = cv::Mat::ones(pts_scales.rows, pts_scales.cols, pts_scales.type());

                cv::Matx44f T_org_to_prev;
                cv::Mat R_org_to_prev_float(R_org_to_prev);
                R_org_to_prev_float.convertTo(R_org_to_prev_float, CV_32FC1);
                cv::Mat t_org_to_prev_in_org_float(t_org_to_prev_in_org);
                t_org_to_prev_in_org_float.convertTo(t_org_to_prev_in_org_float, CV_32FC1);
                cv::Matx34f Rt_org_to_prev;
                cv::hconcat(R_org_to_prev_float, t_org_to_prev_in_org_float, Rt_org_to_prev);
                cv::Matx14f T_4th_row{0.0, 0.0, 0.0, 1.0};
                cv::vconcat(Rt_org_to_prev, T_4th_row, T_org_to_prev);
                pts4d_in_org = T_org_to_prev*pts4d_in_ref;
                cv::Mat pts3d_in_org = pts4d_in_org(cv::Rect(0, 0, pts4d_in_org.cols, 3));

                output_pc_buf_ptr->emplace(std::make_pair(ros::Time::now().toNSec(), pts3d_in_org.clone()));


                is_initialized = true;
                continue;
            }        

            //--------------------------------------------------------

            //ref_info_ptr->GetPts2d(time_prev, img_prev, kCamMat, kDistCoeffs);
            std::vector<cv::Point2f> cur_pts2d;
            cv::Mat img, img_color;
            uint64_t time_now = input_img_buf_ptr->front().first;
            ResizeImg(input_img_buf_ptr->front(), img, img_color);
            input_img_buf_ptr->pop();

            ref_info_ptr->GetPts2d(ref_info_ptr->time, ref_info_ptr->img, kCamMat, kDistCoeffs);
            //return;

            std::vector<cv::Point2f> cur_pts2d_raw;
            std::vector<uchar> is_pts2d_passed_OF;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK
            (
                ref_info_ptr->img, 
                img,
                ref_info_ptr->pts2d_raw,
                cur_pts2d_raw,
                is_pts2d_passed_OF,
                err,
                cv::Size(kOFPatchSz, kOFPatchSz),
                kOFPyrLv
            );

            std::vector<std::vector<cv::Point2f>*> pts2d_want_reordering_ptrs
            {&ref_info_ptr->pts2d_raw, &cur_pts2d_raw};
            std::vector<cv::Mat*> pts4d_want_reordering_ptrs{&pts4d_in_org};
            RmOutliersForPts(is_pts2d_passed_OF, pts2d_want_reordering_ptrs, pts4d_want_reordering_ptrs);
            printf("pass rm 1\n");
            if(pts4d_in_org.cols > 10)
            {
                cv::Mat pts3d_in_org = pts4d_in_org(cv::Rect(0, 0, pts4d_in_org.cols, 3));
                cv::Mat r, t;
                std::vector<uint32_t> inlier_indices;
                //std::cout<<cur_pts2d_raw<<std::endl;
                
                cv::Mat cur_pts2d_raw_mat(cur_pts2d_raw.size(), 2, CV_32FC1, cur_pts2d_raw.data());
                //std::cout<<cur_pts2d_raw_mat<<std::endl;
                cv::Mat pts3d_in_org_T = pts3d_in_org.t();
                std::cout<<pts3d_in_org_T.size()<<std::endl;
                std::cout<<cur_pts2d_raw_mat(cv::Rect(0, 0, 2, pts3d_in_org_T.rows)).size()<<std::endl;
                cv::solvePnPRansac
                (
                    pts3d_in_org_T, 
                    cur_pts2d_raw_mat(cv::Rect(0, 0, 2, pts3d_in_org_T.rows)), 
                    kCamMat, 
                    kDistCoeffs, 
                    r, 
                    t_org_to_cur_in_org, 
                    false, 
                    20, 
                    5.0, 
                    0.99, 
                    inlier_indices, 
                    cv::SOLVEPNP_ITERATIVE
                );
                std::vector<uchar> is_inlier(pts3d_in_org_T.rows, 0);
                for(auto idx_idx : inlier_indices)
                    is_inlier[idx_idx] = 1;

                printf("\npass pnp %d %d\n", inlier_indices.size(), pts3d_in_org_T.rows);
                pts2d_want_reordering_ptrs = std::vector<std::vector<cv::Point2f>*>
                {&ref_info_ptr->pts2d_raw, &cur_pts2d_raw};
                pts4d_want_reordering_ptrs = std::vector<cv::Mat*>{&pts4d_in_org};
                RmOutliersForPts(is_inlier, pts2d_want_reordering_ptrs, pts4d_want_reordering_ptrs);
                printf("pass rm 2\n");
                cv::Rodrigues(r, R_org_to_cur);

                R_org_to_cur = R_org_to_cur.t();
                t_org_to_cur_in_org = -R_org_to_cur*t_org_to_cur_in_org;

                SendPose();


                
                DrawInitOF(time_now, std::vector<uchar>(), img_color, cur_pts2d_raw);
                output_pc_buf_ptr->emplace(std::make_pair(ros::Time::now().toNSec(), pts3d_in_org.clone()));

            }


            is_initialized = false;
            ref_info_ptr->Clear();
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
        std::vector<cv::Point2f>& cur_pts2d,
        bool& is_received_init_pose
    )
    {
        cv::Mat img, img_color;
        uint64_t time_now = input_img_buf_ptr->front().first;
        ResizeImg(input_img_buf_ptr->front(), img, img_color);
        input_img_buf_ptr->pop();

        if(ref_info_ptr->IsEmpty())
        {
            ref_info_ptr->GetPts2d(time_now, img, kCamMat, kDistCoeffs);
            return;
        }
    
        bool is_passed_this_test = false;
        std::vector<cv::Point2f> cur_pts2d_raw;
        TrackCurrnetPts2d(is_passed_this_test, cur_pts2d_raw, cur_pts2d, img);
        if(!is_passed_this_test)
            return;

        is_passed_this_test = false;
        std::vector<uchar> is_inliers;
        cv::Matx33d E;
        CalcE(is_passed_this_test, is_inliers, E, cur_pts2d);
        if(!is_passed_this_test)
            return;

        is_passed_this_test = false;
        CalcInitPose
        (is_passed_this_test, is_inliers, E, cur_pts2d, cur_pts2d_raw, Rt_cur_to_ref);
        if(!is_passed_this_test)
            return;
        
        if(kWantVisualize)
            DrawInitOF(time_now, is_inliers, img_color, cur_pts2d_raw);

        is_received_init_pose = true;
        ref_info_ptr->img = std::move(img);
        ref_info_ptr->time = time_now;
        ref_info_ptr->pts2d_raw = std::move(cur_pts2d_raw);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::TrackCurrnetPts2d
    (
        bool& is_passed_this_test,
        std::vector<cv::Point2f>& cur_pts2d_raw, 
        std::vector<cv::Point2f>& cur_pts2d, 
        const cv::Mat& img
    )
    {
        std::vector<uchar> is_pts2d_passed_OF;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK
        (
            ref_info_ptr->img, 
            img,
            ref_info_ptr->pts2d_raw,
            cur_pts2d_raw,
            is_pts2d_passed_OF,
            err,
            cv::Size(kOFPatchSz, kOFPatchSz),
            kOFPyrLv
        );

        std::vector<std::vector<cv::Point2f>*> pts2d_want_reordering_ptrs
        {&ref_info_ptr->pts2d_raw, &ref_info_ptr->pts2d, &cur_pts2d_raw};
        std::vector<cv::Mat*> empty_mat_pts_ptrs;
        RmOutliersForPts(is_pts2d_passed_OF, pts2d_want_reordering_ptrs, empty_mat_pts_ptrs);

        double mean_disparity = 0.0;
        auto num_pts2d = ref_info_ptr->pts2d_raw.size();
        for(auto idx = 0; idx<num_pts2d; ++idx)
            mean_disparity += cv::norm(cur_pts2d_raw[idx] - ref_info_ptr->pts2d_raw[idx]);
        mean_disparity /= num_pts2d;

        if(mean_disparity < kMinDisparity)
        {
            is_passed_this_test = false;
            return;
        }
        is_passed_this_test = true;
        cv::undistortPoints(cur_pts2d_raw, cur_pts2d, kCamMat, kDistCoeffs);

        printf("test OF : %ld, dist %f\n", cur_pts2d_raw.size(), mean_disparity);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RmOutliersForPts
    (
        const std::vector<uchar> &is_inliers, 
        const std::vector<std::vector<cv::Point2f>*>& vec_pts_ptrs, 
        const std::vector<cv::Mat*>& mat_pts_ptrs
    )
    {
        if(!mat_pts_ptrs.empty())
            printf("%d %d %d\n", is_inliers.size(), vec_pts_ptrs[0]->size(), mat_pts_ptrs[0]->cols);
        auto num_test_passed = 0;
        auto test_sz = is_inliers.size();
        auto mat_roi_w = 0;
        if(!mat_pts_ptrs.empty())
            mat_roi_w = std::accumulate(is_inliers.begin(), is_inliers.begin() + mat_pts_ptrs[0]->cols, 0);

        for(auto idx = 0; idx<is_inliers.size(); ++idx)
        {
            if(is_inliers[idx] == 0)
                continue;

            for(auto& vec_pts_ptr : vec_pts_ptrs)
                (*vec_pts_ptr)[num_test_passed] = (*vec_pts_ptr)[idx];

            for(auto& mat_pts_ptr : mat_pts_ptrs)
            {
                if(idx >= mat_pts_ptr->cols)
                    continue;

                for(auto row = 0; row < mat_pts_ptr->rows; ++row)
                    mat_pts_ptr->at<float>(row, num_test_passed) = mat_pts_ptr->at<float>(row, idx);
            }
            
            ++num_test_passed;
        }

        for(auto& vec_pts_ptr : vec_pts_ptrs)
        {
            if(is_inliers.size() < vec_pts_ptr->size())
            {
                vec_pts_ptr->insert(vec_pts_ptr->begin()+num_test_passed, vec_pts_ptr->begin()+is_inliers.size(), vec_pts_ptr->end());
                vec_pts_ptr->resize(num_test_passed + vec_pts_ptr->size()-is_inliers.size());
            }
            else
                vec_pts_ptr->resize(num_test_passed);
        }

        for(auto& mat_pts_ptr : mat_pts_ptrs)
        {
            std::cout<<mat_pts_ptr->size()<<std::endl;
            printf("%d \n", mat_roi_w);
            *mat_pts_ptr = (*mat_pts_ptr)(cv::Rect(0, 0, mat_roi_w, mat_pts_ptr->rows));
        }
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::CalcE
    (
        bool& is_passed_this_test,
        std::vector<uchar>& is_inliers,
        cv::Matx33d& E,
        std::vector<cv::Point2f>& cur_pts2d
    )
    {
        E = cv::findEssentialMat
        (
            ref_info_ptr->pts2d, 
            cur_pts2d, 
            1.0, 
            cv::Point2f(0, 0), 
            cv::LMEDS, 
            kLMedSProp, 
            1.0, 
            is_inliers
        );
        auto num_pts2d_passed_E = std::accumulate
        (is_inliers.begin(), is_inliers.end(), 0);
        if(num_pts2d_passed_E < kMinPts2dPassedE)
        {
            ref_info_ptr->Clear();
            is_passed_this_test = false;
            return;
        }

        is_passed_this_test = true;
        printf("test E : %d\n", num_pts2d_passed_E);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::CalcInitPose
    (
        bool& is_passed_this_test, 
        std::vector<uchar>& is_inliers,
        cv::Matx33d& E,
        std::vector<cv::Point2f>& cur_pts2d,
        std::vector<cv::Point2f>& cur_pts2d_raw,
        cv::Matx34d& Rt_cur_to_ref
    )
    {
        cv::Matx33d R_cur_to_ref;
        cv::Matx31d t_cur_to_ref_in_cur;
        cv::recoverPose
        (
            E,
            ref_info_ptr->pts2d, 
            cur_pts2d,
            R_cur_to_ref, 
            t_cur_to_ref_in_cur,
            1.0, 
            cv::Point2f(0, 0), 
            is_inliers
        );
        bool has_wrong_R = R_cur_to_ref(0,0) < 0 || R_cur_to_ref(1,1) < 0 || R_cur_to_ref(2,2) < 0;
        if(has_wrong_R)
        {
            is_passed_this_test = false;
            return;    
        }

        is_passed_this_test = true;
        std::vector<std::vector<cv::Point2f>*> pts2d_want_reordering_ptrs
        {&ref_info_ptr->pts2d, &ref_info_ptr->pts2d_raw, &cur_pts2d, &cur_pts2d_raw};
        std::vector<cv::Mat*> empty_mat_pts_ptrs;
        RmOutliersForPts(is_inliers, pts2d_want_reordering_ptrs, empty_mat_pts_ptrs);
        printf("test pose : %ld\n", cur_pts2d.size());
        cv::hconcat(R_cur_to_ref, t_cur_to_ref_in_cur, Rt_cur_to_ref);
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::DrawInitOF
    (
        uint64_t time_now,
        const std::vector<uchar>& is_inliers,
        cv::Mat& img_color,
        const std::vector<cv::Point2f>& cur_pts2d_raw
    )
    {
        for (int idx = 0; idx < ref_info_ptr->pts2d_raw.size(); ++idx)
        {
            cv::arrowedLine
            (
                img_color, 
                ref_info_ptr->pts2d_raw[idx], 
                cur_pts2d_raw[idx], 
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