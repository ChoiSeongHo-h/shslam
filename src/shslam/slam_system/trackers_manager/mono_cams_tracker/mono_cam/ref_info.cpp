#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/trackers_manager.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam.hpp>
#include <shslam/slam_system/trackers_manager/mono_cams_tracker/mono_cam/ref_info.hpp>

namespace shslam
{
    SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo::RefInfo
    (
        const int32_t max_pts2d,
        const double rejection_ratio,
        const double min_pts2d_gap,
        const int32_t min_ref_pts2d
    ) :
    kMaxPts2d(max_pts2d),
    kRejectionRatio(rejection_ratio),
    kMinPts2dGap(min_pts2d_gap),
    kMinRefPts2d(min_ref_pts2d)
    {
        printf("    max finding features : %d\n", kMaxPts2d);
        printf("    bad features rejection ratio : %f\n", kRejectionRatio);
        printf("    min features gap : %f\n", kMinPts2dGap);
        printf("    min reference features : %d\n", kMinRefPts2d);
    }

    bool SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo::IsEmpty()
    {
        if (time == -1)
            return true;
        else
            return false;
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo::Clear()
    {
        pts2d_raw.clear();
        pts2d.clear();
        pts2d_raw.reserve(kMaxPts2d);
        pts2d.reserve(kMaxPts2d);
        time = -1;
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo::GetPts2d
    (
        const uint64_t& time_now,
        const cv::Mat& img,
        const cv::Matx33d& kCamMat,
        const cv::Matx<double, 1, 5>& kDistCoeffs
    )
    {
        auto num_want_extra_pts2d = kMaxPts2d - int(pts2d_raw.size());
        if(num_want_extra_pts2d < 0)
            return;

        printf("next adding pts : %d\n", num_want_extra_pts2d);
        std::vector<cv::Point2f> pts2d_raw_added;
        goodFeaturesToTrack(img, pts2d_raw_added, num_want_extra_pts2d, kRejectionRatio, kMinPts2dGap);
        auto num_all_pts2d = pts2d_raw.size() + pts2d_raw_added.size();
        if(num_all_pts2d < kMinRefPts2d)
            return;
        
        pts2d_raw.insert(pts2d_raw.end(), pts2d_raw_added.begin(), pts2d_raw_added.end() );
        printf("after adding pts : %d\n", pts2d_raw.size());

        std::vector<cv::Point2f> pts2d_added;
        cv::undistortPoints(pts2d_raw_added, pts2d_added, kCamMat, kDistCoeffs);
        pts2d.insert(pts2d.end(), pts2d_added.begin(), pts2d_added.end());

        time = time_now;
        this->img = std::move(img);
    }

}
