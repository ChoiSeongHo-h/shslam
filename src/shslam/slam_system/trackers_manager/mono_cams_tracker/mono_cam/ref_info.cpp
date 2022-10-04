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
        const int32_t max_features,
        const double rejection_ratio,
        const double min_features_gap,
        const int32_t min_ref_features
    ) :
    kMaxFeatures(max_features),
    kRejectionRatio(rejection_ratio),
    kMinFeaturesGap(min_features_gap),
    kMinRefFeatures(min_ref_features)
    {
        printf("    max finding features : %d\n", kMaxFeatures);
        printf("    bad features rejection ratio : %f\n", kRejectionRatio);
        printf("    min features gap : %f\n", kMinFeaturesGap);
        printf("    min reference features : %d\n", kMinRefFeatures);
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
        time = -1;
    }

    void SlamSystem::TrackersManager::MonoCamsTracker::MonoCam::RefInfo::GetFeatures
    (
        const uint64_t& time_now,
        const cv::Mat& img,
        const cv::Matx33d& kCamMat,
        const cv::Matx<double, 1, 5>& kDistCoeffs
    )
    {
        goodFeaturesToTrack(img, features_raw, kMaxFeatures, kRejectionRatio, kMinFeaturesGap);
        if(features_raw.size() < kMinRefFeatures)
            return;
            
        time = time_now;
        this->img = std::move(img);
        cv::undistortPoints(features_raw, features, kCamMat, kDistCoeffs);
    }

}
