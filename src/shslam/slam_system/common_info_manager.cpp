#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/common_info_manager.hpp>

namespace shslam
{
    shslam::SlamSystem::CommonInfoManager::CommonInfoManager() :
    num_sensors_ptr{std::make_shared<shslam::NumSensors>()}
    {}

    std::shared_ptr<shslam::NumSensors> shslam::SlamSystem::CommonInfoManager::GetNumSensorsPtr() const
    {
        return num_sensors_ptr;
    }

    void shslam::SlamSystem::CommonInfoManager::ApplyConfig(const YAML::Node& config)
    {
        this->num_sensors_ptr->mono_cams = config["mono_cameras"].size();
    }
}
