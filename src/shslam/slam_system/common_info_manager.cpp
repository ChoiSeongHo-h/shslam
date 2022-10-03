#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/common_info_manager.hpp>

namespace shslam
{
    SlamSystem::CommonInfoManager::CommonInfoManager() :
    num_sensors_ptr(std::make_shared<NumSensors>())
    {}

    std::shared_ptr<const NumSensors> SlamSystem::CommonInfoManager::GetNumSensorsPtr() const
    {
        return num_sensors_ptr;
    }

    void SlamSystem::CommonInfoManager::ApplyConfig(const YAML::Node& config)
    {
        this->num_sensors_ptr->mono_cams = config["mono_cameras"].size();
    }
}
