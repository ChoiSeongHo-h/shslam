#include <shslam/shslam_base.hpp>
#include <shslam/structs.hpp>
#include <shslam/slam_system.hpp>
#include <shslam/slam_system/common_info_manager.hpp>

namespace shslam
{
    std::shared_ptr<shslam::NumSensors> shslam::SlamSystem::CommonInfoManager::GetNumSensorsPtr() const
    {
        return num_sensors_ptr;
    }

    void shslam::SlamSystem::CommonInfoManager::SetNumSensors(std::shared_ptr<shslam::NumSensors> num_sensors_ptr)
    {
        this->num_sensors_ptr = num_sensors_ptr;
    }
}
