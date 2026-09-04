#include "utils/common.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace modbus_tool
{

namespace
{
constexpr auto lockPath = "/run/lock/phosphor-modbus.lock";
} // namespace

InstanceLock::~InstanceLock()
{
    if (fd >= 0)
    {
        // Closing the descriptor releases the lock.
        close(fd);
    }
}

auto InstanceLock::acquire() -> bool
{
    fd = open(lockPath, O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (fd < 0)
    {
        return false;
    }

    if (flock(fd, LOCK_EX | LOCK_NB) < 0)
    {
        close(fd);
        fd = -1;
        return false;
    }

    return true;
}

// Name a register type as the profile schema spells it.

auto inventoryName(ProfileIntf::InventoryDataType type) -> std::string_view
{
    switch (type)
    {
        case ProfileIntf::InventoryDataType::buildDate:
            return "BuildDate";
        case ProfileIntf::InventoryDataType::manufacturer:
            return "Manufacturer";
        case ProfileIntf::InventoryDataType::model:
            return "Model";
        case ProfileIntf::InventoryDataType::partNumber:
            return "PartNumber";
        case ProfileIntf::InventoryDataType::serialNumber:
            return "SerialNumber";
        case ProfileIntf::InventoryDataType::sparePartNumber:
            return "SparePartNumber";
        case ProfileIntf::InventoryDataType::unknown:
            return "Unknown";
    }
    return "Unknown";
}

auto statusTypeName(ProfileIntf::StatusType type) -> std::string_view
{
    switch (type)
    {
        case ProfileIntf::StatusType::controllerFailure:
            return "ControllerFailure";
        case ProfileIntf::StatusType::fanFailure:
            return "FanFailure";
        case ProfileIntf::StatusType::filterFailure:
            return "FilterFailure";
        case ProfileIntf::StatusType::powerFault:
            return "PowerFault";
        case ProfileIntf::StatusType::pumpFailure:
            return "PumpFailure";
        case ProfileIntf::StatusType::leakDetectedCritical:
            return "LeakDetectedCritical";
        case ProfileIntf::StatusType::leakDetectedWarning:
            return "LeakDetectedWarning";
        case ProfileIntf::StatusType::sensorFailure:
            return "SensorFailure";
        case ProfileIntf::StatusType::sensorReadingCritical:
            return "SensorReadingCritical";
        case ProfileIntf::StatusType::sensorReadingWarning:
            return "SensorReadingWarning";
        case ProfileIntf::StatusType::unknown:
            return "Unknown";
    }
    return "Unknown";
}

auto configTypeName(ProfileIntf::ConfigType type) -> std::string_view
{
    switch (type)
    {
        case ProfileIntf::ConfigType::unixTime:
            return "UnixTime";
        case ProfileIntf::ConfigType::init:
            return "Init";
        case ProfileIntf::ConfigType::unknown:
            return "Unknown";
    }
    return "Unknown";
}

} // namespace modbus_tool
