#include "device_utils.hpp"

namespace phosphor::modbus::rtu::device
{

auto getUnitSuffix(ProfileIntf::SensorType type) -> std::string_view
{
    switch (type)
    {
        case ProfileIntf::SensorType::fanTach:
            return "_RPM";
        case ProfileIntf::SensorType::liquidFlow:
            return "_LPM";
        case ProfileIntf::SensorType::power:
            return "_W";
        case ProfileIntf::SensorType::pressure:
            return "_PA";
        case ProfileIntf::SensorType::temperature:
            return "_C";
        case ProfileIntf::SensorType::voltage:
            return "_V";
        case ProfileIntf::SensorType::current:
            return "_A";
        case ProfileIntf::SensorType::airflow:
            return "_CFM";
        case ProfileIntf::SensorType::altitude:
            return "_M";
        case ProfileIntf::SensorType::energy:
            return "_J";
        case ProfileIntf::SensorType::frequency:
            return "_HZ";
        case ProfileIntf::SensorType::humidity:
            return "_RH";
        case ProfileIntf::SensorType::utilization:
        case ProfileIntf::SensorType::valve:
            return "_PCT";
        case ProfileIntf::SensorType::charge:
            return "_AH";
        case ProfileIntf::SensorType::rotationalPosition:
            return "_RAD";
        case ProfileIntf::SensorType::unknown:
            return "";
    }
    return "";
}

auto getMetricUnitSuffix(ProfileIntf::MetricType type) -> std::string_view
{
    switch (type)
    {
        case ProfileIntf::MetricType::valveClosedDuration:
        case ProfileIntf::MetricType::valveOpenDuration:
            return "_SEC";
        case ProfileIntf::MetricType::unknown:
            return "";
    }
    return "";
}

} // namespace phosphor::modbus::rtu::device
