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

auto matchesProbeValue(std::span<const uint16_t> readBuffer,
                       const ProfileIntf::ProbeRegister& probe) -> bool
{
    return std::visit(
        [&readBuffer](const auto& expected) -> bool {
            using T = std::decay_t<decltype(expected)>;
            if constexpr (std::is_same_v<T, uint64_t>)
            {
                uint64_t value = 0;
                for (const auto& reg : readBuffer)
                {
                    value = (value << 16) | reg;
                }
                return value == expected;
            }
            else // std::string
            {
                std::string value;
                for (const auto& reg : readBuffer)
                {
                    value += static_cast<char>((reg >> 8) & 0xFF);
                    value += static_cast<char>(reg & 0xFF);
                }
                // Remove null characters
                std::erase(value, '\0');
                return value == expected;
            }
        },
        probe.expectedValue);
}

} // namespace phosphor::modbus::rtu::device
