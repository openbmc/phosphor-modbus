#include "device_utils.hpp"

#include <phosphor-logging/lg2.hpp>

#include <bit>
#include <stdexcept>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

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

auto getUnit(ProfileIntf::SensorType type) -> SensorUnit
{
    switch (type)
    {
        case ProfileIntf::SensorType::fanTach:
            return SensorUnit::RPMS;
        case ProfileIntf::SensorType::liquidFlow:
            return SensorUnit::LPM;
        case ProfileIntf::SensorType::power:
            return SensorUnit::Watts;
        case ProfileIntf::SensorType::pressure:
            return SensorUnit::Pascals;
        case ProfileIntf::SensorType::temperature:
            return SensorUnit::DegreesC;
        case ProfileIntf::SensorType::voltage:
            return SensorUnit::Volts;
        case ProfileIntf::SensorType::current:
            return SensorUnit::Amperes;
        case ProfileIntf::SensorType::airflow:
            return SensorUnit::CFM;
        case ProfileIntf::SensorType::altitude:
            return SensorUnit::Meters;
        case ProfileIntf::SensorType::energy:
            return SensorUnit::Joules;
        case ProfileIntf::SensorType::frequency:
            return SensorUnit::Hertz;
        case ProfileIntf::SensorType::humidity:
            return SensorUnit::PercentRH;
        case ProfileIntf::SensorType::utilization:
            return SensorUnit::Percent;
        case ProfileIntf::SensorType::valve:
            return SensorUnit::Percent;
        case ProfileIntf::SensorType::charge:
            return SensorUnit::AmpereHours;
        case ProfileIntf::SensorType::rotationalPosition:
            return SensorUnit::Radians;
        case ProfileIntf::SensorType::unknown:
            throw std::invalid_argument("Unknown sensor type");
    }
    throw std::invalid_argument("Unknown sensor type");
}

auto getMetricUnit(ProfileIntf::MetricType type) -> MetricUnit
{
    switch (type)
    {
        case ProfileIntf::MetricType::valveClosedDuration:
        case ProfileIntf::MetricType::valveOpenDuration:
            return MetricUnit::Seconds;
        case ProfileIntf::MetricType::unknown:
            throw std::invalid_argument("Unknown metric type");
    }
    throw std::invalid_argument("Unknown metric type");
}

static auto getRawIntegerFromRegister(std::span<const uint16_t> reg, bool sign)
    -> int64_t
{
    if (reg.empty())
    {
        return 0;
    }

    uint64_t accumulator = 0;
    for (auto val : reg)
    {
        accumulator = (accumulator << 16) | val;
    }

    int64_t result = 0;

    if (sign)
    {
        if (reg.size() == 1)
        {
            result = static_cast<int16_t>(accumulator);
        }
        else if (reg.size() == 2)
        {
            result = static_cast<int32_t>(accumulator);
        }
        else
        {
            result = static_cast<int64_t>(accumulator);
        }
    }
    else
    {
        if (reg.size() == 1)
        {
            result = static_cast<uint16_t>(accumulator);
        }
        else if (reg.size() == 2)
        {
            result = static_cast<uint32_t>(accumulator);
        }
        else
        {
            result = static_cast<int64_t>(accumulator);
        }
    }

    return result;
}

static auto getFloat32FromRegister(std::span<const uint16_t> reg) -> double
{
    uint32_t rawBits = (static_cast<uint32_t>(reg[0]) << 16) |
                       static_cast<uint32_t>(reg[1]);

    return static_cast<double>(std::bit_cast<float>(rawBits));
}

auto convertRegisterValue(std::span<const uint16_t> reg,
                          ProfileIntf::SensorFormat format, bool isSigned,
                          uint8_t precision, double scale, double shift)
    -> double
{
    switch (format)
    {
        case ProfileIntf::SensorFormat::fixedPoint:
        {
            auto raw =
                static_cast<double>(getRawIntegerFromRegister(reg, isSigned));

            return shift + (scale * (raw / (1ULL << precision)));
        }
        case ProfileIntf::SensorFormat::float32:
        {
            auto raw = getFloat32FromRegister(reg);
            return shift + (scale * (raw / (1ULL << precision)));
        }
        case ProfileIntf::SensorFormat::integer:
            return static_cast<double>(
                getRawIntegerFromRegister(reg, isSigned));
        default:
            error("Unknown sensor register format");
            return 0.0;
    }
}

} // namespace phosphor::modbus::rtu::device
