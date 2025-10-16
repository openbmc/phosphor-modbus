#include "base_device.hpp"

#include <phosphor-logging/lg2.hpp>

#include <numeric>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

BaseDevice::BaseDevice(sdbusplus::async::context& ctx,
                       const config::Config& config, PortIntf& serialPort) :
    ctx(ctx), config(config), serialPort(serialPort)
{
    createSensors();

    info("Successfully created device {NAME}", "NAME", config.name);
}

static auto getObjectPath(const std::string& sensorType,
                          const std::string& sensorName)
    -> sdbusplus::message::object_path
{
    return sdbusplus::message::object_path(
        std::string(SensorValueIntf::namespace_path::value) + "/" + sensorType +
        "/" + sensorName);
}

auto BaseDevice::createSensors() -> void
{
    for (const auto& sensorRegister : config.sensorRegisters)
    {
        SensorValueIntf::properties_t initProperties = {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), sensorRegister.unit};

        auto sensorPath = getObjectPath(
            sensorRegister.pathSuffix, config.name + "_" + sensorRegister.name);

        auto sensor = std::make_unique<SensorValueIntf>(
            ctx, sensorPath.str.c_str(), initProperties);

        sensor->emit_added();

        sensors.emplace(sensorRegister.name, std::move(sensor));
    }

    return;
}

static auto getRawIntegerFromRegister(const std::vector<uint16_t>& reg,
                                      bool sign) -> int64_t
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

auto BaseDevice::readSensorRegisters() -> sdbusplus::async::task<void>
{
    while (!ctx.stop_requested())
    {
        for (const auto& sensorRegister : config.sensorRegisters)
        {
            auto sensor = sensors.find(sensorRegister.name);
            if (sensor == sensors.end())
            {
                error("Sensor not found for {NAME}", "NAME",
                      sensorRegister.name);
                continue;
            }

            if (sensorRegister.size > 4)
            {
                error("Unsupported size for register {NAME}", "NAME",
                      sensorRegister.name);
                continue;
            }

            auto registers = std::vector<uint16_t>(sensorRegister.size);
            auto ret = co_await serialPort.readHoldingRegisters(
                config.address, sensorRegister.offset, config.baudRate,
                config.parity, registers);
            if (!ret)
            {
                error(
                    "Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
                    "NAME", sensorRegister.name, "DEVICE_ADDRESS",
                    config.address);
                continue;
            }

            double regVal = static_cast<double>(
                getRawIntegerFromRegister(registers, sensorRegister.isSigned));
            if (sensorRegister.format == config::SensorFormat::floatingPoint)
            {
                regVal = sensorRegister.shift +
                         (sensorRegister.scale *
                          (regVal / (1ULL << sensorRegister.precision)));
            }

            sensor->second->value(regVal);
        }

        constexpr auto pollInterval = 3;
        co_await sdbusplus::async::sleep_for(
            ctx, std::chrono::seconds(pollInterval));
        debug("Polling sensors for {NAME} in {INTERVAL} seconds", "NAME",
              config.name, "INTERVAL", pollInterval);
    }

    co_return;
}

} // namespace phosphor::modbus::rtu::device
