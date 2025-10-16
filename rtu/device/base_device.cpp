#include "base_device.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <numeric>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

BaseDevice::BaseDevice(sdbusplus::async::context& ctx,
                       const config::Config& config, PortIntf& serialPort,
                       EventIntf::Events& events) :
    ctx(ctx), config(config), serialPort(serialPort), events(events)
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

        co_await readStatusRegisters();

        constexpr auto pollInterval = 3;
        co_await sdbusplus::async::sleep_for(
            ctx, std::chrono::seconds(pollInterval));
        debug("Polling sensors for {NAME} in {INTERVAL} seconds", "NAME",
              config.name, "INTERVAL", pollInterval);
    }

    co_return;
}

static auto getObjectPath(const config::Config& config, config::StatusType type,
                          const std::string& name)
    -> sdbusplus::message::object_path
{
    switch (type)
    {
        case config::StatusType::sensorReadingCritical:
        case config::StatusType::sensorReadingWarning:
        case config::StatusType::sensorFailure:
            return sdbusplus::message::object_path(
                std::string(SensorValueIntf::namespace_path::value) + "/" +
                name);
        case config::StatusType::controllerFailure:
            return config.inventoryPath;
        case config::StatusType::pumpFailure:
            return sdbusplus::message::object_path(
                "/xyz/openbmc_project/state/pump/" + name);
        case config::StatusType::filterFailure:
            return sdbusplus::message::object_path(
                "/xyz/openbmc_project/state/filter/" + name);
        case config::StatusType::powerFault:
            return sdbusplus::message::object_path(
                "/xyz/openbmc_project/state/power_rail/" + name);
        case config::StatusType::fanFailure:
            return sdbusplus::message::object_path(
                "/xyz/openbmc_project/state/fan/" + name);
        case config::StatusType::leakDetectedCritical:
        case config::StatusType::leakDetectedWarning:
            using DetectorIntf =
                sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector<
                    Device>;
            return sdbusplus::message::object_path(
                std::string(DetectorIntf::namespace_path::value) + "/" +
                DetectorIntf::namespace_path::detector + "/" + name);
        case config::StatusType::unknown:
            error("Unknown status type for {NAME}", "NAME", name);
    }

    return sdbusplus::message::object_path();
}

auto BaseDevice::readStatusRegisters() -> sdbusplus::async::task<void>
{
    for (const auto& [address, statusBits] : config.statusRegisters)
    {
        auto registers = std::vector<uint16_t>(1);
        auto ret = co_await serialPort.readHoldingRegisters(
            config.address, address, config.baudRate, config.parity, registers);
        if (!ret)
        {
            error("Failed to read holding registers for {DEVICE_ADDRESS}",
                  "DEVICE_ADDRESS", config.address);
            continue;
        }

        for (const auto& statusBit : statusBits)
        {
            auto statusBitValue = (registers[0] & (1 << statusBit.bitPosition));
            auto statusAsserted = (statusBitValue == statusBit.value);
            auto objectPath =
                getObjectPath(config, statusBit.type, statusBit.name);
            double sensorValue = std::numeric_limits<double>::quiet_NaN();
            SensorValueIntf::Unit sensorUnit = SensorValueIntf::Unit::Percent;
            auto sensorIter = sensors.find(statusBit.name);
            if (sensorIter != sensors.end())
            {
                sensorValue = sensorIter->second->value();
                sensorUnit = sensorIter->second->unit();
            }

            co_await generateEvent(statusBit, objectPath, sensorValue,
                                   sensorUnit, statusAsserted);
        }
    }

    co_return;
}

auto BaseDevice::generateEvent(
    const config::StatusBit& statusBit,
    const sdbusplus::message::object_path& objectPath, double sensorValue,
    SensorValueIntf::Unit sensorUnit, bool statusAsserted)
    -> sdbusplus::async::task<void>
{
    switch (statusBit.type)
    {
        case config::StatusType::sensorReadingCritical:
            co_await events.generateSensorReadingEvent(
                objectPath, EventIntf::EventLevel::critical, sensorValue,
                sensorUnit, statusAsserted);
            break;
        case config::StatusType::sensorReadingWarning:
            co_await events.generateSensorReadingEvent(
                objectPath, EventIntf::EventLevel::warning, sensorValue,
                sensorUnit, statusAsserted);
            break;
        case config::StatusType::sensorFailure:
            co_await events.generateSensorFailureEvent(objectPath,
                                                       statusAsserted);
            break;
        case config::StatusType::controllerFailure:
            co_await events.generateControllerFailureEvent(
                objectPath, statusBit.name, statusAsserted);
            break;
        case config::StatusType::powerFault:
            co_await events.generatePowerFaultEvent(objectPath, statusBit.name,
                                                    statusAsserted);
            break;
        case config::StatusType::filterFailure:
            co_await events.generateFilterFailureEvent(objectPath,
                                                       statusAsserted);
            break;
        case config::StatusType::pumpFailure:
            co_await events.generatePumpFailureEvent(objectPath,
                                                     statusAsserted);
            break;
        case config::StatusType::fanFailure:
            co_await events.generateFanFailureEvent(objectPath, statusAsserted);
            break;
        case config::StatusType::leakDetectedCritical:
            co_await events.generateLeakDetectedEvent(
                objectPath, EventIntf::EventLevel::critical, statusAsserted);
            break;
        case config::StatusType::leakDetectedWarning:
            co_await events.generateLeakDetectedEvent(
                objectPath, EventIntf::EventLevel::warning, statusAsserted);
            break;
        case config::StatusType::unknown:
            error("Unknown status type for {NAME}", "NAME", statusBit.name);
            break;
    }
}

} // namespace phosphor::modbus::rtu::device
