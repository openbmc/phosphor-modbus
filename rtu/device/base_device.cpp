#include "base_device.hpp"

#include "modbus_rtu_config.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <algorithm>
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
    buildSensorBuckets();

    if (!config.firmwareRegisters.empty())
    {
        currentFirmware =
            std::make_unique<DeviceFirmware>(ctx, config, serialPort);
        ctx.spawn(currentFirmware->readVersionRegister());
    }

    info("Successfully created device {NAME}", "NAME", config.name);
}

static auto getObjectPath(const std::string& sensorType,
                          const std::string& sensorName)
    -> sdbusplus::object_path
{
    return sdbusplus::object_path(
        std::string(SensorIntf::namespace_path::value) + "/" + sensorType +
        "/" + sensorName);
}

auto BaseDevice::createSensors() -> void
{
    constexpr SensorIntf::Availability::properties_t initAvailability{};
    constexpr SensorIntf::OperationalStatus::properties_t initOperationalStatus{
        true};
    constexpr SensorIntf::Warning::properties_t initWarning{};
    constexpr SensorIntf::Critical::properties_t initCritical{};
    const SensorIntf::Definitions::properties_t initAssociations{
        {{"monitoring", "monitored_by", config.inventoryPath},
         {"inventory", "sensors", config.inventoryPath},
         {"inventory", "all_sensors", config.inventoryPath}}};

    static constexpr auto maxRegisterSize = 4;
    for (const auto& sensorRegister : config.sensorRegisters)
    {
        if (sensorRegister.size > maxRegisterSize)
        {
            error("Unsupported size for register {NAME}, skipping", "NAME",
                  sensorRegister.name);
            continue;
        }

        SensorIntf::Value::properties_t initValue = {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), sensorRegister.unit};

        auto sensorPath = getObjectPath(
            sensorRegister.pathSuffix, config.name + "_" + sensorRegister.name);

        auto sensor = std::make_unique<SensorIntf>(
            ctx, sensorPath.str.c_str(), initValue, initAvailability,
            initOperationalStatus, initWarning, initCritical, initAssociations);

        sensor->Value::emit_added();
        sensor->Availability::emit_added();
        sensor->OperationalStatus::emit_added();
        sensor->Warning::emit_added();
        sensor->Critical::emit_added();
        sensor->Definitions::emit_added();

        sensorEntries.push_back({sensorRegister, *sensor});

        sensors.emplace(sensorRegister.name, std::move(sensor));
    }
}

auto BaseDevice::buildSensorBuckets() -> void
{
    // Group sensor entries by poll interval into buckets. Each bucket
    // tracks the sensorEntries indices that share the same interval.
    std::vector<std::pair<std::chrono::seconds, std::vector<size_t>>>
        bucketIndices;

    for (size_t i = 0; i < sensorEntries.size(); i++)
    {
        auto interval = sensorEntries[i].reg.pollInterval;
        auto it = std::find_if(
            bucketIndices.begin(), bucketIndices.end(),
            [&](const auto& entry) { return entry.first == interval; });
        if (it != bucketIndices.end())
        {
            it->second.push_back(i);
        }
        else
        {
            bucketIndices.emplace_back(interval, std::vector<size_t>{i});
        }
    }

    for (auto& [interval, indices] : bucketIndices)
    {
        std::vector<RegisterInfo> regInfos;
        regInfos.reserve(indices.size());
        for (auto idx : indices)
        {
            const auto& reg = sensorEntries[idx].reg;
            regInfos.push_back({.offset = reg.offset, .size = reg.size});
        }

        auto spans = buildRegisterSpans(regInfos, maxRegisterSpanLength,
                                        maxRegisterSpanGap);

        // Remap span indices from local regInfos back to sensorEntries indices.
        for (auto& span : spans)
        {
            for (auto& localIdx : span.registerIndices)
            {
                localIdx = indices[localIdx];
            }
        }

        sensorBuckets.push_back({.pollInterval = interval,
                                 .spans = std::move(spans),
                                 .nextPollTime = {}});
    }

    uint16_t maxSpanSize = 0;
    for (const auto& bucket : sensorBuckets)
    {
        for (const auto& span : bucket.spans)
        {
            maxSpanSize = std::max(maxSpanSize, span.totalSize);
        }
    }
    readBuffer.resize(maxSpanSize);
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

auto BaseDevice::pollSensorBucket(SensorBucket& bucket)
    -> sdbusplus::async::task<void>
{
    for (const auto& span : bucket.spans)
    {
        std::fill(readBuffer.begin(), readBuffer.end(), 0);
        auto ret = co_await serialPort.readHoldingRegisters(
            config.address, span.startOffset, config.baudRate, config.parity,
            readBuffer);
        if (!ret)
        {
            for (auto idx : span.registerIndices)
            {
                auto& [sensorRegister, sensor] = sensorEntries[idx];
                error(
                    "Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
                    "NAME", sensorRegister.name, "DEVICE_ADDRESS",
                    config.address);
                sensor.value(std::numeric_limits<double>::quiet_NaN());
                sensor.functional(false);
            }
            continue;
        }

        for (auto idx : span.registerIndices)
        {
            auto& [sensorRegister, sensor] = sensorEntries[idx];
            auto regStart = sensorRegister.offset - span.startOffset;
            auto regSlice = std::vector<uint16_t>(
                readBuffer.begin() + regStart,
                readBuffer.begin() + regStart + sensorRegister.size);

            double regVal = static_cast<double>(
                getRawIntegerFromRegister(regSlice, sensorRegister.isSigned));
            if (sensorRegister.format == config::SensorFormat::floatingPoint)
            {
                regVal = sensorRegister.shift +
                         (sensorRegister.scale *
                          (regVal / (1ULL << sensorRegister.precision)));
            }

            sensor.value(regVal);
        }
    }
}

// Sensor polling loop with per-bucket scheduling.
//
// Sensors are grouped into buckets by poll interval (e.g. 2s, 5s).
// Each bucket tracks its own nextPollTime. On every iteration:
//  1. Poll each bucket whose nextPollTime has passed, then advance its
//     timer. Skip buckets that aren't due yet.
//  2. Track the earliest nextPollTime across all buckets while iterating.
//  3. Sleep until that earliest time, then repeat.
//
// Example with 2s and 5s buckets:
//   t=0: poll both  -> sleep 2s
//   t=2: poll 2s    -> sleep 2s
//   t=4: poll 2s    -> sleep 1s  (5s bucket due at t=5)
//   t=5: poll 5s    -> sleep 1s  (2s bucket due at t=6)
auto BaseDevice::readSensorRegisters() -> sdbusplus::async::task<void>
{
    while (!ctx.stop_requested())
    {
        auto now = std::chrono::steady_clock::now();
        auto earliestNextPoll = std::chrono::steady_clock::time_point::max();

        for (auto& bucket : sensorBuckets)
        {
            if (now < bucket.nextPollTime)
            {
                earliestNextPoll =
                    std::min(earliestNextPoll, bucket.nextPollTime);
                continue;
            }
            co_await pollSensorBucket(bucket);
            bucket.nextPollTime = now + bucket.pollInterval;
            earliestNextPoll = std::min(earliestNextPoll, bucket.nextPollTime);
        }

        co_await readStatusRegisters();

        auto sleepDuration = std::chrono::duration_cast<std::chrono::seconds>(
            earliestNextPoll - std::chrono::steady_clock::now());
        if (sleepDuration > std::chrono::seconds(0))
        {
            co_await sdbusplus::async::sleep_for(ctx, sleepDuration);
        }
        else
        {
            warning(
                "No idle time between poll cycles for {NAME}, check poll interval configuration",
                "NAME", config.name);
        }
        debug("Polling sensors for {NAME}", "NAME", config.name);
    }

    co_return;
}

static auto getObjectPath(const config::Config& config, config::StatusType type,
                          const std::string& name) -> sdbusplus::object_path
{
    switch (type)
    {
        case config::StatusType::sensorReadingCritical:
        case config::StatusType::sensorReadingWarning:
        case config::StatusType::sensorFailure:
            return sdbusplus::object_path(
                std::string(SensorIntf::namespace_path::value) + "/" + name);
        case config::StatusType::controllerFailure:
            return config.inventoryPath;
        case config::StatusType::pumpFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/pump/" + name);
        case config::StatusType::filterFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/filter/" + name);
        case config::StatusType::powerFault:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/power_rail/" + name);
        case config::StatusType::fanFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/fan/" + name);
        case config::StatusType::leakDetectedCritical:
        case config::StatusType::leakDetectedWarning:
            using DetectorIntf =
                sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector<
                    BaseDevice>;
            return sdbusplus::object_path(
                std::string(DetectorIntf::namespace_path::value) + "/" +
                DetectorIntf::namespace_path::detector + "/" + name);
        case config::StatusType::unknown:
            error("Unknown status type for {NAME}", "NAME", name);
    }

    return sdbusplus::object_path();
}

static auto updateSensorOnStatusChange(
    SensorIntf& sensor, config::StatusType statusType, bool statusAsserted)
{
    if (statusType == config::StatusType::sensorReadingCritical)
    {
        sensor.critical_alarm_high(statusAsserted);
    }
    else if (statusType == config::StatusType::sensorReadingWarning)
    {
        sensor.warning_alarm_high(statusAsserted);
    }
    else if (statusType == config::StatusType::sensorFailure)
    {
        if (statusAsserted)
        {
            sensor.value(std::numeric_limits<double>::quiet_NaN());
        }
        sensor.available(!statusAsserted);
        sensor.functional(!statusAsserted);
    }
}

auto BaseDevice::readStatusRegisters() -> sdbusplus::async::task<void>
{
    for (const auto& [address, statusBits] : config.statusRegisters)
    {
        static constexpr auto maxRegisterSize = 1;
        auto registers = std::vector<uint16_t>(maxRegisterSize);

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
            static constexpr auto maxBitPoistion = 15;
            if (statusBit.bitPosition > maxBitPoistion)
            {
                error("Invalid status bit position {POSITION} for {NAME}",
                      "POSITION", statusBit.bitPosition, "NAME",
                      statusBit.name);
                continue;
            }
            auto statusBitValue =
                ((registers[0] & (1 << statusBit.bitPosition)) != 0);
            auto statusAsserted = (statusBitValue == statusBit.value);
            auto objectPath =
                getObjectPath(config, statusBit.type, statusBit.name);
            double sensorValue = std::numeric_limits<double>::quiet_NaN();
            SensorIntf::Unit sensorUnit = SensorIntf::Unit::Percent;
            auto sensorIter = sensors.find(statusBit.name);
            if (sensorIter != sensors.end())
            {
                sensorValue = sensorIter->second->value();
                sensorUnit = sensorIter->second->unit();

                updateSensorOnStatusChange(*(sensorIter->second),
                                           statusBit.type, statusAsserted);
            }

            co_await generateEvent(statusBit, objectPath, sensorValue,
                                   sensorUnit, statusAsserted);
        }
    }

    co_return;
}

auto BaseDevice::generateEvent(const config::StatusBit& statusBit,
                               const sdbusplus::object_path& objectPath,
                               double sensorValue, SensorIntf::Unit sensorUnit,
                               bool statusAsserted)
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
