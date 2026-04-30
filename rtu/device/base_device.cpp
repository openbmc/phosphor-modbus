#include "base_device.hpp"

#include "modbus_rtu_config.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <algorithm>
#include <numeric>
#include <span>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

auto getPathSuffix(ProfileIntf::SensorType type) -> std::string_view
{
    switch (type)
    {
        case ProfileIntf::SensorType::fanTach:
            return SensorValueIntf::namespace_path::fan_tach;
        case ProfileIntf::SensorType::liquidFlow:
            return SensorValueIntf::namespace_path::liquidflow;
        case ProfileIntf::SensorType::power:
            return SensorValueIntf::namespace_path::power;
        case ProfileIntf::SensorType::pressure:
            return SensorValueIntf::namespace_path::pressure;
        case ProfileIntf::SensorType::temperature:
            return SensorValueIntf::namespace_path::temperature;
        case ProfileIntf::SensorType::voltage:
            return SensorValueIntf::namespace_path::voltage;
        case ProfileIntf::SensorType::current:
            return SensorValueIntf::namespace_path::current;
        case ProfileIntf::SensorType::airflow:
            return SensorValueIntf::namespace_path::airflow;
        case ProfileIntf::SensorType::altitude:
            return SensorValueIntf::namespace_path::altitude;
        case ProfileIntf::SensorType::energy:
            return SensorValueIntf::namespace_path::energy;
        case ProfileIntf::SensorType::frequency:
            return SensorValueIntf::namespace_path::frequency;
        case ProfileIntf::SensorType::humidity:
            return SensorValueIntf::namespace_path::humidity;
        case ProfileIntf::SensorType::utilization:
            return SensorValueIntf::namespace_path::utilization;
        case ProfileIntf::SensorType::valve:
            return SensorValueIntf::namespace_path::valve;
        case ProfileIntf::SensorType::unknown:
            throw std::invalid_argument("Unknown sensor type");
    }
    throw std::invalid_argument("Unknown sensor type");
}

auto getUnit(ProfileIntf::SensorType type) -> SensorValueIntf::Unit
{
    switch (type)
    {
        case ProfileIntf::SensorType::fanTach:
            return SensorValueIntf::Unit::RPMS;
        case ProfileIntf::SensorType::liquidFlow:
            return SensorValueIntf::Unit::LPM;
        case ProfileIntf::SensorType::power:
            return SensorValueIntf::Unit::Watts;
        case ProfileIntf::SensorType::pressure:
            return SensorValueIntf::Unit::Pascals;
        case ProfileIntf::SensorType::temperature:
            return SensorValueIntf::Unit::DegreesC;
        case ProfileIntf::SensorType::voltage:
            return SensorValueIntf::Unit::Volts;
        case ProfileIntf::SensorType::current:
            return SensorValueIntf::Unit::Amperes;
        case ProfileIntf::SensorType::airflow:
            return SensorValueIntf::Unit::CFM;
        case ProfileIntf::SensorType::altitude:
            return SensorValueIntf::Unit::Meters;
        case ProfileIntf::SensorType::energy:
            return SensorValueIntf::Unit::Joules;
        case ProfileIntf::SensorType::frequency:
            return SensorValueIntf::Unit::Hertz;
        case ProfileIntf::SensorType::humidity:
            return SensorValueIntf::Unit::PercentRH;
        case ProfileIntf::SensorType::utilization:
            return SensorValueIntf::Unit::Percent;
        case ProfileIntf::SensorType::valve:
            return SensorValueIntf::Unit::Percent;
        case ProfileIntf::SensorType::unknown:
            throw std::invalid_argument("Unknown sensor type");
    }
    throw std::invalid_argument("Unknown sensor type");
}

BaseDevice::BaseDevice(sdbusplus::async::context& ctx,
                       const config::Config& config, PortIntf& serialPort,
                       EventIntf::Events& events) :
    ctx(ctx), config(config), serialPort(serialPort), events(events)
{
    createSensors();
    buildPollBuckets();

    if (!config.profile.firmwareRegisters.empty())
    {
        currentFirmware =
            std::make_unique<DeviceFirmware>(ctx, config, serialPort);
        ctx.spawn(currentFirmware->readVersionRegister());
    }

    info("Successfully created device {NAME}", "NAME", config.name);
}

static auto getObjectPath(std::string_view sensorType,
                          const std::string& sensorName)
    -> sdbusplus::object_path
{
    return sdbusplus::object_path(
        std::string(SensorIntf::namespace_path::value) + "/" +
        std::string(sensorType) + "/" + sensorName);
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
    for (const auto& sensorRegister : config.profile.sensorRegisters)
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
            std::numeric_limits<double>::quiet_NaN(),
            getUnit(sensorRegister.type)};

        auto sensorPath =
            getObjectPath(getPathSuffix(sensorRegister.type),
                          config.name + "_" + sensorRegister.name);

        auto sensor = std::make_unique<SensorIntf>(
            ctx, sensorPath.str.c_str(), initValue, initAvailability,
            initOperationalStatus, initWarning, initCritical, initAssociations);

        sensor->Value::emit_added();
        sensor->Availability::emit_added();
        sensor->OperationalStatus::emit_added();
        sensor->Warning::emit_added();
        sensor->Critical::emit_added();
        sensor->Definitions::emit_added();

        sensors.emplace(sensorRegister.name, std::move(sensor));
    }
}

auto BaseDevice::buildPollBuckets() -> void
{
    // Helper to find or create a bucket for a given poll interval.
    auto findOrCreateBucket =
        [this](std::chrono::seconds interval) -> PollBucket& {
        auto it = std::find_if(
            pollBuckets.begin(), pollBuckets.end(),
            [&](const auto& b) { return b.pollInterval == interval; });
        if (it != pollBuckets.end())
        {
            return *it;
        }
        pollBuckets.push_back({.pollInterval = interval,
                               .entries = {},
                               .spans = {},
                               .nextPollTime = {}});
        return pollBuckets.back();
    };

    // Group sensor entries into buckets by poll interval.
    for (const auto& reg : config.profile.sensorRegisters)
    {
        auto sensorIter = sensors.find(reg.name);
        if (sensorIter == sensors.end())
        {
            continue;
        }
        auto& bucket = findOrCreateBucket(reg.pollInterval);
        bucket.entries.emplace_back(SensorEntry{reg, *(sensorIter->second)});
    }

    // Add status entries to the default poll interval bucket.
    for (const auto& [address, statusBits] : config.profile.statusRegisters)
    {
        auto& bucket = findOrCreateBucket(defaultSensorPollInterval);
        bucket.entries.emplace_back(StatusEntry{address, &statusBits});
    }

    // Build register spans for each bucket.
    uint16_t maxSpanSize = 0;
    for (auto& bucket : pollBuckets)
    {
        buildBucketSpans(bucket);
        for (const auto& span : bucket.spans)
        {
            maxSpanSize = std::max(maxSpanSize, span.totalSize);
        }
    }

    readBuffer.resize(maxSpanSize);
}

auto BaseDevice::buildBucketSpans(PollBucket& bucket) -> void
{
    std::vector<RegisterInfo> regInfos;
    regInfos.reserve(bucket.entries.size());
    for (const auto& entry : bucket.entries)
    {
        if (std::holds_alternative<SensorEntry>(entry))
        {
            const auto& reg = std::get<SensorEntry>(entry).reg;
            regInfos.push_back({.offset = reg.offset, .size = reg.size});
        }
        else
        {
            const auto& status = std::get<StatusEntry>(entry);
            regInfos.push_back({.offset = status.address, .size = 1});
        }
    }

    bucket.spans = buildRegisterSpans(regInfos, maxRegisterSpanLength);
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

auto BaseDevice::processSensorEntry(const SensorEntry& entry,
                                    std::span<const uint16_t> spanBuffer,
                                    uint16_t spanStartOffset) -> void
{
    auto& [sensorRegister, sensor] = entry;
    auto regStart = sensorRegister.offset - spanStartOffset;
    auto regSlice = std::span<const uint16_t>(spanBuffer.data() + regStart,
                                              sensorRegister.size);

    double regVal = static_cast<double>(
        getRawIntegerFromRegister(regSlice, sensorRegister.isSigned));
    if (sensorRegister.format == ProfileIntf::SensorFormat::floatingPoint)
    {
        regVal = sensorRegister.shift +
                 (sensorRegister.scale *
                  (regVal / (1ULL << sensorRegister.precision)));
    }

    sensor.value(regVal);
}

auto BaseDevice::pollBucket(PollBucket& bucket) -> sdbusplus::async::task<void>
{
    for (const auto& span : bucket.spans)
    {
        auto spanBuffer = std::span(readBuffer.data(), span.totalSize);
        std::fill(spanBuffer.begin(), spanBuffer.end(), 0);
        auto ret = co_await serialPort.readHoldingRegisters(
            config.address, span.startOffset, config.profile.baudRate,
            config.profile.parity, spanBuffer);
        if (!ret)
        {
            for (auto idx : span.registerIndices)
            {
                if (std::holds_alternative<SensorEntry>(bucket.entries[idx]))
                {
                    auto& [sensorRegister,
                           sensor] = std::get<SensorEntry>(bucket.entries[idx]);
                    error(
                        "Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
                        "NAME", sensorRegister.name, "DEVICE_ADDRESS",
                        config.address);
                    sensor.value(std::numeric_limits<double>::quiet_NaN());
                    sensor.functional(false);
                }
                else
                {
                    error(
                        "Failed to read status registers for {DEVICE_ADDRESS}",
                        "DEVICE_ADDRESS", config.address);
                }
            }
            continue;
        }

        for (auto idx : span.registerIndices)
        {
            if (std::holds_alternative<SensorEntry>(bucket.entries[idx]))
            {
                processSensorEntry(std::get<SensorEntry>(bucket.entries[idx]),
                                   spanBuffer, span.startOffset);
            }
            else
            {
                co_await processStatusEntry(
                    std::get<StatusEntry>(bucket.entries[idx]), spanBuffer,
                    span.startOffset);
            }
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
auto BaseDevice::pollRegisters() -> sdbusplus::async::task<void>
{
    while (!ctx.stop_requested() && !stopRequested)
    {
        auto now = std::chrono::steady_clock::now();
        auto earliestNextPoll = std::chrono::steady_clock::time_point::max();

        for (auto& bucket : pollBuckets)
        {
            if (now < bucket.nextPollTime)
            {
                earliestNextPoll =
                    std::min(earliestNextPoll, bucket.nextPollTime);
                continue;
            }
            co_await pollBucket(bucket);
            bucket.nextPollTime = now + bucket.pollInterval;
            earliestNextPoll = std::min(earliestNextPoll, bucket.nextPollTime);
        }

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

    stopped = true;
}

static auto getObjectPath(const config::Config& config,
                          ProfileIntf::StatusType type, const std::string& name)
    -> sdbusplus::object_path
{
    switch (type)
    {
        case ProfileIntf::StatusType::sensorReadingCritical:
        case ProfileIntf::StatusType::sensorReadingWarning:
        case ProfileIntf::StatusType::sensorFailure:
            return sdbusplus::object_path(
                std::string(SensorIntf::namespace_path::value) + "/" + name);
        case ProfileIntf::StatusType::controllerFailure:
            return config.inventoryPath;
        case ProfileIntf::StatusType::pumpFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/pump/" + name);
        case ProfileIntf::StatusType::filterFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/filter/" + name);
        case ProfileIntf::StatusType::powerFault:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/power_rail/" + name);
        case ProfileIntf::StatusType::fanFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/fan/" + name);
        case ProfileIntf::StatusType::leakDetectedCritical:
        case ProfileIntf::StatusType::leakDetectedWarning:
            using DetectorIntf =
                sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector<
                    BaseDevice>;
            return sdbusplus::object_path(
                std::string(DetectorIntf::namespace_path::value) + "/" +
                DetectorIntf::namespace_path::detector + "/" + name);
        case ProfileIntf::StatusType::unknown:
            error("Unknown status type for {NAME}", "NAME", name);
    }

    return sdbusplus::object_path();
}

static auto updateSensorOnStatusChange(
    SensorIntf& sensor, ProfileIntf::StatusType statusType, bool statusAsserted)
{
    if (statusType == ProfileIntf::StatusType::sensorReadingCritical)
    {
        sensor.critical_alarm_high(statusAsserted);
    }
    else if (statusType == ProfileIntf::StatusType::sensorReadingWarning)
    {
        sensor.warning_alarm_high(statusAsserted);
    }
    else if (statusType == ProfileIntf::StatusType::sensorFailure)
    {
        if (statusAsserted)
        {
            sensor.value(std::numeric_limits<double>::quiet_NaN());
        }
        sensor.available(!statusAsserted);
        sensor.functional(!statusAsserted);
    }
}

auto BaseDevice::processStatusEntry(const StatusEntry& entry,
                                    std::span<const uint16_t> spanBuffer,
                                    uint16_t spanStartOffset)
    -> sdbusplus::async::task<void>
{
    auto regOffset = entry.address - spanStartOffset;

    for (const auto& statusBit : *entry.statusBits)
    {
        static constexpr auto maxBitPosition = 15;
        if (statusBit.bitPosition > maxBitPosition)
        {
            error("Invalid status bit position {POSITION} for {NAME}",
                  "POSITION", statusBit.bitPosition, "NAME", statusBit.name);
            continue;
        }
        auto statusBitValue =
            ((spanBuffer[regOffset] & (1 << statusBit.bitPosition)) != 0);
        auto statusAsserted = (statusBitValue == statusBit.value);
        auto objectPath = getObjectPath(config, statusBit.type, statusBit.name);
        double sensorValue = std::numeric_limits<double>::quiet_NaN();
        SensorIntf::Unit sensorUnit = SensorIntf::Unit::Percent;
        auto sensorIter = sensors.find(statusBit.name);
        if (sensorIter != sensors.end())
        {
            sensorValue = sensorIter->second->value();
            sensorUnit = sensorIter->second->unit();

            updateSensorOnStatusChange(*(sensorIter->second), statusBit.type,
                                       statusAsserted);
        }

        co_await generateEvent(statusBit, objectPath, sensorValue, sensorUnit,
                               statusAsserted);
    }
}

auto BaseDevice::generateEvent(const ProfileIntf::StatusBit& statusBit,
                               const sdbusplus::object_path& objectPath,
                               double sensorValue, SensorIntf::Unit sensorUnit,
                               bool statusAsserted)
    -> sdbusplus::async::task<void>
{
    switch (statusBit.type)
    {
        case ProfileIntf::StatusType::sensorReadingCritical:
            co_await events.generateSensorReadingEvent(
                objectPath, EventIntf::EventLevel::critical, sensorValue,
                sensorUnit, statusAsserted);
            break;
        case ProfileIntf::StatusType::sensorReadingWarning:
            co_await events.generateSensorReadingEvent(
                objectPath, EventIntf::EventLevel::warning, sensorValue,
                sensorUnit, statusAsserted);
            break;
        case ProfileIntf::StatusType::sensorFailure:
            co_await events.generateSensorFailureEvent(objectPath,
                                                       statusAsserted);
            break;
        case ProfileIntf::StatusType::controllerFailure:
            co_await events.generateControllerFailureEvent(
                objectPath, statusBit.name, statusAsserted);
            break;
        case ProfileIntf::StatusType::powerFault:
            co_await events.generatePowerFaultEvent(objectPath, statusBit.name,
                                                    statusAsserted);
            break;
        case ProfileIntf::StatusType::filterFailure:
            co_await events.generateFilterFailureEvent(objectPath,
                                                       statusAsserted);
            break;
        case ProfileIntf::StatusType::pumpFailure:
            co_await events.generatePumpFailureEvent(objectPath,
                                                     statusAsserted);
            break;
        case ProfileIntf::StatusType::fanFailure:
            co_await events.generateFanFailureEvent(objectPath, statusAsserted);
            break;
        case ProfileIntf::StatusType::leakDetectedCritical:
            co_await events.generateLeakDetectedEvent(
                objectPath, EventIntf::EventLevel::critical, statusAsserted);
            break;
        case ProfileIntf::StatusType::leakDetectedWarning:
            co_await events.generateLeakDetectedEvent(
                objectPath, EventIntf::EventLevel::warning, statusAsserted);
            break;
        case ProfileIntf::StatusType::unknown:
            error("Unknown status type for {NAME}", "NAME", statusBit.name);
            break;
    }
}

} // namespace phosphor::modbus::rtu::device
