#include "base_device.hpp"

#include "device_utils.hpp"
#include "modbus_rtu_config.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <algorithm>
#include <bit>
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
        case ProfileIntf::SensorType::charge:
            return SensorValueIntf::namespace_path::charge;
        case ProfileIntf::SensorType::rotationalPosition:
            return SensorValueIntf::namespace_path::rotational_position;
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
        case ProfileIntf::SensorType::charge:
            return SensorValueIntf::Unit::AmpereHours;
        case ProfileIntf::SensorType::rotationalPosition:
            return SensorValueIntf::Unit::Radians;
        case ProfileIntf::SensorType::unknown:
            throw std::invalid_argument("Unknown sensor type");
    }
    throw std::invalid_argument("Unknown sensor type");
}

auto getMetricPathSuffix(ProfileIntf::MetricType type) -> std::string_view
{
    switch (type)
    {
        case ProfileIntf::MetricType::valveClosedDuration:
            return MetricIntf::namespace_path::valve_closed_duration;
        case ProfileIntf::MetricType::valveOpenDuration:
            return MetricIntf::namespace_path::valve_open_duration;
        case ProfileIntf::MetricType::unknown:
            throw std::invalid_argument("Unknown metric type");
    }
    throw std::invalid_argument("Unknown metric type");
}

auto getMetricUnit(ProfileIntf::MetricType type) -> MetricIntf::Unit
{
    switch (type)
    {
        case ProfileIntf::MetricType::valveClosedDuration:
        case ProfileIntf::MetricType::valveOpenDuration:
            return MetricIntf::Unit::Seconds;
        case ProfileIntf::MetricType::unknown:
            throw std::invalid_argument("Unknown metric type");
    }
    throw std::invalid_argument("Unknown metric type");
}

BaseDevice::BaseDevice(sdbusplus::async::context& ctx,
                       const config::Config& config, PortIntf& serialPort,
                       EventIntf::Events& events) :
    ctx(ctx), config(config), serialPort(serialPort), events(events),
    deviceConfig(this->config, serialPort)
{
    createSensors();
    createMetrics();
    buildPollBuckets();

    if (!config.profile.firmwareRegisters.empty())
    {
        currentFirmware =
            std::make_unique<DeviceFirmware>(ctx, config, serialPort);
        ctx.spawn(currentFirmware->readVersionRegister());
    }

    info("Successfully created device {NAME}", "NAME", config.name);
}

BaseDevice::~BaseDevice()
{
    for (auto& [name, sensor] : sensors)
    {
        sensor->Value::emit_removed();
        sensor->Availability::emit_removed();
        sensor->OperationalStatus::emit_removed();
        sensor->Warning::emit_removed();
        sensor->Critical::emit_removed();
        sensor->Definitions::emit_removed();
    }
    for (auto& [name, metric] : metrics)
    {
        metric->Value::emit_removed();
        metric->Definitions::emit_removed();
    }
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
        {{"monitoring", "monitored_by", config.parentInventoryPath},
         {"inventory", "sensors", config.parentInventoryPath},
         {"inventory", "all_sensors", config.parentInventoryPath},
         {"monitoring", "monitored_by", config.inventoryPath},
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

        auto sensorName = config.name + "_" + sensorRegister.name;
        if constexpr (appendUnitSuffix)
        {
            sensorName += getUnitSuffix(sensorRegister.type);
        }
        auto sensorPath =
            getObjectPath(getPathSuffix(sensorRegister.type), sensorName);

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

static auto getMetricObjectPath(std::string_view metricType,
                                const std::string& metricName)
    -> sdbusplus::object_path
{
    return sdbusplus::object_path(
        std::string(MetricIntf::namespace_path::value) + "/" +
        std::string(metricType) + "/" + metricName);
}

auto BaseDevice::createMetrics() -> void
{
    const MetricIntf::Definitions::properties_t initAssociations{
        {{"measuring", "measured_by", config.parentInventoryPath},
         {"measuring", "measured_by", config.inventoryPath}}};

    static constexpr auto maxRegisterSize = 4;
    for (const auto& metricRegister : config.profile.metricRegisters)
    {
        if (metricRegister.size > maxRegisterSize)
        {
            error("Unsupported size for metric register {NAME}, skipping",
                  "NAME", metricRegister.name);
            continue;
        }

        MetricIntf::Value::properties_t initValue = {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            getMetricUnit(metricRegister.type)};

        auto metricName = config.name + "_" + metricRegister.name;
        if constexpr (appendUnitSuffix)
        {
            metricName += getMetricUnitSuffix(metricRegister.type);
        }
        auto metricPath = getMetricObjectPath(
            getMetricPathSuffix(metricRegister.type), metricName);

        auto metric = std::make_unique<MetricIntf>(ctx, metricPath.str.c_str(),
                                                   initValue, initAssociations);

        metric->Value::emit_added();
        metric->Definitions::emit_added();

        metrics.emplace(metricRegister.name, std::move(metric));
    }
}

auto BaseDevice::findOrCreateBucket(std::chrono::seconds interval)
    -> PollBucket&
{
    auto it =
        std::find_if(pollBuckets.begin(), pollBuckets.end(),
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
}

auto BaseDevice::buildPollBuckets() -> void
{
    auto& bucket = findOrCreateBucket(config.pollRate);

    for (const auto& reg : config.profile.sensorRegisters)
    {
        auto sensorIter = sensors.find(reg.name);
        if (sensorIter == sensors.end())
        {
            continue;
        }
        bucket.entries.emplace_back(SensorEntry{reg, *(sensorIter->second)});
    }

    for (const auto& reg : config.profile.metricRegisters)
    {
        auto metricIter = metrics.find(reg.name);
        if (metricIter == metrics.end())
        {
            continue;
        }
        bucket.entries.emplace_back(MetricEntry{reg, *(metricIter->second)});
    }

    for (const auto& [address, statusBits] : config.profile.statusRegisters)
    {
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
        else if (std::holds_alternative<MetricEntry>(entry))
        {
            const auto& reg = std::get<MetricEntry>(entry).reg;
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

static auto getFloat32FromRegister(std::span<const uint16_t> reg) -> double
{
    uint32_t rawBits = (static_cast<uint32_t>(reg[0]) << 16) |
                       static_cast<uint32_t>(reg[1]);

    return static_cast<double>(std::bit_cast<float>(rawBits));
}

static auto convertRegisterValue(
    std::span<const uint16_t> reg, ProfileIntf::SensorFormat format,
    bool isSigned, uint8_t precision, double scale, double shift) -> double
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

auto BaseDevice::processSensorEntry(const SensorEntry& entry,
                                    std::span<const uint16_t> spanBuffer,
                                    uint16_t spanStartOffset) -> void
{
    auto& [sensorRegister, sensor] = entry;
    auto regStart = sensorRegister.offset - spanStartOffset;
    auto regSlice = std::span<const uint16_t>(spanBuffer.data() + regStart,
                                              sensorRegister.size);

    auto regVal = convertRegisterValue(
        regSlice, sensorRegister.format, sensorRegister.isSigned,
        sensorRegister.precision, sensorRegister.scale, sensorRegister.shift);

    sensor.value(regVal);
    sensor.functional(true);
    sensor.available(true);
}

auto BaseDevice::processMetricEntry(const MetricEntry& entry,
                                    std::span<const uint16_t> spanBuffer,
                                    uint16_t spanStartOffset) -> void
{
    auto& [metricRegister, metric] = entry;
    auto regStart = metricRegister.offset - spanStartOffset;
    auto regSlice = std::span<const uint16_t>(spanBuffer.data() + regStart,
                                              metricRegister.size);

    auto regVal = convertRegisterValue(
        regSlice, metricRegister.format, metricRegister.isSigned,
        metricRegister.precision, metricRegister.scale, metricRegister.shift);

    metric.value(regVal);
}

auto BaseDevice::handleSpanReadFailure(PollBucket& bucket,
                                       const RegisterSpan& span) -> void
{
    for (auto idx : span.registerIndices)
    {
        if (std::holds_alternative<SensorEntry>(bucket.entries[idx]))
        {
            auto& [sensorRegister,
                   sensor] = std::get<SensorEntry>(bucket.entries[idx]);
            error(
                "Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
                "NAME", sensorRegister.name, "DEVICE_ADDRESS", lg2::hex,
                config.address);
            sensor.value(std::numeric_limits<double>::quiet_NaN());
            sensor.functional(false);
        }
        else if (std::holds_alternative<MetricEntry>(bucket.entries[idx]))
        {
            auto& [metricRegister,
                   metric] = std::get<MetricEntry>(bucket.entries[idx]);
            error(
                "Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
                "NAME", metricRegister.name, "DEVICE_ADDRESS", lg2::hex,
                config.address);
            metric.value(std::numeric_limits<double>::quiet_NaN());
        }
        else
        {
            error("Failed to read status registers for {DEVICE_ADDRESS}",
                  "DEVICE_ADDRESS", lg2::hex, config.address);
        }
    }
}

auto BaseDevice::handleSpanBusy(PollBucket& bucket, const RegisterSpan& span)
    -> void
{
    // The port is in use (e.g. firmware update), so the read was not
    // attempted. Mark sensors unavailable and blank the reading to signal
    // transient unavailability, without faulting them (functional untouched).
    for (auto idx : span.registerIndices)
    {
        if (std::holds_alternative<SensorEntry>(bucket.entries[idx]))
        {
            auto& sensor = std::get<SensorEntry>(bucket.entries[idx]).sensor;
            sensor.value(std::numeric_limits<double>::quiet_NaN());
            sensor.available(false);
        }
    }
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
        if (ret == port::OperationStatus::busy)
        {
            handleSpanBusy(bucket, span);
            continue;
        }
        if (ret != port::OperationStatus::success)
        {
            handleSpanReadFailure(bucket, span);
            continue;
        }

        for (auto idx : span.registerIndices)
        {
            if (std::holds_alternative<SensorEntry>(bucket.entries[idx]))
            {
                processSensorEntry(std::get<SensorEntry>(bucket.entries[idx]),
                                   spanBuffer, span.startOffset);
            }
            else if (std::holds_alternative<MetricEntry>(bucket.entries[idx]))
            {
                processMetricEntry(std::get<MetricEntry>(bucket.entries[idx]),
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
    co_await deviceConfig.writeInitial();

    auto configNextWrite = std::chrono::steady_clock::now();

    while (!ctx.stop_requested() && !stopRequested)
    {
        auto earliestNextPoll = std::chrono::steady_clock::time_point::max();

        for (auto& bucket : pollBuckets)
        {
            auto now = std::chrono::steady_clock::now();
            if (now < bucket.nextPollTime)
            {
                earliestNextPoll =
                    std::min(earliestNextPoll, bucket.nextPollTime);
                continue;
            }
            co_await pollBucket(bucket);
            bucket.nextPollTime =
                std::chrono::steady_clock::now() + bucket.pollInterval;
            earliestNextPoll = std::min(earliestNextPoll, bucket.nextPollTime);
        }

        if (std::chrono::steady_clock::now() >= configNextWrite)
        {
            configNextWrite = co_await deviceConfig.writePeriodic();
        }
        earliestNextPoll = std::min(earliestNextPoll, configNextWrite);

        co_await sleepUntilNextPoll(earliestNextPoll);
        debug("Polling sensors for {NAME}", "NAME", config.name);
    }

    stopped = true;
}

auto BaseDevice::sleepUntilNextPoll(
    std::chrono::steady_clock::time_point nextPoll)
    -> sdbusplus::async::task<void>
{
    auto sleepDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
        nextPoll - std::chrono::steady_clock::now());
    if (sleepDuration <= std::chrono::milliseconds(0))
    {
        debug(
            "No idle time between poll cycles for {NAME}, check poll interval configuration",
            "NAME", config.name);
        co_return;
    }

    // Sleep in short intervals so we can respond to stop requests promptly
    // instead of blocking for the full poll interval.
    constexpr auto stopCheckInterval = std::chrono::milliseconds(3000);
    for (auto elapsed = std::chrono::milliseconds(0);
         elapsed < sleepDuration && !ctx.stop_requested() && !stopRequested;)
    {
        auto sleepTime = std::min(sleepDuration - elapsed, stopCheckInterval);
        co_await sdbusplus::async::sleep_for(ctx, sleepTime);
        elapsed += sleepTime;
    }
}

static auto getObjectPath(const config::Config& config,
                          ProfileIntf::StatusType type, const std::string& name)
    -> sdbusplus::object_path
{
    auto fullName = config.name + "_" + name;
    switch (type)
    {
        case ProfileIntf::StatusType::sensorReadingCritical:
        case ProfileIntf::StatusType::sensorReadingWarning:
        case ProfileIntf::StatusType::sensorFailure:
            return sdbusplus::object_path(
                std::string(SensorIntf::namespace_path::value) + "/" +
                fullName);
        case ProfileIntf::StatusType::controllerFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/smc/" + fullName);
        case ProfileIntf::StatusType::pumpFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/pump/" + fullName);
        case ProfileIntf::StatusType::filterFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/filter/" + fullName);
        case ProfileIntf::StatusType::powerFault:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/power_rail/" + fullName);
        case ProfileIntf::StatusType::fanFailure:
            return sdbusplus::object_path(
                "/xyz/openbmc_project/state/fan/" + fullName);
        case ProfileIntf::StatusType::leakDetectedCritical:
        case ProfileIntf::StatusType::leakDetectedWarning:
            using DetectorIntf =
                sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector<
                    BaseDevice>;
            return sdbusplus::object_path(
                std::string(DetectorIntf::namespace_path::value) + "/" +
                DetectorIntf::namespace_path::detector + "/" + fullName);
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
