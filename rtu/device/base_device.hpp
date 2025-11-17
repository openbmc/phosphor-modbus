#pragma once

#include "base_config.hpp"
#include "common/events.hpp"
#include "firmware/device_firmware.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Critical/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/aserver.hpp>

namespace phosphor::modbus::rtu::device
{

class BaseDevice;

using SensorIntf = sdbusplus::async::server_t<
    BaseDevice, sdbusplus::aserver::xyz::openbmc_project::sensor::Value,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::Availability,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::
        OperationalStatus,
    sdbusplus::aserver::xyz::openbmc_project::sensor::threshold::Warning,
    sdbusplus::aserver::xyz::openbmc_project::sensor::threshold::Critical,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

using PortIntf = phosphor::modbus::rtu::port::BasePort;
namespace EventIntf = phosphor::modbus::events;

class BaseDevice
{
  public:
    BaseDevice() = delete;

    explicit BaseDevice(sdbusplus::async::context& ctx,
                        const config::Config& config, PortIntf& serialPort,
                        EventIntf::Events& events);

    auto readSensorRegisters() -> sdbusplus::async::task<void>;

  private:
    auto createSensors() -> void;

    auto readStatusRegisters() -> sdbusplus::async::task<void>;

    auto generateEvent(const config::StatusBit& statusBit,
                       const sdbusplus::message::object_path& objectPath,
                       double sensorValue, SensorIntf::Unit sensorUnit,
                       bool statusAsserted) -> sdbusplus::async::task<void>;

    using sensors_map_t =
        std::unordered_map<std::string, std::unique_ptr<SensorIntf>>;
    sdbusplus::async::context& ctx;
    const config::Config config;
    PortIntf& serialPort;
    EventIntf::Events& events;
    std::unique_ptr<DeviceFirmware> currentFirmware;
    sensors_map_t sensors;
};

} // namespace phosphor::modbus::rtu::device
