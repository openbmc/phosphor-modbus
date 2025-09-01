#pragma once

#include "common/events.hpp"
#include "modbus/modbus.hpp"
#include "serial_port.hpp"

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>

namespace phosphor::modbus::rtu::device
{

class Device;

using SensorValueIntf =
    sdbusplus::aserver::xyz::openbmc_project::sensor::Value<Device>;
using SerialPortIntf = phosphor::modbus::rtu::serial_port::SerialPort;
namespace ModbusIntf = phosphor::modbus::rtu;
namespace EventIntf = phosphor::modbus::events;

namespace config
{

enum class SensorFormat
{
    floatingPoint,
    integer,
    unknown
};

struct SensorRegister
{
    std::string name = "unknown";
    std::string pathSuffix = "unknown";
    SensorValueIntf::Unit unit;
    uint16_t offset = 0;
    uint8_t size = 0;
    uint8_t precision = 0;
    double scale = 1.0;
    double shift = 0.0;
    bool isSigned = false;
    SensorFormat format = SensorFormat::unknown;
};

enum class StatusType
{
    controllerFailure,
    fanFailure,
    filterFailure,
    powerFault,
    pumpFailure,
    leakDetectedCritical,
    leakDetectedWarning,
    sensorFailure,
    sensorReadingCritical,
    sensorReadingWarning,
    unknown
};

struct StatusBit
{
    std::string name = "unknown";
    StatusType type = StatusType::unknown;
    uint8_t bitPostion = 0;
    bool value = false;
};

enum class FirmwareRegisterType
{
    version,
    update,
    unknown
};

struct FirmwareRegister
{
    std::string name = "unknown";
    FirmwareRegisterType type = FirmwareRegisterType::unknown;
    uint16_t offset = 0;
    uint8_t size = 0;
};

enum class DeviceType
{
    reservoirPumpUnit,
    heatExchanger,
    flowMeter,
    unknown
};

enum class DeviceModel
{
    RDF040DSS5193E0,
    unknown
};

struct Config
{
    using sensor_registers_t = std::vector<SensorRegister>;
    using status_registers_t =
        std::unordered_map<uint16_t, std::vector<StatusBit>>;
    using firmware_registers_t = std::vector<FirmwareRegister>;

    uint8_t address = 0;
    std::string name = "unknown";
    std::string portName = "unknown";
    DeviceType deviceType = DeviceType::unknown;
    DeviceModel deviceModel = DeviceModel::unknown;
    sdbusplus::message::object_path inventoryPath;
    sensor_registers_t sensorRegisters;
    status_registers_t statusRegisters;
    firmware_registers_t firmwareRegisters;
};

static constexpr auto ModbusRDF040DSS5193E0ReservoirPumpUnitInterface =
    "xyz.openbmc_project.Configuration.ModbusRDF040DSS5193E0ReservoirPumpUnit";
static constexpr auto ModbusRDF040DSS5193E0HeatExchangerInterface =
    "xyz.openbmc_project.Configuration.ModbusRDF040DSS5193E0HeatExchanger";

auto getConfig(sdbusplus::async::context& ctx,
               const sdbusplus::message::object_path& objectPath,
               const std::string& interfaceName)
    -> sdbusplus::async::task<std::optional<Config>>;

} // namespace config

class Device
{
  public:
    Device() = delete;

    explicit Device(sdbusplus::async::context& ctx,
                    const config::Config& config,
                    const std::unique_ptr<SerialPortIntf>& serialPort,
                    ModbusIntf::Parity parity, uint32_t baudRate,
                    EventIntf::Events& events);

  private:
    auto createSensors() -> sdbusplus::async::task<void>;

    auto readSensorRegisters() -> sdbusplus::async::task<void>;

    auto readStatusRegisters() -> sdbusplus::async::task<void>;

    auto generateEvent(const config::StatusBit& statusBit,
                       const sdbusplus::message::object_path& objectPath,
                       double sensorValue, SensorValueIntf::Unit sensorUnit,
                       bool statusAsserted) -> sdbusplus::async::task<void>;

    using sensors_map_t =
        std::unordered_map<std::string, std::unique_ptr<SensorValueIntf>>;
    // Event Intf
    sdbusplus::async::context& ctx;
    const config::Config config;
    const std::unique_ptr<SerialPortIntf>& serialPort;
    ModbusIntf::Parity parity;
    uint32_t baudRate;
    EventIntf::Events& events;
    sensors_map_t sensors;
};

} // namespace phosphor::modbus::rtu::device
