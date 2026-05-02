#pragma once

#include "modbus/modbus.hpp"
#include "modbus_rtu_config.hpp"

#include <xyz/openbmc_project/Sensor/Value/client.hpp>

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace phosphor::modbus::rtu::profile
{

using SensorValueIntf =
    sdbusplus::client::xyz::openbmc_project::sensor::Value<>;

enum class InventoryDataType
{
    buildDate,
    manufacturer,
    model,
    partNumber,
    serialNumber,
    sparePartNumber,
    unknown
};

struct InventoryRegister
{
    std::string name = "unknown";
    InventoryDataType type = InventoryDataType::unknown;
    uint16_t offset = 0;
    uint8_t size = 0;
};

enum class SensorFormat
{
    floatingPoint,
    integer,
    unknown
};

enum class SensorType
{
    fanTach,
    liquidFlow,
    power,
    pressure,
    temperature,
    voltage,
    current,
    airflow,
    altitude,
    energy,
    frequency,
    humidity,
    utilization,
    valve,
    unknown
};

struct SensorRegister
{
    std::string name = "unknown";
    SensorType type = SensorType::unknown;
    uint16_t offset = 0;
    uint8_t size = 0;
    uint8_t precision = 0;
    double scale = 1.0;
    double shift = 0.0;
    bool isSigned = false;
    SensorFormat format = SensorFormat::unknown;
    std::chrono::seconds pollInterval = defaultSensorPollInterval;
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
    uint8_t bitPosition = 0;
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
    batteryBackupUnit,
    capacitorBankUnit,
    flowMeter,
    heatExchanger,
    powerMonitorModule,
    powerSupplyUnit,
    reservoirPumpUnit,
    valve,
    unknown
};

enum class DeviceModel
{
    Artesyn7000433970000,
    Danfoss003Z8540,
    DeltaECD70000020,
    DeltaRDF040DSS5193E0,
    PanasonicBJBPM102A0001,
    unknown
};

struct ProbeRegister
{
    uint16_t offset = 0;
    uint8_t size = 0;
    std::variant<uint64_t, std::string> expectedValue;
};

struct DeviceProfile
{
    Parity parity;
    uint32_t baudRate;
    ProbeRegister probeRegister;
    std::vector<InventoryRegister> inventoryRegisters;
    std::vector<SensorRegister> sensorRegisters;
    std::unordered_map<uint16_t, std::vector<StatusBit>> statusRegisters;
    std::vector<FirmwareRegister> firmwareRegisters;
};

/** @brief Returns the device profile for a given device type.
 *  @param type The device type string (e.g., "DeltaRDF040DSS5193E0RPU").
 *  @throws std::out_of_range if type is not found. */
auto getDeviceProfile(std::string_view type) -> const DeviceProfile&;

/** @brief Returns the DeviceType for a given interface name.
 *  @throws std::out_of_range if type is not found. */
auto getDeviceType(std::string_view type) -> DeviceType;

/** @brief Returns the DeviceModel for a given interface name.
 *  @throws std::out_of_range if type is not found. */
auto getDeviceModel(std::string_view type) -> DeviceModel;

/** @brief Returns all device profile names. */
auto getProfileNames() -> std::vector<std::string>;

} // namespace phosphor::modbus::rtu::profile
