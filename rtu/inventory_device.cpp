#include "inventory_device.hpp"

#include "common/entity_manager_interface.hpp"

#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Configuration/ModbusRTUDetect/client.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>

namespace phosphor::modbus::rtu::inventory
{
PHOSPHOR_LOG2_USING;

namespace config
{

using BasicVariantType =
    std::variant<std::vector<std::string>, std::vector<uint8_t>, std::string,
                 int64_t, uint64_t, double, int32_t, uint32_t, int16_t,
                 uint16_t, uint8_t, bool>;
using InventoryBaseConfigMap =
    boost::container::flat_map<std::string, BasicVariantType>;
using InventoryData =
    boost::container::flat_map<std::string, InventoryBaseConfigMap>;
using ManagedObjectType =
    boost::container::flat_map<sdbusplus::message::object_path, InventoryData>;

static constexpr std::array<std::pair<std::string_view, Parity>, 3>
    validParities = {
        {{"Odd", Parity::odd}, {"Even", Parity::even}, {"None", Parity::none}}};

// TODO: This API will be dropped once EM supports non-indexed interfaces for
// array objects.
static auto processModbusAddressInterface(
    Config& config, const InventoryBaseConfigMap& configMap) -> bool
{
    info("Processing ModbusAddress {NAME}", "NAME", config.name);

    auto rangeStartIter = configMap.find("RangeStart");
    if (rangeStartIter == configMap.end())
    {
        error("Missing RangeStart for {NAME}", "NAME", config.name);
        return false;
    }
    auto rangeStart = std::get<uint64_t>(rangeStartIter->second);

    auto rangeEndIter = configMap.find("RangeEnd");
    if (rangeEndIter == configMap.end())
    {
        error("Missing RangeEnd for {NAME}", "NAME", config.name);
        return false;
    }
    auto rangeEnd = std::get<uint64_t>(rangeEndIter->second);

    auto serialPortIter = configMap.find("SerialPort");
    if (serialPortIter == configMap.end())
    {
        error("Missing SerialPort for {NAME}", "NAME", config.name);
        return false;
    }
    auto serialPort = std::get<std::string>(serialPortIter->second);

    config.addressMap[serialPort].push_back({rangeStart, rangeEnd});

    info("ModbusAddress {NAME} {PORT} {START} {END}", "NAME", config.name,
         "PORT", serialPort, "START", rangeStart, "END", rangeEnd);

    return true;
}

// TODO: This API will be dropped once EM supports non-indexed interfaces for
// array objects.
static auto processModbusRegistersInterface(
    Config& config, const InventoryBaseConfigMap& configMap) -> bool
{
    info("Processing ModbusRegisters {NAME}", "NAME", config.name);
    Register registerConfig = {};

    auto nameIter = configMap.find("Name");
    if (nameIter == configMap.end())
    {
        error("Missing Name for {NAME}", "NAME", config.name);
        return false;
    }
    registerConfig.name = std::get<std::string>(nameIter->second);

    auto address = configMap.find("Address");
    if (address == configMap.end())
    {
        error("Missing Address for {NAME}", "NAME", config.name);
        return false;
    }
    registerConfig.offset = std::get<uint64_t>(address->second);

    auto sizeIter = configMap.find("Size");
    if (sizeIter == configMap.end())
    {
        error("Missing Size for {NAME}", "NAME", config.name);
        return false;
    }
    registerConfig.size = std::get<uint64_t>(sizeIter->second);

    config.registers.push_back(registerConfig);

    info("ModbusRegisters {NAME} {ADDRESS} {SIZE}", "NAME", registerConfig.name,
         "ADDRESS", registerConfig.offset, "SIZE", registerConfig.size);

    return true;
}

static auto printConfig(const Config& config) -> void
{
    info("Inventory device config: {NAME} {BAUDRATE} {PARITY}", "NAME",
         config.name, "BAUDRATE", config.baudRate, "PARITY", config.parity);

    for (const auto& [port, addressRanges] : config.addressMap)
    {
        for (const auto& addressRange : addressRanges)
        {
            info(
                "Inventory device config: {PORT} {ADDRESS_START} {ADDRESS_END}",
                "PORT", port, "ADDRESS_START", std::get<0>(addressRange),
                "ADDRESS_END", std::get<1>(addressRange));
        }
    }

    for (const auto& registerConfig : config.registers)
    {
        info("Inventory device config: {NAME} {ADDRESS} {SIZE}", "NAME",
             registerConfig.name, "ADDRESS", registerConfig.offset, "SIZE",
             registerConfig.size);
    }
}

auto getConfig(sdbusplus::async::context& ctx,
               sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<Config>>
{
    using ModbusRTUDetectIntf = sdbusplus::client::xyz::openbmc_project::
        configuration::ModbusRTUDetect<>;

    Config config = {};

    auto properties =
        co_await ModbusRTUDetectIntf(ctx)
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(objectPath.str)
            .properties();

    config.name = properties.name;
    config.baudRate = properties.baud_rate;

    for (const auto& [parityStr, parity] : config::validParities)
    {
        if (parityStr == properties.data_parity)
        {
            config.parity = parity;
            break;
        }
    }
    if (config.parity == Parity::unknown)
    {
        error("Invalid parity {PARITY} for {NAME}", "PARITY",
              properties.data_parity, "NAME", properties.name);
        co_return std::nullopt;
    }

    constexpr auto modbusAddressInterface =
        "xyz.openbmc_project.Configuration.ModbusRTUDetect.Address";
    constexpr auto modbusRegistersInterface =
        "xyz.openbmc_project.Configuration.ModbusRTUDetect.Registers";

    using InventoryIntf =
        sdbusplus::client::xyz::openbmc_project::inventory::Item<>;

    // TODO: This needs to be changed to get the related interfaces from the
    // same subtree as ModbusRTUDetect interface.
    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(InventoryIntf::namespace_path)
            .interface("org.freedesktop.DBus.ObjectManager");

    for (const auto& [path, deviceConfig] :
         co_await entityManager.call<ManagedObjectType>(ctx,
                                                        "GetManagedObjects"))
    {
        if (!(path.str).starts_with(objectPath.str))
        {
            debug("Skipping device {PATH}", "PATH", path.str);
            continue;
        }
        info("Processing device {PATH}", "PATH", path.str);
        for (const auto& [interfaceName, interfaceConfig] : deviceConfig)
        {
            if (interfaceName.starts_with(modbusAddressInterface))
            {
                if (!processModbusAddressInterface(config, interfaceConfig))
                {
                    error("Failed to process {INTERFACE} for {NAME}",
                          "INTERFACE", modbusAddressInterface, "NAME",
                          config.name);
                    co_return std::nullopt;
                }
            }
            else if (interfaceName.starts_with(modbusRegistersInterface))
            {
                if (!processModbusRegistersInterface(config, interfaceConfig))
                {
                    error("Failed to process {INTERFACE} for {NAME}",
                          "INTERFACE", modbusRegistersInterface, "NAME",
                          config.name);
                    co_return std::nullopt;
                }
            }
        }
    }

    printConfig(config);

    co_return config;
}

} // namespace config

Device::Device(sdbusplus::async::context& ctx, const config::Config& config,
               serial_port_map_t& serialPorts) :
    ctx(ctx), config(config), serialPorts(serialPorts)
{
    for (const auto& [serialPort, addressRanges] : config.addressMap)
    {
        if (serialPorts.find(serialPort) == serialPorts.end())
        {
            throw std::runtime_error("Serial port not found");
        }
    }
}

auto Device::probePorts() -> sdbusplus::async::task<void>
{
    info("Probing ports for {NAME}", "NAME", config.name);
    while (!ctx.stop_requested())
    {
        for (const auto& [serialPort, addressRanges] : config.addressMap)
        {
            ctx.spawn(probePort(serialPort));
        }
        constexpr auto probeInterval = 3;
        co_await sdbusplus::async::sleep_for(
            ctx, std::chrono::seconds(probeInterval));
        info("Probing ports for {NAME} in {INTERVAL} seconds", "NAME",
             config.name, "INTERVAL", probeInterval);
    }
}

auto Device::probePort(std::string portName) -> sdbusplus::async::task<void>
{
    info("Probing port {PORT}", "PORT", portName);

    auto portConfig = config.addressMap.find(portName);
    if (portConfig == config.addressMap.end())
    {
        error("Serial port {PORT} address map not found for {NAME}", "PORT",
              portName, "NAME", config.name);
        co_return;
    }
    auto addressRanges = portConfig->second;

    auto port = serialPorts.find(portName);
    if (port == serialPorts.end())
    {
        error("Serial port {PORT} not found for {NAME}", "PORT", portName,
              "NAME", config.name);
        co_return;
    }

    for (const auto& addressRange : addressRanges)
    {
        auto rangeStart = std::get<0>(addressRange);
        auto rangeEnd = std::get<1>(addressRange);
        for (auto address = rangeStart; address <= rangeEnd; address++)
        {
            co_await probeDevice(address, portName, port->second);
        }
    }
}

auto Device::probeDevice(uint8_t address, const std::string& portName,
                         const std::unique_ptr<SerialPortIntf>& port)
    -> sdbusplus::async::task<void>
{
    info("Probing device at {ADDRESS} on port {PORT}", "ADDRESS", address,
         "PORT", portName);

    if (config.registers.size() == 0)
    {
        error("No registers configured for {NAME}", "NAME", config.name);
        co_return;
    }
    auto probeRegister = config.registers[0].offset;
    auto registers = std::vector<uint16_t>(config.registers[0].size);

    auto ret = co_await port->readHoldingRegisters(
        address, probeRegister, config.baudRate, config.parity, registers);
    if (ret)
    {
        if (inventorySources.find(address) == inventorySources.end())
        {
            info("Device found at {ADDRESS}", "ADDRESS", address);
            co_await addInventorySource(address, portName, port);
        }
        else
        {
            info("Device already exists at {ADDRESS}", "ADDRESS", address);
        }
    }
    else
    {
        if (inventorySources.find(address) != inventorySources.end())
        {
            info(
                "Device removed at {ADDRESS} due to probe failure for {PROBE_REGISTER}",
                "ADDRESS", address, "PROBE_REGISTER", probeRegister);
            inventorySources[address]->emit_removed();
            inventorySources.erase(address);
        }
    }
}

auto Device::addInventorySource(uint8_t address, const std::string& portName,
                                const std::unique_ptr<SerialPortIntf>& port)
    -> sdbusplus::async::task<void>
{
    InventorySourceIntf::properties_t properties;

    constexpr auto partNumber = "PartNumber";
    constexpr auto sparePartNumber = "SparePartNumber";
    constexpr auto serialNumber = "SerialNumber";
    constexpr auto buildDate = "BuildDate";
    constexpr auto model = "Model";
    constexpr auto manufacturer = "Manufacturer";

    for (const auto& reg : config.registers)
    {
        auto registers = std::vector<uint16_t>(reg.size);
        auto ret = co_await port->readHoldingRegisters(
            address, reg.offset, config.baudRate, config.parity, registers);
        if (!ret)
        {
            error(
                "Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
                "NAME", reg.name, "DEVICE_ADDRESS", address);
            continue;
        }

        std::string strValue = "";

        for (const auto& value : registers)
        {
            strValue += static_cast<char>((value >> 8) & 0xFF);
            strValue += static_cast<char>(value & 0xFF);
        }

        if (reg.name == partNumber)
        {
            properties.part_number = strValue;
        }
        else if (reg.name == sparePartNumber)
        {
            properties.spare_part_number = strValue;
        }
        else if (reg.name == serialNumber)
        {
            properties.serial_number = strValue;
        }
        else if (reg.name == buildDate)
        {
            properties.build_date = strValue;
        }
        else if (reg.name == model)
        {
            properties.model = strValue;
        }
        else if (reg.name == manufacturer)
        {
            properties.manufacturer = strValue;
        }
    }

    auto objectPath =
        std::string(InventorySourceIntf::namespace_path) + "/" + config.name +
        "_" + std::to_string(address) + "_" + portName;

    properties.name = config.name;
    properties.address = address;
    properties.link_tty = portName;

    inventorySources[address] = std::make_unique<InventorySourceIntf>(
        ctx, objectPath.c_str(), properties);
    inventorySources[address]->emit_added();
}

} // namespace phosphor::modbus::rtu::inventory
