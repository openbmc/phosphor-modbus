#include "cmd/dump.hpp"
#include "test_base.hpp"
#include "utils/json_writer.hpp"
#include "utils/port_reservation.hpp"

#include <nlohmann/json.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>
#include <xyz/openbmc_project/Object/Enable/aserver.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

using namespace std::literals;
using namespace modbus_tool;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace ProfileIntf = phosphor::modbus::rtu::profile;

namespace
{

/** @brief A device that read cleanly, with one register of each interesting
 *  shape: a multi word inventory register and a status register with bits. */
auto readDevice() -> DeviceDump
{
    DeviceDump device{
        .name = "PSU_1_1",
        .type = "DeltaECD17020037PowerSupplyUnit",
        .address = 0x90,
        .serialPort = "ttyRS485-1",
        .result = Result::success,
    };

    device.registers.inventory.emplace_back(RegisterDump{
        .name = "Model",
        .offset = 0x8,
        .size = 4,
        .read = true,
        .raw = {0x4543, 0x4431, 0x3730, 0x3230},
    });

    device.registers.status.emplace_back(RegisterDump{
        .name = "PFC_ALARM",
        .offset = 0x3D,
        .size = 1,
        .read = true,
        .raw = {0x0100},
        .bits = {BitDump{.position = 0,
                         .name = "AC_UNDER_VOLTAGE",
                         .type = "SensorReadingCritical",
                         .asserted = false},
                 BitDump{.position = 8,
                         .name = "AC_NOT_OK",
                         .type = "PowerFault",
                         .asserted = true}},
    });

    return device;
}

} // namespace

// Every dump carries the schema version and what produced it, so a consumer
// can tell whether it understands the rest.
TEST(ModbusToolSchema, TestMetadata)
{
    auto json = toJson(Dump{});

    const auto& metadata = json.at("Metadata");
    EXPECT_EQ(metadata.at("SchemaVersion"), "1.0.0");
    EXPECT_EQ(metadata.at("Tool"), "modbus-tool");
    EXPECT_TRUE(metadata.contains("BMCVersion"));
    EXPECT_TRUE(metadata.contains("Timestamp"));
    EXPECT_TRUE(json.at("Devices").empty());
}

// Devices are an array whatever was asked for, so both modes of the tool
// produce the same shape.
TEST(ModbusToolSchema, TestDevicesAreAlwaysAnArray)
{
    Dump dump;
    dump.devices.emplace_back(readDevice());

    auto json = toJson(dump);

    ASSERT_TRUE(json.at("Devices").is_array());
    EXPECT_EQ(json.at("Devices").size(), 1U);
}

// Offsets and register contents are hex, matching how the profiles read.
TEST(ModbusToolSchema, TestDeviceFields)
{
    Dump dump;
    dump.devices.emplace_back(readDevice());

    auto json = toJson(dump);

    const auto& device = json.at("Devices").at(0);
    EXPECT_EQ(device.at("Name"), "PSU_1_1");
    EXPECT_EQ(device.at("Type"), "DeltaECD17020037PowerSupplyUnit");
    EXPECT_EQ(device.at("Address"), "0x90");
    EXPECT_EQ(device.at("SerialPort"), "ttyRS485-1");
    EXPECT_EQ(device.at("Result"), "Success");
    EXPECT_FALSE(device.contains("Reason"));
}

// Registers are grouped the way the profile groups them. A group the profile
// has none of is still written, as an empty array, so every device carries the
// same keys and a reader never has to check whether one is there.
TEST(ModbusToolSchema, TestRegisterGroupsArePresent)
{
    Dump dump;
    dump.devices.emplace_back(readDevice());

    auto json = toJson(dump);

    const auto& registers = json.at("Devices").at(0).at("Registers");
    for (const auto* group :
         {"Inventory", "Firmware", "Sensor", "Status", "Metric", "Config"})
    {
        ASSERT_TRUE(registers.contains(group)) << group;
        EXPECT_TRUE(registers.at(group).is_array()) << group;
    }
    EXPECT_TRUE(registers.at("Firmware").empty());
}

TEST(ModbusToolSchema, TestRegisterIsRaw)
{
    Dump dump;
    dump.devices.emplace_back(readDevice());

    auto json = toJson(dump);

    const auto& reg =
        json.at("Devices").at(0).at("Registers").at("Inventory").at(0);
    EXPECT_EQ(reg.at("Name"), "Model");
    EXPECT_EQ(reg.at("Offset"), "0x8");
    EXPECT_EQ(reg.at("Size"), 4);
    EXPECT_EQ(reg.at("ReadStatus"), "Success");
    EXPECT_EQ(reg.at("Raw"),
              nlohmann::ordered_json({"0x4543", "0x4431", "0x3730", "0x3230"}));
    // Only status registers describe bits.
    EXPECT_FALSE(reg.contains("Bits"));
}

// Status registers report the bits the profile models, and whether the word
// read has them set.
TEST(ModbusToolSchema, TestStatusBits)
{
    Dump dump;
    dump.devices.emplace_back(readDevice());

    auto dumped = toJson(dump);

    const auto& status =
        dumped.at("Devices").at(0).at("Registers").at("Status").at(0);
    EXPECT_EQ(status.at("Raw"), nlohmann::ordered_json({"0x0100"}));

    const auto& bits = status.at("Bits");
    ASSERT_EQ(bits.size(), 2U);
    EXPECT_EQ(bits.at(0).at("Position"), 0);
    EXPECT_EQ(bits.at(0).at("Name"), "AC_UNDER_VOLTAGE");
    EXPECT_EQ(bits.at(0).at("Type"), "SensorReadingCritical");
    EXPECT_EQ(bits.at(0).at("Asserted"), false);
    EXPECT_EQ(bits.at(1).at("Position"), 8);
    EXPECT_EQ(bits.at(1).at("Asserted"), true);
}

// A device that could not be read says why, and a register that failed still
// gets an entry, so what a device reports does not depend on what it answered
// for.
TEST(ModbusToolSchema, TestFailuresAreReported)
{
    Dump dump;
    DeviceDump device{
        .name = "PSU_1_9",
        .type = "DeltaECD17020037PowerSupplyUnit",
        .address = 0x98,
        .serialPort = "ttyRS485-1",
        .result = Result::failure,
        .reason = "No response",
    };
    device.registers.sensor.emplace_back(
        RegisterDump{.name = "INLET_SENSOR0_TEMP", .offset = 0x45, .size = 1});
    dump.devices.emplace_back(std::move(device));

    auto dumped = toJson(dump);

    const auto& json = dumped.at("Devices").at(0);
    EXPECT_EQ(json.at("Result"), "Failure");
    EXPECT_EQ(json.at("Reason"), "No response");

    const auto& reg = json.at("Registers").at("Sensor").at(0);
    EXPECT_EQ(reg.at("ReadStatus"), "Failure");
    EXPECT_TRUE(reg.at("Raw").empty());
}

// When none of a second sourced device's variants match, every one of them is
// reported, so the same name can appear more than once.
TEST(ModbusToolSchema, TestUnmatchedVariantsAreBothReported)
{
    Dump dump;
    for (const auto* type : {"DeltaECD17020037PowerSupplyUnit",
                             "Artesyn7000552480000PowerSupplyUnit"})
    {
        dump.devices.emplace_back(DeviceDump{
            .name = "PSU_1_1",
            .type = type,
            .result = Result::failure,
            .reason = "Probe value mismatch",
        });
    }

    auto json = toJson(dump);

    const auto& devices = json.at("Devices");
    ASSERT_EQ(devices.size(), 2U);
    EXPECT_EQ(devices.at(0).at("Name"), devices.at(1).at("Name"));
    EXPECT_NE(devices.at(0).at("Type"), devices.at(1).at("Type"));
}

// The dump flow, end to end, against the mock Modbus server over a socat
// pair. Entity Manager is not involved: dumpDevices is given the devices and
// a way to reach their port, which is what runDump would have looked up.
class DumpFlowTest;

using ConnectorIntf =
    sdbusplus::aserver::xyz::openbmc_project::object::Enable<DumpFlowTest>;
using InventoryIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::Item<>;

class DumpFlowTest : public BaseTest
{
  public:
    static constexpr auto clientPathPrefix = "/tmp/ttyModbusToolV0";
    static constexpr auto serverPathPrefix = "/tmp/ttyModbusToolV1";
    static constexpr auto portName = "TestPort0";
    static constexpr auto connectorPath =
        "/xyz/openbmc_project/inventory/system/connector/TestPort0";
    // Not the reported model: a probe compares the register with its nulls
    // removed, which leaves the trailing '0'.
    static constexpr auto probeValue = "RDF040DSS5190";

    DumpFlowTest() :
        BaseTest(clientPathPrefix, serverPathPrefix, modbus_tool::daemonService)
    {}

    void SetUp() override
    {
        // Stand in for the daemon's connector object, so reserving the port
        // has something to write Enabled on. It starts enabled, as the daemon
        // publishes it.
        connector = std::make_unique<ConnectorIntf>(
            ctx, connectorPath, ConnectorIntf::properties_t{.enabled = true});
        BaseTest::SetUp();
    }

    /** @brief A profile whose registers the mock server answers. */
    static auto testProfile(std::string expectedValue)
        -> ProfileIntf::DeviceProfile
    {
        return {
            .parity = ModbusIntf::Parity::none,
            .baudRate = baudRate,
            .probeRegister =
                {.offset = TestIntf::testReadHoldingRegisterModelOffset,
                 .size = TestIntf::testReadHoldingRegisterModelCount,
                 .expectedValue = std::move(expectedValue)},
            .inventoryRegisters =
                {{.type = ProfileIntf::InventoryDataType::model,
                  .offset = TestIntf::testReadHoldingRegisterModelOffset,
                  .size = TestIntf::testReadHoldingRegisterModelCount}},
            .sensorRegisters =
                {{.name = "INLET_TEMP",
                  .type = ProfileIntf::SensorType::temperature,
                  .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
                  .size = TestIntf::testReadHoldingRegisterTempCount}},
            .statusRegisters = {},
            .metricRegisters = {},
            .firmwareRegisters = {},
        };
    }

    static auto testConfig(const ProfileIntf::DeviceProfile& profile)
        -> ConfigIntf::Config
    {
        return {
            .name = "PSU_1_1",
            .type = "TestDevice",
            .address = TestIntf::testDeviceAddress,
            .serialPort = portName,
            .parentInventoryPath = {},
            .inventoryPath = {},
            .profile = profile,
            .pollRate = 1s,
        };
    }

    static auto testDevice(const ProfileIntf::DeviceProfile& profile)
        -> modbus_tool::DeviceVariants
    {
        return {.name = "PSU_1_1", .configs = {testConfig(profile)}};
    }

    /** @brief A profile whose probe register the mock server never answers,
     *  as an absent device would not. */
    static auto absentProfile() -> ProfileIntf::DeviceProfile
    {
        auto profile = testProfile(probeValue);
        profile.probeRegister.offset = TestIntf::testFailureReadHoldingRegister;
        profile.probeRegister.size = 1;
        return profile;
    }

    /** @brief Whether the port was left usable by the daemon. */
    auto portEnabled() const -> bool
    {
        return connector->enabled();
    }

    /** @brief Stands in for the entity-manager lookup, handing back the socat
     *  device instead of a real serial port. */
    auto portLookup() -> modbus_tool::PortLookup
    {
        return [this](sdbusplus::async::context&, const std::string& name)
                   -> sdbusplus::async::task<modbus_tool::PortDetails> {
            EXPECT_EQ(name, portName);
            modbus_tool::PortDetails port;
            port.config =
                std::make_unique<PortIntf::config::PortFactoryConfig>();
            port.config->name = portName;
            port.config->baudRate = baudRate;
            port.devicePath = clientDevicePath;
            co_return std::move(port);
        };
    }

    /** @brief A lookup that finds nothing, as an unconfigured port would. */
    static auto missingPortLookup() -> modbus_tool::PortLookup
    {
        return [](sdbusplus::async::context&, const std::string&)
                   -> sdbusplus::async::task<modbus_tool::PortDetails> {
            co_return modbus_tool::PortDetails{};
        };
    }

    auto run(const std::vector<modbus_tool::DeviceVariants>& devices,
             const modbus_tool::PortLookup& lookup) -> Dump
    {
        Dump dump;
        ctx.spawn(modbus_tool::dumpDevices(ctx, devices, lookup) |
                  sdbusplus::async::execution::then([&](Dump dumped) {
                      dump = std::move(dumped);
                      ctx.request_stop();
                  }));

        ctx.run();
        return dump;
    }

  private:
    sdbusplus::server::manager_t manager{ctx, InventoryIntf::namespace_path};
    std::unique_ptr<ConnectorIntf> connector;
};

// A device that answers its probe is read through, and the words reported are
// the ones the device returned.
TEST_F(DumpFlowTest, TestDeviceIsRead)
{
    auto profile = testProfile(probeValue);
    std::vector<modbus_tool::DeviceVariants> devices;
    devices.emplace_back(testDevice(profile));

    auto dump = run(devices, portLookup());

    ASSERT_EQ(dump.devices.size(), 1U);
    const auto& device = dump.devices.front();
    EXPECT_EQ(device.name, "PSU_1_1");
    EXPECT_EQ(device.result, Result::success);
    EXPECT_TRUE(device.reason.empty());

    ASSERT_EQ(device.registers.inventory.size(), 1U);
    // Names carry the device, as the daemon names what it publishes.
    EXPECT_EQ(device.registers.inventory.front().name, "PSU_1_1_Model");
    EXPECT_EQ(device.registers.inventory.front().raw,
              TestIntf::testReadHoldingRegisterModel);

    ASSERT_EQ(device.registers.sensor.size(), 1U);
    EXPECT_EQ(device.registers.sensor.front().name, "PSU_1_1_INLET_TEMP");
    EXPECT_TRUE(device.registers.sensor.front().read);
    EXPECT_EQ(device.registers.sensor.front().raw,
              TestIntf::testReadHoldingRegisterTempUnsigned);

    // The port is handed back once the dump is done.
    EXPECT_TRUE(portEnabled());
}

// A device that answers, but not with what the profile expects, is not the
// device the profile describes. Its probe register is reported even so, so the
// dump shows what it did answer with and why the profile did not match.
TEST_F(DumpFlowTest, TestProbeMismatchIsReported)
{
    auto profile = testProfile("SomeOtherModel");
    std::vector<modbus_tool::DeviceVariants> devices;
    devices.emplace_back(testDevice(profile));

    auto dump = run(devices, portLookup());

    ASSERT_EQ(dump.devices.size(), 1U);
    const auto& device = dump.devices.front();
    EXPECT_EQ(device.result, Result::failure);
    EXPECT_EQ(device.reason, "Probe value mismatch");

    ASSERT_EQ(device.registers.inventory.size(), 1U);
    EXPECT_EQ(device.registers.inventory.front().raw,
              TestIntf::testReadHoldingRegisterModel);
    // Nothing beyond the probe is read once the device is not identified.
    EXPECT_TRUE(device.registers.sensor.empty());
}

// A device that does not answer costs one read: nothing past the probe is
// attempted.
TEST_F(DumpFlowTest, TestSilentDeviceIsReported)
{
    auto profile = absentProfile();
    std::vector<modbus_tool::DeviceVariants> devices;
    devices.emplace_back(testDevice(profile));

    auto dump = run(devices, portLookup());

    ASSERT_EQ(dump.devices.size(), 1U);
    const auto& device = dump.devices.front();
    EXPECT_EQ(device.result, Result::failure);
    EXPECT_EQ(device.reason, "No response");
    EXPECT_TRUE(device.registers.inventory.empty());
}

// A port that cannot be reached fails its devices rather than the run.
TEST_F(DumpFlowTest, TestUnreachablePortIsReported)
{
    auto profile = testProfile(probeValue);
    std::vector<modbus_tool::DeviceVariants> devices;
    devices.emplace_back(testDevice(profile));

    auto dump = run(devices, missingPortLookup());

    ASSERT_EQ(dump.devices.size(), 1U);
    EXPECT_EQ(dump.devices.front().result, Result::failure);
    EXPECT_EQ(dump.devices.front().reason, "Port unavailable");
}

// A name with no configuration is reported, so asking for a device that does
// not exist says so rather than going missing from the dump.
TEST_F(DumpFlowTest, TestUnconfiguredDeviceIsReported)
{
    std::vector<modbus_tool::DeviceVariants> devices;
    devices.emplace_back(
        modbus_tool::DeviceVariants{.name = "PSU_9_9", .configs = {}});

    auto dump = run(devices, portLookup());

    ASSERT_EQ(dump.devices.size(), 1U);
    EXPECT_EQ(dump.devices.front().name, "PSU_9_9");
    EXPECT_EQ(dump.devices.front().result, Result::failure);
    EXPECT_EQ(dump.devices.front().reason, "Not configured");
}

// Only the variant that answers is kept, so a second sourced device that is
// present appears once.
TEST_F(DumpFlowTest, TestOnlyTheMatchingVariantIsKept)
{
    auto wrong = testProfile("SomeOtherModel");
    auto right = testProfile(probeValue);
    std::vector<modbus_tool::DeviceVariants> devices;
    devices.emplace_back(modbus_tool::DeviceVariants{
        .name = "PSU_1_1", .configs = {testConfig(wrong), testConfig(right)}});

    auto dump = run(devices, portLookup());

    ASSERT_EQ(dump.devices.size(), 1U);
    EXPECT_EQ(dump.devices.front().result, Result::success);
}
