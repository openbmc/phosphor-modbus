#include "inventory/modbus_inventory.hpp"
#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"
#include "test_base.hpp"

#include <xyz/openbmc_project/Inventory/Source/Modbus/FRU/client.hpp>

#include <gtest/gtest.h>

using namespace std::literals;
using namespace testing;
using InventorySourceIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::source::modbus::FRU<>;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace PortConfigIntf = PortIntf::config;
namespace InventoryIntf = phosphor::modbus::rtu::inventory;
namespace InventoryConfigIntf = InventoryIntf::config;

class MockPort : public PortIntf::BasePort
{
  public:
    MockPort(sdbusplus::async::context& ctx,
             const PortConfigIntf::Config& config,
             const std::string& devicePath) : BasePort(ctx, config, devicePath)
    {}
};

class InventoryTest : public BaseTest
{
  public:
    static constexpr const char* clientDevicePath =
        "/tmp/ttyInventoryTestPort0";
    static constexpr const char* serverDevicePath =
        "/tmp/ttyInventoryTestPort1";
    static constexpr const auto deviceName = "Test1";
    static constexpr auto serviceName = "xyz.openbmc_project.TestModbusRTU";
    PortConfigIntf::Config portConfig;

    InventoryTest() : BaseTest(clientDevicePath, serverDevicePath, serviceName)
    {
        portConfig.name = "TestPort1";
        portConfig.portMode = PortConfigIntf::PortMode::rs485;
        portConfig.baudRate = 115200;
        portConfig.rtsDelay = 1;
    }

    auto testInventorySourceCreation(std::string objPath)
        -> sdbusplus::async::task<void>
    {
        InventoryConfigIntf::Config::port_address_map_t addressMap;
        addressMap[portConfig.name] = {{.start = TestIntf::testDeviceAddress,
                                        .end = TestIntf::testDeviceAddress}};
        InventoryConfigIntf::Config deviceConfig = {
            .name = deviceName,
            .addressMap = addressMap,
            .registers = {{"Model",
                           TestIntf::testReadHoldingRegisterModelOffset,
                           TestIntf::testReadHoldingRegisterModelCount}},
            .parity = ModbusIntf::Parity::none,
            .baudRate = 115200};
        InventoryIntf::Device::serial_port_map_t ports;
        ports[portConfig.name] =
            std::make_unique<MockPort>(ctx, portConfig, clientDevicePath);

        auto inventoryDevice =
            std::make_unique<InventoryIntf::Device>(ctx, deviceConfig, ports);

        co_await inventoryDevice->probePorts();

        // Create InventorySource client interface to read back D-Bus properties
        auto properties = co_await InventorySourceIntf(ctx)
                              .service(serviceName)
                              .path(objPath)
                              .properties();

        constexpr auto defaultInventoryValue = "Unknown";

        EXPECT_EQ(properties.name,
                  std::format("{} {} {}", deviceName,
                              TestIntf::testDeviceAddress, portConfig.name))
            << "Name mismatch";
        EXPECT_EQ(properties.address, TestIntf::testDeviceAddress)
            << "Address mismatch";
        EXPECT_EQ(properties.link_tty, portConfig.name) << "Link TTY mismatch";
        EXPECT_EQ(properties.model, TestIntf::testReadHoldingRegisterModelStr)
            << "Model mismatch";
        EXPECT_EQ(properties.serial_number, defaultInventoryValue)
            << "Part Number mismatch";

        co_return;
    }
};

TEST_F(InventoryTest, TestAddInventorySource)
{
    auto objPath =
        std::format("{}/{}_{}_{}", InventorySourceIntf::namespace_path,
                    deviceName, TestIntf::testDeviceAddress, portConfig.name);

    ctx.spawn(testInventorySourceCreation(objPath));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
