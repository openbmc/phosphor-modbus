#include "inventory/modbus_inventory.hpp"
#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"

#include <fcntl.h>

#include <xyz/openbmc_project/Inventory/Source/Modbus/FRU/client.hpp>

#include <gtest/gtest.h>

using namespace std::literals;
using namespace testing;
using InventorySourceIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::source::modbus::FRU<>;

namespace TestIntf = phosphor::modbus::test;
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

class InventoryTest : public ::testing::Test
{
  public:
    PortConfigIntf::Config portConfig;
    static constexpr const char* clientDevicePath =
        "/tmp/ttyInventoryTestPort0";
    static constexpr const char* serverDevicePath =
        "/tmp/ttyInventoryTestPort1";
    static constexpr const auto defaultBaudeRate = "b115200";
    static constexpr const auto deviceName = "Test1";
    static constexpr auto serviceName = "xyz.openbmc_project.TestModbusRTU";
    int socat_pid = -1;
    sdbusplus::async::context ctx;
    int fdClient = -1;
    std::unique_ptr<TestIntf::ServerTester> serverTester;
    int fdServer = -1;

    InventoryTest()
    {
        portConfig.name = "TestPort1";
        portConfig.portMode = PortConfigIntf::PortMode::rs485;
        portConfig.baudRate = 115200;
        portConfig.rtsDelay = 1;

        std::string socatCmd = std::format(
            "socat -x -v -d -d pty,link={},rawer,echo=0,parenb,{} pty,link={},rawer,echo=0,parenb,{} & echo $!",
            serverDevicePath, defaultBaudeRate, clientDevicePath,
            defaultBaudeRate);

        // Start socat in the background and capture its PID
        FILE* fp = popen(socatCmd.c_str(), "r");
        EXPECT_NE(fp, nullptr) << "Failed to start socat: " << strerror(errno);
        EXPECT_GT(fscanf(fp, "%d", &socat_pid), 0);
        pclose(fp);

        // Wait for socat to start up
        sleep(1);

        fdClient = open(clientDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(fdClient, -1)
            << "Failed to open serial port " << clientDevicePath
            << " with error: " << strerror(errno);

        fdServer = open(serverDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(fdServer, -1)
            << "Failed to open serial port " << serverDevicePath
            << " with error: " << strerror(errno);

        ctx.request_name(serviceName);

        serverTester = std::make_unique<TestIntf::ServerTester>(ctx, fdServer);
    }

    ~InventoryTest() noexcept override
    {
        if (fdClient != -1)
        {
            close(fdClient);
            fdClient = -1;
        }
        if (fdServer != -1)
        {
            close(fdServer);
            fdServer = -1;
        }
        kill(socat_pid, SIGTERM);
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

    void SetUp() override
    {
        // Process request for probe device call
        ctx.spawn(serverTester->processRequests());

        // Process request to read `Model` holding register call
        ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
                  sdbusplus::async::execution::then([&]() {
                      ctx.spawn(serverTester->processRequests());
                  }));
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
