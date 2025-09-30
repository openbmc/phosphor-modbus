#include "inventory_device.hpp"
#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"

#include <fcntl.h>

#include <sdbusplus/test/sdbus_mock.hpp>
#include <xyz/openbmc_project/Inventory/Source/Modbus/FRU/client.hpp>

#include <gmock/gmock.h>
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

struct SdbusInventoryMock : public sdbusplus::SdBusImpl
{
    SdbusInventoryMock() : sdbusplus::SdBusImpl()
    {
        sd_bus_open(&busp);
    }

    ~SdbusInventoryMock()
    {
        sd_bus_unref(busp);
    }

    MOCK_METHOD(int, sd_bus_call_async,
                (sd_bus*, sd_bus_slot**, sd_bus_message*,
                 sd_bus_message_handler_t, void*, uint64_t),
                (override));

    MOCK_METHOD(int, sd_watchdog_enabled, (int, uint64_t* usec), (override));

    sdbusplus::bus::busp_t busp = nullptr;
};

class InventoryTest : public ::testing::Test
{
  public:
    static constexpr PortConfigIntf::Config portConfig = {
        "TestPort1", PortConfigIntf::PortMode::rs485, 115200, 1};
    static constexpr const char* clientDevicePath =
        "/tmp/ttyInventoryTestPort0";
    static constexpr const char* serverDevicePath =
        "/tmp/ttyInventoryTestPort1";
    static constexpr const auto defaultBaudeRate = "b115200";
    static constexpr const auto deviceName = "Test1";
    int socat_pid = -1;
    sdbusplus::SdBusMock sdbusMock;
    sdbusplus::bus_t bus = sdbusplus::get_mocked_new(&sdbusMock);
    sdbusplus::async::context ctx;
    int fdClient = -1;
    std::unique_ptr<TestIntf::ServerTester> serverTester;
    int fdServer = -1;
    std::unique_ptr<MockPort> testPort;

    InventoryTest() : ctx(std::move(bus))
    {
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

        serverTester = std::make_unique<TestIntf::ServerTester>(ctx, fdServer);

        testPort =
            std::make_unique<MockPort>(ctx, portConfig, clientDevicePath);
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

    auto testInventorySourceCreation() -> sdbusplus::async::task<void>
    {
        InventoryConfigIntf::Config::port_address_map_t addressMap;
        addressMap[portConfig.name] = {{.start = TestIntf::testDeviceAddress,
                                        .end = TestIntf::testDeviceAddress}};
        InventoryConfigIntf::Config deviceConfig = {
            .name = deviceName,
            .addressMap = addressMap,
            .registers = {{"PartNumber",
                           TestIntf::testSuccessReadHoldingRegisterOffset,
                           TestIntf::testSuccessReadHoldingRegisterCount}},
            .parity = ModbusIntf::Parity::none,
            .baudRate = 115200};
        InventoryIntf::Device::serial_port_map_t ports;
        ports[portConfig.name] = std::move(testPort);

        auto inventoryDevice =
            std::make_unique<InventoryIntf::Device>(ctx, deviceConfig, ports);

        co_await inventoryDevice->addInventorySource(
            TestIntf::testDeviceAddress, portConfig.name,
            *ports[portConfig.name]);

        co_return;
    }

    void SetUp() override
    {
        ctx.spawn(serverTester->processRequests());
    }
};

TEST_F(InventoryTest, TestAddInventorySource)
{
    auto objPath =
        std::format("{}/{}_{}_{}", InventorySourceIntf::namespace_path,
                    deviceName, TestIntf::testDeviceAddress, portConfig.name);

    EXPECT_CALL(sdbusMock,
                sd_bus_emit_interfaces_added_strv(_, StrEq(objPath), _))
        .WillOnce([](sd_bus*, const char*, char** interfaces) {
            constexpr auto expectedInterfaceCount = 1;
            auto interfacesCount = 0;

            for (; interfaces[interfacesCount] != nullptr; ++interfacesCount)
            {
                EXPECT_EQ(std::string(interfaces[interfacesCount]),
                          InventorySourceIntf::interface)
                    << "Interface name mismatch";
            }
            EXPECT_EQ(interfacesCount, expectedInterfaceCount)
                << "Interface count mismatch";

            return 0;
        });

    EXPECT_CALL(sdbusMock,
                sd_bus_emit_interfaces_removed_strv(_, StrEq(objPath), _))
        .Times(1);

    ctx.spawn(testInventorySourceCreation());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
