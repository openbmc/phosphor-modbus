#include "modbus/modbus.hpp"
#include "modbus_server_tester.hpp"
#include "test_base.hpp"

#include <gtest/gtest.h>

using namespace std::literals;

namespace RTUIntf = phosphor::modbus::rtu;
using ModbusIntf = RTUIntf::Modbus;

class ModbusTest : public BaseTest
{
  public:
    static constexpr auto clientDevicePath = "/tmp/ttyV0";
    static constexpr auto serverDevicePath = "/tmp/ttyV1";
    static constexpr auto serviceName = "xyz.openbmc_project.TestModbus";
    std::unique_ptr<ModbusIntf> modbus;

    ModbusTest() : BaseTest(clientDevicePath, serverDevicePath, serviceName)
    {
        modbus = std::make_unique<ModbusIntf>(ctx, fdClient, 115200, 0);
    }

    auto TestHoldingRegisters(uint16_t registerOffset, bool res)
        -> sdbusplus::async::task<void>
    {
        std::cout << "TestHoldingRegisters() start" << std::endl;

        std::vector<uint16_t> registers(
            TestIntf::testSuccessReadHoldingRegisterCount);

        auto ret = co_await modbus->readHoldingRegisters(
            TestIntf::testDeviceAddress, registerOffset, registers);

        EXPECT_EQ(ret, res) << "Failed to read holding registers";

        if (!res)
        {
            co_return;
        }

        for (auto i = 0; i < TestIntf::testSuccessReadHoldingRegisterCount; i++)
        {
            EXPECT_EQ(registers[i],
                      TestIntf::testSuccessReadHoldingRegisterResponse[i]);
        }

        co_return;
    }
};

TEST_F(ModbusTest, TestReadHoldingRegisterSuccess)
{
    ctx.spawn(TestHoldingRegisters(
        TestIntf::testSuccessReadHoldingRegisterOffset, true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestReadHoldingRegisterSegmentedSuccess)
{
    ctx.spawn(TestHoldingRegisters(
        TestIntf::testSuccessReadHoldingRegisterSegmentedOffset, true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestReadHoldingRegisterFailure)
{
    ctx.spawn(
        TestHoldingRegisters(TestIntf::testFailureReadHoldingRegister, false));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
