#include "modbus/modbus.hpp"
#include "modbus_server_tester.hpp"
#include "test_base.hpp"

#include <array>

#include <gtest/gtest.h>

using namespace std::literals;

namespace RTUIntf = phosphor::modbus::rtu;
using ModbusIntf = RTUIntf::Modbus;

class ModbusTest : public BaseTest
{
  public:
    static constexpr auto clientPathPrefix = "/tmp/ttyV0";
    static constexpr auto serverPathPrefix = "/tmp/ttyV1";
    static constexpr auto serviceName = "xyz.openbmc_project.TestModbus";
    std::unique_ptr<ModbusIntf> modbus;

    ModbusTest() : BaseTest(clientPathPrefix, serverPathPrefix, serviceName)
    {
        modbus = std::make_unique<ModbusIntf>(
            ctx, fdClient, 115200, 0, std::chrono::microseconds(300000));
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

    /** @brief Read one group of records from a file. */
    auto TestReadFileRecord(uint16_t fileNumber, uint16_t recordNumber,
                            uint16_t recordCount, bool res)
        -> sdbusplus::async::task<void>
    {
        std::cout << "TestReadFileRecord() start" << std::endl;

        std::vector<uint16_t> data(recordCount);
        std::array<RTUIntf::FileRecord, 1> records{
            {{fileNumber, recordNumber, data}}};

        auto ret = co_await modbus->readFileRecord(TestIntf::testDeviceAddress,
                                                   records);

        EXPECT_EQ(ret, res) << "Failed to read file record";

        if (!res)
        {
            co_return;
        }

        // A read returns the records it asked for.
        for (auto i = 0; i < recordCount; i++)
        {
            EXPECT_EQ(data[i], TestIntf::testFileRecord[recordNumber + i]);
        }

        co_return;
    }

    /** @brief Read two non-contiguous groups of records in a single
     *  request, each carried as its own sub request. */
    auto TestReadFileRecords() -> sdbusplus::async::task<void>
    {
        std::cout << "TestReadFileRecords() start" << std::endl;

        std::vector<uint16_t> first(2);
        std::vector<uint16_t> second(3);
        std::array<RTUIntf::FileRecord, 2> records{
            {{TestIntf::testFileNumber, 0, first},
             {TestIntf::testFileNumber, 5, second}}};

        auto ret = co_await modbus->readFileRecord(TestIntf::testDeviceAddress,
                                                   records);

        EXPECT_TRUE(ret) << "Failed to read file records";
        if (!ret)
        {
            co_return;
        }

        for (size_t i = 0; i < first.size(); i++)
        {
            EXPECT_EQ(first[i], TestIntf::testFileRecord[i]);
        }
        for (size_t i = 0; i < second.size(); i++)
        {
            EXPECT_EQ(second[i], TestIntf::testFileRecord[5 + i]);
        }

        co_return;
    }

    auto TestWriteSingleRegister(uint16_t registerOffset, bool res)
        -> sdbusplus::async::task<void>
    {
        std::cout << "TestWriteSingleRegister() start" << std::endl;

        auto ret = co_await modbus->writeSingleRegister(
            TestIntf::testDeviceAddress, registerOffset,
            TestIntf::testWriteSingleRegisterValue);

        EXPECT_EQ(ret, res) << "Failed to write single register";

        co_return;
    }

    auto TestWriteMultipleRegisters(uint16_t registerOffset, bool res)
        -> sdbusplus::async::task<void>
    {
        std::cout << "TestWriteMultipleRegisters() start" << std::endl;

        auto ret = co_await modbus->writeMultipleRegisters(
            TestIntf::testDeviceAddress, registerOffset,
            TestIntf::testWriteMultipleRegistersData);

        EXPECT_EQ(ret, res) << "Failed to write multiple registers";

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

TEST_F(ModbusTest, TestReadFileRecordSuccess)
{
    ctx.spawn(TestReadFileRecord(TestIntf::testFileNumber, 0,
                                 TestIntf::testFileRecord.size(), true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// A file longer than one response is read as several, so a read has to be
// able to start part way in.
TEST_F(ModbusTest, TestReadFileRecordFromAnOffset)
{
    ctx.spawn(TestReadFileRecord(TestIntf::testFileNumber, 2, 3, true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestReadFileRecordsInOneRequest)
{
    ctx.spawn(TestReadFileRecords());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestReadFileRecordFailure)
{
    ctx.spawn(TestReadFileRecord(TestIntf::testFailureFileNumber, 0, 1, false));

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

TEST_F(ModbusTest, TestReadHoldingRegisterIllegalDataAddress)
{
    ctx.spawn(
        TestHoldingRegisters(TestIntf::testIllegalDataAddressRegister, false));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestWriteSingleRegisterSuccess)
{
    ctx.spawn(TestWriteSingleRegister(
        TestIntf::testSuccessWriteSingleRegisterOffset, true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestWriteSingleRegisterFailure)
{
    ctx.spawn(TestWriteSingleRegister(
        TestIntf::testFailureWriteSingleRegisterOffset, false));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestWriteMultipleRegistersSuccess)
{
    ctx.spawn(TestWriteMultipleRegisters(
        TestIntf::testSuccessWriteMultipleRegistersOffset, true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestWriteMultipleRegistersFailure)
{
    ctx.spawn(TestWriteMultipleRegisters(
        TestIntf::testFailureWriteMultipleRegistersOffset, false));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ModbusTest, TestWriteMultipleRegistersFlaky)
{
    // First attempt fails, retry succeeds
    ctx.spawn(TestWriteMultipleRegisters(
        TestIntf::testFlakyWriteMultipleRegistersOffset, true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
