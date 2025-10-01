#include "modbus/modbus_commands.hpp"
#include "modbus/modbus_exception.hpp"

#include <cstring>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace RTUIntf = phosphor::modbus::rtu;

class ReadHoldingRegistersResponseTest :
    public RTUIntf::ReadHoldingRegistersResponse
{
  public:
    ReadHoldingRegistersResponseTest(uint8_t address,
                                     std::vector<uint16_t>& registers) :
        RTUIntf::ReadHoldingRegistersResponse(address, registers)
    {}

    template <std::size_t Length>
    ReadHoldingRegistersResponseTest& operator=(
        const std::array<uint8_t, Length>& expectedMessage)
    {
        len = Length;
        std::copy(expectedMessage.begin(), expectedMessage.end(), raw.begin());
        return *this;
    }
};

class ModbusCommandTest : public ::testing::Test
{
  public:
    ModbusCommandTest() = default;

    static constexpr auto failureMessageLength = 11;

    template <typename ExceptionType, std::size_t Length>
    void TestReadHoldingRegisterResponseFailure(
        const std::array<uint8_t, Length>& expectedMessage)
    {
        constexpr uint8_t expectedAddress = 0xa;
        constexpr size_t expectedRegisterSize = 3;
        std::vector<uint16_t> registers(expectedRegisterSize);
        ReadHoldingRegistersResponseTest response(expectedAddress, registers);
        response = expectedMessage;

        EXPECT_THROW(response.decode(), ExceptionType);
    }
};

TEST_F(ModbusCommandTest, TestReadHoldingRegistersRequestSucess)
{
    constexpr size_t expectedLength = 8;
    constexpr uint8_t expectedAddress = 0x1;
    constexpr std::array<uint8_t, expectedLength> expectedMessage = {
        expectedAddress, // addr(1) = 0x01
        0x03,            // func(1) = 0x03
        0x12,            // reg_off(2) = 0x1234
        0x34,
        0x00,            // reg_cnt(2) = 0x0020,
        0x20,
        0x00,            // crc(2) (Pre-computed) = 0xa4
        0xa4};

    RTUIntf::ReadHoldingRegistersRequest request(expectedAddress, 0x1234, 0x20);
    request.encode();

    std::array<uint8_t, expectedLength> actualMessage;
    std::memcpy(actualMessage.data(), request.raw.data(), expectedLength);

    EXPECT_EQ(actualMessage, expectedMessage);
    EXPECT_EQ(request.address, expectedAddress);
}

TEST_F(ModbusCommandTest, TestReadHoldingRegistersResponseSucess)
{
    constexpr size_t expectedLength = 11;
    constexpr uint8_t expectedAddress = 0xa;
    constexpr size_t expectedRegisterSize = 3;
    constexpr std::array<uint8_t, expectedLength> expectedMessage = {
        expectedAddress, // addr(1) = 0x0a
        0x03,            // func(1) = 0x03
        0x06,            // bytes(1) = 0x06
        0x11,            // regs(3*2) = 0x1122, 0x3344, 0x5566
        0x22,
        0x33,
        0x44,
        0x55,
        0x66,
        0x59, // crc(2) = 0x5928 (pre-computed)
        0x28};
    const std::vector<uint16_t> expectedRegisters{0x1122, 0x3344, 0x5566};
    std::vector<uint16_t> registers(expectedRegisterSize);

    RTUIntf::ReadHoldingRegistersResponse response(expectedAddress, registers);
    EXPECT_EQ(response.len, expectedLength);

    std::copy(expectedMessage.begin(), expectedMessage.end(),
              response.raw.begin());

    EXPECT_EQ(response.len, expectedLength);
    response.decode();
    EXPECT_EQ(registers, expectedRegisters);
}

TEST_F(ModbusCommandTest, TestReadHoldingRegistersResponseError)
{
    constexpr std::array<uint8_t, 5> expectedMessage = {
        0xa,  // addr(1) = 0x0a
        0x83, // func(1) = 0x83
        0x03, // exception code(1) = 0x03
        0x70, // crc(2) = 0x70f3 (pre-computed)
        0xf3};

    TestReadHoldingRegisterResponseFailure<RTUIntf::ModbusException>(
        expectedMessage);
}

TEST_F(ModbusCommandTest, TestReadHoldingRegistersResponseBadAddress)
{
    constexpr std::array<uint8_t, failureMessageLength> expectedMessage = {
        0x1,  // Bad address(1), should be 0x0a
        0x03, // func(1) = 0x03
        0x06, // bytes(1) = 0x06
        0x11, // regs(3*2) = 0x1122, 0x3344, 0x5566
        0x22, 0x33, 0x44, 0x55, 0x66,
        0x2a, // crc(2) = 0x2a18 (pre-computed)
        0x18};

    TestReadHoldingRegisterResponseFailure<RTUIntf::ModbusBadResponseException>(
        expectedMessage);
}

TEST_F(ModbusCommandTest, TestReadHoldingRegistersResponseBadCRC)
{
    constexpr std::array<uint8_t, failureMessageLength> expectedMessage = {
        0xa,  // addr(1) = 0x0a
        0x03, // func(1) = 0x03
        0x06, // bytes(1) = 0x06
        0x11, // regs(3*2) = 0x1122, 0x3344, 0x5566
        0x22, 0x33, 0x44, 0x55, 0x66,
        0x59, // Bad crc(2), should be 0x5928
        0x29};

    TestReadHoldingRegisterResponseFailure<RTUIntf::ModbusCRCException>(
        expectedMessage);
}

TEST_F(ModbusCommandTest, TestReadHoldingRegistersResponseBadFunctionCode)
{
    constexpr std::array<uint8_t, failureMessageLength> expectedMessage = {
        0xa,  // addr(1) = 0x0a
        0x04, // Bad function code(1), should be 0x03
        0x06, // bytes(1) = 0x06
        0x11, // regs(3*2) = 0x1122, 0x3344, 0x5566
        0x22, 0x33, 0x44, 0x55, 0x66,
        0x18, // crc(2) = 0x5928 (pre-computed)
        0xce};

    TestReadHoldingRegisterResponseFailure<RTUIntf::ModbusBadResponseException>(
        expectedMessage);
}

TEST_F(ModbusCommandTest, TestReadHoldingRegistersResponseBadByteCount)
{
    constexpr std::array<uint8_t, failureMessageLength> expectedMessage = {
        0xa,  // addr(1) = 0x0a
        0x03, // func(1) = 0x03
        0x04, // Bad bytes(1), should be 0x06
        0x11, // regs(3*2) = 0x1122, 0x3344, 0x5566
        0x22, 0x33, 0x44, 0x55, 0x66,
        0x7a, // crc(2) = 0x7ae8 (pre-computed)
        0xe8};

    TestReadHoldingRegisterResponseFailure<RTUIntf::ModbusBadResponseException>(
        expectedMessage);
}
