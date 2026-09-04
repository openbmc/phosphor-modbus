#pragma once

#include <sdbusplus/async.hpp>

#include <string>

namespace modbus_tool
{

/** @brief The service whose monitoring a reservation pauses. */
inline constexpr auto daemonService = "xyz.openbmc_project.ModbusRTU";

/** @brief Holds a serial port reserved so the daemon stops using the bus.
 *
 *  Writing Enabled false reserves the port, but the property only clears once
 *  operations already in progress have finished, so reserve() waits for it to
 *  read false before reporting success. */
class PortReservation
{
  public:
    explicit PortReservation(std::string portName);
    PortReservation(const PortReservation&) = delete;
    PortReservation& operator=(const PortReservation&) = delete;
    PortReservation(PortReservation&&) = delete;
    PortReservation& operator=(PortReservation&&) = delete;
    ~PortReservation();

    /** @brief Reserve the port and wait for the bus to go quiet.
     *  @return False if the port is held by another client. */
    auto reserve(sdbusplus::async::context& ctx)
        -> sdbusplus::async::task<bool>;

    /** @brief Release the port. Does nothing if it is not held. */
    auto release(sdbusplus::async::context& ctx)
        -> sdbusplus::async::task<void>;

    /** @brief Release over a connection of its own, for giving the port back
     *  once the async context has stopped. The destructor falls back to this
     *  so an interrupted run does not leave the port reserved. */
    auto releaseBlocking() -> void;

  private:
    std::string portName;
    std::string objectPath;
    bool held = false;
};

} // namespace modbus_tool
