# Mock Modbus Test Device

## mock-modbus-device

The `mock-modbus-device` daemon launches a simulated Modbus server on a
specified PTY port ID. This server listens for Modbus requests and returns
static data. At present, it only supports the ReadHoldingRegisters command, with
plans to add more command support in the future.

## start_mock_server.sh

The `start_mock_server.sh` script acts as a wrapper for `mock-modbus-device`. It
accepts a count parameter and starts a separate mocked Modbus server for each
PTY port. The script also configures the necessary environment variables to
enable client communication with the mocked servers. It utilizes socat to create
pseudo-terminals (PTYs) and initiates a `mock-modbus-device` instance for each
PTY.

This approach enables clients to interact with simulated Modbus devices,
facilitating testing across different scenarios without requiring physical
hardware. It is particularly suited for Qemu-based testing. To use this setup,
users must manually copy the required artifacts to the Qemu VM and execute the
script to start the mocked Modbus servers.
