# Testing

## Mocked Modbus Testing

The `socat` command is utilized to create pseudo-terminals (PTYs), enabling the
setup of virtual serial ports for communication between a Modbus client and
server. In this testing framework, the Modbus client (implemented as a test
client using gtest) sends Modbus commands to a test server, which processes
these requests and returns responses. The test environment setup is responsible
for configuring the pseudo-terminals and launching the test server instance. The
server logic resides in `modbus_server_tester.cpp::ServerTester`, with further
details provided in the following sections.

### ServerTester

`ServerTester` acts as a mock Modbus server, intercepting Modbus messages and
generating appropriate responses. The replies are determined by the mocked
Modbus addresses, allowing targeted code paths to be exercised on the client
side. `ServerTester` is capable of handling both single and segmented response
scenarios, and also provides error responses to facilitate negative testing. The
test server currently handles the following Modbus command:

- ReadHoldingRegisters
