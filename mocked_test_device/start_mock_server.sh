#!/bin/bash

set -e
set -u
set -o pipefail

function cleanup() {
    echo -e "\nCaught EXIT signal. Terminating background processes."
    jobs -p | xargs -r kill 2>/dev/null
    echo "Cleanup complete."
}
trap cleanup EXIT

DEFAULT_COUNT=2
MODBUS_SERVER="/usr/libexec/phosphor-modbus/mock-modbus-server"

if [ "$#" -eq 0 ]; then
    echo "No count provided. Starting with default count of $DEFAULT_COUNT servers and serial port pairs."
    SERVER_COUNT=$DEFAULT_COUNT
elif [ "$#" -eq 1 ]; then
    SERVER_COUNT=$1
    if ! [[ "$SERVER_COUNT" =~ ^[0-9]+$ ]]; then
        echo "Error: Count must be a non-negative integer." >&2
        exit 1
    fi
    echo "Starting $SERVER_COUNT $MODBUS_SERVER instances and virtual port pairs."
else
    echo "Error: Too many arguments." >&2
    echo "Usage: $0 [<count>]" >&2
    exit 1
fi

# Create the necessary directory structure for serial ports
echo "Creating directory /dev/serial/by-path"
mkdir -p /dev/serial/by-path

# Remove old symlinks to prevent conflicts
echo "Removing old symlinks from /dev/serial/by-path/..."
rm -f /dev/serial/by-path/platform-1e6a1000.usb-usb-0:1:1.*-port0

# Loop to create virtual serial port pairs and start modbus servers
echo "Starting $SERVER_COUNT virtual serial port pairs and $MODBUS_SERVER instances."
for (( i=0; i<SERVER_COUNT; i++ )); do
    TTY_USB="/dev/ttyUSB$i"
    TTY_V="/dev/ttyV$i"

    # Start the socat process for this pair
    echo "  - Starting socat for $TTY_USB and $TTY_V."
    socat -v -x -d -d pty,link="$TTY_USB",rawer,echo=0,b115200 pty,rawer,echo=0,link="$TTY_V",b115200 &

    # Wait a moment for socat to create the devices
    sleep 0.5

    echo "  - Creating symlink for $TTY_USB..."
    ln -sf $TTY_USB "/dev/serial/by-path/platform-1e6a1000.usb-usb-0:1:1.$i-port0"

    # Start the $MODBUS_SERVER instance
    echo "  - Starting $MODBUS_SERVER instance $i."
    $MODBUS_SERVER $i &
done

echo "All background processes have been started."
echo "Press Ctrl+C to terminate all processes gracefully."

# Keep the script running to manage background jobs
wait
