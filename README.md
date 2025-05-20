# phosphor-modbus

phosphor-modbus provides a set of software applications to query sensors info,
inventory data, firmware info of various modbus devices.

## Dependencies

The phosphor-modbus requires phosphor-dbus-interfaces, sdbusplus and
phosphor-logging.

## Building

The phosphor-modbus is built using meson.

```sh
meson build && ninja -C build
```
