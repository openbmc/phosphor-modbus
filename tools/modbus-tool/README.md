# modbus-tool

Reads a device's registers directly and writes them to JSON. Given a device name
it looks up the entity-manager configuration, resolves the serial port, loads
the device profile for its type, reserves the port, and reads every register the
profile defines.

The register contents are reported raw, so a dump shows exactly what the device
returned.

## Usage

```sh
modbus-tool dump --devices PSU_1_1
modbus-tool dump --devices PSU_1_1,PSU_1_2,BBU_1_1
modbus-tool dump --all --output dump.json
```

| Option            | Description                              |
| ----------------- | ---------------------------------------- |
| `--devices NAMES` | Comma separated list of devices to dump. |
| `--all`           | Every device in the allowlist.           |
| `--output FILE`   | Write the JSON here instead of stdout.   |
| `-y`, `--yes`     | Do not ask before pausing monitoring.    |

A dump pauses `xyz.openbmc_project.ModbusRTU` on the ports involved, so the tool
says so and waits for a yes first. `--yes` skips the prompt, and is required
when stdin is not a terminal so a script never blocks on it.

`--devices` and `--all` are mutually exclusive, and one of them is required.

A device name is its entity-manager `Name`, for example `PSU_1_1`, matching the
names in `allowed-devices.json`. Spaces are replaced with underscores, as they
are in the allowlist.

`--all` reads the allowlist from `allowed-devices.json`. If no allowlist is
configured, or it is empty, name the devices with `--devices` instead.

### Exit codes

| Code | Meaning                                             |
| ---- | --------------------------------------------------- |
| 0    | A dump was produced. Check `Result` on each device. |
| 1    | Nothing could be dumped.                            |

The exit code only says whether there is output to read. Anything that stops one
device being read, such as a port held by another client or a name that is not
configured, is reported as that device's `Result` and `Reason`, so the rest of
the dump survives. Exit 1 is for the cases that leave nothing at all: no
allowlist to expand, none of the named devices could be attempted, or the lock
could not be acquired.

Only one instance runs at a time, held by an exclusive `flock` on
`/run/lock/phosphor-modbus.lock`. The reservation alone cannot tell two
invocations apart, because a port one of them holds already reads as disabled to
the other, which would then release a reservation it does not own. The kernel
drops the lock when the process ends, so it cannot go stale.

JSON goes to stdout, so it can be redirected or piped on its own. Everything
else goes to stderr. Exit 1 says why there is no dump:

```text
Another modbus-tool is already running
No allowed-devices list configured; name devices with --devices
```

A dump that was produced still reports what went wrong in it, alongside exit 0,
so that a failure is visible without reading the JSON:

```text
PSU_1_9: Not configured in entity-manager
ttyRS485-2: Reserved by another client
PSU_1_4: 12 of 17 registers failed
3 of 24 devices did not respond
```

Messages summarise; the per-register detail is in the JSON. A dump with nothing
to report prints nothing to stderr.

## Port reservation

Reading directly would collide with the daemon's polling, so the tool reserves
the port first by writing `Enabled` false on
`/xyz/openbmc_project/inventory/system/connector/<PortName>`.

The write does not mean the port is free, only that it is reserved, so the tool
waits for `Enabled` to read false before it transmits. If it stays true the port
is held by another client and its devices are reported as failures.

Reservation is per port, not per device, so every device on that bus stops being
polled for the duration and its sensors read as unavailable. `--all` therefore
reserves each port once and dumps every device on it before moving on. The
reservation is released on exit, including on `SIGINT` and `SIGTERM`. If the
tool is killed outright the port stays reserved; re-enable it with:

```sh
busctl set-property xyz.openbmc_project.ModbusRTU \
  /xyz/openbmc_project/inventory/system/connector/ttyRS485_1 \
  xyz.openbmc_project.Object.Enable Enabled b true
```

## Output

Register contents are reported raw. The tool does no scaling, sign handling or
string assembly, so the output cannot disagree with the profile - decoding is
left to the consumer, which needs the profile's `Format`, `Precision`,
`IsSigned` and `Scale`. Status registers are the one exception: the profile's
bit definitions are copied through with an `Asserted` flag, since that is a bit
test rather than an interpretation.

```json
{
  "Metadata": {
    "SchemaVersion": "1.0.0",
    "Tool": "modbus-tool",
    "BMCVersion": "ventura2-v2026.33.0",
    "Timestamp": "2026-08-31T17:42:11Z"
  },
  "Devices": [
    {
      "Name": "PSU_1_1",
      "Type": "DeltaECD17020037PowerSupplyUnit",
      "Address": "0x90",
      "SerialPort": "ttyRS485-1",
      "Result": "Success",
      "Registers": {
        "Inventory": [
          {
            "Name": "Model",
            "Offset": "0x8",
            "Size": 8,
            "ReadStatus": "Success",
            "Raw": ["0x4543", "0x4431", "0x3730", "0x3230"]
          }
        ],
        "Firmware": [
          {
            "Name": "PSU_FW_Revision",
            "Offset": "0x30",
            "Size": 4,
            "ReadStatus": "Success",
            "Raw": ["0x0100", "0x0000", "0x0000", "0x0000"]
          }
        ],
        "Sensor": [
          {
            "Name": "INLET_SENSOR0_TEMP",
            "Offset": "0x45",
            "Size": 1,
            "ReadStatus": "Success",
            "Raw": ["0x01F4"]
          }
        ],
        "Status": [
          {
            "Name": "PFC_ALARM",
            "Offset": "0x3D",
            "Size": 1,
            "ReadStatus": "Success",
            "Raw": ["0x0100"],
            "Bits": [
              {
                "Position": 0,
                "Name": "AC_UNDER_VOLTAGE",
                "Type": "SensorReadingCritical",
                "Asserted": false
              },
              {
                "Position": 8,
                "Name": "AC_NOT_OK",
                "Type": "PowerFault",
                "Asserted": true
              }
            ]
          }
        ],
        "Metric": [],
        "Config": [
          {
            "Name": "UnixTime",
            "Offset": "0x5A",
            "Size": 2,
            "ReadStatus": "Success",
            "Raw": ["0x68B1", "0x2C40"]
          }
        ]
      }
    }
  ]
}
```

### Metadata

- `SchemaVersion` is the version of this format, described under Versioning
  below.
- `Tool` names the program that produced the dump.
- `BMCVersion` is `VERSION_ID` from `/etc/os-release`, which already carries the
  machine name, for example `ventura2-v2026.33.0`. It reads `unknown` if the
  file or key is missing.
- `Timestamp` is when the dump was taken, ISO 8601 in UTC. Note that sensor
  registers change between reads, so two dumps of the same device differ whether
  or not this field is present.

### Devices

Always an array, even for a single device, so both modes produce the same shape.
`Result` is per device, so one unreachable device does not hide the others:

| Result    | Meaning                                         |
| --------- | ----------------------------------------------- |
| `Success` | Every register read.                            |
| `Partial` | Some registers failed. `ReadStatus` says which. |
| `Failure` | Nothing was read. `Reason` says why.            |

`Reason` is present only on `Failure`, and is one of:

| Reason                 | Meaning                                      |
| ---------------------- | -------------------------------------------- |
| `No response`          | The device did not answer the probe.         |
| `Probe value mismatch` | It answered, but not as this variant.        |
| `Port unavailable`     | The port was held by another client.         |
| `Not configured`       | The name is not in entity-manager.           |
| `No profile`           | No profile is installed for the device type. |

A device is identified by reading the profile's probe register and comparing it
against the expected value, so an absent device costs one read rather than a
timeout on every span.

Devices that are second sourced carry a configuration for each variant on the
same entity-manager object, and only one of them is really present. When a
variant is identified the others are dropped, and the dump holds one entry. If
none of them match, every variant is reported, so **`Name` is not unique within
`Devices`** and a consumer should key on `Name` and `Type` together, or simply
iterate. Only failed entries are ever duplicated.

Since the probe register is also an inventory register, a device that answers
with an unexpected value still reports that register, which shows what it
actually returned against what each variant expected. A device that does not
answer at all reports no registers.

### Registers

Grouped the same way the device profile groups them, so a profile and a dump can
be read side by side. Every entry carries:

| Field        | Description                                          |
| ------------ | ---------------------------------------------------- |
| `Name`       | Register name.                                       |
| `Offset`     | Register offset, hex. Unique within a device.        |
| `Size`       | Length in 16-bit registers.                          |
| `ReadStatus` | `Success` or `Failure`.                              |
| `Raw`        | `Size` register values, hex, most significant first. |

Registers that failed to read keep their entry with `ReadStatus` `Failure` and
an empty `Raw`, so the set of keys does not depend on which reads succeeded.

`Name` comes from the profile's `Name` where it has one. Inventory registers and
the `UnixTime` config register are identified by `Type` in the profile instead,
and that value is used as the name here. `Offset` identifies a register
unambiguously in all cases.

Status registers carry an additional `Bits` array holding only the positions the
profile defines, each with its `Position`, `Name`, `Type` and whether it is
`Asserted`. Positions the profile does not model are absent, so compare against
`Raw` to find bits the profile is missing.

### Versioning

`SchemaVersion` is `major.minor.patch`. Adding a field is a minor bump; renaming
or removing one, or changing what a field means, is a major bump. Consumers
should pin the major version.
