# modbus-tool

Reads a device's registers directly and writes them to JSON. Given a device name
it looks up the entity-manager configuration, resolves the serial port, loads
the device profile for its type, reserves the port, and reads every register the
profile defines.

Each register is reported both raw, exactly as the device returned it, and
processed into its value.

## Usage

```sh
modbus-tool dump --devices PSU_1_1
modbus-tool dump --devices PSU_1_1,PSU_1_2,BBU_1_1
modbus-tool dump --all --output dump.json
```

| Option                  | Description                              |
| ----------------------- | ---------------------------------------- |
| `-d`, `--devices NAMES` | Comma separated list of devices to dump. |
| `-a`, `--all`           | Every device in the allowlist.           |
| `-b`, `--blackbox`      | Also read the blackbox.                  |
| `-o`, `--output FILE`   | Write the JSON here instead of stdout.   |
| `-y`, `--yes`           | Do not ask before pausing monitoring.    |
| `-v`, `--verbose`       | Log everything the read does.            |

Reads log to stderr on a terminal. Only warnings and worse are shown, unless
`--verbose` or an `LG2_LOG_LEVEL` in the environment asks for more.

A dump pauses `xyz.openbmc_project.ModbusRTU` on the ports involved, so the tool
says so and waits for a yes first. `--yes` skips the prompt, and is required
when stdin is not a terminal so a script never blocks on it.

`--devices` and `--all` are mutually exclusive, and one of them is required.

A device name is its entity-manager `Name`, for example `PSU_1_1`, matching the
names in `allowed-devices.json`. Spaces are replaced with underscores, as they
are in the allowlist.

`--all` reads the allowlist from `allowed-devices.json` and dumps those devices
in name order, so two dumps of the same platform can be compared. If no
allowlist is configured, or it is empty, there is no set to read and the tool
says to name the devices with `--devices` instead. A name the allowlist has but
entity-manager does not is reported as `Not configured`, rather than dropped, so
the two configurations disagreeing is visible.

### Exit codes

| Code | Meaning                                             |
| ---- | --------------------------------------------------- |
| 0    | A dump was produced. Check `Result` on each device. |
| 1    | Nothing could be dumped.                            |

The exit code only says whether there is output to read. Anything that stops one
device being read, such as a port held by another client or a name that is not
configured, is reported as that device's `Result` and `Reason`, so the rest of
the dump survives. Exit 1 is for the cases that leave nothing to read: no
allowlist to expand, no device that could be read, the output file could not be
written, or the lock could not be acquired.

Only one instance runs at a time, held by an exclusive `flock` on
`/run/lock/modbus.lock`. The reservation alone cannot tell two invocations
apart, because a port one of them holds already reads as disabled to the other,
which would then release a reservation it does not own. The kernel drops the
lock when the process ends, so it cannot go stale.

JSON goes to stdout, so it can be redirected or piped on its own. Everything
else goes to stderr. Exit 1 says why there is no dump:

```text
Another modbus-tool is already running
No allowlist is configured, so there is no set of devices to read. Name the
devices with --devices.
```

A dump that was produced still reports what went wrong in it, alongside exit 0,
so that a failure is visible without reading the JSON:

```text
PSU_1_9: Not configured
PSU_1_4: Port unavailable
2 of 24 devices could not be read
```

One line per device that could not be read at all, then a count. A device that
read only partly is not listed, since it is in the dump; `ReadStatus` says which
of its registers failed. A dump with nothing to report prints nothing to stderr.

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

The format is described below, and as a JSON Schema in
[schemas/dump.json](schemas/dump.json) for validating a dump.

Register contents are reported raw, and alongside them the processed `Value`,
with the `Unit` of each sensor and metric. `Raw` is kept so a consumer can still
check the value against the profile. Status registers have no `Value`: the
profile's bit definitions are copied through with an `Asserted` flag instead.

```json
{
  "Metadata": {
    "SchemaVersion": "1.2.0",
    "Tool": "modbus-tool",
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
            "Name": "PSU_1_1_Model",
            "Offset": "0x8",
            "Size": 8,
            "ReadStatus": "Success",
            "Value": "ECD17020",
            "Raw": ["0x4543", "0x4431", "0x3730", "0x3230"]
          }
        ],
        "Firmware": [
          {
            "Name": "PSU_1_1_PSU_FW_Revision",
            "Offset": "0x30",
            "Size": 4,
            "ReadStatus": "Success",
            "Value": "V1.00",
            "Raw": ["0x5631", "0x2E30", "0x3000", "0x0000"]
          }
        ],
        "Sensor": [
          {
            "Name": "PSU_1_1_INLET_SENSOR0_TEMP",
            "Offset": "0x45",
            "Size": 1,
            "ReadStatus": "Success",
            "Value": 25.0,
            "Unit": "DegreesC",
            "Raw": ["0x0C80"]
          }
        ],
        "Status": [
          {
            "Name": "PSU_1_1_PFC_ALARM",
            "Offset": "0x3D",
            "Size": 1,
            "ReadStatus": "Success",
            "Raw": ["0x0100"],
            "Bits": [
              {
                "Name": "PSU_1_1_AC_UNDER_VOLTAGE",
                "Position": 0,
                "Type": "SensorReadingCritical",
                "Asserted": false
              },
              {
                "Name": "PSU_1_1_AC_NOT_OK",
                "Position": 8,
                "Type": "PowerFault",
                "Asserted": true
              }
            ]
          }
        ],
        "Metric": [],
        "Config": [
          {
            "Name": "PSU_1_1_UnixTime",
            "Offset": "0x5A",
            "Size": 2,
            "ReadStatus": "Success",
            "Value": 1756441664,
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

| Reason                 | Meaning                               |
| ---------------------- | ------------------------------------- |
| `No response`          | The device did not answer the probe.  |
| `Probe value mismatch` | It answered, but not as this variant. |
| `Port unavailable`     | The port was held by another client.  |
| `Not configured`       | The name is not in entity-manager.    |

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
| `Name`       | Device name, then the register name.                 |
| `Offset`     | Register offset, hex. Unique within a device.        |
| `Size`       | Length in 16-bit registers.                          |
| `ReadStatus` | `Success` or `Failure`.                              |
| `Value`      | The register decoded, as below. Absent for `Status`. |
| `Unit`       | The unit `Value` is in. `Sensor` and `Metric` only.  |
| `Raw`        | `Size` register values, hex, most significant first. |

`Value` holds the processed data:

| Group                   | `Value`                                                 |
| ----------------------- | ------------------------------------------------------- |
| `Inventory`, `Firmware` | String, with the nulls the device pads it with removed. |
| `Sensor`, `Metric`      | Number, scaled as the profile describes.                |
| `Config`                | Unsigned integer.                                       |

`Unit` is the D-Bus unit without its interface prefix, such as `DegreesC`,
`Volts` or `Seconds`.

Registers that failed to read keep their entry with `ReadStatus` `Failure`, an
empty `Raw` and a `Value` of `null`, so the set of keys does not depend on which
reads succeeded. `Value` is also `null` for a sensor or metric that is not a
finite number, and for a config register wider than 64 bits, since JSON cannot
hold either.

Keys are written in the order shown rather than sorted, so what a device is
comes before what it read.

`Name` is the device name and the register name joined by an underscore, so a
sensor or status bit reads as the daemon publishes it and a name taken from an
event log can be found here. The register part comes from the profile's `Name`
where it has one; inventory registers and the `UnixTime` config register are
identified by `Type` in the profile instead, and that value is used. `Offset`
identifies a register unambiguously in all cases.

Status registers carry an additional `Bits` array holding only the positions the
profile defines, each with its `Name`, `Position`, `Type` and whether it is
`Asserted`. Bit names carry the device the same way. Positions the profile does
not model are absent, so compare against `Raw` to find bits the profile is
missing.

### Blackbox

A device that records a blackbox reports it under `Blackbox`, one entry per
section, but only when `--blackbox` asked for it.

| Field        | Description                             |
| ------------ | --------------------------------------- |
| `Section`    | The file number, or record index, read. |
| `ReadStatus` | `Success` or `Failure`.                 |
| `Raw`        | The section's registers.                |

A section that failed part way through keeps what it read, so `Raw` may be
shorter than the profile's length even though `ReadStatus` is `Failure`.

### Versioning

`SchemaVersion` is `major.minor.patch`. Adding a field is a minor bump; renaming
or removing one, or changing what a field means, is a major bump. Consumers
should pin the major version.

[schemas/dump.json](schemas/dump.json) records the version it describes in its
top-level `version`, and the tool takes `SchemaVersion` from it at build time.
