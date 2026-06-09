# Runtime Configuration

## allowed-devices.json

Controls which devices are probed. If the file exists, only listed devices are
probed. If absent, all devices from entity-manager are probed.

**Path:** `/var/lib/phosphor-modbus/allowed-devices.json`

**Format:**

```json
{
  "AllowedDevices": ["PSU_1_1", "PSU_1_2", "BBU_SHELF_1", "PSU_PMM_1"]
}
```

- Device names must match the entity-manager `Name` property.
- Spaces in names are replaced with underscores.
- An empty list blocks all devices from probing.
- Changes to the file are detected automatically via inotify and the allowlist
  is reloaded without restarting the service.
