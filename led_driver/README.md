# led_driver
Drives the safety-light LED strip over serial based on the robot's current e-stop value, teleop activity, and state.
Waits up to `ready_timeout_s` for the Arduino to send `READY` before starting.

## Read Files
- `/tmp/estop_value.txt` - e-stop state written by `estop_driver`

## Subscribed Topics
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - marks teleop as active for `teleop_timeout_s` after the last message
- `state` (`std_msgs/String`) - current state

## LED Priority
| Priority | Condition | Code | Effect |
|---|---|---|---|
| 1 | `/tmp/estop_value.txt` is `"1"` | `6` | Flashing red |
| 2 | `teleop_cmd_vel` received within `teleop_timeout_s` | `1` | Solid blue |
| 3 | `state` = `normal` | `2` | Flashing blue |
| 3 | `state` = `no_mans_land` | `3` | Flashing green |
| 3 | `state` = `recovery` | `4` | Flashing yellow |
| 4 | `state` is unknown | `9` | Rainbow |

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `serial_port` | `Path` | `/dev/led` | Path to the serial device |
| `baud_rate` | `int` | `9600` | Serial baud rate |
| `ready_timeout_s` | `float` | `5.0` | Seconds to wait for the Arduino `READY` message before giving up |
| `teleop_timeout_s` | `float` | `1.0` | Seconds after the last teleop message before teleop is considered inactive |
| `update_period_s` | `float` | `0.1` | Period (s) at which the LED state is evaluated and sent |
| `estop_file_path` | `Path` | `/tmp/estop_value.txt` | Path to the file containing the e-stop state |
