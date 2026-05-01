# estop_driver
Reads from the serial e-stop device and atomically writes its state to `/tmp/estop_value.txt`. All other nodes read that 
file directly to determine whether the robot is estopped.

Does not publish the e-stop value to a ROS topic — this ensures the e-stop still works even if there are ROS network
issues.

## Written Files
- `/tmp/estop_value.txt` - e-stop state; `"1"` = estopped, `"0"` = safe to drive

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `serial_port` | `Path` | `/dev/estop` | Path to the serial device |
| `baud_rate` | `int` | `9600` | Serial baud rate |
| `poll_period_s` | `float` | `0.05` | Period (s) at which the serial port is polled for new data |
| `estop_file_path` | `Path` | `/tmp/estop_value.txt` | Path to the file the e-stop state is written to |
