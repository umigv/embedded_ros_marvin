# embedded_ros_marvin

## Project structure
```
├── embedded_ros_marvin/              # ROS 2 node implementations
│   ├── odrive_two_motors.py          # /cmd_vel → ODrives; publishes /enc_vel/raw (TwistWithCovarianceStamped)
│   ├── led_subscriber.py             # Drives safety-light LED based on estop / teleop / autonomy state
│   ├── serial_estop_monitor.py       # Monitors remote estop state and write to /tmp/estop_value.txt
│   └── recovery_executable.py        # /state-driven recovery; publishes /recovery_cmd_vel
├── launch/
│   └── embedded.launch.py            # Launches dual_odrive_controller + LED_subscriber + serial_estop_monitor
```

The estop writes the current estop state into `/tmp/estop_value.txt` (`"1"` = estopped). All embedded nodes read that file directly.

---

## To run the robot for the 2026 competition

### 1. Motor calibration
- Make sure the power switch is on and the physical estop is unpressed.
- In a new terminal: `odrivetool`
- `odrv0` and `odrv1` should both connect. If not, check USB and (if needed) reboot the computer.
- `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`
  - You should hear beeping. If the indicator light is red instead, run `odrv0.clear_errors()` first.
  - Light flashes green during calibration, blue when done.
- Repeat for `odrv1`.
  - On the test stand you can calibrate both at once. On the ground, do them one at a time.
- `quit()` to exit.
- If an ODrive errors at runtime, re-enter `odrivetool` and run `odrv0.clear_errors()` / `odrv1.clear_errors()`. If errors persist, re-check power and the physical estop.

### 2. Launch the embedded nodes
Make sure `odrivetool` is closed first (it holds the USB connection).
```sh
ros2 launch embedded_ros_marvin embedded.launch.py
```
ODrive indicator lights should now flash green.

### 3. Launch the recovery executable
`recovery_executable` is **not** included in `embedded.launch.py` — start it in its own terminal when running nav:
```sh
ros2 run embedded_ros_marvin recovery_executable
```
It waits on the `state/set_recovery` service, so bring up nav stack first.

---

## Nodes

### `dual_odrive_controller` ([odrive_two_motors.py](embedded_ros_marvin/odrive_two_motors.py))
- **Sub:** `cmd_vel` (`geometry_msgs/Twist`)
- **Pub:** `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) — remapped from `enc_vel` in the launch file
- Reads `/tmp/estop_value.txt`; commands zero velocity while estopped.


### `LED_subscriber` ([led_subscriber.py](embedded_ros_marvin/led_subscriber.py))

| Priority | Source | Code |
|---|---|---|
| 1 | `/tmp/estop_value.txt` is `"1"` | `6` |
| 2 | `teleop_cmd_vel` (`geometry_msgs/Twist`) message in the last 1 s | `1` |
| 3 | `state` = `normal` | `2` |
| 3 | `state` = `no_mans_land` | `3` |
| 3 | `state` = `recovery` | `4` |


### `recovery_executable` ([recovery_executable.py](embedded_ros_marvin/recovery_executable.py))
- **Sub:** `state` (`std_msgs/String`)
- **Pub:** `recovery_cmd_vel` (`geometry_msgs/Twist`)
- **Service client:** `state/set_recovery` (`std_srvs/SetBool`)


## Wireless access point password
`64182087`
