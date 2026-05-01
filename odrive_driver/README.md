# odrive_driver
Controls two ODrives (left and right) over USB. Converts `cmd_vel` twist commands to per-motor velocity setpoints and
publishes encoder-derived velocity with propagated covariance.

## Read Files
- `/tmp/estop_value.txt` - e-stop state written by `estop_driver`; commands zero velocity to both motors while estopped

## Subscribed Topics
- `cmd_vel` (`geometry_msgs/Twist`)

## Published Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - encoder derived linear/angular velocity with dynamic 
covariance

## Config Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `sample_time_s` | `float` | `0.01` | Period (s) of the encoder publish timer |
| `timestamp_delay_s` | `float` | `0.0` | Subtracted from the publish timestamp to compensate read and processing latency (s) |
| `frame_id` | `str` | `"base_link"` | TF frame ID attached to the published twist header |
| `estop_file_path` | `Path` | `/tmp/estop_value.txt` | Path to the e-stop flag file |

### Geometry
| Parameter | Type | Default | Description |
|---|---|---|---|
| `track_width_m` | `float` | `0.764` | Distance between left and right wheel contact points (m) |
| `wheel_diameter_m` | `float` | `0.18423` | Diameter of each drive wheel (m) |
| `gear_ratio` | `float` | `170/9 ≈ 18.89` | Motor-to-wheel gear ratio (motor revolutions per wheel revolution) |

### Covariance
Variance scales with speed to reflect increased uncertainty at higher velocities:

- `linear_variance  = linear_variance_static  + linear_variance_gain  * linear_mps²`
- `angular_variance = angular_variance_static + angular_variance_gain * (linear_mps² / track_width_m² + angular_radps²)`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `linear_variance_static` | `float` | `1e-6` | Baseline linear velocity variance, independent of speed (m²/s²) |
| `linear_variance_gain` | `float` | `0.0004` | Speed-dependent gain on linear velocity variance (m²/s² per (m/s)²) |
| `angular_variance_static` | `float` | `1e-6` | Baseline angular velocity variance, independent of speed (rad²/s²) |
| `angular_variance_gain` | `float` | `0.0004` | Speed-dependent gain on angular velocity variance (rad²/s² per (m/s)²) |

## Motor calibration
Before running any script, make sure `odrivetool` and this node is closed first as they hold the USB connection.

Calibrate the odrive every time they turn on after a power cycle.
```sh
python3 scripts/calibrate_odrive.py
```
You should hear a beep when calibration starts. The indicator light will flash green while calibrating and be solid blue
when done. If the indicator light is red, run the script again.

If an ODrive errors (flashing red) at runtime:
```sh
python3 scripts/clear_odrive_errors.py
```
The indicator light should stop flashing red.

If errors persist or if the scripts can't detect an ODrive, troubleshoot in the following order:
1. Re-check power and the physical e-stop
2. Unplug and replug the ODrive from both the USB hub and the computer USB port
3. Run `odrivetool` and see if the official tool is able to detect them
4. Close the terminal and open a new one
5. Restart the laptop
