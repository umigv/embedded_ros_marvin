# embedded_ros_marvin

## Key Project Structrure
```
├── embedded_ros_marvin/              # Source folder containing ROS 2 node implementations
│   ├── led_subscriber.py             # Subscribe to /is_auto and control safety light LED
│   ├── odrive_two_motors.py          # Subscribe to /joy_cmd_vel, control the odrives and publish /enc_vel/raw
│   └── recovery_executable.py        # Subscribe to /state and call /state/set_recovery, controls recovery and publish /recovery_cmd_vel
├── launch/                           
│   └── launch_embedded.py            # Launches embedded nodes
├── sdr_estop/                        
│   ├── estopnew.grc                  # GNU radio flowgraph
│   └── estop.py                      # Auto generated python code from flowgraph
│   └── estop_epy_block_3.py          # Embedded python block for integer toggle
```

---

## **To run the robot for 2025 competition**
### **1. Motor Calibration**
- Make sure power switch is on and physical estop is unpressed
- Type `odrivetool` in a new terminal
- odrv0 and odrv1 should both be connected. If not, check USB connection and optionally restart computer.
- Type `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE` to calibrate odrv0
    - You should hear a beeping sound when calibration starts. If not, check if odrive indicator light is red. If yes, type `odrv0.clear_errors()`
    - During calibration, the light should flash green. When calibration completes, light will flash blue.
- Do the same for odrv1
    - When robot is on test stand, you can calibrate both odrives at the same time. When on the ground, calibrate one at a time.
- Exit odrivetool by typing `quit()`
- If odrive errors during runtime, enter odrivetool and type `odrv0.clear_errors()`, `odrv1.clear_errors()`
    - If error persists and odrive still flash red, double check if power is on and physical estop is unpressed

### **2. Launch Emedded Nodes**
- Type `ros2 launch embedded_ros_marvin launch_embedded.py`. Make sure to quit odrivetool first
- Odrive indicator lights should now flash green

### **3. To start remote estop**
- Open estopnew.grc located in the sdr_estop folder
- Run the flowgraph 

#### Notes

- Restarting the flowgraph will default estop value to 1 (no estop).

- If estop flowgraph is not ran, estop value will be default to 1 (no estop).

---

## **Launch Files**

### **1. Launch Embedded Nodes**
This launch file starts the odrive controller node. Make sure to first run motor calibration (but no need to set to closed loop control).

```sh
ros2 launch embedded_ros_marvin launch_embedded.py
```

#### **Parameters:**
- **`use_LED`** (default: `true`)
  - If `false`, the LED subscriber node will be disabled.

---

## Setup for indicator LED on a new device

### 1. Create a udev rule file

Run the following command to open the rule file:
```bash
sudo nano /etc/udev/rules.d/99-arduino.rules
```

### 2. Add the following rule

Copy and paste this line into the file:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", ATTRS{serial}=="557363130383514121D2", SYMLINK+="LED_Arduino", MODE="0666"
```

### 3. Reboot the system

### Notes

- To get the `idVendor` and `idProduct`, run:
  ```bash
  lsusb
  ```
- To get the serial number (optional), make sure the device is connected to `ACM0`, then run:
  ```bash
  udevadm info -a -n /dev/ttyACM0 | grep 'ATTRS{serial}'
  ```

---

## Wireless access point password:
Password: `64182087`
