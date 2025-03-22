# embedded_ros_marvin

## **Launch Files**

### **1. Launch Embedded Nodes**
This launch file starts the odrive controller node. Make sure to first run motor calibration (but no need to set to closed loop control).

```sh
ros2 launch embedded_ros_marvin launch_embedded.py
```

#### **Parameters:**
- **`use_LED`** (default: `true`)
  - If `false`, the LED subscriber node will be disabled.
  
- **`use_enc_odom`** (default: `false`)
  - If `true`, an odom message will be published using encoder-based estimation, and the TF transform will be broadcast.
  - This should be set to `false` when using sensor fusion.

---

### **2. Launch EKF**
This launch file starts the robot_localization EKF node

```sh
ros2 launch embedded_ros_marvin launch_ekf.py
```

  - The EKF node uses parameters specified in: params/arv_ekf.yaml

## To start remote estop

### 1. Open estopnew.grc located in the sdr_estop folder

### 2. Run the flowgraph 

### Notes

- Restarting the flowgraph will default estop value to 1 (no estop).

- If estop flowgraph is not ran, estop value will be default to 1 (no estop).

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
### Router password:
Password: 64182087
