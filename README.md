# embedded_ros_marvin

## Setup for Indicator LED

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
