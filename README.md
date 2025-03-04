# Embedded_ROS_Marvin
Setup for indicator LED

1. Run this in terminal: 
        sudo nano /etc/udev/rules.d/99-arduino.rules
2. In the file that opens, add: 
        SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", ATTRS{serial}=="557363130383514121D2", SYMLINK+="LED_Arduino", MODE="0666"
3. Reboot system

Note:

Run "lsusb" to get the idVendor and idProduct.

To get serial number (optional), make sure device is connected to ACM0, then run " udevadm info -a -n /dev/ttyACM0 | grep 'ATTRS{serial}' ".
