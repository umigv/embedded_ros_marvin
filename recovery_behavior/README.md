# recovery_behavior

State-machine-driven recovery node. When the robot enters the `recovery` state it sweeps left/right using an ultrasonic 
sensor to find a clear path, then backs up.

Reads ultrasonic distance from `/dev/ultrasonic` at 9600 baud.

## Subscribed Topics
- `state` (`std_msgs/String`) — triggers recovery when value is `"recovery"`

## Published Topics
- `recovery_cmd_vel` (`geometry_msgs/Twist`)

## Service Clients
- `state/set_recovery` (`std_srvs/SetBool`) — signals recovery end back to the nav stack
