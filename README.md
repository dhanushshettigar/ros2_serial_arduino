# ROS2 Serial Communication Package(Arduino)

## Overview

This ROS2 package facilitates serial communication between a Raspberry Pi and Arduino within a Docker container. It consists of three nodes, each serving a specific purpose.

## Nodes

1. **list_ports**
    - Lists available serial ports on the Raspberry Pi.

2. **serial_reader**
    - Reads data from the Arduino over serial communication.
    - Parameters:
        - `baud` (int): Baud rate for serial communication.
        - `port` (string): Serial port to connect to Arduino.
        - `publish_topic` (string): ROS2 topic to publish received data.

3. **serial_writer**
    - Writes data to the Arduino over serial communication.
    - Parameters:
        - `baud` (int): Baud rate for serial communication.
        - `port` (string): Serial port to connect to Arduino.
        - `subscribe_topic` (string): ROS2 topic to subscribe for data to be sent.

## Clone the repository

**Step 1**: Navigate to your desired workspace directory (e.g., ros2_ws)

```bash
cd home/ros2_ws
```

**Step 2**: Clone the repository
```bash
https://github.com/dhanushshettigar/ros2_serial_arduino.git
```

**Step 3**: Build the package with colcon
```bash
colcon build --symlink-install
```

**Step 4**: Source the setup.bash file
```bash
source install/setup.bash
```

**Optional**: Add the setup.bash file to .bashrc for automatic sourcing
```bash
echo "source home/ros2_ws/install/setup.bash" >> ~/.bashrc
```

##  ROS2-Serial-Arduino Package Usage

### Finding Serial Ports

To simplify the process of identifying the correct serial port, a utility node named `list_ports` has been provided. Run the following command to list available port paths and names:

```bash
ros2 run ros2_serial_arduino list_ports
```

### Communicating with Serial Port

To `read` data from a specific serial port and `publish` that data, use the `serial_read` node. Ensure you provide the correct parameters:

```bash
ros2 run ros2_serial_arduino serial_reader --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=<9600> -p publish_to:=/<my_custom_topic>
```

To `write` data to a specific serial port based on the `subscribed topic` then use the `serial_write` node. Ensure you provide the correct parameters:

```bash
ros2 run ros2_serial_arduino serial_writer --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=<9600> -p subscribe_to:=/<my_custom_topic>
```
**Note**: Omit the **<>** characters when entering your actual values.
