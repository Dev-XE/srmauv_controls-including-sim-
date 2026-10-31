# joystick_pacs

A ROS2 Python package for joystick-based control of underwater vehicles (AUV/ROV). This package provides nodes for reading joystick inputs, publishing velocity commands, interfacing with serial hardware (thrusters, IMU), and offering GUI tools for vehicle control.

**Usage:**
# Terminal 1: 
```bash
ros2 run joystick_pacs joy_node
```

**Usage:**
# Terminal 2:
```bash
ros2 run joystick_pacs slider_gui
```

## Features

- **Joystick Control**: Read inputs from gamepad/joystick controllers and publish ROS2 velocity commands
- **IMU Integration**: Serial interface for ESP32-based IMU sensors with orientation, angular velocity, and acceleration data
- **Thruster Control**: Send thruster commands via serial to microcontroller
- **Calibration Tools**: Interactive joystick axis calibration
- **GUI Tools**: PyQt5-based GUIs for arm/disarm control and parameter adjustment
- **Depth Lock**: Toggle depth hold mode from joystick

## Dependencies

- `rclpy` - ROS2 Python client library
- `pygame` - Joystick input handling
- `nemo_interfaces` - Custom ROS2 message definitions (RovCommands)
- `pyserial` - Serial communication
- `PyQt5` - GUI components (for GUI nodes)

```bash
pip install pygame pyserial PyQt5
```

## Nodes

### joy_node

Main joystick control node that reads joystick axes and publishes velocity commands.



**Published Topics:**
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/nemo_auv/cmd_vel_raw` | `geometry_msgs/Twist` | Linear and angular velocity commands |
| `/nemo_auv/input_cmd` | `nemo_interfaces/RovCommands` | Surge, sway, heave, yaw commands |
| `/nemo_auv/depth_lock` | `std_msgs/Bool` | Depth lock toggle status |

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `surgeInput` | `double[]` | [1.0, 0.019, -1.0] | Calibrated surge axis input mapping |
| `yawInput` | `double[]` | [0.99, -0.01, -1.00] | Calibrated yaw axis input mapping |
| `heaveInput` | `double[]` | [0.99, 0.0, -0.99] | Calibrated heave axis input mapping |
| `swayInput` | `double[]` | [-1.00, 0.00, 0.99] | Calibrated sway axis input mapping |
| `surgeVelVector` | `double[]` | [-1.00, 0.00, 1.00] | Surge velocity output mapping |
| `yawVelVector` | `double[]` | [-1.00, 0.00, 1.00] | Yaw velocity output mapping (deg/s) |
| `heaveVelVector` | `double[]` | [-1.00, 0.00, 1.00] | Heave velocity output mapping |
| `swayVelVector` | `double[]` | [-1.00, 0.00, 1.00] | Sway velocity output mapping |

**Features:**
- Interactive calibration on startup (optional)
- Button 3 toggles depth lock mode

---

### armPub

Publishes arm/disarm status from joystick button 0.

**Usage:**
```bash
ros2 run joystick_pacs armPub
```

**Published Topics:**
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/arm_status` | `std_msgs/Bool` | Armed (True) / Disarmed (False) status |

**Behavior:**
- Reads button 0 state at 50ms intervals
- Publishes on state change or heartbeat every 0.5s

---

### serialPub

Subscribes to thruster values and sends them via serial to microcontroller.

**Usage:**
```bash
ros2 run joystick_pacs serialPub
```

**Subscribed Topics:**
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/nemo_auv/thruster_vals` | `std_msgs/Int8MultiArray` | Array of 8 thruster values |

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `SerialPort` | `string` | `/dev/ttyUSB0` | Serial port device path |

**Serial Protocol:**
- Baud rate: 115200
- Packet format: `[0xAA, 8 thruster bytes, 0x55]`

---

### controlled_pub

Subscribes to RovCommands and sends formatted thruster commands via serial.

**Usage:**
```bash
ros2 run joystick_pacs controlled_pub
```

**Subscribed Topics:**
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/nemo_auv/input_cmd` | `nemo_interfaces/RovCommands` | Vehicle motion commands |

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `SerialPort` | `string` | `/dev/ttyUSB0` | Serial port device path |
| `Baud` | `int` | 115200 | Serial baud rate |

---

### imu_pub

Reads IMU data from ESP32 via serial and publishes ROS IMU messages.

**Usage:**
```bash
ros2 run joystick_pacs imu_pub
```

**Published Topics:**
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/imu/data_raw` | `sensor_msgs/Imu` | Orientation, angular velocity, linear acceleration |
| `/imu/mag` | `sensor_msgs/MagneticField` | Magnetic field data |

**Serial Protocol:**
- Baud rate: 115200
- Packet size: 52 bytes
- Start byte: `0xAA`, Stop byte: `0x55`
- Data: pitch, pitch_rate, roll, roll_rate, yaw, yaw_rate, accel_x, accel_y, accel_z

---

### calibration_node

Interactive joystick calibration tool.

**Usage:**
```bash
ros2 run joystick_pacs calibration_node
```

**Process:**
1. Prompts to calibrate each axis (surge, yaw, heave, sway)
2. For each axis: center position → max position → min position
3. Calculates average values for each position
4. Outputs calibrated parameters

---

### arm_gui

PyQt5 GUI for arm/disarm control.

**Usage:**
```bash
ros2 run joystick_pacs arm_gui
```

**Published Topics:**
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/nemo_auv/arm` | `std_msgs/Bool` | Arm command |

**Features:**
- Toggle button (red = disarmed, green = armed)
- 10ms ROS spin rate integrated with Qt event loop

---

### param_gui

Parameter adjustment GUI (see source for details).

**Usage:**
```bash
ros2 run joystick_pacs param_gui
```

---

### slider_gui

Slider-based control GUI (see source for details).



---

### better_imu

Updated IMU publisher with improved functionality.

**Usage:**
```bash
ros2 run joystick_pacs better_imu
```

---

## Axis Mapping (Default)

| Axis ID | Motion | Description |
|---------|--------|-------------|
| 0 | Yaw | Rotation around vertical axis |
| 1 | Surge | Forward/backward motion |
| 3 | Sway | Left/right motion |
| 4 | Heave | Up/down motion |

## Building

```bash
cd ~/ros2_ws/src
git clone <repository-url> joystick_pacs
cd ..
colcon build --packages-select joystick_pacs
source install/setup.bash
```

## Running a Complete Control Stack

```bash
# Terminal 1: Joystick input
ros2 run joystick_pacs joy_node

# Terminal 2: Serial to thrusters
ros2 run joystick_pacs serialPub

# Terminal 3: IMU data (if using external IMU)
ros2 run joystick_pacs imu_pub

# Terminal 4: Arm GUI (optional)
ros2 run joystick_pacs arm_gui
```

## License

Specify your license here (e.g., MIT, BSD, Apache-2.0)

## Maintainer

- **zappington** - abhinavs1306@gmail.com

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.