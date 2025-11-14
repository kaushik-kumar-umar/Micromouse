Micromouse Maze Solver
A competition-winning autonomous maze-solving robot based on the FloodFill algorithm, built with ESP32 and VL53L0X ToF sensors.
🏆 Winner - IIT BHU Technnex'25 , IIIT Nagpur TantraFiesta'25
📋 Table of Contents

Overview
Hardware Components
Features
Algorithm
Setup Instructions
Code Structure
Key Functions
Tuning Parameters
Circuit Diagram
Usage
Demo
Contributing
License

Overview
This project implements a micromouse - a small autonomous robot designed to navigate and solve mazes using the classic FloodFill algorithm. The robot uses time-of-flight sensors for wall detection and encoder feedback for precise movement control with PID-based wall following.
Key Specifications

Maze Size: 18x18 cells
Target: Center 4 cells (configurable)
Control: Dual PID loops (wall following + distance control)
Movement: Trapezoidal velocity profiling for smooth acceleration/deceleration

Hardware Components
Microcontroller ESP32 
Motors N20 DC Geared Motors (300 RPM) with encoders
Motor Driver   TB6612FNG 
Distance Sensors VL53L0X Time-of-Flight (ToF)
I2C Multiplexer TCA9548A (for multiple VL53L0X)
Voltage Regulator 7805 (5V)
BatteryLiPo 7.4V (2S)
Wheels 32mm diameter

Pin Configuration
cpp// Motor Driver Pins
LEFT_MOTOR_A:  26
LEFT_MOTOR_B:  25
LEFT_PWM:      19
RIGHT_MOTOR_A: 27
RIGHT_MOTOR_B: 14
RIGHT_PWM:     18

// Encoder Pins
LEFT_ENCODER:  13
RIGHT_ENCODER: 15

// Control Button
BUTTON:        2

// I2C (for sensors)
SDA:           21 (default)
SCL:           22 (default)
```

## ✨ Features

- **FloodFill Algorithm**: Dynamically updates shortest path as new walls are discovered
- **Wall Following**: PID-controlled navigation using left/right wall sensors
- **Smooth Motion**: Trapezoidal velocity profile for acceleration/deceleration
- **Encoder Feedback**: Precise distance tracking (620 ticks per cell)
- **Adaptive Turning**: Auto-adjusts 180° turns based on wall proximity
- **Real-time Maze Mapping**: Updates internal maze representation during exploration
- **Priority-based Navigation**: Prefers forward movement, then turns based on current direction
- **Sensor Calibration**: Button-based threshold calibration before run

## 🧠 Algorithm

### FloodFill Algorithm
The robot uses FloodFill to find the shortest path to the goal:

1. **Initialize**: Set destination cells (center 4) to flood value 0
2. **Propagate**: Assign values to neighboring cells (value = current + 1)
3. **Navigate**: Always move to accessible neighbor with lowest flood value
4. **Update**: When walls are discovered, re-flood the maze
5. **Repeat**: Until goal is reached

### Movement Strategy
```
Priority Order (when facing north):
1. North (forward)
2. East (right turn)
3. West (left turn)
4. South (180° turn)
```

## 📥 Setup Instructions

### Software Requirements
- Arduino IDE 1.8.x or later
- ESP32 Board Package
- Required Libraries:
```
  - Wire (built-in)
  - Adafruit_VL53L0X
  - Standard C++ queue
Installation Steps

Clone the repository

bash   git clone https://github.com/kaushik-kumar-umar/micromouse.git
   cd micromouse
```

2. **Install Libraries**
   - Open Arduino IDE
   - Go to `Sketch > Include Library > Manage Libraries`
   - Search and install: `Adafruit_VL53L0X`

3. **Configure Board**
   - Select `Tools > Board > ESP32 Dev Module`
   - Set upload speed to 115200

4. **Upload Code**
   - Open `micromouse.ino`
   - Click Upload

### Hardware Setup

1. **Mount Sensors**:
   - Left sensor: Channel 1 on TCA9548A
   - Front sensor: Channel 2 on TCA9548A
   - Right sensor: Channel 0 on TCA9548A

2. **Calibration Procedure**:
```
   1. Place robot in maze start position
   2. Press button once (LED indicates ready)
   3. Sensors calibrate left/right wall thresholds
   4. Press button again to start run
```

    # Standard maze specifications
🔧 Key Functions
Navigation Functions
FunctionDescriptionforwardOneCell()Moves robot forward one cell with PID wall followingturn90DegreesLeft()Executes left turn + backward adjustmentturn90DegreesRight()Executes right turn + backward adjustmentturn180Degrees()Performs 180° turn with wall-preference logic
Algorithm Functions
FunctionDescriptioninitializeMaze()Sets up 18x18 maze with boundary wallsfloodFill()Implements FloodFill algorithm using BFS queuefindNextCell()Selects next cell based on flood values & direction prioritysetWalls()Updates maze walls based on ToF sensor readingsresetMaze()Clears flood values before re-flooding
Control Functions
FunctionDescriptionupdateDistancePID()Implements trapezoidal velocity profilingreadSensors()Reads all three VL53L0X sensors via I2C multiplexerstopMotors()Emergency stop with all motor pins LOW
Utility Functions
FunctionDescriptiontcaSelect(channel)Switches I2C multiplexer channelprintDebugInfo()Outputs sensor readings to SerialprintMaze()Displays flood values in grid format
⚙️ Tuning Parameters
Movement Parameters
cppCELL_DISTANCE:        620    // Encoder ticks per cell
TURN_90_DEGREES:      140    // Encoder ticks for 90° turn
TURN_180_DEGREES:     400    // Encoder ticks for 180° turn
BASE_SPEED:           140    // Normal movement speed (PWM)
TURN_SPEED:           180    // Turning speed (PWM)
MIN_SPEED:            80     // Minimum speed during accel/decel
MAX_SPEED:            140    // Maximum cruising speed
PID Constants
Wall Following PID:
cppKp = 0.4   // Proportional gain
Ki = 0.0   // Integral gain (disabled)
Kd = 1.8   // Derivative gain
Distance PID (currently using trapezoidal profile):
cppACCEL_DISTANCE = 200   // Ticks to reach max speed
DECEL_DISTANCE = 200   // Ticks to decelerate
Sensor Thresholds
cppWALL_THRESHOLD_SIDE:  140 mm  // Left/Right wall detection
WALL_THRESHOLD_FRONT: 150 mm  // Front wall detection
LEFT_WALL_SETPOINT:   55 mm   // Target distance for wall following
```


### Wiring Summary
```
ESP32 → TB6612FNG
  - Pin 26 → AIN1 (Left Motor)
  - Pin 25 → AIN2 (Left Motor)
  - Pin 19 → PWMA (Left Motor)
  - Pin 27 → BIN1 (Right Motor)
  - Pin 14 → BIN2 (Right Motor)
  - Pin 18 → PWMB (Right Motor)

ESP32 → TCA9548A:
  - Pin 21 (SDA) → SDA
  - Pin 22 (SCL) → SCL
  - 3.3V → VCC
  - GND → GND

TCA9548A → VL53L0X Sensors:
  - SD0/SC0 → Right Sensor
  - SD1/SC1 → Left Sensor
  - SD2/SC2 → Front Sensor
```

## 🚀 Usage

### Quick Start
1. Place robot at maze start position (corner)
2. Press button to calibrate sensors
3. Press button again to start maze solving
4. Robot will navigate to center and stop

### Serial Monitor Output
Enable Serial Monitor at **115200 baud** to view:
- Sensor readings (Left, Right, Front distances)
- Current position coordinates
- Flood fill values
- Movement progress
- Debug information

### Expected Behavior
```
1. Robot reads initial walls
2. Calculates shortest path using FloodFill
3. Moves forward with wall following
4. Updates maze when new walls discovered
5. Re-calculates path dynamically
6. Reaches center (flood value = 0)
7. Stops and displays final maze
