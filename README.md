# Quadrotor Autopilot Simulation for Webots

A comprehensive quadrotor drone simulation with autonomous flight capabilities, wind effects, and obstacle navigation built for the Webots robotics simulator.

![Webots Version](https://img.shields.io/badge/Webots-R2023b-blue)
![Python Version](https://img.shields.io/badge/Python-3.8%2B-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## Features

### 🚁 Autonomous Flight Control
- **PID-based autopilot system** for stable flight control
- Multi-axis control: altitude, roll, pitch, and yaw
- Waypoint navigation with automatic path following
- Smooth takeoff and landing capabilities

### 🌬️ Wind Simulation
- Realistic wind effects with base wind velocity
- Dynamic wind gusts using sinusoidal patterns
- Random turbulence for enhanced realism
- Automatic wind disturbance compensation

### 🏢 Environment
- Multiple building structures as obstacles
- 50m × 50m flight arena
- Textured ground and realistic lighting
- Customizable environment layout

### 📡 Sensor Suite
- **IMU** (Inertial Measurement Unit) - orientation tracking
- **GPS** - position tracking
- **Gyroscope** - angular velocity measurement
- **Compass** - heading information
- **Camera** - forward-facing vision (640×480)

## Installation

### Prerequisites
- **Webots R2023b** or later ([Download here](https://cyberbotics.com/))
- **Python 3.8+** (included with Webots)

### Setup

1. Clone this repository:
```bash
git clone https://github.com/yourusername/quadrotor-autopilot-webots.git
cd quadrotor-autopilot-webots
```

2. Copy the world file to your Webots worlds directory:
```bash
cp worlds/quadrotor_world.wbt /path/to/webots/worlds/
```

3. Copy the controller to your Webots controllers directory:
```bash
cp -r controllers/quadrotor_autopilot /path/to/webots/controllers/
```

Alternatively, you can place these files in your project's local `worlds/` and `controllers/` directories.

## Project Structure

```
quadrotor-autopilot-webots/
│
├── worlds/
│   └── quadrotor_world.wbt          # Main simulation world
│
├── controllers/
│   └── quadrotor_autopilot/
│       └── quadrotor_autopilot.py   # Autopilot controller
│
├── docs/
│   └── images/                       # Screenshots and diagrams
│
└── README.md
```

## Usage

### Running the Simulation

1. Open Webots
2. Load the world file: `File → Open World → quadrotor_world.wbt`
3. Click the **Play** button to start the simulation
4. Watch the quadrotor autonomously navigate through waypoints

### Console Output

The controller provides real-time flight information:
```
Quadrotor Autopilot Initialized
Mission: Navigate through 5 waypoints
Wind simulation: Enabled
Time: 2.0s | Pos: (0.15, 0.08, 2.95) | Target WP0: (0.0, 0.0, 3.0) | Dist: 0.17m | Wind: (2.3, 1.2, 0.4)
Waypoint 0 reached. Moving to next waypoint.
```

## Configuration

### Modifying Waypoints

Edit the waypoints list in `quadrotor_autopilot.py`:

```python
self.waypoints = [
    (x, y, z, yaw),  # (meters, meters, meters, radians)
    (0, 0, 3, 0),
    (5, 5, 4, 0.785),
    (10, 0, 3.5, 1.57),
    # Add more waypoints...
]
```

### Adjusting Wind Parameters

Modify wind characteristics in the controller:

```python
self.wind_base = [2.0, 1.0, 0.5]      # Base wind [x, y, z] m/s
self.wind_gust_amplitude = 1.5         # Gust strength
self.wind_gust_frequency = 0.3         # Gust frequency
self.wind_enabled = True               # Enable/disable wind
```

### Tuning PID Controllers

Fine-tune flight behavior by adjusting PID gains:

```python
self.altitude_pid = PIDController(kp=20.0, ki=0.5, kd=15.0)
self.roll_pid = PIDController(kp=40.0, ki=0.1, kd=20.0)
self.pitch_pid = PIDController(kp=40.0, ki=0.1, kd=20.0)
self.yaw_pid = PIDController(kp=30.0, ki=0.05, kd=10.0)
```

### Adding Buildings

Add more structures in the world file:

```python
SimpleBuilding {
  translation x y 0
  corners [
    0 0
    0 width
    length width
    length 0
  ]
  wallHeight height
}
```

## Control Architecture

The autopilot implements a cascaded control system:

```
Position Control → Velocity Control → Attitude Control → Motor Control
                      ↓
                Wind Compensation
```

**Control Loop:**
1. GPS measures current position
2. Error calculated from target waypoint
3. Velocity controller generates desired velocities
4. Wind compensation adjusts for disturbances
5. Attitude controller converts to roll/pitch commands
6. Motor mixer distributes thrust to 4 propellers

## Technical Details

### Motor Configuration
- **Front Left (FL)**: Counter-clockwise (red)
- **Front Right (FR)**: Clockwise (green)
- **Rear Left (RL)**: Clockwise (blue)
- **Rear Right (RR)**: Counter-clockwise (yellow)

### Quadrotor Specifications
- **Mass**: 0.5 kg
- **Size**: 0.3m diagonal (motor to motor)
- **Max Motor Speed**: 600 rad/s
- **Thrust Constant**: 12.5 N/rpm
- **Torque Constant**: 0.0015 Nm/rpm

## Troubleshooting

**Quadrotor falls immediately:**
- Check that all 4 motors are properly initialized
- Verify PID gains are not too low
- Ensure base_velocity is sufficient (try 350-400)

**Unstable oscillations:**
- Reduce PID proportional gains
- Increase derivative gains for damping
- Lower timestep in world file (8ms recommended)

**Waypoints not reached:**
- Increase waypoint threshold distance
- Reduce wind strength for testing
- Check GPS sensor is enabled

**Wind too strong:**
- Reduce `wind_base` values
- Decrease `wind_gust_amplitude`
- Adjust wind compensation factors


**Note**: This is a simulation for educational and research purposes. Real-world drone flight requires proper licensing, safety measures, and compliance with local regulations.
