# ENGR-2350 Lab 5 - Gesture RC Car

A multi-part lab implementing gesture-based control and autonomous navigation for the TI-RSLK robot using I2C sensors.

## Project Overview

This lab is divided into three parts:

- **Part A & B**: Gesture control using CMPS12 compass sensor with two control modes
- **Part C**: Autonomous navigation using ultrasonic ranging

## Repository Structure

```
Lab5/
├── PartA_B_GestureControl.c    # Gesture-based RC control (compass)
├── PartC_AutonomousNav.c       # Autonomous navigation (ultrasonic)
├── engr2350_mspm0.h            # Hardware abstraction library (if included)
├── README.md                   # This file
└── .gitignore                  # Git ignore rules
```

---

# Part A & B: Gesture RC Car

Control the robot using hand movements detected by a CMPS12 electronic compass.

## Hardware Requirements (Parts A & B)

- TI-RSLK Robotic Car
- CMPS12 Electronic Compass (I2C address: 0x60)
- SRF08 Ultrasonic Ranger (I2C address: 0x70)
- Slide switch for mode selection
- Long wires (~1m) for hand-held compass module

## Features

### Part A: Roll/Pitch Control
- **Pitch** (tilt forward/back) controls speed
  - Forward tilt → speed up
  - Backward tilt → slow down/reverse
- **Roll** (tilt left/right) controls turning
  - Left tilt → turn left
  - Right tilt → turn right
- **Collision avoidance**: Stops if obstacle within 25 cm

### Part B: Heading-Based PD Control  
- **Pitch** still controls speed
- **Compass heading** controls direction using PD control
  - Rotate hand-held compass → robot turns to match heading
  - Uses encoder-based relative heading measurement
  - PD control ensures smooth turning without oscillation

### Mode Selection
- Slide switch toggles between Part A (roll) and Part B (heading) control
- LED or serial output indicates current mode

## Control Specifications (Parts A & B)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Max Speed | 90% | Maximum duty cycle |
| Min Speed | 15% | Below this, motors stop |
| Dead Zone (Pitch) | ±5° | No speed change in this range |
| Dead Zone (Roll) | ±5° | Drive straight in this range |
| Collision Range | 25 cm | Auto-stop distance |
| Update Rate | 100 ms | Control loop period |
| Kp (Heading) | 0.03 | Proportional gain (tuned) |
| Kd (Heading) | 0.001 | Derivative gain (tuned) |
| Wheel Base | 149 mm | Distance between wheels |
| Encoder Resolution | 0.611 mm/count | Distance per encoder tick |

## Pin Configuration (Parts A & B)

### I2C Sensors
- **PA15**: I2C1 SDA (open-drain with pull-up)
- **PA16**: I2C1 SCL (open-drain with pull-up)
- **CMPS12 Address**: 0x60
- **SRF08 Address**: 0x70

### Mode Selection
- **PA18**: Slide switch (LOW=Roll control, HIGH=Heading control)

### Motor Control
- **PB15**: Left motor enable
- **PB16**: Right motor enable
- **PB0**: Left motor direction
- **PB8**: Right motor direction

### PWM & Encoders
- **PB7**: Left motor PWM
- **PB6**: Right motor PWM
- **PB10**: Left encoder
- **PB11**: Right encoder

## Part A: Roll/Pitch Control Algorithm

```c
// Read compass pitch and roll
pitch = readCompassRegister(4);  // -90° to +90°
roll = readCompassRegister(5);   // -90° to +90°

// Convert pitch to speed
if (|pitch| < 5°):
    desired_speed = 0
else:
    desired_speed = (pitch / 90) × 0.9

// Convert roll to differential speed
if (|roll| < 5°):
    desired_diff_speed = 0  // Drive straight
else:
    desired_diff_speed = (roll / 90) × 0.9

// Collision avoidance
if (range < 25 cm AND moving forward):
    desired_speed = 0

// Calculate wheel speeds
left_speed = desired_speed - desired_diff_speed
right_speed = desired_speed + desired_diff_speed
```

## Part B: Heading-Based PD Control

### Relative Heading Calculation
The robot tracks its orientation using encoder measurements:

```c
// In encoder interrupt:
if (motor_forward):
    enc_counts_segment++
else:
    enc_counts_segment--

// In main loop every 100ms:
dist_left = enc_counts_left_segment × 0.611 mm
dist_right = enc_counts_right_segment × 0.611 mm

delta_theta = (dist_right - dist_left) / 149 mm
measured_heading += delta_theta × (180/π)

// Keep in 0-360° range
while (measured_heading >= 360°): measured_heading -= 360°
while (measured_heading < 0°): measured_heading += 360°

// Reset segment counters
enc_counts_left_segment = 0
enc_counts_right_segment = 0
```

### PD Control for Turning

```c
// Read desired heading from hand-held compass
desired_heading = compassHeading / 10.0  // Convert tenths to degrees

// Calculate error with wrapping
heading_error = desired_heading - measured_heading
if (heading_error > 180°): heading_error -= 360°
if (heading_error < -180°): heading_error += 360°

// PD control
heading_derivative = heading_error - heading_error_prev
desired_diff_speed = Kp × heading_error + Kd × heading_derivative

// Clamp differential speed
if (desired_diff_speed > 0.9): desired_diff_speed = 0.9
if (desired_diff_speed < -0.9): desired_diff_speed = -0.9

heading_error_prev = heading_error
```

### Why Angle Wrapping is Critical

Without wrapping:
```
desired = 350°, measured = 10°
error = 350 - 10 = 340°  ❌ WRONG! (turns long way)

With wrapping:
error = 340° - 360° = -20°  ✅ CORRECT! (short turn)
```

## Debug Output (Parts A & B)

```
Mode:HEAD | P: 15 | R:  3 | H:1850 | MH:45.2 | Rng: 50 | Vd:0.15 | Vdiff:-0.05
Mode:ROLL | P:-20 | R:-10 | H:1850 | MH:45.2 | Rng: 30 | Vd:-0.22 | Vdiff:-0.11
```

Shows:
- **Mode**: ROLL or HEAD (heading control)
- **P**: Pitch angle (degrees)
- **R**: Roll angle (degrees)
- **H**: Compass heading (tenths of degrees, 0-3599)
- **MH**: Measured heading from encoders (degrees, 0-360)
- **Rng**: Range to obstacle (cm)
- **Vd**: Desired speed (duty cycle)
- **Vdiff**: Differential speed (duty cycle)

---

# Part C: Autonomous Navigation

Autonomous obstacle avoidance using ultrasonic ranging and intelligent turn strategy.

## Hardware Requirements (Part C)

- TI-RSLK Robotic Car
- SRF08 Ultrasonic Ranger (I2C address: 0x70)
- Push button (BMP1) for start trigger
- No compass required

## Features

- **Automatic obstacle detection** at 25 cm
- **Intelligent turn strategy**: Alternates left/right, increases turn angle
- **Encoder-based turn precision**: No compass needed
- **Dual-wheel speed control**: Maintains straight-line motion
- **Systematic exploration**: Avoids getting stuck in corners

## Control Specifications (Part C)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Range Limit | 25 cm | Obstacle detection threshold |
| Turn Angle | 60° | Default turn magnitude |
| Turn Increment | 5° | Angle increase per direction change |
| Wheel Base | 149 mm | Distance between wheels |
| Turn PWM | 300 | 30% duty cycle during turns |
| Speed Setpoint | 80,000 | Encoder period target |

## Navigation Algorithm

```
Startup:
    Wait for BMP1 button press
    Initialize forward motion

Main Loop:
    Read ultrasonic range
    
    If range < 25 cm:
        Stop motors
        
        If completed 2 turns in current direction:
            Reverse turn direction (left ↔ right)
            Increase turn angle by 5°
            Reset turn counter
        
        Pause 100ms
        Execute turn (calculated from encoders)
        Pause 1.5s
        Increment turn counter
    
    Else:
        Drive forward
    
    Update every 100ms
```

## Turn Pattern Example

```
Turn 1:  Right 60°
Turn 2:  Right 60°  → Switch direction
Turn 3:  Left -65°  (increased by 5°)
Turn 4:  Left -65°  → Switch direction
Turn 5:  Right 70°  (increased by 5°)
Turn 6:  Right 70°  → Switch direction
...and so on...
```

This creates systematic exploration and prevents loops.

## Encoder-Based Turn Calculation

```c
// Arc length for turn angle
arcLength = (π × wheelBase × |degrees|) / 180°
          = (π × 149 × 60) / 180 = 155.5 mm

// Distance per encoder tick
mmPerTick = (2π × wheelRadius) / 360
          = (2π × 35) / 360 = 0.611 mm

// Target encoder counts
targetCounts = arcLength / mmPerTick
             = 155.5 / 0.611 ≈ 254 ticks

// Execute turn:
If turning right:
    Left wheel on, right wheel off
    Wait until enc_total_events ≥ targetCounts

If turning left:
    Right wheel on, left wheel off
    Wait until enc_total_events ≥ targetCounts
```

## Debug Output (Part C)

```
RANGE: 045 | PWM_L: 800 | PWM_R: 795 | turnDirection: 60 RIGHT
RANGE: 018 | PWM_L: 800 | PWM_R: 798 | turnDirection: 60 RIGHT

TURNED RIGHT

RANGE: 120 | PWM_L: 802 | PWM_R: 799 | turnDirection: -65 LEFT
```

---

# Common Elements (All Parts)

## I2C Communication

### CMPS12 Compass (Parts A & B)
```c
// Read 4 bytes starting at register 2
uint8_t data[4];
I2C_readData(I2C1, 0x60, 2, data, 4);

heading = (data[0] << 8) | data[1];  // 0-3599 (tenths of degrees)
pitch = (int8_t)data[2];             // -90 to +90
roll = (int8_t)data[3];              // -90 to +90
```

### SRF08 Ultrasonic Ranger (All Parts)
```c
// Trigger measurement
uint8_t command = 0x51;  // Range in cm
I2C_writeData(I2C1, 0x70, 0, &command, 1);

// Wait 80ms, then read result
uint8_t data[2];
I2C_readData(I2C1, 0x70, 2, data, 2);
range = (data[0] << 8) | data[1];
```

## Encoder-Based Speed Control (All Parts)

Both wheels use identical integral control:

```c
// Every 6 encoder events:
averageSpeed = sumOfLast6Periods / 6

if (averageSpeed > setpoint):
    currentPWM--  // Too fast, reduce PWM
else if (averageSpeed < setpoint):
    currentPWM++  // Too slow, increase PWM

Apply new PWM value
```

**Note**: Lower encoder period = faster speed (inverse relationship)

## Tuning Guides

### Parts A & B: Gesture Control

**Pitch/Roll sensitivity:**
- Modify the `/90.0` divisor to change sensitivity
- Smaller divisor = more sensitive (e.g., `/45.0`)
- Larger divisor = less sensitive (e.g., `/120.0`)

**PD Control (Part B):**
1. Start with `Kp = 0.001`, `Kd = 0`
2. Increase `Kp` until robot responds quickly but oscillates
3. Increase `Kd` until oscillation stops
4. Final values: `Kp = 0.03`, `Kd = 0.001`

### Part C: Autonomous Navigation

**Turn precision:**
- Verify `WHEEL_BASE = 149 mm` (measure your robot!)
- Adjust `turnDegrees` initial value for sharper/wider turns
- Modify turn increment (default 5°)

**Exploration pattern:**
- Change turns-per-direction (default: 2)
- Adjust initial `turnDirection` (default: 60°)

---

# Project Files

## PartA_B_GestureControl.c
Contains:
- CMPS12 compass reading
- Roll/pitch to speed conversion
- Heading-based PD control
- Mode switching logic
- Encoder-based heading calculation
- Collision avoidance

## PartC_AutonomousNav.c  
Contains:
- Ultrasonic ranging
- Turn execution with encoder feedback
- Intelligent turn strategy
- Speed control for straight driving
- Start button logic

---

# Course Information

- **Course**: ENGR-2350 Embedded Systems
- **Lab**: Lab 5 - Gesture RC Car & Autonomous Navigation
- **Authors**: Jun Iguchi
- **RIN**: 662107130
- **Institution**: Rensselaer Polytechnic Institute

## License

Educational use only - RPI ENGR-2350

## Acknowledgments

- TI-RSLK robot platform
- EmCon HAL library for MSPM0G3507  
- CMPS12 compass and SRF08 ranger documentation
- Course materials from RPI ENGR-2350
