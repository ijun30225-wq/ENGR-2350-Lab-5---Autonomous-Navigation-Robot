# ENGR-2350 Lab 5 - Autonomous Navigation Robot

An autonomous navigation system for the TI-RSLK robot using ultrasonic ranging and encoder-based turning control.

## Project Overview

This lab implements an autonomous robot that navigates through an environment by:
- Detecting obstacles using an ultrasonic range sensor
- Making intelligent turning decisions to avoid collisions
- Using encoder feedback for precise turn angle control
- Alternating turn directions to explore the environment systematically

## Hardware Requirements

- TI-RSLK Robotic Car
- MSPM0G3507 microcontroller
- SRF08 Ultrasonic Ranger (I2C address: 0x70)
- CMPS12 Electronic Compass (optional, I2C capable)
- Rotary encoders on both wheels
- Push button (BMP1) for start trigger

## Features

### 1. Ultrasonic Collision Avoidance
- Continuously monitors distance to obstacles
- Triggers avoidance behavior when object detected within 25 cm
- Non-blocking range measurements

### 2. Intelligent Turn Strategy
- Alternates between left and right turns
- Performs 2 turns in one direction before switching
- Adjustable turn angle (default: 60°)
- Increases turn sharpness after 2 consecutive turns in same direction

### 3. Encoder-Based Turn Control
- Calculates precise turn angles using wheel encoders
- Accounts for wheel base geometry (149 mm)
- Uses arc length calculations for accurate rotations
- No compass required for basic operation

### 4. Dual-Wheel Speed Control
- Independent integral control for each wheel
- 6-sample averaging for stable speed measurements
- Automatic PWM adjustment to maintain target speed
- Prevents wheel speed drift during straight-line motion

## Control Specifications

| Parameter | Value | Description |
|-----------|-------|-------------|
| Range Limit | 25 cm | Obstacle detection threshold |
| Turn Angle | 60° | Default turn magnitude (±5° adjustment) |
| Wheel Base | 149 mm | Distance between wheel centers |
| Wheel Radius | 35 mm | Radius of drive wheels |
| PWM Period | 1000 counts | 32 kHz PWM frequency |
| Turn PWM | 300 | PWM value during turns (30% duty cycle) |
| Speed Setpoint | 80,000 counts | Target encoder period (≈25% speed) |
| Encoder Ticks | 360 per revolution | Encoder resolution |

## Pin Configuration

### I2C (Ultrasonic Ranger)
- **PA15**: I2C1 SDA (open-drain with pull-up)
- **PA16**: I2C1 SCL (open-drain with pull-up)
- **I2C Clock**: 100 kHz
- **SRF08 Address**: 0x70

### Motor Control
- **PB15**: Left motor enable
- **PB16**: Right motor enable
- **PB8**: Left motor direction
- **PB0**: Right motor direction

### PWM Outputs (32 kHz)
- **PB7 (TIMG8_CCR1)**: Left motor PWM
- **PB6 (TIMG8_CCR0)**: Right motor PWM

### Encoder Inputs
- **PB10 (TIMG0_CCR0)**: Left wheel encoder
- **PB11 (TIMG0_CCR1)**: Right wheel encoder
- **Edge Trigger**: Rising edge
- **Max Period**: 65,535 counts (overflow tracking enabled)

### User Input
- **PA7**: BMP1 button (start trigger, pull-up enabled)
- **PA18**: Mode selection switch (reserved, pull-up enabled)

## Operation Algorithm

### Startup Sequence
1. System initialization (GPIO, timers, I2C)
2. Wait for BMP1 button press
3. Set initial PWM values (80% duty cycle)
4. Configure forward direction
5. Take initial range measurement
6. Begin autonomous navigation

### Navigation Loop
```
While forever:
    1. Read ultrasonic range
    2. Drive forward (both motors enabled)
    
    3. If obstacle detected (range < 25 cm):
        a. Stop motors
        b. Check turn counter
        c. If 2 turns completed in current direction:
           - Reverse turn direction
           - Increase turn sharpness by 5°
           - Reset turn counter
        d. Pause 100ms
        e. Execute turn (calculated angle)
        f. Pause 1.5 seconds
        g. Increment turn counter
    
    4. Display status (range, PWM, turn direction)
    5. Update every 100ms
```

### Turn Execution Algorithm
```c
turn(int16_t turnDegrees):
    1. Calculate target encoder counts:
       arcLength = (π × wheelBase × |degrees|) / 180
       mmPerTick = (2π × wheelRadius) / 360
       targetCounts = arcLength / mmPerTick
    
    2. If turning right (degrees > 0):
       - Left wheel forward
       - Right wheel stopped
    
    3. If turning left (degrees < 0):
       - Left wheel stopped  
       - Right wheel forward
    
    4. Wait until encoder count reaches target
    5. Stop both motors
```

## Encoder-Based Speed Control

Each wheel uses independent integral control:

```c
Every 6 encoder pulses:
    averageSpeed = sumOfLastSixPeriods / 6
    
    if (averageSpeed > setpoint):
        // Too fast - reduce PWM
        currentPWM--
    else if (averageSpeed < setpoint):
        // Too slow - increase PWM
        currentPWM++
    
    Apply new PWM value
```

**Key Feature**: Lower encoder period = faster wheel speed, so control is inverted compared to typical speed controllers.

## Turn Geometry

The robot uses differential drive kinematics:

```
Arc Length = (π × wheelBase × θ) / 180°

For 60° turn with 149mm wheel base:
Arc Length = (π × 149 × 60) / 180 = 155.5 mm

Encoder Counts = Arc Length / mmPerTick
              = 155.5 / 0.611 ≈ 254 ticks
```

Where:
- `wheelBase = 149 mm` (measured between wheel centers)
- `wheelRadius = 35 mm`
- `mmPerTick = (2π × 35) / 360 = 0.611 mm`

## I2C Communication

### SRF08 Ultrasonic Ranger
```c
// Trigger new measurement
command = 0x51  // Range in cm
I2C_writeData(I2C1, 0x70, 0, &command, 1)

// Read previous result (after 80ms)
I2C_readData(I2C1, 0x70, 2, data, 2)
range = (data[0] << 8) | data[1]
```

**Important**: First reading after power-on is discarded (returns 0xFFFF)

## Navigation Behavior

### Turn Pattern Example
```
Initial: Right turn (60°)
Turn 1:  Right turn (60°)
Turn 2:  Right turn (60°) → Switch direction
Turn 3:  Left turn (-65°)  [increased by 5°]
Turn 4:  Left turn (-65°)
Turn 5:  Left turn (-65°) → Switch direction  
Turn 6:  Right turn (70°)  [increased by 5°]
...continues...
```

This creates a systematic exploration pattern that prevents the robot from getting stuck in corners or loops.

## Debug Output

Serial output (115200 baud) displays:
```
RANGE: 045 | PWM_L: 800 | PWM_R: 795 | turnDirection: 60 RIGHT
```

Status updates every 100ms showing:
- Current range measurement (cm)
- Left wheel PWM value
- Right wheel PWM value  
- Turn direction and angle

## Tuning Guide

### Speed Control
- Increase `SETPOINT` → slower robot
- Decrease `SETPOINT` → faster robot
- Adjust PWM limits (100-900) for min/max speed

### Turn Precision
- Verify `WHEEL_BASE` measurement (measure your robot!)
- Verify `wheelRadius` (35mm is typical for RSLK)
- Fine-tune `encoderCountsPerDegree` calculation if turns are consistently over/under

### Turn Strategy
- Modify `turnDirection` initial value (default: 60°)
- Adjust angle increment (default: ±5°)
- Change turn counter threshold (default: 2 turns before switch)

### Range Detection
- Modify `RANGE_LIMIT` (default: 25 cm)
- Increase for more cautious navigation
- Decrease for tighter spaces

## Common Issues & Solutions

**Issue**: Robot turns too far or not far enough
- **Solution**: Verify wheel base measurement, check encoder connections

**Issue**: Robot doesn't detect obstacles
- **Solution**: Check I2C connections, verify SRF08 address (0x70), ensure 80ms delay between readings

**Issue**: One wheel significantly faster than other
- **Solution**: Check encoder connections, verify both speed controllers are active, ensure motors are enabled

**Issue**: Robot gets stuck in corners
- **Solution**: Increase turn angle, reduce number of turns before direction switch

**Issue**: Encoder overflow (counts > 65535)
- **Solution**: Already handled! Overflow tracking adds 65536 on timer reset

## Project Structure

```
Lab5/
├── main.c              # Complete navigation code
├── engr2350_mspm0.h    # Hardware abstraction library  
├── README.md           # This file
└── .gitignore          # Git ignore file
```

## Future Enhancements

Possible improvements for this design:
- Add compass-based heading control (CMPS12)
- Implement mapping of explored area
- Add multiple ultrasonic sensors for 360° awareness
- Optimize turn angles based on detected gap width
- Add "stuck detection" using encoder monitoring
- Implement return-to-start functionality

## Course Information

- **Course**: ENGR-2350 Embedded Systems
- **Lab**: Lab 5 - Autonomous Navigation Robot
- **Authors**: Cori DeBeatham, Jun Iguchi
- **RIN**: 662061765
- **Institution**: Rensselaer Polytechnic Institute

## License

Educational use only - RPI ENGR-2350

## Acknowledgments

- TI-RSLK robot platform
- EmCon HAL library for MSPM0G3507
- SRF08 ultrasonic ranger documentation
- Course materials from RPI ENGR-2350
