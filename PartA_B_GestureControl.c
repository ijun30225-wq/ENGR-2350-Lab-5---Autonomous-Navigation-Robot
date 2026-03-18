////////////////////////////////////////////////////////////////////////
//** ENGR-2350 Lab 5: Gesture RC Car
//** NAME: XXXX
//** RIN: XXXX
//** This lab implements gesture-based control using CMPS12 compass sensor
//** Part A: Roll/Pitch control with ultrasonic collision avoidance
//** Part B: Heading-based PD control with pitch speed control
////////////////////////////////////////////////////////////////////////

#include "engr2350_mspm0.h"
#include <math.h>
#include <stdlib.h>

// Function Prototypes
void GPIOinit();
void I2CInit();
void timerInit();
void TIMG0_IRQHandler();
void readCompassData();
uint16_t readRanger();
void calculateDesiredSpeeds(int8_t pitch, int8_t roll, uint16_t heading,
                           uint16_t range, bool useHeadingControl);
float wrapAngle(float error);

// I2C Configuration
I2C_ControllerConfig i2cConfig;

// Timer configurations
Timers_TimerConfig tim0cfg;
Timers_CaptureConfig tim0cap;
Timers_TimerConfig tim8cfg;
Timers_CompareConfig tim8cmp;

// Encoder tracking variables
volatile int32_t encCountsBetweenEdgesLeft = 0;
volatile int32_t encCountsBetweenEdgesRight = 0;

// Wheel speed control (Integral control from Lab 4)
volatile uint32_t sumWheelSpeedMeasurementsLeft = 0;
volatile uint32_t numWheelSpeedMeasurementsLeft = 0;
volatile uint32_t currentPWMLeft = 500;

volatile uint32_t sumWheelSpeedMeasurementsRight = 0;
volatile uint32_t numWheelSpeedMeasurementsRight = 0;
volatile uint32_t currentPWMRight = 500;

uint32_t SETPOINT = 80000; 

// Speed and control variables
float desired_speed = 0;        // Vd in duty cycle (0-1.0)
float desired_diff_speed = 0;   // Vdiff in duty cycle
float desired_left_speed = 0;   // VL
float desired_right_speed = 0;  // VR

#define MAX_SPEED 0.9
#define MIN_SPEED 0.15 
#define MOTOR_PWM_PERIOD 1000   // Timer period for PWM

// PD Control variables for heading control (Part B)
float heading_error = 0;
float heading_error_prev = 0;
float Kp_heading = 0.03;
float Kd_heading = 0.001; 

// Encoder-based heading tracking for Part B
volatile int32_t enc_counts_left_segment = 0;    // Counts for current segment
volatile int32_t enc_counts_right_segment = 0;   // Counts for current segment
float measured_heading = 0;                      // Accumulated relative heading (degrees)
#define WHEEL_BASE 149.0
#define MM_PER_COUNT 0.611

// Variables for sensor readings
int8_t pitch = 0, roll = 0;
uint16_t heading = 0, range = 0;

// Mode selection
bool useHeadingControl = false;

// Motor direction tracking (for encoder direction)
volatile uint8_t motor_dir_left = 0;   // 0 = forward, 1 = reverse
volatile uint8_t motor_dir_right = 0;  // 0 = forward, 1 = reverse

int main() {
    sysInit();
    GPIOinit();
    I2CInit();
    timerInit();
    
    // Initialize motors off
    GPIO_clearPins(GPIOB, GPIO_PIN16 | GPIO_PIN15);
    GPIO_clearPins(GPIOB, GPIO_PIN0 | GPIO_PIN8); // Forward direction default
    
    printf("Lab 5: Gesture RC Car Initialized\r\n");
    printf("Calibrate compass by rotating hand-held breadboard...\r\n\r\n");
    delay_cycles(3.2e6);
    
    while(1) {
        // Read slide switch to determine control mode
        // Switch LOW (grounded) = Part A (roll control)
        // Switch HIGH (pulled up) = Part B (heading control)
        useHeadingControl = (GPIO_readPins(GPIOA, GPIO_PIN18) & GPIO_PIN18) != 0;
        
        // Read compass data
        readCompassData();
        
        // Read ultrasonic ranger
        range = readRanger();
        
        // Calculate delta_theta for this segment before resetting counters
        float dist_left_segment = enc_counts_left_segment * MM_PER_COUNT;
        float dist_right_segment = enc_counts_right_segment * MM_PER_COUNT;
        float delta_theta_rad = (dist_right_segment - dist_left_segment) / WHEEL_BASE;
        float delta_theta_deg = delta_theta_rad * (180.0 / M_PI);
        
        // Accumulate to measured heading
        measured_heading += delta_theta_deg;
        
        // Keep heading in 0-360 range
        while (measured_heading >= 360.0) measured_heading -= 360.0;
        while (measured_heading < 0.0) measured_heading += 360.0;
        
        // Reset segment counters for next iteration
        enc_counts_left_segment = 0;
        enc_counts_right_segment = 0;
        
        // Calculate desired wheel speeds based on mode
        calculateDesiredSpeeds(pitch, roll, heading, range, useHeadingControl);
        
        // Apply speeds to motors
        if (desired_left_speed > 0) {
            GPIO_clearPins(GPIOB, GPIO_PIN0); // Forward
            GPIO_setPins(GPIOB, GPIO_PIN15);  // Enable
            motor_dir_left = 0;
            currentPWMLeft = MOTOR_PWM_PERIOD - (uint32_t)(desired_left_speed * MOTOR_PWM_PERIOD);
        } else if (desired_left_speed < 0) {
            GPIO_setPins(GPIOB, GPIO_PIN0);   // Reverse
            GPIO_setPins(GPIOB, GPIO_PIN15);  // Enable
            motor_dir_left = 1;
            currentPWMLeft = MOTOR_PWM_PERIOD - (uint32_t)(-desired_left_speed * MOTOR_PWM_PERIOD);
        } else {
            GPIO_clearPins(GPIOB, GPIO_PIN15); // Disable
            currentPWMLeft = 0;
        }
        
        if (desired_right_speed > 0) {
            GPIO_clearPins(GPIOB, GPIO_PIN8);  // Forward
            GPIO_setPins(GPIOB, GPIO_PIN16);   // Enable
            motor_dir_right = 0;
            currentPWMRight = MOTOR_PWM_PERIOD - (uint32_t)(desired_right_speed * MOTOR_PWM_PERIOD);
        } else if (desired_right_speed < 0) {
            GPIO_setPins(GPIOB, GPIO_PIN8);    // Reverse
            GPIO_setPins(GPIOB, GPIO_PIN16);   // Enable
            motor_dir_right = 1;
            currentPWMRight = MOTOR_PWM_PERIOD - (uint32_t)(-desired_right_speed * MOTOR_PWM_PERIOD);
        } else {
            GPIO_clearPins(GPIOB, GPIO_PIN16); // Disable
            currentPWMRight = 0;
        }
        
        Timers_setCCRValue(TIMG8, TIMER_CCR_CCR1, currentPWMLeft);
        Timers_setCCRValue(TIMG8, TIMER_CCR_CCR0, currentPWMRight);
        
        // Debug output with mode indication
        printf("Mode:%s | P:%3d | R:%3d | H:%4u | MH:%.1f | dT:%.2f | Rng:%3u | Vd:%.2f | Vdiff:%.2f | DesSpR:%.2f | DesSpL:%.2f\r\n",
               useHeadingControl ? "HEAD" : "ROLL",
               pitch, roll, heading, measured_heading, delta_theta_deg, range, 
               desired_speed, desired_diff_speed, desired_right_speed, desired_left_speed);
        
        delay_cycles(3200000); // 100ms update rate
    }
}

void GPIOinit() {
    // Motor control pins
    GPIO_initDigitalOutput(GPIOB, GPIO_PIN0 | GPIO_PIN8 | GPIO_PIN16 | GPIO_PIN15);
    
    // Mode selection switch on PA18
    GPIO_initDigitalInput(GPIOA, GPIO_PIN18);
    GPIO_setInternalResistor(GPIOA, GPIO_PIN18, GPIO_PULL_UP);
    
    // I2C pins
    GPIO_initPeripheralFunction(GPIOA, GPIO_PIN15 | GPIO_PIN16, 4);
    GPIO_setOpenDrain(GPIOA, GPIO_PIN15 | GPIO_PIN16);
    
    // PWM pins
    GPIO_initPeripheralFunction(GPIOB, GPIO_PIN6 | GPIO_PIN7, 5);
    
    // Encoder pins
    GPIO_initPeripheralFunction(GPIOB, GPIO_PIN10 | GPIO_PIN11, 2);
}

void I2CInit() {
    i2cConfig.busclkRate = 32000000;
    i2cConfig.bitRate = 100000;
    i2cConfig.addrMode = I2C_ADDR_MODE_7BIT;
    I2C_initController(I2C1, &i2cConfig);
}

void timerInit() {
    // PWM Timer (TIMG8)
    tim8cfg.mode = TIMER_MODE_PERIODIC_UP;
    tim8cfg.clksrc = TIMER_CLOCK_BUSCLK;
    tim8cfg.clkdivratio = TIMER_CLOCK_DIVIDE_1;
    tim8cfg.period = MOTOR_PWM_PERIOD;
    Timers_initTimer(TIMG8, &tim8cfg);
    
    tim8cmp.ccrn = TIMER_CCR_CCR1 | TIMER_CCR_CCR0;
    tim8cmp.action = TIMER_CCR_ACTION_ZERO_CLEAR | TIMER_CCR_ACTION_UPCOMPARE_SET;
    tim8cmp.value = 0;
    tim8cmp.invertOutput = 0;
    Timers_initCompare(TIMG8, &tim8cmp);
    
    // Encoder Timer (TIMG0)
    tim0cfg.mode = TIMER_MODE_PERIODIC_UP;
    tim0cfg.clksrc = TIMER_CLOCK_BUSCLK;
    tim0cfg.clkdivratio = TIMER_CLOCK_DIVIDE_1;
    tim0cfg.clkprescale = 0;
    tim0cfg.period = 65535;
    Timers_initTimer(TIMG0, &tim0cfg);
    
    tim0cap.ccrn = TIMER_CCR_CCR0 | TIMER_CCR_CCR1;
    tim0cap.inputSel = TIMER_CCR_INPUT_CCPn;
    tim0cap.edge = TIMER_CCR_EDGE_RISE;
    tim0cap.invertInput = 0;
    Timers_initCapture(TIMG0, &tim0cap);
    
    Timers_enableInterrupt(TIMG0, TIMER_INTSRC_ZERO);
    Timers_enableInterrupt(TIMG0, TIMER_INTSRC_CCR0_UP);
    Timers_enableInterrupt(TIMG0, TIMER_INTSRC_CCR1_UP);
    
    NVIC_EnableIRQ(TIMG0_INT_IRQn);
    
    Timers_startTimer(TIMG8);
    Timers_startTimer(TIMG0);
}

void readCompassData() {
    uint8_t data[4];
    
    // Read 4 bytes starting from register 2: heading (2 bytes), pitch, roll
    I2C_readData(I2C1, 0x60, 2, data, 4);
    
    heading = (data[0] << 8) | data[1]; // Tenths of degrees (0-3599)
    pitch = (int8_t)data[2];            // Signed degrees
    roll = (int8_t)data[3];             // Signed degrees
}

uint16_t readRanger() {
    static uint8_t firstRead = 1;
    uint8_t data[2];
    
    I2C_readData(I2C1, 0x70, 2, data, 2);
    uint16_t distance = (data[0] << 8) | data[1];
    
    uint8_t command = 0x51;
    I2C_writeData(I2C1, 0x70, 0, &command, 1);
    
    if (firstRead) {
        firstRead = 0;
        return 0xFFFF;
    }
    
    return distance;
}

void calculateDesiredSpeeds(int8_t pitch, int8_t roll, uint16_t heading,
                           uint16_t range, bool useHeadingControl) {
    // Convert pitch to desired speed
    // Pitch > 0: tilting forward -> speed up
    // Pitch < 0: tilting backward -> slow down/reverse
    desired_speed = (pitch / 90.0) * MAX_SPEED;
    
    // Apply dead zone for pitch
    if (fabs(pitch) < 5) {
        desired_speed = 0;
    }
    
    // Clamp speed
    if (fabs(desired_speed) < MIN_SPEED && desired_speed != 0) {
        desired_speed = (desired_speed > 0) ? MIN_SPEED : -MIN_SPEED;
    }
    if (desired_speed > MAX_SPEED) desired_speed = MAX_SPEED;
    if (desired_speed < -MAX_SPEED) desired_speed = -MAX_SPEED;
    
    // Collision avoidance: stop if object within 25 cm and moving forward
    if (range < 25 && range != 0xFFFF && desired_speed > 0) {
        desired_speed = 0;
    }
    
    if (!useHeadingControl) {
        // Part A: Roll-based turning
        if (fabs(roll) < 5) {
            // Dead zone: drive straight
            desired_diff_speed = 0;
        } else {
            // Map roll to differential speed
            // Roll > 0 (tilt right) -> turn right -> positive diff speed
            desired_diff_speed = (roll / 90.0) * MAX_SPEED;
            
            // Clamp differential speed
            if (desired_diff_speed > MAX_SPEED) desired_diff_speed = MAX_SPEED;
            if (desired_diff_speed < -MAX_SPEED) desired_diff_speed = -MAX_SPEED;
        }
    } else {
        // Part B: Heading-based PD control
        float desired_heading = heading / 10.0; // Convert tenths to degrees
        
        // Calculate heading error with wrapping
        heading_error = desired_heading - measured_heading;
        heading_error = wrapAngle(heading_error);
        
        // Calculate derivative
        float heading_error_derivative = heading_error - heading_error_prev;
        
        // PD control for differential speed
        desired_diff_speed = Kp_heading * heading_error + 
                            Kd_heading * heading_error_derivative;
        
        // Clamp differential speed
        if (desired_diff_speed > MAX_SPEED) desired_diff_speed = MAX_SPEED;
        if (desired_diff_speed < -MAX_SPEED) desired_diff_speed = -MAX_SPEED;
        
        heading_error_prev = heading_error;
    }
    
    // Calculate individual wheel speeds
    //if(desired_speed == 0) {
        // Forward motion: normal differential
        desired_left_speed  = desired_speed - desired_diff_speed;
        desired_right_speed = desired_speed + desired_diff_speed;
    
    /*} else {
        desired_left_speed  = desired_speed + desired_diff_speed;
        desired_right_speed = desired_speed - desired_diff_speed;

    }*/
    
    
    // Clamp wheel speeds
    if (desired_left_speed > MAX_SPEED) desired_left_speed = MAX_SPEED;
    if (desired_left_speed < -MAX_SPEED) desired_left_speed = -MAX_SPEED;
    if (desired_right_speed > MAX_SPEED) desired_right_speed = MAX_SPEED;
    if (desired_right_speed < -MAX_SPEED) desired_right_speed = -MAX_SPEED;
}

float wrapAngle(float error) {
    // Wrap angle to -180 to +180 range
    while (error > 180.0) {
        error -= 360.0;
    }
    while (error < -180.0) {
        error += 360.0;
    }
    return error;
}

void TIMG0_IRQHandler() {
    uint32_t pending = Timers_getPendingInterrupts(TIMG0);
    
    // Timer overflow
    if (pending & TIMER_INTSRC_ZERO) {
        Timers_clearInterrupt(TIMG0, TIMER_INTSRC_ZERO);
        encCountsBetweenEdgesLeft += 65536;
        encCountsBetweenEdgesRight += 65536;
    }
    
    // Left encoder event
    if (pending & TIMER_INTSRC_CCR0_UP) {
        Timers_clearInterrupt(TIMG0, TIMER_INTSRC_CCR0_UP);
        
        uint16_t capture_value = Timers_getCCRValue(TIMG0, TIMER_CCR_CCR0);
        int32_t enc_counts = encCountsBetweenEdgesLeft + capture_value;
        encCountsBetweenEdgesLeft = -capture_value;
        
        // Track encoder direction based on motor direction
        // Forward = increment, Reverse = decrement
        if (motor_dir_left == 0) {
            // Forward
            enc_counts_left_segment++;
        } else {
            // Reverse
            enc_counts_left_segment--;
        }
        
        // Wheel speed control (Integral from Lab 4)
        // Only adjust PWM if motor is actually running
        if (currentPWMLeft > 0) {
            sumWheelSpeedMeasurementsLeft += enc_counts;
            numWheelSpeedMeasurementsLeft++;
            
            if (numWheelSpeedMeasurementsLeft == 6) {
                uint32_t average_speed = sumWheelSpeedMeasurementsLeft / 6;
                
                if (average_speed > SETPOINT && currentPWMLeft > 100) {
                    currentPWMLeft--;
                } else if (average_speed < SETPOINT && currentPWMLeft < 900) {
                    currentPWMLeft++;
                }
                
                sumWheelSpeedMeasurementsLeft = 0;
                numWheelSpeedMeasurementsLeft = 0;
            }
        }
    }
    
    // Right encoder event
    if (pending & TIMER_INTSRC_CCR1_UP) {
        Timers_clearInterrupt(TIMG0, TIMER_INTSRC_CCR1_UP);
        
        uint16_t capture_value = Timers_getCCRValue(TIMG0, TIMER_CCR_CCR1);
        int32_t enc_counts = encCountsBetweenEdgesRight + capture_value;
        encCountsBetweenEdgesRight = -capture_value;
        
        // Track encoder direction based on motor direction
        if (motor_dir_right == 0) {
            // Forward
            enc_counts_right_segment++;
        } else {
            // Reverse
            enc_counts_right_segment--;
        }
        
        // Wheel speed control (Integral from Lab 4)
        if (currentPWMRight > 0) {
            sumWheelSpeedMeasurementsRight += enc_counts;
            numWheelSpeedMeasurementsRight++;
            
            if (numWheelSpeedMeasurementsRight == 6) {
                uint32_t average_speed = sumWheelSpeedMeasurementsRight / 6;
                
                if (average_speed > SETPOINT && currentPWMRight > 100) {
                    currentPWMRight--;
                } else if (average_speed < SETPOINT && currentPWMRight < 900) {
                    currentPWMRight++;
                }
                
                sumWheelSpeedMeasurementsRight = 0;
                numWheelSpeedMeasurementsRight = 0;
            }
        }
    }
}