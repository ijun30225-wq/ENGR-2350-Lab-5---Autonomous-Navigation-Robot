/*
Lab 5
*/

#include "engr2350_mspm0.h"
#include <math.h>


// Function Prototypes
void GPIOinit();
void timerInit();
void TIMG0_IRQHandler();

void turn(int16_t turnDegrees);

uint16_t readRanger();
void I2CInit();

// I2C Configuration
I2C_ControllerConfig i2cConfig;

Timers_TimerConfig tim0cfg; //Compare for PWM 
Timers_CaptureConfig tim0cap; //Capture for encoders

Timers_TimerConfig tim8cfg;
Timers_CompareConfig tim8cmp;

int32_t encCountsBetweenEdgesLeft = 0; //Count number of counts between edges for the left wheel
int32_t encCountsBetweenEdgesRight = 0; //Count number of counts between edges for the right wheel

uint32_t sumWheelSpeedMeasurementsLeft = 0; 
uint32_t numWheelSpeedMeasurementsLeft = 0;
uint32_t currentPWMLeft = 800; //Start with 20% duty cycle

uint32_t sumWheelSpeedMeasurementsRight = 0;
uint32_t numWheelSpeedMeasurementsRight = 0;
uint32_t currentPWMRight = 800; //Start with 20% duty cycle

uint32_t enc_total_events = 0; //Number of total encoder events (edges)
int32_t enc_counts_track = 0; //Number of total counts for these events
int32_t enc_counts = 0; //Number of total counts for these events
uint8_t enc_flag = 0;
//radius = 35mm
uint32_t SETPOINT = 80000;//25%a

uint16_t range = 1000; //Some default value for the ranger measurments

#define RANGE_LIMIT 25//Point at which ranger makes a decision
uint8_t straight_distance = 100; //Distance in cm to travel at a time
int8_t turnDirection = 60; //First turn should be a right turn.

int main() {
    //Local Variables

    sysInit();
    GPIOinit();
    timerInit();
    I2CInit();

    printf("SYSTEM INITIALIZED. Press BMP1 to Begin.       \n");
    while(((GPIO_readPins(GPIOA , GPIO_PIN7) & (1 << 7))) != 0){} //Wait for the start bumper to be pressed.
    printf("BMP1 pressed, proceeding.     \n");

    //Set initial PWM values for each wheel.
    Timers_setCCRValue(TIMG8, TIMER_CCR_CCR1, currentPWMLeft);
    Timers_setCCRValue(TIMG8, TIMER_CCR_CCR0, currentPWMRight);
   
    // Set forward direction
    GPIO_clearPins(GPIOB, GPIO_PIN16 | GPIO_PIN15); // Keep motors off
    GPIO_clearPins(GPIOB, GPIO_PIN0 | GPIO_PIN8); // Forward direction

    uint8_t turnsForThisDirection = 1; //First turn should happen only once in one direction.
    
    range = readRanger();
    delay_cycles(32000000); // Give ranger a moment.

    while(1){  
        range = readRanger();
        //drive straight until we hit a wall.
        GPIO_setPins(GPIOB, GPIO_PIN16 | GPIO_PIN15);
        GPIO_clearPins(GPIOB, GPIO_PIN0 | GPIO_PIN8);
        
        if (range < RANGE_LIMIT){ //Need to make a decision about a turn.
            //Stop while a turn is decided
            GPIO_clearPins(GPIOB, GPIO_PIN16 | GPIO_PIN15);
            if (turnsForThisDirection == 2){ //If weve already turned twice in one direction, start turning the other way.
                if(turnDirection < 0) turnDirection += 5;
                turnDirection = -turnDirection; //Turn the other way.
                turnsForThisDirection = 1; //New direction, reset the counts for the number of times we turned this direction.
                if (turnDirection < 0) turnDirection = turnDirection - 5;
            }
            else turnsForThisDirection++;

            delay_cycles(3200000); // Pause for a moment before turning.

            turn(turnDirection);
    
            delay_cycles(48000000); // Pause for a after before turning.
            if(turnDirection < 0) printf("\nTURNED LEFT          \r\n\n");
            if(turnDirection > 0) printf("\nTURNED RIGHT          \r\n\n");
        }


        printf("RANGE: %u | PWM_L: %u | PWM_R: %u | turnDirection: %d  %s              \r", range, currentPWMLeft, currentPWMRight, turnDirection, (turnDirection > 0 ? "RIGHT" : "LEFT") );
        range = readRanger();
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

    //BMP1
    GPIO_initDigitalInput(GPIOA, GPIO_PIN7);
    GPIO_setInternalResistor(GPIOA, GPIO_PIN7, GPIO_PULL_UP);
}

void timerInit() {
    //PWM TIMER CONFIG 
    tim8cfg.mode = TIMER_MODE_PERIODIC_UP;
    tim8cfg.clksrc = TIMER_CLOCK_BUSCLK;
    tim8cfg.clkdivratio = TIMER_CLOCK_DIVIDE_1;
    tim8cfg.period = 1000; //Nperiod = Tperiod / Ttimclk = (1/32000) / (1/32000000) = 32000000/32000 = 1000
    Timers_initTimer(TIMG8, &tim8cfg);

    //Configuring Timer Compare for PWM 
    tim8cmp.ccrn =  TIMER_CCR_CCR1 | TIMER_CCR_CCR0 ; //Left, Right
    tim8cmp.action = TIMER_CCR_ACTION_ZERO_CLEAR | TIMER_CCR_ACTION_UPCOMPARE_SET;
    tim8cmp.value = 1000 * 0.9; //90% of the way there, clear
    tim8cmp.invertOutput = 0;
    Timers_initCompare(TIMG8, &tim8cmp);

    //CAPTURE TIMER CONFIG FOR ENCODERS
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

    // Enable interrupts
    Timers_enableInterrupt(TIMG0, TIMER_INTSRC_ZERO);
    Timers_enableInterrupt(TIMG0, TIMER_INTSRC_CCR0_UP);
    Timers_enableInterrupt(TIMG0, TIMER_INTSRC_CCR1_UP);
   
    // Enable CPU interrupt for TIMG0
    NVIC_EnableIRQ(TIMG0_INT_IRQn);

    //Start timers
    Timers_startTimer(TIMG8);
    Timers_startTimer(TIMG0);
}

void I2CInit() {
    i2cConfig.busclkRate = 32000000;
    i2cConfig.bitRate = 100000;
    i2cConfig.addrMode = I2C_ADDR_MODE_7BIT;
    I2C_initController(I2C1, &i2cConfig);
}

void turn(int16_t turnDegrees) {
    GPIO_clearPins(GPIOB, GPIO_PIN16 | GPIO_PIN15);
    uint32_t turnPWM = 300;
    enc_total_events = 0;

    // Arc length for 1 degree turn = (π × wheelBase × 1) / 180
    float arcLengthPerDegree = (M_PI * 149) / 180.0;
   
    // Wheel circumference = 2 × π × radius (mm)
    float wheelCircumference = 2 * M_PI * 35;
   
    // Each encoder tick represents 1 degree of wheel rotation (360 ticks per revolution)
    // So distance per encoder tick = wheelCircumference / 360
    float mmPerEncoderTick = wheelCircumference / 360.0;
   
    // Encoder counts per degree of turning = arcLengthPerDegree / mmPerEncoderTick
    float encoderCountsPerDegree = arcLengthPerDegree / mmPerEncoderTick;

    uint32_t targetEncoderCounts = 0; //[WASNT DECLARED]

    if (turnDegrees > 0) {
         // Turn left
        GPIO_setPins(GPIOB, GPIO_PIN0);
        GPIO_clearPins(GPIOB, GPIO_PIN8);
       
        // Set PWM value
        Timers_setCCRValue(TIMG8, TIMER_CCR_CCR1, turnPWM);
        Timers_setCCRValue(TIMG8, TIMER_CCR_CCR0, 0);

        //calculate target counts
         targetEncoderCounts = turnDegrees * encoderCountsPerDegree;
       
    } else {
        // Turn right
        GPIO_clearPins(GPIOB, GPIO_PIN0);
        GPIO_setPins(GPIOB, GPIO_PIN8);
       
        // Set PWM value
        Timers_setCCRValue(TIMG8, TIMER_CCR_CCR1, 0);
        Timers_setCCRValue(TIMG8, TIMER_CCR_CCR0, turnPWM);

        //calculate target counts
         targetEncoderCounts = -turnDegrees * encoderCountsPerDegree;

    }
   
    // Enable motors
    GPIO_setPins(GPIOB, GPIO_PIN16 | GPIO_PIN15);
   
    // Wait until target encoder counts reached
    while (enc_total_events < targetEncoderCounts) {
        // Wait until target encoder counts reached

    }
   
    //stop motors
    GPIO_clearPins(GPIOB, GPIO_PIN16 | GPIO_PIN15);

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










// Interrupts
// Add interrupt functions last so they are easy to find
void TIMG0_IRQHandler() {
 
    // Check pending
    uint32_t pending = Timers_getPendingInterrupts(TIMG0);
   
    // Check if the timer reset in between encoder edges
    if (pending & TIMER_INTSRC_ZERO) {
        Timers_clearInterrupt(TIMG0, TIMER_INTSRC_ZERO);
        encCountsBetweenEdgesLeft += 65536;
        encCountsBetweenEdgesRight += 65536;
    }
   
    //Check if the left encoder caused an interrupt (rising edge)
    if (pending & TIMER_INTSRC_CCR0_UP) {
        Timers_clearInterrupt(TIMG0, TIMER_INTSRC_CCR0_UP);
       
        // Get the capture value
        uint16_t capture_value = Timers_getCCRValue(TIMG0, TIMER_CCR_CCR0);

        // Increment total encoder events
        enc_total_events++;

        // Calculate final counts between capture events
        int32_t enc_counts = encCountsBetweenEdgesLeft + capture_value;
       
        // Reset counts track with negative capture value
        encCountsBetweenEdgesLeft = -capture_value;

        //Controlling left wheel
        sumWheelSpeedMeasurementsLeft += enc_counts;
        numWheelSpeedMeasurementsLeft++;
       
       if (numWheelSpeedMeasurementsLeft == 6) {
            uint32_t average_speed = sumWheelSpeedMeasurementsLeft / 6;
           
            // Adjust PWM based on speed - TYPE 2 PWM (higher PWM = faster)
            if (average_speed > SETPOINT) {
                // Wheel is spinning too fast - decrease PWM
                if (currentPWMLeft > 100) {
                    currentPWMLeft--;
                }
            } else if (average_speed < SETPOINT) {
                // Wheel is spinning too slow - increase PWM
                if (currentPWMLeft < 900) {
                    currentPWMLeft++;
                }
            }
           
            // Apply new PWM value
            Timers_setCCRValue(TIMG8, TIMER_CCR_CCR1, currentPWMLeft);
           
            // Reset averaging variables
            sumWheelSpeedMeasurementsLeft = 0;
            numWheelSpeedMeasurementsLeft = 0;
        }

    }

    //Check if the right encoder caused an interrupt (rising edge)
    if (pending & TIMER_INTSRC_CCR1_UP) {
        Timers_clearInterrupt(TIMG0, TIMER_INTSRC_CCR1_UP);
       
        // Get the capture value
        uint16_t capture_value = Timers_getCCRValue(TIMG0, TIMER_CCR_CCR1);

       // Increment total encoder events
        enc_total_events++;

        // Calculate final counts between capture events
        int32_t enc_counts = encCountsBetweenEdgesRight + capture_value;
       
        // Reset counts track with negative capture value
        encCountsBetweenEdgesRight = -capture_value;

        //Controlling left wheel
        sumWheelSpeedMeasurementsRight += enc_counts;
        numWheelSpeedMeasurementsRight++;
       
       if (numWheelSpeedMeasurementsRight == 6) {
            uint32_t average_speed = sumWheelSpeedMeasurementsRight / 6;
           
            // Adjust PWM based on speed - TYPE 2 PWM (higher PWM = faster)
            if (average_speed > SETPOINT) {
                // Wheel is spinning too fast - decrease PWM
                if (currentPWMRight > 100) {
                    currentPWMRight--;
                }
            } else if (average_speed < SETPOINT) {
                // Wheel is spinning too slow - increase PWM
                if (currentPWMRight < 900) {
                    currentPWMRight++;
                }
            }
           
            // Apply new PWM value
            Timers_setCCRValue(TIMG8, TIMER_CCR_CCR0, currentPWMRight);
           
            // Reset averaging variables
            sumWheelSpeedMeasurementsRight = 0;
            numWheelSpeedMeasurementsRight = 0;
        }

    }
    uint16_t distance = ((2*M_PI*35)/360) * enc_total_events;
}
