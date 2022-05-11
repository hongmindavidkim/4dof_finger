#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"
#include "VL6180X.h"
#include <cstdint>
#include "actuator_transformation.h"

// Initialize stuff for dynamixels
#define WAIT_TIME_MS 1
#define LEN 100 
#define wait_ms(x) wait_us(x*1000)
#define rad2pulse_t(x) uint32_t(rad2pulse(x))

bool state_change = false;
bool servo_on = false;

RawSerial uart(D1, D0);
DigitalInOut RTS(D2);
volatile uint8_t waitForReceive = 0;
volatile uint8_t nextReload = 15;
uint8_t rx_buffer[LEN];

uint8_t dxl_ID[] =  {1,2,3,4};
int8_t idLength = sizeof(dxl_ID) / sizeof(uint8_t);

// Debugging LEDs
DigitalOut led_pwr(LED1);
DigitalOut led_mot(LED2);
DigitalOut led_com(LED3);

// Initialize serial port
RawSerial pc(USBTX, USBRX, 921600);
Timer t;
int loop_time;

// Initial Positions

uint32_t multiHomePos[] = {rad2pulse_t(0),rad2pulse_t(0),rad2pulse_t(0),rad2pulse_t(0)};
uint16_t multiGoalCurrent[] = {1193,1193,1193,1193};

uint32_t* tempPos1 = ActuatorTransformation(PI/6, -PI/4, -PI/4, PI/4);
uint32_t* tempPos2 = ActuatorTransformation(-PI/6, PI/4, PI/4, -PI/4);

int32_t dxl_position[4];
int32_t dxl_velocity[4];
int16_t dxl_current[4];
float desired_current[4];
uint16_t current_command[4];

float currentPos[4];
float currentVel[4];
float currentCur[4];

float KdJ = 0.1f; // bonus joint damping
float Kt = 3.0f/2.3f; // effective torque constant, Nm/A
float Kt_inv = 1.0f/Kt;
float pulse_to_rad = (2.0f*3.14159f)/4096.0f; // = 0.001534
float rpm_to_rads = (0.229f*2.0f*3.14159f)/60.0f; // = 0.0239
float current_limit = 2.30f; // in A, max is 3.2(?)
float torque_limit = 0.75f; // in Nm, stall is 3.0Nm at 12V

// ToF sensor i2c busses
/**
I2C i2c1(PF_0, PF_1); //(PB_9, PB_8); // SDA, SCL
// initialize sensors
VL6180X tof1; // right finger inner

int range[1];
float range_m[1]; // range in m
int range_status[1];
uint16_t range_period = 30;
**/

// Tickers for motor commands and data logging
Ticker motor_cmd;
Ticker motor_data;
volatile int motor_cmd_flag = 0;
volatile int motor_cmd_pos = 0;
volatile int motor_data_flag = 0;
void send_motor_cmd(){
    motor_cmd_flag = 1;
}
void get_motor_data(){
    motor_data_flag = 1;
}


// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {
    
    /**
    // ToF Setup
    i2c1.frequency(400000);
    pc.printf("Sensor 1...\n\r");
    wait_us(100);
    if(!tof1.begin(&i2c1)){
        pc.printf("Sensor 1 init failed.\n\r");
    }
    wait_us(100);
    tof1.stopRangeContinuous();
    wait_us(100);
    tof1.startRangeContinuous(range_period);
    wait_us(1000);
    **/

    // Set up dynamixel
    // wait_ms(300);
    // RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    // RTS.mode(OpenDrainNoPull);
    // RTS.output();
    // uart.baud(1000000);
    // RTS = 0;
    // wait_ms(500);
    

    pc.printf("Initializing Dynamixels.\n\r");

    XM430_bus dxl_bus(1000000, D1, D0, D2); // baud, tx, rx, rts

    for (int i=0; i<idLength; i++) {
        dxl_bus.SetTorqueEn(dxl_ID[i],0x00);    
        dxl_bus.SetRetDelTime(dxl_ID[i],0x32); // TODO: make this shorter?
        dxl_bus.SetControlMode(dxl_ID[i], POSITION_CONTROL);
        // dxl_bus.SetControlMode(dxl_ID[i], CURRENT_CONTROL);
        wait_ms(100);    
        dxl_bus.TurnOnLED(dxl_ID[i], 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID[i],0x01); // Enable motors
        wait_ms(100);
    }

    for (int i=0; i<idLength; i++) {
        dxl_bus.SetVelocityProfile(dxl_ID[i], 414); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID[i], 80); // 17166 rev/min^2
    }

    pc.printf("Setting home position.\n\r");

    dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos); 
    wait_ms(2000);

    pc.printf("Starting...\n\r");

    // Attach interrupts
    motor_cmd.attach_us(&send_motor_cmd, 500000); // send every 0.5 seconds
    motor_data.attach_us(&get_motor_data, 5000); // get data every 5ms (200Hz)
    // Start timer
    t.reset();
    t.start();

    // On every received message, run dynamixel control loop...eventually move this to an interrupt on received message
    while (true) {

        if (motor_cmd_flag==1){
            motor_cmd_flag = 0;
            if (motor_cmd_pos==1){
                dxl_bus.SetMultGoalPositions(dxl_ID, idLength, tempPos1);
                motor_cmd_pos = 0;
            } else if (motor_cmd_pos==0){
                dxl_bus.SetMultGoalPositions(dxl_ID, idLength, tempPos2);
                motor_cmd_pos = 1;
            }
        }

        if (motor_data_flag==1){
            motor_data_flag = 0;
            // get data
            dxl_bus.GetMultPositions(dxl_position, dxl_ID, idLength);
            dxl_bus.GetMultVelocities(dxl_velocity, dxl_ID, idLength);
            dxl_bus.GetMultCurrents(dxl_current, dxl_ID, idLength);
            // convert states
            for(int i=0; i<idLength; i++){
                currentPos[i] = (pulse_to_rad*(float)dxl_position[i]);
                currentVel[i] = rpm_to_rads*(float)dxl_velocity[i];
                currentCur[i] = 0.001f*(float)dxl_current[i];
            }
            // print data
            pc.printf("%.3f, %.3f, %.3f, %.3f, %.3f\n\r", t.read(), currentPos[0], currentPos[1], currentPos[2], currentPos[3]);
        }

        /**
        wait_us(10);
        range[0] = tof1.readRangeResult();
        wait_us(10);
        range_status[0] = tof1.readRangeStatus();
        **/
    }

}

