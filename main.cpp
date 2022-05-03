
#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"
#include "VL6180X.h"
#include <cstdint>


// Initialize stuff for dynamixels
#define M_PI 3.141592653  /* pi */
#define WAIT_TIME_MS 1
#define LEN 100 // should be 34? = 5*7-1
#define wait_ms(x) wait_us(x*1000)
RawSerial uart(D1, D0);
DigitalInOut RTS(D2);
volatile uint8_t waitForReceive = 0;
volatile uint8_t nextReload = 15;
uint8_t rx_buffer[LEN];

// Dynamixel parameters 
//uint8_t dxl_ID[] =  {1,2,3,4};
//uint8_t idLength = sizeof(dxl_ID) / sizeof(uint8_t);
uint8_t dxl_ID = 1;

// Debugging LEDs
DigitalOut led_pwr(LED1);
DigitalOut led_mot(LED2);
DigitalOut led_com(LED3);

// Initialize serial port
RawSerial pc(USBTX, USBRX, 921600);
Timer t;
int loop_time;

// Initial Position
//uint32_t initPos[] = {2048, 2048, 2048, 2048};
uint32_t initPos = 2048;

// Final Position
//uint32_t finPos[] = {2200, 2200, 2200, 2200};
uint32_t finPos = 2300;

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
// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {
    
    // Board has power, main loop is running
    led_pwr = 1;
    led_mot = 0;
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
    wait_ms(300);
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RTS.mode(OpenDrainNoPull);
    RTS.output();
    uart.baud(1000000);
    RTS = 0;
    wait_ms(500); // wait(1); // does this need to be 1 second?
    
    XM430_bus dxl_bus(1000000, D1, D0, D2); // baud, tx, rx, rts
   
    // Enable dynamixels and set to position mode
    
        dxl_bus.SetTorqueEn(dxl_ID,0x00);    
        dxl_bus.SetRetDelTime(dxl_ID,0x32); // 4us delay time?
        dxl_bus.SetControlMode(dxl_ID, POSITION_CONTROL);
        wait_ms(100);    
        dxl_bus.TurnOnLED(dxl_ID, 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID,0x01); //to be able to move 
        wait_ms(100);
    
    
    pc.printf("initialize\n\r");
    
    led_mot = 1; // Nucleo LED shows that motors are enabled

    // On every received message, run dynamixel control loop...eventually move this to an interrupt on received message
    while (true) {
        dxl_bus.SetGoalPosition(dxl_ID, initPos);
        wait_ms(1000);
        pc.printf("%d\n\r", dxl_bus.GetPosition(dxl_ID));

        dxl_bus.SetGoalPosition(dxl_ID, finPos);
        wait_ms(1000);
        pc.printf("%d\n\r", dxl_bus.GetPosition(dxl_ID));
        
        /**
        wait_us(10);
        range[0] = tof1.readRangeResult();
        wait_us(10);
        range_status[0] = tof1.readRangeStatus();
        **/
       
    }

}

