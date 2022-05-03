
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
#define deg(x) x*(4096/360) + 2048 // 0 deg = upright pos // CW +, CCW -

//Mode Define
#define REST_MODE 0
#define RAPID_FIRE_MODE 1
#define SMOOTH_MODE 2
#define CLAY_MODE 3
#define COMPLIANT_MODE 4
uint8_t state = 0; // Operating State

RawSerial uart(D1, D0);
DigitalInOut RTS(D2);
volatile uint8_t waitForReceive = 0;
volatile uint8_t nextReload = 15;
uint8_t rx_buffer[LEN];

// Dynamixel parameters 
//uint8_t dxl_ID[] =  {1,2,3,4};
uint8_t dxl_ID[] =  {4};
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
//uint32_t initPos[] = {2048, 2048, 2048, 2048};
uint32_t homePos = deg(0);
uint32_t rightPos = deg(70);
uint32_t leftPos = deg(-70);

// Final Position
//uint32_t finPos[] = {2200, 2200, 2200, 2200};
uint32_t finPos = 2300;
uint32_t defaultPos = deg(0);

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

// Options Menu
void display_options(void) {
    pc.printf("\n\r\n\r\n\r");
    pc.printf("4DOF FINGER CONTROL v0.1 by HDK\n\r\n\r");
    pc.printf(" Commands:\n\r");
    wait_us(10);
    pc.printf(" h - Home Position\n\r");
    wait_us(10);
    pc.printf(" r - Rapid Fire\n\r");
    wait_us(10);
    pc.printf(" s - Smooth Motion\n\r");
    wait_us(10);
    pc.printf(" l - Clay Mode\n\r");
    wait_us(10);
    pc.printf(" c - Compliant\n\r");
    wait_us(10);
    pc.printf(" esc - Exit to Menu\n\r");
    wait_us(10);
}

// Interrupt 
void serial_interrupt(void){
    while(pc.readable()){
        char c = pc.getc();
        if(c == 27){
            state = REST_MODE;
        }
        else if(c == 104){
            state = REST_MODE;
        }
        else if(c == 114){
            state = RAPID_FIRE_MODE;
        }
        else if(c == 115){
            state = SMOOTH_MODE;
        }
        else if(c == 108){
            state = CLAY_MODE;
        }
        else if(c == 99){
            state = COMPLIANT_MODE;
        }
    }
}

// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {
    pc.attach(&serial_interrupt);
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
    for (int i=0; i<idLength; i++) {
        dxl_bus.SetTorqueEn(dxl_ID[i],0x00);    
        dxl_bus.SetRetDelTime(dxl_ID[i],0x32); // 4us delay time?
        dxl_bus.SetControlMode(dxl_ID[i], POSITION_CONTROL);
        wait_ms(100);    
        dxl_bus.TurnOnLED(dxl_ID[i], 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID[i],0x01); //to be able to move 
        wait_ms(100);
    }
        
    dxl_bus.SetVelocityProfile(dxl_ID[0], 200);
    dxl_bus.SetAccelerationProfile(dxl_ID[0], 20);
    pc.printf("Dynamixel Initialized...\n\r");
    led_mot = 1; // Nucleo LED shows that motors are enabled
    wait_ms(3000);
    display_options();
    // On every received message, run dynamixel control loop...eventually move this to an interrupt on received message
    while (true) {
        /*
        dxl_bus.SetGoalPosition(dxl_ID[0], homePos);
        wait_ms(100);
        pc.printf("%d\n\r", dxl_bus.GetPosition(dxl_ID[0]));

        dxl_bus.SetGoalPosition(dxl_ID[0], finPos);
        wait_ms(100);
        pc.printf("%d\n\r", dxl_bus.GetPosition(dxl_ID[0]));
        */

        if (state == RAPID_FIRE_MODE) {
        dxl_bus.SetVelocityProfile(4, 0); // 
        dxl_bus.SetAccelerationProfile(4, 0); //
        dxl_bus.SetGoalCurrent(4, 1193);
        dxl_bus.SetGoalPosition(4, deg(69));
        wait_ms(600);
        dxl_bus.SetGoalPosition(4, deg(-69));
        wait_ms(600);
        }

        else if (state == SMOOTH_MODE) {
        //dxl_bus.SetPosDGain(4,2000);
        //dxl_bus.SetPosDGain(4,600);
        dxl_bus.SetVelocityProfile(4, 200); // 
        dxl_bus.SetAccelerationProfile(4, 20); //
        //dxl_bus.SetGoalCurrent(4, 1193);
        dxl_bus.SetGoalPosition(4, rightPos);
        wait_ms(850);
        pc.printf("Current Position: %d \r\n",dxl_bus.GetPosition(4));
        dxl_bus.SetGoalPosition(4, leftPos);
        wait_ms(850);
        pc.printf("Current Position: %d \r\n",dxl_bus.GetPosition(4));
        }

        else if (state == COMPLIANT_MODE) {

        dxl_bus.SetGoalPosition(4, homePos);
        dxl_bus.SetGoalCurrent(4, 40);
        wait_ms(20);
        }

        else if (state == CLAY_MODE) {
        pc.printf("Previous Position: %d \r\n",dxl_bus.GetPosition(4));
        dxl_bus.SetGoalCurrent(4, 10);
        dxl_bus.SetGoalPosition(4, defaultPos);
        wait_ms(5);
        pc.printf("Current Position: %d \r\n",dxl_bus.GetPosition(4));
        defaultPos = dxl_bus.GetPosition(4);
        wait_ms(5);
        }

        else if (state == REST_MODE) {
        dxl_bus.SetGoalCurrent(4, 1193);
        dxl_bus.SetGoalPosition(4, homePos);
        wait_ms(800);
        }

        /**
        wait_us(10);
        range[0] = tof1.readRangeResult();
        wait_us(10);
        range_status[0] = tof1.readRangeStatus();
        **/
       
    }

}

