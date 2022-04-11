
#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"


// Initialize stuff for dynamixels
#define M_PI 3.141592653  /* pi */
#define WAIT_TIME_MS 1
#define LEN 100 // should be 34? = 5*7-1
RawSerial uart(D1, D0);
DigitalInOut RTS(D2);
volatile uint8_t waitForReceive = 0;
volatile uint8_t nextReload = 15;
uint8_t rx_buffer[LEN];

// Dynamixel parameters 
uint8_t dxl_ID[] =  {1,2,3,4};
int8_t idLength = sizeof(dxl_ID) / sizeof(uint8_t);


// Debugging LEDs
DigitalOut led_pwr(LED1);
DigitalOut led_mot(LED2);
DigitalOut led_com(LED3);

// Initialize serial port
Serial pc(USBTX, USBRX, 9600);
Timer t;
int loop_time;

// Initial Position
uint32_t initPos[] = {128, 128, 128, 128};

// Final Position
uint32_t finPos[] = {3000, 3000, 3000, 3000};

// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {
    

    // Board has power, main loop is running
    led_pwr = 1;
    led_mot = 0;

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
    for(int i=0; i<4; i++){
       
        dxl_bus.SetTorqueEn(dxl_ID[i],0x00);    
        dxl_bus.SetRetDelTime(dxl_ID[i],0x32); // 4us delay time?
        dxl_bus.SetControlMode(dxl_ID[i], POSITION_CONTROL);
        wait_ms(100);    
        dxl_bus.TurnOnLED(dxl_ID[i], 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID[i],0x01); //to be able to move 
        wait_ms(100);
    } 
    
    pc.printf("initialize\n\r");
    
    led_mot = 1; // Nucleo LED shows that motors are enabled

    // On every received message, run dynamixel control loop...eventually move this to an interrupt on received message
    while (true) {
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, initPos);
        wait_ms(1000);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, finPos);
        wait_ms(1000);
    }

}

