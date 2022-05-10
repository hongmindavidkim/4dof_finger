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
#define LEN 100 // should be 34? = 5*7-1
#define wait_ms(x) wait_us(x*1000)
#define rad2pulse_t(x) uint32_t(rad2pulse(x))

//Mode Define
#define REST_MODE 0
#define RAPID_FIRE_MODE 1
#define SMOOTH_MODE 2
#define TEACHING_MODE 3
#define IMPEDANCE_CONTROL_MODE 4
#define SERVO_ON 5
#define SERVO_OFF 6
#define SWITCH_TO_CURRENT_CONTROL 7

uint8_t state = 0; // Operating State
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
//uint32_t initPos[] = {2048, 2048, 2048, 2048};
uint32_t homePos = rad2pulse_t(0);
uint32_t rightPos = rad2pulse_t(70);
uint32_t leftPos = rad2pulse_t(-70);
uint32_t multiHomePos[] = {rad2pulse_t(0),rad2pulse_t(0),rad2pulse_t(0),rad2pulse_t(0)};
uint16_t multiGoalCurrent[] = {1193,1193,1193,1193};
uint16_t multiGoalCurrentImp[] = {30,30,30,40};

uint8_t first_enter = 0;

uint32_t RF_POS_1[] = {1150,3600,800,2048};
uint32_t RF_POS_2[] = {2890,1387,2411,2048};

uint32_t RF_POS_3[] = {582,3163,278,1450};
uint32_t RF_POS_4[] = {2031,4047,927,2748};

uint32_t RT_POS[] = {1245,2776,841,1623};

//uint32_t TEMP_POS1[] = {rad2pulse(0),rad2pulse(0),rad2pulse(0),rad2pulse(63)};
//uint32_t TEMP_POS2[] = {rad2pulse(0),rad2pulse(0),rad2pulse(0),rad2pulse(-63)};

uint32_t ST_POS1[] = {rad2pulse_t(PULLEY_RATIO_1*(PI/4)), rad2pulse_t(PULLEY_RATIO_2*(PI/4)), rad2pulse_t(PULLEY_RATIO_3*(PI/4)), rad2pulse_t(PI/4)};
uint32_t ST_POS2[] = {rad2pulse_t(PULLEY_RATIO_1*-(PI/4)), rad2pulse_t(PULLEY_RATIO_2*-(PI/4)), rad2pulse_t(PULLEY_RATIO_3*-(PI/4)), rad2pulse_t(-PI/4)};

uint32_t* tempPos1 = ActuatorTransformation(PI/4, -PI/4, -PI/4, PI/4);
uint32_t* tempPos2 = ActuatorTransformation(PI/4, PI/4, PI/4, -PI/4);

uint32_t RT_POS2[] = {1119,2557,1883,1466};

// Final Position
//uint32_t finPos[] = {2200, 2200, 2200, 2200};
uint32_t finPos = 2300;
uint32_t defaultPos = rad2pulse(0);

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
    pc.printf(" p - Power On / Torque Enable\n\r");
    wait_us(10);
    pc.printf(" h - Home Position\n\r");
    wait_us(10);
    pc.printf(" r - Rapid Fire\n\r");
    wait_us(10);
    pc.printf(" s - Smooth Motion\n\r");
    wait_us(10);
    pc.printf(" i - Impedance Control\n\r");
    wait_us(10);
    pc.printf(" t - Teaching Mode\n\r");
    wait_us(10);
    pc.printf(" c - Switch to Current Control\n\r");
    wait_us(10);
    pc.printf(" space - Abort / Torque Disable\n\r");
    wait_us(10);
}

// Interrupt 
void serial_interrupt(void){
    while(pc.readable()) {
        char input = pc.getc();
        if(input == 32) { //esc

            state = SERVO_OFF;
        }
        else if(input == 104) { //h
            if (state != REST_MODE) state_change = !state_change;
            else state_change = false;
            state = REST_MODE;
        }
        else if(input == 114) { //r
            if (state != RAPID_FIRE_MODE) state_change = !state_change;
            else state_change = false;
            state = RAPID_FIRE_MODE;
        }
        else if(input == 115) { //s
            state = SMOOTH_MODE;
            if (state != SMOOTH_MODE) state_change = !state_change;
            else state_change = false;
        }
        else if(input == 116) { //t
            state = TEACHING_MODE;
            if (state != TEACHING_MODE) state_change = !state_change;
            else state_change = false;
        }
        else if(input == 105) { //i
            if (state != IMPEDANCE_CONTROL_MODE) state_change = !state_change;
            else state_change = false;
            state = IMPEDANCE_CONTROL_MODE;
        }
        else if(input == 112) { //p
            if (state != SERVO_ON) state_change = !state_change;
            else state_change = false;
            state = SERVO_ON;
        }
        else if(input == 99) { //c
            if (state != SERVO_ON) state_change = !state_change;
            else state_change = false;
            state = SWITCH_TO_CURRENT_CONTROL;
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
           //if (state_change) pc.printf("-----Rapid Fire-----\n\r");
        dxl_bus.SetMultGoalCurrents(dxl_ID, idLength, multiGoalCurrent);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, RF_POS_1);
        wait_ms(350);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, RF_POS_2);
        wait_ms(350);
        }

        else if (state == SMOOTH_MODE) {
            if (first_enter == 0) {
                for (int i=0; i<idLength; i++) {
                dxl_bus.SetVelocityProfile(dxl_ID[i], 414);
                dxl_bus.SetAccelerationProfile(dxl_ID[i], 80);
                first_enter++;
                }                
            }
        /*
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, TEMP_POS1);
        wait_ms(1500);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, TEMP_POS2);
        wait_ms(1500);
        */

        /*
        pc.printf("%d, %d, %d, %d\n\r", tempPos1[0], tempPos1[1], tempPos1[2], tempPos1[3]);
        pc.printf("%d, %d, %d, %d\n\r", ST_POS1[0], ST_POS1[1], ST_POS1[2], ST_POS1[3]);
        wait_ms(1500);
        pc.printf("%d, %d, %d, %d\n\r", tempPos2[0], tempPos2[1], tempPos2[2], tempPos2[3]);
        pc.printf("%d, %d, %d, %d\n\r", ST_POS2[0], ST_POS2[1], ST_POS2[2], ST_POS2[3]);
        wait_ms(1500);
        */
        //tempPos1 = ActuatorTransformation(PI/4, -PI/4, -PI/4, PI/4);
        //tempPos2 = ActuatorTransformation(PI/4, PI/4, PI/4, -PI/4);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, tempPos1);
        wait_ms(500);
        pc.printf("%d, %d, %d, %d\n\r", tempPos1[0], tempPos1[1], tempPos1[2], tempPos1[3]);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, tempPos2);
        pc.printf("%d, %d, %d, %d\n\r", tempPos2[0], tempPos2[1], tempPos2[2], tempPos2[3]);
        wait_ms(500);
        free(tempPos1);
        free(tempPos2);
        /*
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, RT_POS2);
        wait_ms(2500);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);  
        wait_ms(2500);     
        */

            //if (state_change) pc.printf("-----Smooth Motion-----\n\r");
        /*
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

        else if (state == IMPEDANCE_CONTROL_MODE) {
            if (state_change) pc.printf("-----Impedance Control-----\n\r");

        dxl_bus.SetGoalPosition(4, homePos);
        dxl_bus.SetGoalCurrent(4, 40);
        wait_ms(20);
        */
        }

        else if (state == TEACHING_MODE) { 
            //if (state_change) pc.printf("-----Teaching Mode-----\n\r");
            /*
        pc.printf("Previous Position: %d \r\n",dxl_bus.GetPosition(4));
        dxl_bus.SetGoalCurrent(4, 10);
        dxl_bus.SetGoalPosition(4, defaultPos);
        wait_ms(5);
        pc.printf("Current Position: %d \r\n",dxl_bus.GetPosition(4));
        defaultPos = dxl_bus.GetPosition(4);
        wait_ms(5);
        */


        }

        else if (state == REST_MODE) {
            //if (state_change) pc.printf("-----Home Position-----\n\r");
        dxl_bus.SetMultGoalCurrents(dxl_ID, idLength, multiGoalCurrent);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);
        wait_ms(100);
        }

        else if (state == IMPEDANCE_CONTROL_MODE) {
            //if (state_change) pc.printf("-----Impedance Control-----\n\r");
            //pc.printf("-----Impedance Control-----\n\r");
        dxl_bus.SetMultGoalCurrents(dxl_ID, idLength, multiGoalCurrentImp);
        dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);
        wait_ms(30);
        }

        else if (state == SERVO_ON) {
            if (!dxl_bus.GetTorqueEn(1)) {
            pc.printf("Turning All Servos On...\n\r");
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

                for (int i=0; i<idLength; i++){    
                dxl_bus.SetVelocityProfile(dxl_ID[i], 0);
                dxl_bus.SetAccelerationProfile(dxl_ID[i], 0);
                pc.printf("Dynamixel No.%d Initialized...\n\r",i+1);
                }
                //dxl_bus.SetMultGoalCurrents(dxl_ID, idLength, multiGoalCurrent);
                //dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);
                wait_ms(800);
                state = REST_MODE;
            }
            else {
                if (state_change) {
                    //pc.printf("Servo Already On!..Returning to Home Position\n\r");
                    state = REST_MODE;
                }
            }
        }
        else if (state == SERVO_OFF) {
            
            if (dxl_bus.GetTorqueEn(1)){
                pc.printf("Abort / Turning All Servos Off...\n\r");
                for (int i=0; i<idLength; i++) {
                dxl_bus.TurnOnLED(dxl_ID[i], 0x00);
                dxl_bus.SetTorqueEn(dxl_ID[i],0x00);    
                wait_ms(100);
                }
                state = REST_MODE;
            }

        }
        else if (state == SWITCH_TO_CURRENT_CONTROL) {
            
                pc.printf("Switching to Current Control Mode...\n\r");
                for (int i=0; i<idLength; i++) {
                dxl_bus.TurnOnLED(dxl_ID[i], 0x00);
                dxl_bus.SetTorqueEn(dxl_ID[i],0x00);    
                wait_ms(100);
                }
                for (int i=0; i<idLength; i++) {
                dxl_bus.SetRetDelTime(dxl_ID[i],0x32); // 4us delay time?
                dxl_bus.SetControlMode(dxl_ID[i], CURRENT_POS_CONTROL);
                wait_ms(100);
                dxl_bus.TurnOnLED(dxl_ID[i], 0x01);
                dxl_bus.SetTorqueEn(dxl_ID[i],0x01);    
                wait_ms(100);                
                }
                state = REST_MODE;
        }
        /**
        wait_us(10);
        range[0] = tof1.readRangeResult();
        wait_us(10);
        range_status[0] = tof1.readRangeStatus();
        **/
       
    }

}

