
#define trying 5
//trying is either linefollowing or servoo or shaky or ultrasonic
#if trying == 1
/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
 #include "mbed.h"
 //#include "LM75B.h"
//#include "mbed_wait_api.h"
 #include "tca9548a.h"
 #include "motoresDC.h"

 //Serial pc(SERIAL_TX,SERIAL_RX);

//Put the sda and scl pins in here maybe this (PB_9, PB_8);
 TCA9548A i2c_sw(PB_9, PB_8); //default address 0x70 applied
 MotoresDC car(PB_3, PB_10, PA_8, PA_10, PB_4, PB_5);
 DigitalOut led(LED1);
Timer t;
 int main() {
     //float normalSpeed = 0.15;
    float normalSpeed = 0.18;
    float Kp = 0.12;
    float Kd = 0.0;
    float Ki = 0;
    float I = 0;
    float e_prev = 0;
    float redRatio = 0.41;
    float greenRatio = 0.43;    
    float blueRatio = 0.57;
    float SENSOR_ON_LINE_VALUE = 0.7;
    //leftToRight
    int minClearValues[3] = {0, 0, 0};
    int maxClearValues[3] = {0, 0, 0};

    //  float blueRatio = 0.42;
    bool stopAtGreen = false;
    bool stopAt3Red = false;
    bool stopAtBlue = false;
    float ROBOT_GOING_LEFT = false;
    float ROBOT_GOING_RIGHT = false;
    //LEFT TO RIGHT OF 0 to 2
    float PREVIOUS_SENSOR = 1;
    bool PAST_THE_SENSOR_ON_WAY_BACK = false;
    float LOST = false;
    int iterations = 0;



    // By default TCA9548A performs a power on reset and all downstream ports are deselected
    I2C i2c0(PB_9, PB_8); //pins for I2C communication (SDA, SCL)
    i2c0.frequency(400000);
    int sensor_addr = 41 << 1;
    for (int i = 2; i <= 4; i++) {
        i2c_sw.select(i);               //  select  the channel 0
        printf("i2c%d freq set\n", i);
        //LM75B tmp0(PB_9, PB_8);
        char id_regval0[1] = {146};
        char data0[1] = {0};
        i2c0.write(sensor_addr, id_regval0, 1, true);
        i2c0.read(sensor_addr,data0,1,false);
        printf("i2c%d read and write completed\n", i);
        printf("This is data i2c %c\n", data0[0]);
        if (68 == data0[0]) printf("i2c1 data is 68\n");
        //OneCycleValue
        char timing_register[2] = {129, 255};
        //This is the code for 10 cycles
        //char timing_register[2] = {129, 246};
        i2c0.write(sensor_addr,timing_register,2,false);
        char control_register[2] = {143,0};
        i2c0.write(sensor_addr,control_register,2,false);
        char enable_register[2] = {128,3};
        i2c0.write(sensor_addr,enable_register,2,false);
    }
    
    /*i2c_sw.select(3);               //  select  the channel 1
    I2C i2c1(PB_9, PB_8); //pins for I2C communication (SDA, SCL)
    printf("i2c1 freq set\n");
    i2c1.frequency(40000);
    //LM75B tmp1(PB_9, PB_8); 
    //int sensor_addr = 41 << 1;
    char id_regval1[1] = {146};
    char data1[1] = {0};
    i2c1.write(sensor_addr,id_regval1,1, true);
    i2c1.read(sensor_addr,data1,1,false);
    printf("i2c01 read and write completed\n");
    printf("This is data i2c1 %c\n", data1[0]);
    if (68 == data1[0]) printf("i2c1 data is 68\n");
    i2c1.write(sensor_addr,timing_register,2,false);
    i2c1.write(sensor_addr,control_register,2,false);
    i2c1.write(sensor_addr,enable_register,2,false);

    i2c_sw.select(4);               //  select  the channel 2
    //LM75B tmp2(PB_9, PB_8);  
    I2C i2c2(PB_9, PB_8); //pins for I2C communication (SDA, SCL)
    i2c2.frequency(40000);
    printf("i2c2 freq set\n");
    //int sensor_addr = 41 << 1;
    char id_regval2[1] = {146};
    char data2[1] = {0};
    i2c2.write(sensor_addr,id_regval2,1, true);
    i2c2.read(sensor_addr,data2,1,false);
    printf("i2c2 read and write completed\n");
    printf("This is data i2c2 %c\n", data2[0]);
    if (68 == data2[0]) printf("i2c2 data is 68\n");
    i2c2.write(sensor_addr,timing_register,2,false);
    i2c2.write(sensor_addr,control_register,2,false);
    i2c2.write(sensor_addr,enable_register,2,false);*/

    //printf("Ready in 5 seconds");
    printf("Do iterations in 5 seconds");
    wait_us(5000000);
        //calibrations is starting
    led = !led;
    for (int iterations1 = 0; iterations1 < 350; iterations1++) {
        for(int i = 2; i <= 4; i++) {
            i2c_sw.select(i);               //  select  the channel 2
            char clear_reg[1] = {148};
            char clear_data[2] = {0,0};
            i2c0.write(sensor_addr,clear_reg,1, true);
            i2c0.read(sensor_addr,clear_data,2, false);
            int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
            
            if (clear_value < minClearValues[i-2]) {
                minClearValues[i-2] = clear_value;
            } else if (clear_value > maxClearValues[i-2]) {
                maxClearValues[i-2] = clear_value;
            }
        }
    }
    led = !led;
    wait_us(5000000);

    //car.conducir(1, 200000);
     while(1) {
        // t.reset();
        // t.start();
        //bool isRed[3] = {0, 0, 0};
        float clearValuePercentage[3] = {0, 0, 0};
        float e = 0.0;
        for (int i = 2; i <= 4; i++){
            i2c_sw.select(i);               //  select  the channel 2
            char clear_reg[1] = {148};
            char clear_data[2] = {0,0};
            i2c0.write(sensor_addr,clear_reg,1, true);
            i2c0.read(sensor_addr,clear_data,2, false);
            int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
            
            char red_reg[1] = {150};
            char red_data[2] = {0,0};
            i2c0.write(sensor_addr,red_reg,1, true);
            i2c0.read(sensor_addr,red_data,2, false);
            int red_value = ((int)red_data[1] <<8) | red_data[0];
            
            char green_reg[1] = {152};
            char green_data[2] = {0,0};
            i2c0.write(sensor_addr,green_reg,1, true);
            i2c0.read(sensor_addr,green_data,2, false);
            int green_value = ((int)green_data[1] << 8) | green_data[0];
            
            char blue_reg[1] = {154};
            char blue_data[2] = {0,0};
            i2c0.write(sensor_addr,blue_reg,1, true);
            i2c0.read(sensor_addr,blue_data,2, false);
            int blue_value = ((int)blue_data[1] << 8) | blue_data[0];
            
            // printf("%d\n", red_value);
            // // printf("%d\n", blue_value);
            // // printf("%d\n", green_value);
            

            float total = red_value+blue_value+green_value;
            //printf("%f\n", red_value/total);
            // if (i == 3) {
            
            //if (i == 4) printf("\n");
            bool checkIfRed = redRatio <= red_value/total;
            
            clearValuePercentage[i-2] = ((float)clear_value - minClearValues[i-2])/(maxClearValues[i-2] - minClearValues[i-2]);



            // if (checkIfRed) {
            //     isRed[i-2] = 1;
            // }
            if (stopAtBlue == true && blueRatio <= blue_value/total) {
                while(1) {
                    car.detener();
                }
            }
            if (stopAtGreen == true && greenRatio <= green_value/total) {
                while(1) {
                    car.detener();
                }
            }
        }

        //Generate an Error
        //TO know where it previosuly was use the previous error
   
        //SENSOR 0 TO 2 WITH LEFT BEING 0 AND RIGHT BEING 1
            //Put the percentage on a scale from 0.5 to 1 based of the clear value is below a threashold.
            if (clearValuePercentage[0] < SENSOR_ON_LINE_VALUE) {
                if (PREVIOUS_SENSOR == 1) {
                    ROBOT_GOING_LEFT = true;
                    LOST = false;
                    PREVIOUS_SENSOR = 0;
                }
                //PreviousSensorIsItself
                if (LOST) {
                    e = -1;
                } else if (!LOST && ROBOT_GOING_LEFT) {
                    //this function lets 0.7 = 0.5 and lets 0 = 1
                    e = -(clearValuePercentage[0]*-10/13 + 27.0/26);
                } else if (!LOST && ROBOT_GOING_RIGHT && PAST_THE_SENSOR_ON_WAY_BACK) {
                    //this function lets 0.7 = 0.5 and lets 0 = 1
                   e = -(clearValuePercentage[0]*-10/13 + 27.0/26);
                } else if (!LOST && ROBOT_GOING_RIGHT) {
                    //this functions lets 0.7 = 1.5 and lets 0 = 1
                    e = -(clearValuePercentage[0]*10/13 + 25.0/26);
                }
                if (clearValuePercentage[0] < 0.05 && ROBOT_GOING_LEFT) {
                    LOST = true;
                    e = -1;
                    iterations = 50;
                }
                if (clearValuePercentage[0] < 0.05 && ROBOT_GOING_RIGHT) {
                    PAST_THE_SENSOR_ON_WAY_BACK = true;
                }
            } else  if (clearValuePercentage[2] < SENSOR_ON_LINE_VALUE) {
                if (PREVIOUS_SENSOR == 1) {
                    ROBOT_GOING_RIGHT = true;
                    LOST = false;
                    PREVIOUS_SENSOR = 2;
                }
                //PreviousSensorIsItself
                if (LOST) {
                    e = -1;
                } else if (!LOST && ROBOT_GOING_RIGHT) {
                    //this function lets 0.7 = 0.5 and lets 0 = 1
                    e = -(clearValuePercentage[2]*-10/13 + 27.0/26);
                } else if (!LOST && ROBOT_GOING_LEFT && PAST_THE_SENSOR_ON_WAY_BACK) {
                    //this function lets 0.7 = 0.5 and lets 0 = 1
                   e = -(clearValuePercentage[2]*-10/13 + 27.0/26);
                } else if (!LOST && ROBOT_GOING_LEFT) {
                    //this functions lets 0.7 = 1.5 and lets 0 = 1
                    e = -(clearValuePercentage[2]*10/13 + 25.0/26);
                }
                if (clearValuePercentage[2] < 0.05 && ROBOT_GOING_RIGHT) {
                    LOST = true;
                    e = -1;
                    iterations = 50;
                }
                if (clearValuePercentage[2] < 0.05 && ROBOT_GOING_LEFT) {
                    PAST_THE_SENSOR_ON_WAY_BACK = true;
                }
            } else if (clearValuePercentage[1] < SENSOR_ON_LINE_VALUE) {
                //coming from 0
                if (ROBOT_GOING_RIGHT) {
                    e = -5.0/7*clearValuePercentage[1];
                } else if (ROBOT_GOING_LEFT) {
                    //coming from 2
                     e = 5.0/7*clearValuePercentage[1];
                }
                if (clearValuePercentage[1] < 0.05) {
                    ROBOT_GOING_LEFT = false;
                    ROBOT_GOING_RIGHT = false;
                    PREVIOUS_SENSOR = 0;
                    PAST_THE_SENSOR_ON_WAY_BACK = false;
                    LOST = true;
                }
            } else {
                //Wont affect sensor 0
                iterations--;
                if (LOST && iterations == 0) {
                    if (PREVIOUS_SENSOR == 0) {
                        e = -2;
                        ROBOT_GOING_RIGHT = true;
                        LOST = false;
                    } else if (PREVIOUS_SENSOR == 2) {
                        e = 2;
                        ROBOT_GOING_LEFT = true;
                        LOST = false;
                    } else {
                        e = 0;
                    }
               }
               if (e > 1 || e < -1) e = e_prev;
            }
           


        // float numRed = 0;
        // for (int i = 0; i <= 2; i++) {
        //     if (isRed[i] == 1) {
        //         e += i-1;
        //         numRed++;
        //     }
        // }
        // if (numRed == 0){
        //     numRed++;
        //     // if (e_prev == -1) e = -1.5;
        //     // else if (e_prev == 1) e = 1.5;
        //     e = e_prev;
        // // } 
        // if (stopAt3Red && numRed == 3) {
        //     while(1) {
        //         car.detener();
        //     }
        // }
        // printf("%f",e);
        float numer = 0;
        float denum = 0;
        for (int i = 0; i <=2 ; i++){
            numer += i*1000*(1-clearValuePercentage[i])*1000;
            denum += (1-clearValuePercentage[i])*1000;
        }
        printf("Readline: %f\n",numer/denum);
        float P = Kp*e;
        I = I + e;
        float D = ((e - e_prev)/0.0000023)*Kd;
        e_prev = e;
        float PID = P+I*Ki+D;
        // printf("PID value %f ", PID);
       // printf("Error Value %f\n", e);
        //wait_us(1000);
        //Left Engine
        float leftMotorSpeed = normalSpeed+PID;
        // if (leftMotorSpeed < 0.3) leftMotorSpeed = -0.5;
        float rightMotorSpeed = normalSpeed-PID;
        //if (rightMotorSpeed < 0.3) rightMotorSpeed = -0.5;
        //car.motorIzq(leftMotorSpeed);
        // Right Engine
        //car.motorDer(rightMotorSpeed);
        //t.stop();
        //printf("time %f\n  ", t.read());
    }
 }


#elif trying ==  2

#include "mbed.h"
#include "ServoSimon/Servo.h"
#include <cmath>

// Blinking rate in milliseconds
#define BLINKING_RATE     500ms

int main()
{

    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    //PA_6 is the gripper; PA_5 is shaky
    Servo gripperMotor(PA_6);
    //first param is the range in seconds from middle to end and second param is the angle from middle to end
    //the second param degrees doesnt do anything. 
    //to control the angle, 0.0005 gave ~100 degrees range
    //basically for the gripper, we had to recalibrate it because 0 wasnt exactly the closing position 
    //so i set the range to be greater than 90 degrees but then never use anything in the 50~100 of that range
    //because that would cause the gripper to bend more and break
    //so set the range wider and use just 0~40% of that range

    //40 is open, 0 is closed
    //dont change this value
    gripperMotor.calibrate(0.0009,45);

    for (int i = 0; i<10; i++)
    {
        led = !led;        
        ThisThread::sleep_for(BLINKING_RATE);
    } 

    while(1) 
    {
        // for(int i=0; i<40; i++) 
        // {
        //     gripperMotor = i/100.0;
        //     wait_us(20000);
        // }       
        // for(int i =40; i>0; i--) 
        // {
        //     gripperMotor = i/100.0;
        //     wait_us(20000);      
        // }
        gripperMotor = 0.4;
     }    
}

#elif trying ==  3

#include "mbed.h"
#include "ServoSimon/Servo.h"

int main(){

    Servo gripperMotor(PA_6);
    gripperMotor.calibrate(0.0009,45);
    for(int i =40; i>0; i--) 
    {
        gripperMotor = i/100.0;
        wait_us(20000);      
    }
    wait_us(2000000);      

    //dont change these values
    Servo shakyMotor(PA_5);
    shakyMotor.calibrate(0.0006,45);

    //90 is left corner, 0 is right corner
    while(1) 
    {
        //it was leaning more towards the left side so it goes from 10-80 not 20-80
        for(int i=0; i<90; i++) 
        {
            shakyMotor = i/100.0;
            wait_us(20000);
        }       
        for(int i=90; i>0; i--) 
        {
            shakyMotor = i/100.0;
            wait_us(20000);      
        }
     }    
}

#elif trying ==4
#include "mbed.h"
#include "HCSR04Antonio/HCSR04.h"

//trigger, echo in this order
HCSR04 USsensor(D8, D9);

int main()
{   
    while(1) {
        float distance = USsensor.distance();   
        printf("distanza  %f  \n",distance);
        wait_us(100000); 
    }
}

#elif trying ==5
#include "mbed.h"
#include "tca9548a.h"
#include "motoresDC.h"
#include "ServoSimon/Servo.h"
#include "HCSR04Antonio/HCSR04.h"


float SENSOR_ON_LINE_VALUE = 0.60;
float LOW_CLR_VAL = 0.1;
int EDGE_MOST_SENSOR = 2;
DigitalOut led(LED1);
TCA9548A i2c_sw(PB_9, PB_8); //default address 0x70 applied
MotoresDC car(PB_3, PB_10, PA_8, PA_10, PB_4, PB_5);
//trigger, echo in this order
HCSR04 USsensor(D8, D9);

float generateLinerizedError(int edgeSensor, int middleSensor, float* clearValuePercentage) {
    //this function lets 0.7 = 0.5 and lets 0 = 1
    float e = 0;
    int sign = 1;
    if (edgeSensor < 0) sign = -1;
    else if (edgeSensor == 0 && middleSensor == 1) sign = -1;
    //The above is because this doesn't cover the right side coming back and this is a bandaid fixed it

    if (clearValuePercentage[edgeSensor+2] <= SENSOR_ON_LINE_VALUE) {
        //If edge sensor is activated
        e = middleSensor + (clearValuePercentage[edgeSensor+2]*-10.0/13 + 27.0/26)*sign;
    } else if (clearValuePercentage[middleSensor+2] <= SENSOR_ON_LINE_VALUE) {
        //If current sensor is activated
        e = middleSensor + 5.0/7*clearValuePercentage[middleSensor+2]*sign;
    } else {
        e = middleSensor + 0.5*sign;
    }
    return e;
}

void CloseGripper(Servo &gripperMotor){
    for(int i =40; i>0; i--) 
    {
        gripperMotor = i/100.0;
        wait_us(20000);      
    }   

}

void OpenGripper(Servo &gripperMotor){
    for(int i=0; i<40; i++) 
    {
        gripperMotor = i/100.0;
        wait_us(20000);
    } 

}
int main() {
float normalSpeed = 0.22;
float Kp = 0.09;
float Kd = 0.07;
float Ki = 0;
float I = 0;
float e_prev = 0;
float redRatio = 0.41;
float greenRatio = 0.43;    
float blueRatio = 0.57;
//leftToRight
int minClearValues[5] = {INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX};
int maxClearValues[5] = {0, 0, 0, 0, 0};

//  float blueRatio = 0.42;
bool stopAtGreen = false;
bool stopAt3Red = false;
bool stopAtBlue = true;

Servo gripperMotor(PA_6);
//40 is open, 0 is closed
//dont change this value
gripperMotor.calibrate(0.0009,45); 


printf("Clear logs now");
// By default TCA9548A performs a power on reset and all downstream ports are deselected
I2C i2c0(PB_9, PB_8); //pins for I2C communication (SDA, SCL)
i2c0.frequency(400000);
int sensor_addr = 41 << 1;
for (int i = 1; i <= 5; i++) {
    i2c_sw.select(i);               //  select  the channel 0
    char id_regval0[1] = {146};
    char data0[1] = {0};
    i2c0.write(sensor_addr, id_regval0, 1, true);
    i2c0.read(sensor_addr,data0,1,false);
    if (68 == data0[0]) printf("Read data is 68 for sensor %d (working)\n", i);
    //OneCycleValue
    char timing_register[2] = {129, 255};
    //This is the code for 10 cycles
    //char timing_register[2] = {129, 246};
    i2c0.write(sensor_addr,timing_register,2,false);
    char control_register[2] = {143,0};
    i2c0.write(sensor_addr,control_register,2,false);
    char enable_register[2] = {128,3};
    i2c0.write(sensor_addr,enable_register,2,false);
}

printf("Calabration is beginning\n");
led = !led;
wait_us(5000000);

    //calibrations is starting
led = !led;
for (int iterations1 = 0; iterations1 < 3000; iterations1++) {
    for(int i = 1; i <= 5; i++) {
        i2c_sw.select(i);               //  select  the channel 2
        char clear_reg[1] = {148};
        char clear_data[2] = {0,0};
        i2c0.write(sensor_addr,clear_reg,1, true);
        i2c0.read(sensor_addr,clear_data,2, false);
        int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
        //printf("%d ", clear_value);
        if (clear_value < minClearValues[i-1]) {
            minClearValues[i-1] = clear_value;
        } else if (clear_value > maxClearValues[i-1]) {
            maxClearValues[i-1] = clear_value;
        }
    }
}
//maxClearValues[4] = (maxClearValues[0] + maxClearValues[1] + maxClearValues[2] + maxClearValues[3])/4;

printf("Max values are %d, %d, %d, %d, %d\n", maxClearValues[0], maxClearValues[1], maxClearValues[2], maxClearValues[3], maxClearValues[4]);
printf("Min values are %d, %d, %d, %d, %d\n", minClearValues[0], minClearValues[1], minClearValues[2], minClearValues[3], minClearValues[4]);


led = !led;
wait_us(8000000);
int currentSensor = 0;
bool left = false;
bool right = false;
    while(1) {
        bool isRed[5] = {0, 0, 0, 0, 0};
        bool isGreen[5] = {0, 0, 0, 0, 0};

        float clearValuePercentage[5] = {0, 0, 0, 0, 0};
        float e = 0.0;
        for (int i = 1; i <= 5; i++) {
            i2c_sw.select(i);               //  select  the channel 2
            char clear_reg[1] = {148};
            char clear_data[2] = {0,0};
            i2c0.write(sensor_addr,clear_reg,1, true);
            i2c0.read(sensor_addr,clear_data,2, false);
            int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
            
            char red_reg[1] = {150};
            char red_data[2] = {0,0};
            i2c0.write(sensor_addr,red_reg,1, true);
            i2c0.read(sensor_addr,red_data,2, false);
            int red_value = ((int)red_data[1] <<8) | red_data[0];
            
            char green_reg[1] = {152};
            char green_data[2] = {0,0};
            i2c0.write(sensor_addr,green_reg,1, true);
            i2c0.read(sensor_addr,green_data,2, false);
            int green_value = ((int)green_data[1] << 8) | green_data[0];
            
            char blue_reg[1] = {154};
            char blue_data[2] = {0,0};
            i2c0.write(sensor_addr,blue_reg,1, true);
            i2c0.read(sensor_addr,blue_data,2, false);
            int blue_value = ((int)blue_data[1] << 8) | blue_data[0];

            float total = red_value+blue_value+green_value;
            if (greenRatio <=  green_value/total) {
                isGreen[i-1] = 1;
            }
            bool checkIfRed = redRatio <= red_value/total;
            if (checkIfRed) {
                isRed[i-1] = 1;
            }
            clearValuePercentage[i-1] = ((float)clear_value - minClearValues[i-1])/(maxClearValues[i-1] - minClearValues[i-1]);

            if (stopAtBlue == true && blueRatio <= blue_value/total) {
               // while(1) {
                    car.detener();
                    //look for man using us
                    CloseGripper(gripperMotor);
                    wait_us(3000000);
                    OpenGripper(gripperMotor);
                    wait_us(3000000);

                    //turn
               // }
            }
            if (stopAtGreen == true && greenRatio <= green_value/total) {
                while(1) {
                    car.detener();
                }
            }
        }

        e = currentSensor;
        if(currentSensor == -EDGE_MOST_SENSOR){
            if(clearValuePercentage[currentSensor +1+2] <= SENSOR_ON_LINE_VALUE){
                left = true;
                right = false;
                currentSensor = currentSensor+1;
            }
        } else if(currentSensor == EDGE_MOST_SENSOR) {
            if(clearValuePercentage[currentSensor -1+2] <= SENSOR_ON_LINE_VALUE){
                right = true;
                left = false;
                currentSensor = currentSensor-1;
            }
        } else {
            //So it still keeps going 
            if (clearValuePercentage[currentSensor-1+2] <= SENSOR_ON_LINE_VALUE && !isGreen[currentSensor-1+2]) {
                left = true;
                right = false;
                //printf("Sensor on Line Left \n");
            } else if (clearValuePercentage[currentSensor+1+2] <= SENSOR_ON_LINE_VALUE && !isGreen[currentSensor+1+2]) {
                right = true;
                left = false;
                //printf("Sensor on Line Right \n");
            }
            if (left == true) {
                e = generateLinerizedError(currentSensor-1, currentSensor, clearValuePercentage);
               // printf("Lin error left is %f\n", e);
            } else if (right == true) {
                e = generateLinerizedError(currentSensor+1, currentSensor, clearValuePercentage);
               // printf("Lin error right is %f\n", e);
            }

            if (std::abs(e -currentSensor)<LOW_CLR_VAL && !isGreen[currentSensor+2]){
                e= currentSensor;
                left = false;
                right = false;
            } else if(std::abs(e -(currentSensor-1))<LOW_CLR_VAL && !isGreen[currentSensor-1+2]){
                currentSensor = currentSensor-1;
                left = false;
                right = false;
                e = currentSensor;
            } else if(std::abs(e -(currentSensor+1))<LOW_CLR_VAL && !isGreen[currentSensor+1+2]){
                currentSensor = currentSensor+1;
                left = false;
                right = false;
                e = currentSensor;
            }
        }
        if (isRed[4]){
            e = 2;
            currentSensor = 2;
            left = false;
            right = false;
        } else if (isRed[0]){
            e = -2;
            currentSensor = -2;
            left = false;
            right = false;
        } else if (isRed[1]) {
           // e = -1;
            currentSensor = -1;
            left = false;
            right = false;
        } else if (isRed[3]) {
            //e = -1;
            currentSensor = 1;
            left = false;
            right = false;
        } else if (isRed[2]) {
            //e = 0;
            currentSensor = 0;
            left = false;
            right = false;
        }

        float P = Kp*e;
        I = I + e;
        float D = ((e - e_prev))*Kd;
        e_prev = e;
        float PID = P+I*Ki+D;
        float leftMotorSpeed = normalSpeed+PID;
        float rightMotorSpeed = normalSpeed-PID;
        //Left Engine
        car.motorIzq(leftMotorSpeed);
        // Right Engine
        car.motorDer(rightMotorSpeed);
            
        printf("%f ", e);
        printf("%f, %f, %f, %f, %f, %d, %d\n", clearValuePercentage[0], clearValuePercentage[1], clearValuePercentage[2], clearValuePercentage[3], clearValuePercentage[4], left, right);
    }
}
#endif
