/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "ServoSimon/Servo.h"

//PA_5 is shaky

// Blinking rate in milliseconds
#define BLINKING_RATE     500ms

void wait_ms(float us){
    wait_us(us);
}
void MoveClawToZero(){
    Servo ServoClaw(PA_6);
    ServoClaw.calibrate(0.0005,70);
    int pos = ServoClaw.read()*70;
    printf("in MoveClawToZero?\n");
    while(ServoClaw.read()> 0.0){
        ServoClaw.position(pos); 
        printf("position %f \n", ServoClaw.read());
        pos = pos-0.01;
        wait_ms(10); 
    }
}
void testClaw(){
    printf("in test?\n");

    Servo ServoClaw(PA_6);
    ServoClaw.calibrate(0.0005,35.0);

    printf("pos: %f (SHOULD BE 0?)\n", ServoClaw.read());

    int pos = ServoClaw.read()*70;

    if(ServoClaw.read()== 0.0){
        while(ServoClaw.read()!= 1.0){
            ServoClaw.position(pos); 
            pos++;
            std::printf("pos: %f \n", ServoClaw.read());
            wait_ms(10); 
        }
    }
    else {
        printf("KSDJFLSKDJF");
    }
        wait_ms(3000);

    if(ServoClaw.read()== 1.0){
        while(ServoClaw.read()!= 0.0){
            ServoClaw.position(pos); 
            pos--;
            std::printf("pos: %f \n", ServoClaw.read());
            wait_ms(10); 
        }
    }
    else{
        printf("DSFL:KDS:FKDS:FJDS");
    }
  
    
}


int main()
{
    float some = 0.0;
    printf("MYTHING%f\n", some);
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    Servo motorWerk(PA_6);
    motorWerk.
    

    for (int i = 0; i<10; i++)
    {
        led = !led;        
        ThisThread::sleep_for(BLINKING_RATE);
    } 

    while(1) {
        for(int i=0; i<100; i++) {
            motorWerk = i/100.0;
            ThisThread::sleep_for(1000ms);
        }
        for(int i=100; i>0; i--) {
            motorWerk = i/100.0;
            ThisThread::sleep_for(1000ms);
        }
     }
    // printf("here?\n");
    // MoveClawToZero();
    // while (true) {
    //     wait_ms(2000);
    //     testClaw();
    // }
    
}
