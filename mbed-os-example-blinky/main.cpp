#define trying 5


#if trying == 5
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

void CloseGripper(Servo *gripperMotor){
    for(int i =40; i>0; i--) 
    {
        *gripperMotor = i/100.0;
        wait_us(20000);      
    }   

}

void OpenGripper(Servo *gripperMotor){
    for(int i=0; i<40; i++) 
    {
        *gripperMotor = i/100.0;
        wait_us(20000);
    } 

}
void celebrate(Servo* gripperMotor){
    CloseGripper(gripperMotor);
    OpenGripper(gripperMotor);
    CloseGripper(gripperMotor);
    OpenGripper(gripperMotor);
    CloseGripper(gripperMotor);
    OpenGripper(gripperMotor);

}

bool checkForReds(I2C &i2c0, float redRatio, bool isRedd[5] ) {
    int sensor_addr = 41 << 1;
    for (int i = 1; i <= 5; i++) {
        i2c_sw.select(i);               //  select  the channel 0

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
        bool checkIfRed = redRatio <= red_value/total;
        if (checkIfRed) {
            isRedd[i-1] = 1;
        }
    }
    return isRedd[2];        
}

bool checkForProximity(){

//trigger, echo in this order
    HCSR04 USsensor(D8, D9);
    float distance = USsensor.distance();   
    //4-7 is the range we want
    return distance<6;
}
int main() {
    float adjusting = 0.4;//0.7; 

    int backwardsIterations = 590;
    float NORMAL_SPEED = 0.245*adjusting;
    float normalSpeed = NORMAL_SPEED;
    float FORWARD_MAN_GRAB_SPEED = NORMAL_SPEED;
    float BACKWARD_MAN_GRAB_SPEED = NORMAL_SPEED;

    float GOING_BACK_AFTER_GRAB_SPEED = NORMAL_SPEED*1.4;
    //float Kp = 0.089*adjusting;
    float K_P = 0.11*adjusting;
    float Kp = K_P;
    float BACKWARD_MAN_GRAB_KP = K_P;
    float FORWARD_MAN_GRAB_KP = K_P*0.4;
    float Kd = 0.07*adjusting;
    float Ki = 0;
    float I = 0;
    float e_prev = 0;
    //For the home track 
    //float redRatio = 0.41;
    //For the home track  2
    //float redRatio = 0.48;

///////////////////////////////////////////////////////////
    //For the school track
    float redRatio = 0.42; 
///////////////////////////////////////////////////////////

    //For the home track 
    //float greenRatio = 0.44;  

///////////////////////////////////////////////////////////
    //For the school Track  
    float greenRatio = 0.46; 
///////////////////////////////////////////////////////////

    //For the home track
    //float blueRatio = 0.55;

///////////////////////////////////////////////////////////
    //For the school track
    float blueRatio = 0.46;
///////////////////////////////////////////////////////////

    //leftToRight
    //For home to calibrate
    //int minClearValues[5] = {INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX};
    //int maxClearValues[5] = {0, 0, 0, 0, 0};

///////////////////////////////////////////////////////////
    //These are the school values
    int minClearValues[5] = {25, 20, 17, 16, 73};
    int maxClearValues[5] = {83, 66, 58, 51, 200};
///////////////////////////////////////////////////////////

    //These are the home values
    //int minClearValues[5] = {29, 23, 22, 20, 86};
    //int maxClearValues[5] = {106, 83, 69, 61, 252};

    //Billy's random number
    //float blueRatio = 0.42;
    bool stopAtGreen = false;
    bool stopAt5Red = true;
    bool stopAtBlue = true;

    // Servo gripperMotor(PA_6);
    // //40 is open, 0 is closed
    // //dont change this value
    // gripperMotor.calibrate(0.00095,45); 

    Servo* shakyMotor;
    Servo* gripperMotor;
    gripperMotor = new Servo(PA_6);
    //40 is open, 0 is closed
    gripperMotor->calibrate(0.00095,45); 
    shakyMotor= new Servo(PA_5);
    shakyMotor->calibrate(0.0006,45);
    for (int i = 50; i>42; i--){
        *shakyMotor = i/100.0;
        printf("here %f\n",shakyMotor->read());
        wait_us(1000);
    }

    // for (int i = 0; i<40; i++){
    //     *shakyMotor = i/100.0;
    //     printf("here %f\n",shakyMotor->read());
    //     wait_us(1000);
    // }
    wait_us(1000000);
    shakyMotor->stop();
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

    // printf("Calabration is beginning\n");

        //calibrations is starting
    // led = !led;
    // for (int iterations1 = 0; iterations1 < 3000; iterations1++) {
    //     for(int i = 1; i <= 5; i++) {
    //         i2c_sw.select(i);               //  select  the channel 2
    //         char clear_reg[1] = {148};
    //         char clear_data[2] = {0,0};
    //         i2c0.write(sensor_addr,clear_reg,1, true);
    //         i2c0.read(sensor_addr,clear_data,2, false);
    //         int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
    //         //printf("%d ", clear_value);
    //         if (clear_value < minClearValues[i-1]) {
    //             minClearValues[i-1] = clear_value;
    //         } else if (clear_value > maxClearValues[i-1]) {
    //             maxClearValues[i-1] = clear_value;
    //         }
    //     }
    // }
    //maxClearValues[4] = (maxClearValues[0] + maxClearValues[1] + maxClearValues[2] + maxClearValues[3])/4;

    printf("Max values are %d, %d, %d, %d, %d\n", maxClearValues[0], maxClearValues[1], maxClearValues[2], maxClearValues[3], maxClearValues[4]);
    printf("Min values areeeee %d, %d, %d, %d, %d\n", minClearValues[0], minClearValues[1], minClearValues[2], minClearValues[3], minClearValues[4]);

    led = !led;
    wait_us(5000000);
    int currentSensor = 0;
    bool left = false;
    bool right = false;
    bool findMan = false;
    int moveDirection = 1;
    HCSR04 USsensor(D8, D9);
    while(1) {
        bool isRed[5] = {0, 0, 0, 0, 0};
        int numRed = 0;
        bool isGreen[5] = {0, 0, 0, 0, 0};

        float clearValuePercentage[5] = {0, 0, 0, 0, 0};
        float e = 0.0;
        // printf("BLUE");
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
                numRed++;
            }
            clearValuePercentage[i-1] = ((float)clear_value - minClearValues[i-1])/(maxClearValues[i-1] - minClearValues[i-1]);

            // printf(" %f ", blue_value/total);
            if (stopAtBlue == true && blueRatio <= blue_value/total) {
            // while(1) {
                printf("IN STOP AT BLUE");
                wait_us(30000);
                car.detener();
                wait_us(1000000);
                // car.conducir(-0.18, 500);
                stopAtBlue = false;
                //look for man using us
                //move back a bit
                // float distance = USsensor.distance(); 
                // while (distance >= 6) {
                //     car.conducir(NORMAL_SPEED);
                //     distance = USsensor.distance(); 
                // }
                //printf("%f\n", distance);
                car.detener();

                CloseGripper(gripperMotor);
                wait_us(1000000);
                gripperMotor->stop();
                //delete gripperMotor;
                //hopefully deleting this keeps it closed

                // shakyMotor= new Servo(PA_5);
                // shakyMotor->calibrate(0.0006,45); gripperMotor->stop();
                shakyMotor->resume();

                for(int i = 42; i<90; i++){
                    *shakyMotor = i/100.0;
                    wait_us(1000);
                }   
                wait_us(1000000);


                shakyMotor->stop();

                
                //turn
                car.conducir(-GOING_BACK_AFTER_GRAB_SPEED, 800);
                car.detener();
                wait_us(1000000);
                checkForReds(i2c0, redRatio, isRed);
                while (!isRed[4]) {
                    //FOr School
                    //car.pivotar(-0.35);
                    car.pivotar(-NORMAL_SPEED*1.5);
                    checkForReds(i2c0, redRatio, isRed);
                }
                while (!isRed[0]) {
                    //FOr School
                    //car.pivotar(-0.35);
                    car.pivotar(-NORMAL_SPEED*1.5);
                    checkForReds(i2c0, redRatio, isRed);
                }
                
                while (!isRed[2]) {
                    car.pivotar(-NORMAL_SPEED*1.2);
                    checkForReds(i2c0, redRatio, isRed);
                    currentSensor = 0;
                }
                car.detener();
                wait_us(1000000);

            

                findMan = false;
                normalSpeed = NORMAL_SPEED;
                Kp = K_P;
                stopAtGreen = true;
                //} 
                // else {
                //     moveDirection = -1;
                //     normalSpeed = BACKWARD_MAN_GRAB_SPEED;
                //     Kp = BACKWARD_MAN_GRAB_KP;
                // }

            }

            if (stopAtGreen == true && greenRatio <= green_value/total && (i == 1 || i == 2)) {
                printf("IN STOP AT GREEN");
                car.detener();
                //shakyMotor->stop();
                wait_us(10000);
                OpenGripper(gripperMotor);
                wait_us(1000000);
                gripperMotor->stop();
                car.conducir(-GOING_BACK_AFTER_GRAB_SPEED*1.2, 450);
                wait_us(10000);
                for(int i = 90; i>=50; i--){
                    *shakyMotor = i/100.0;
                    wait_us(10000);
                } 
                wait_us(1000000);
                shakyMotor->stop();
                stopAtGreen = false;
                currentSensor = 1;
                left = false;
                right = false;
                led = !led;
                wait_us(3000000);
            }
            if (stopAt5Red == true && numRed == 5) {
                printf("IN STOP AT 5 RED");
                car.detener();

                //Servo* gripperMotor = new Servo(PA_6);
                //40 is open, 0 is closed
                //gripperMotor->calibrate(0.00095,45); 
                OpenGripper(gripperMotor);
                         celebrate(gripperMotor);   

                 //lets just not destroy this cuz like it ends here... do we rlly need to care about this
                return 1;
            }
        }
        //printf("\n");

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
        e = e*moveDirection;
        if (moveDirection == -1) {
            backwardsIterations--;
        }

        if (backwardsIterations == 0) {
            moveDirection = 1;
            backwardsIterations--;
            findMan = true;
            normalSpeed = FORWARD_MAN_GRAB_SPEED;
            Kp = FORWARD_MAN_GRAB_KP;
        }

        if (findMan) {
            float distance = USsensor.distance();  
            if (distance <= 6) {
                printf("%f\n", distance);
                car.detener();
               
                CloseGripper(gripperMotor);
                wait_us(1000000);
                //delete gripperMotor;
                //hopefully deleting this keeps it closed

                // shakyMotor= new Servo(PA_5);
                // shakyMotor->calibrate(0.0006,45); gripperMotor->stop();
                shakyMotor->resume();

                for(int i = 50; i<100; i++){
                    *shakyMotor = i/100.0;
                    wait_us(90000);
                }   

                wait_us(1000000);
                
                //turn
                car.conducir(-GOING_BACK_AFTER_GRAB_SPEED, 900);
                car.detener();
                wait_us(1000000);
                checkForReds(i2c0, redRatio, isRed);
                while (!isRed[0]) {
                    //FOr School
                    //car.pivotar(-0.35);
                    car.pivotar(-NORMAL_SPEED*1.5);
                    checkForReds(i2c0, redRatio, isRed);
                }
                
                while (!isRed[2]) {
                    car.pivotar(-NORMAL_SPEED*1.2);
                    checkForReds(i2c0, redRatio, isRed);
                    currentSensor = 0;
                }
                car.detener();
                wait_us(1000000);

               

                findMan = false;
                normalSpeed = NORMAL_SPEED;
                Kp = K_P;
            }
        }

        float P = Kp*e;
        I = I + e;
        float D = ((e - e_prev))*Kd;
        e_prev = e;
        float PID = P+I*Ki+D;
        float leftMotorSpeed = normalSpeed+PID;
        float rightMotorSpeed = normalSpeed-PID;
        //Left Engine
        car.motorIzq(leftMotorSpeed*moveDirection);
        //car.motorIzq(leftMotorSpeed);
        //car.motorIzq(0.16*(adjusting+0.2));
        // Right Engine
        car.motorDer(rightMotorSpeed*moveDirection);
        //car.motorDer(rightMotorSpeed);
       //car.conducir(0.16*(adjusting+0.2));
        // printf("%f ", e);
        // printf("%f, %f, %f, %f, %f, %d, %d\n", clearValuePercentage[0], clearValuePercentage[1], clearValuePercentage[2], clearValuePercentage[3], clearValuePercentage[4], left, right);
    }
}

#elif trying ==6
//this is testing all the sensors w the 9v battery 
#include "mbed.h"
#include "tca9548a.h"
#include "motoresDC.h"
#include "ServoSimon/Servo.h"
#include "HCSR04Antonio/HCSR04.h"

MotoresDC car(PB_3, PB_10, PA_8, PA_10, PB_4, PB_5);
void CloseGripper(Servo *gripperMotor){
    for(int i =40; i>0; i--) 
    {
        *gripperMotor = i/100.0;
        wait_us(20000);      
    }   

}

void OpenGripper(Servo *gripperMotor){
    for(int i=0; i<40; i++) 
    {
        *gripperMotor = i/100.0;
        wait_us(20000);
    } 

}

int main()
{   
    float adjusting = 0.4;//0.7; 

    int backwardsIterations = 590;
    float NORMAL_SPEED = 0.245*adjusting;
    float normalSpeed = NORMAL_SPEED;
    float FORWARD_MAN_GRAB_SPEED = NORMAL_SPEED;
    float BACKWARD_MAN_GRAB_SPEED = NORMAL_SPEED;

    float GOING_BACK_AFTER_GRAB_SPEED = NORMAL_SPEED*1.4;
    //float Kp = 0.089*adjusting;
    float K_P = 0.11*adjusting;
    float Kp = K_P;
    float BACKWARD_MAN_GRAB_KP = K_P;
    float FORWARD_MAN_GRAB_KP = K_P*0.4;
    float Kd = 0.07*adjusting;
    float Ki = 0;
    float I = 0;
    float e_prev = 0;
    

    //For the home track 
   // float redRatio = 0.41;
    //For the home track  2
    //float redRatio = 0.48;
    //For the school track
    float redRatio = 0.42; 
    //For the home track 
    //float greenRatio = 0.40;  
    //For the school Track  
    float greenRatio = 0.46;  
    //For the home track
    //float blueRatio = 0.55;
    //For the school track
    float blueRatio = 0.46;
    //leftToRight
    //For home to calibrate
   // int minClearValues[5] = {INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX};
    //int maxClearValues[5] = {0, 0, 0, 0, 0};

    //These are the school values
    int minClearValues[5] = {25, 20, 17, 16, 73};
    int maxClearValues[5] = {83, 66, 58, 51, 200};

    //These are the home values
    // int minClearValues[5] = {29, 23, 22, 20, 86};
    // int maxClearValues[5] = {106, 83, 69, 61, 252};

    //  float blueRatio = 0.42;
    bool stopAtGreen = false;
    bool stopAt5Red = true;
    bool stopAtBlue = true;

    // Servo gripperMotor(PA_6);
    // //40 is open, 0 is closed
    // //dont change this value
    // gripperMotor.calibrate(0.00095,45); 

    // By default TCA9548A performs a power on reset and all downstream ports are deselected
    I2C i2c0(PB_9, PB_8); //pins for I2C communication (SDA, SCL)
    i2c0.frequency(400000);
    float SENSOR_ON_LINE_VALUE = 0.60;
    float LOW_CLR_VAL = 0.1;
    int EDGE_MOST_SENSOR = 2;
    DigitalOut led(LED1);
    TCA9548A i2c_sw(PB_9, PB_8); //default address 0x70 applied
    MotoresDC car(PB_3, PB_10, PA_8, PA_10, PB_4, PB_5);
    //trigger, echo in this order
    HCSR04 USsensor(D8, D9);
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

    //delete gripperMotor;
    //hopefully deleting this keeps it closed

    // shakyMotor= new Servo(PA_5);
    // shakyMotor->calibrate(0.0006,45); gripperMotor->stop();

    Servo* shakyMotor;
    Servo* gripperMotor;
    gripperMotor = new Servo(PA_6);
    gripperMotor->calibrate(0.00095,45); 

    shakyMotor= new Servo(PA_5);
    shakyMotor->calibrate(0.0006,45);


    CloseGripper(gripperMotor);
    wait_us(1000000);

    shakyMotor->resume();
    gripperMotor->stop();

    for(int i = 50; i<100; i++){
        *shakyMotor = i/100.0;
        wait_us(90000);
    }   

    wait_us(1000000);

    car.detener();
    shakyMotor->stop();
    wait_us(1000);
    OpenGripper(gripperMotor);
    wait_us(1000);
    car.conducir(-GOING_BACK_AFTER_GRAB_SPEED*1.2, 450);
    gripperMotor->stop();
    wait_us(1000);
    for(int i = 100; i>=50; i--){
        *shakyMotor = i/100.0;
        wait_us(90000);
    } 
}
//trying is either linefollowing or servoo or shaky or ultrasonic
#elif trying == 1
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


bool checkForReds(I2C &i2c0, float redRatio, bool isRedd[5] ){
    int sensor_addr = 41 << 1;
    for (int i = 1; i <= 5; i++) {
        i2c_sw.select(i);               //  select  the channel 0

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
        bool checkIfRed = redRatio <= red_value/total;
        if (checkIfRed) {
            isRedd[i-1] = 1;
        }
    }
    return isRedd[2];        
}

bool checkForProximity(){

//trigger, echo in this order
    HCSR04 USsensor(D8, D9);
    float distance = USsensor.distance();   
    //4-7 is the range we want
    return distance<6;
}
int main() {
    float normalSpeed = 0.22;
    float Kp = 0.08;
    float Kd = 0.06;
    float Ki = 0;
    float I = 0;
    float e_prev = 0;
    float redRatio = 0.41;
    float greenRatio = 0.43;    
    float blueRatio = 0.5;
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

    printf("Calibration is starting");
    led = !led;
    wait_us(3000000);
    led = !led;
    // for (int iterations1 = 0; iterations1 < 3000; iterations1++) {
    //     for(int i = 1; i <= 5; i++) {
    //         i2c_sw.select(i);               //  select  the channel 2
    //         char clear_reg[1] = {148};
    //         char clear_data[2] = {0,0};
    //         i2c0.write(sensor_addr,clear_reg,1, true);
    //         i2c0.read(sensor_addr,clear_data,2, false);
    //         int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
    //         //printf("%d ", clear_value);
    //         if (clear_value < minClearValues[i-1]) {
    //             minClearValues[i-1] = clear_value;
    //         } else if (clear_value > maxClearValues[i-1]) {
    //             maxClearValues[i-1] = clear_value;
    //         }
    //     }
    // }
    //maxClearValues[4] = (maxClearValues[0] + maxClearValues[1] + maxClearValues[2] + maxClearValues[3])/4;

   printf("Max values are %d, %d, %d, %d, %d\n", maxClearValues[0], maxClearValues[1], maxClearValues[2], maxClearValues[3], maxClearValues[4]);
    printf("Min values are %d, %d, %d, %d, %d\n", minClearValues[0], minClearValues[1], minClearValues[2], minClearValues[3], minClearValues[4]);


    // led = !led;
    int currentSensor = 0;
    bool left = false;
    bool right = false;
    int iterations2 = 250;
    // printf("Green");
    printf("Entering While loop");
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
                printf("%f " , red_value/total);
                bool checkIfRed = redRatio <= red_value/total;
                if (checkIfRed) {
                    isRed[i-1] = 1;
                }

                //printf("%f ", green_value/total);
                if (i == 5) iterations2--;
                if (iterations2 == 0) return 1;
                
                
                clearValuePercentage[i-1] = ((float)clear_value - minClearValues[i-1])/(maxClearValues[i-1] - minClearValues[i-1]);

               // printf(" %f ", blue_value/total);
                // if (stopAtBlue == true && blueRatio <= blue_value/total) {
                // // while(1) {
                //     //printf("IN STOP AT BLUE");
                //         car.detener();
                //         wait_us(3000000);
                //         car.conducir(-0.18, 500);
                //         stopAtBlue = false;
                //         //look for man using us
                //         //move back a bit
                //         bool isRedd[5] = {0, 0, 0, 0, 0};
                //         bool stopLooking  = false;
                //         while(!checkForReds(i2c0, redRatio, isRedd))
                //         {
                //            // move motors back n forth
                //            if (!stopLooking){
                //                 for(int i = 0; i<5;i++){
                //                     if(isRedd[i]){
                //                         stopLooking = true;
                //                         int sign = 1;
                //                         if (i < 2) sign = -1;
                //                         car.pivotar(0.25*sign);
                //                         break;
                //                     }
                //                 }
                //             }
                //         }
                //         car.detener();
                //         wait_us(3000000);
                //         HCSR04 USsensor(D8, D9);
                //         float distance = USsensor.distance();  
                //         while(distance>5){ //somewhere between 4-7
                //             car.conducir(0.2);
                //             printf("%f\n", distance);
                //             wait_us(2000);
                //             distance = USsensor.distance();  
                //         } 
                //         printf("%f\n", distance);
                //         car.detener();

                //         //while ultrasonic does not see lego man, move forwards slowly
                //         CloseGripper(gripperMotor);
                //         wait_us(3000000);
                //         // OpenGripper(gripperMotor);
                //         // wait_us(3000000);

                //         //turn
                // // }
                //     checkForReds(i2c0, redRatio, isRedd);
                //     while (!isRedd[0]) {
                //         car.pivotar(-0.25);
                //         checkForReds(i2c0, redRatio, isRedd);
                //     }
                // }
                // if (stopAtGreen == true && greenRatio <= green_value/total) {
                //     while(1) {
                //         car.detener();
                //     }
                // }
            }
            printf("\n");

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
            // car.motorIzq(leftMotorSpeed);
            // // Right Engine
            // car.motorDer(rightMotorSpeed);
                
            // printf("%f ", e);
            // printf("%f, %f, %f, %f, %f, %d, %d\n", clearValuePercentage[0], clearValuePercentage[1], clearValuePercentage[2], clearValuePercentage[3], clearValuePercentage[4], left, right);
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
    Servo* gripperMotor = new Servo(PA_6);
    //first param is the range in seconds from middle to end and second param is the angle from middle to end
    //the second param degrees doesnt do anything. 
    //to control the angle, 0.0005 gave ~100 degrees range
    //basically for the gripper, we had to recalibrate it because 0 wasnt exactly the closing position 
    //so i set the range to be greater than 90 degrees but then never use anything in the 50~100 of that range
    //because that would cause the gripper to bend more and break
    //so set the range wider and use just 0~40% of that range

    //40 is open, 0 is closed
    //dont change this value
    gripperMotor->calibrate(0.00095,45);

    // for (int i = 0; i<10; i++)
    // {
    //     led = !led;        
    //     ThisThread::sleep_for(BLINKING_RATE);
    // } 

    // while(1) 
    // {
    //     for(int i=0; i<40; i++) 
    //     {
    //         gripperMotor = i/100.0;
    //         wait_us(20000);
    //     }       
        for(int i =40; i>0; i--) 
        {
            *gripperMotor = i/100.0;
            wait_us(20000);      
        }
        delete gripperMotor;
    //     gripperMotor = 0.4;
    //  }    

    wait_us(3000000);
    Servo* gripperMotorr = new Servo(PA_6);
    //first param is the range in seconds from middle to end and second param is the angle from middle to end
    //the second param degrees doesnt do anything. 
    //to control the angle, 0.0005 gave ~100 degrees range
    //basically for the gripper, we had to recalibrate it because 0 wasnt exactly the closing position 
    //so i set the range to be greater than 90 degrees but then never use anything in the 50~100 of that range
    //because that would cause the gripper to bend more and break
    //so set the range wider and use just 0~40% of that range

    //40 is open, 0 is closed
    //dont change this value
    gripperMotorr->calibrate(0.00095,45);
}

#elif trying ==  3

#include "mbed.h"
#include "ServoSimon/Servo.h"

int main(){

    Servo* gripperMotor = new Servo(PA_6);
    gripperMotor->calibrate(0.0009,45);
    Servo* shakyMotor= new Servo(PA_5);
    shakyMotor->calibrate(0.0006,45);

    shakyMotor->stop();
    printf("stopping shakey \n");
    wait_us(2000000);

    for(int i =40; i>0; i--) 
    {
        *gripperMotor = i/100.0;
        wait_us(20000);      
    }


    
    gripperMotor->stop();
    shakyMotor->resume();

    printf("stopping gripper \n");
    wait_us(2000000);

    for(int i = 50; i<90; i++){
        *shakyMotor = i/100.0;
        wait_us(5000);
        printf("%d\n", i);
    }

    while(1){}

    delete gripperMotor;
    delete shakyMotor;

    // wait_us(2000000);  
    // Servo shakyMotor(PA_5);
    // shakyMotor.calibrate(0.0006,45);
    // wait_us(2000000);      

        

    // //dont change these values
    // // shakyMotor = 0;
    // for(int i = 50; i<90; i++){
    //     shakyMotor = i/100.0;
    //     wait_us(5000);
    //     printf("%d\n", i);
    // }
    //90 is left corner, 0 is right corner
    // while(1) 
    // {
        //it was leaning more towards the left side so it goes from 10-80 not 20-80
        // for(int i=0; i<90; i++) 
        // {
        //     shakyMotor = i/100.0;
        //     wait_us(20000);
        // }       
    //     for(int i=90; i>0; i--) 
    //     {
    //         shakyMotor = i/100.0;
    //         wait_us(20000);      
    //     }
    //  }    
}

#elif trying ==4
#include "mbed.h"
#include "HCSR04Antonio/HCSR04.h"
#include "ServoSimon/Servo.h"
#include "tca9548a.h"

//trigger, echo in this order
HCSR04 USsensor(D8, D9);
TCA9548A i2c_sw(PB_9, PB_8); //default address 0x70 applied

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

bool checkForReds(I2C &i2c0, float redRatio, bool isRedd[5] ) {
    int sensor_addr = 41 << 1;
    for (int i = 1; i <= 5; i++) {
        i2c_sw.select(i);               //  select  the channel 0

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
        printf("red_value sensor %d %d", i, red_value);

        float total = red_value+blue_value+green_value;
        bool checkIfRed = redRatio <= red_value/total;
        if (checkIfRed) {
            isRedd[i-1] = 1;
        }
    }
    return isRedd[2];        
}

int main()
{   
    Servo gripperMotor(PA_6);
    gripperMotor.calibrate(0.0009,45);
    Servo shakyMotor(PA_5);
    shakyMotor.calibrate(0.0006,45);
    wait_us(7000000);
    for (int i = 0; i<20; i++){
       
        float distance = USsensor.distance();   
        printf("distanza  %f  \n",distance);
        wait_us(100000); 
    }

    CloseGripper(gripperMotor);
    wait_us(3000000);
    OpenGripper(gripperMotor);
    wait_us(3000000);

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
    wait_us(3000000);

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
    while(1){
        bool red[5] = {0,0,0,0,0};
        checkForReds(i2c0, 0.42, red);
        for(int i = 0; i<5; i++){
            printf("%d ", red[i]);
        }
        printf("\n");
    }

 

}

#endif
