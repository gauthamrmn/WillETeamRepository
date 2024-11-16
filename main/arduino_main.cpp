// Bare minimum code for spinning motors triggered by controller input

// Assumes servo is connected to pin 15

// Assumes motor controller IN1 and IN2 are connected to pins 14 and 12

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <bits/stdc++.h>
#include <Wire.h>
#include <Arduino_APDS9960.h>
#include <QTRSensors.h>
#include <ESP32SharpIR.h>


#define IN1a 17
#define IN1b 5
#define IN2a 12
#define IN2b 14
#define MY_BLUE_LED_PIN 2

#define APDS9960_INT 0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000


ESP32SharpIR SensorFront(ESP32SharpIR::GP2Y0A21YK0F, 2);
ESP32SharpIR SensorLeft(ESP32SharpIR::GP2Y0A21YK0F, 4);
ESP32SharpIR SensorRight(ESP32SharpIR::GP2Y0A21YK0F, 13);

TwoWire I2C_0 = TwoWire(0);
APDS9960 sensor =  APDS9960(I2C_0, APDS9960_INT);


QTRSensors qtr;
uint16_t sensors[8];

uint16_t black=0;
Servo servo;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];



// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}


static void TurnLeft(){
    //left doesn't move
    digitalWrite(IN1a, HIGH);
    digitalWrite(IN1b, HIGH);
    // // right moves forward 
    digitalWrite(IN2a, HIGH);
    digitalWrite(IN2b, LOW);
    Serial.println("turn left");

}

static void TurnRight(){
    // left moves forward
    digitalWrite(IN1a, HIGH);
    digitalWrite(IN1b, LOW);
    // right doesn't move
    digitalWrite(IN2a, LOW);
    digitalWrite(IN2b, LOW);
    Serial.print("Right\n");
}



static void STOP(){
    // right doesn't move
    digitalWrite(IN1a, LOW);
    digitalWrite(IN1b, HIGH);
    // left doesn't move
    digitalWrite(IN2a, LOW);
    digitalWrite(IN2b, LOW);
    delay(20);
    Serial.print("STOP\n");
}

static void MoveForward(){
    // left moves forward 
    digitalWrite(IN1a, HIGH);
    digitalWrite(IN1b, LOW);
    // right moves forward
    digitalWrite(IN2a, HIGH);
    digitalWrite(IN2b, LOW);
    Serial.print("Forward\n");
}



void setup() {

    Serial.begin(115200);
    SensorFront.setFilterRate(1.0f);
    SensorLeft.setFilterRate(1.0f);
    SensorRight.setFilterRate(1.0f);
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]) {26,25,33,32,35,34,39,36}, 8);

    //calibration sequence
  


    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
 
    servo.attach(15);

    // motor controller outputs
    pinMode(IN1a, OUTPUT); // problem wire, I4
    pinMode(IN1b, OUTPUT); //problem wire, I3 
    pinMode(IN2a, OUTPUT);
    pinMode(IN2b, OUTPUT);
    pinMode(MY_BLUE_LED_PIN, OUTPUT);

    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    sensor.setInterruptPin(APDS9960_INT);
    sensor.begin();
    Serial.begin(115200);
}

void color_sensor(){
        Serial.println("entering color sensor function");
    // while(!sensor.colorAvailable()){
    //     delay(5);
    // }
    // digitalWrite(MY_BLUE_LED_PIN, HIGH);
    bool match=false;
    int color_index=0;

    int r, g, b, a;
while(1){
    while(!sensor.colorAvailable()){\
        Serial.print(sensor.colorAvailable());
        delay(5);
        Serial.println("lolpt2");
    }
    sensor.readColor(r, g, b, a);
    Serial.print("r = ");
    Serial.print(r); 
    Serial.print(" g = ");
    Serial.print(g);
    Serial.print(" b = ");
    Serial.println(b);
    vTaskDelay(1);
}
    if (r>g && r>b){
        color_index=0;
    } 
    if (g>r && g>b){
        color_index=1;
    } 
    if (b>r && b>g){
        color_index=2;        
    } 

    while(match==false){
        sensor.readColor(r, g, b, a);
        MoveForward();
        if ((color_index==0) &&(r>g && r>b)){
            STOP();
            Serial.print("Red");
            match=true;
            return;
        }
         if ((color_index==1) &&(g>r && g>b)){
            STOP();
            Serial.print("Green");
            match=true;
            return;
        }
         if ((color_index==2) &&(b>r && b>g)){
            STOP();
            Serial.print("Blue");
            match=true;
            return;
        }

    }
}

void line_sensor(){
    uint16_t sensors[8];
    while(1){
    qtr.readLineBlack(sensors);
    Serial.print(sensors[0]);
        Serial.print(" ");
        Serial.println(sensors[1]);
        // Serial.print(" ");
        // Serial.println(sensors[2]);
        // Serial.print(" ");
        // Serial.println(sensors[3]);
        // Serial.print(" ");
        // Serial.println(sensors[4]);
        // Serial.print(" ");
        // Serial.println(sensors[5]);
        // Serial.print(" ");
        // Serial.println(sensors[6]);
        // Serial.print(" ");
        // Serial.println(sensors[7]);
        delay(250); 
        delay(150);
        TurnRight();
        delay(150);
        TurnLeft();
        delay(150);
        int flag=0;
        if (((sensors[0]<25)||(sensors[1]<25)||(sensors[2]<25))){
                //left side
                //spin right untill 4 or 3  are black
            while(flag==0){
                TurnLeft();
                delay(150);
                STOP();
                delay(900);
                if ((sensors[2]>25)&&(sensors[1]>25)&&(sensors[0]>25)&&((sensors[4]<25)||(sensors[3]<25))&&(sensors[5]>25)&&(sensors[6]>25)&&(sensors[7]>25)){
                    MoveForward();
                    flag=1;
                    }}}
        if (((sensors[5]<25)||(sensors[6]<25)||(sensors[7]<25))){
            TurnRight();
            delay(150);
            STOP();
            delay(900);
                //spin right untill 4 or 3  are black
            while(flag==0){
                TurnRight();
                if ((sensors[2]>25)&&(sensors[1]>25)&&(sensors[0]>25)&&((sensors[4]<25)||(sensors[3]<25))&&(sensors[5]>25)&&(sensors[6]>25)&&(sensors[7]>25)){
                    MoveForward();
                    flag=1;
                }
            }
        }
                
    //         }

    //         if ((sensors[5]!=0)||(sensors[6]!=0)||(sensors[7]!=0)){
    //             //right
    //         }
    if (((sensors[2]>25)&&(sensors[1]>25)&&(sensors[0]>25)&&((sensors[4]>25)&&(sensors[3]>25))&&(sensors[5]>25)&&(sensors[6]>25)&&(sensors[7]>25))){ 
        return;
    }


    }
}


void arm_movement(){}

void wall_sensor(){

    while(1){
        // digitalWrite(MY_BLUE_LED_PIN, HIGH);
        // Serial.print("Front: "); 
        // Serial.print(SensorFront.getDistanceFloat());
        // Serial.print("Left: "); 
        // Serial.print(SensorLeft.getDistanceFloat());
        // Serial.print("Right: "); 
        // Serial.println(SensorRight.getDistanceFloat());
        MoveForward();

        //turn left
        while ((SensorLeft.getDistanceFloat()>15)){
            TurnLeft();
            delay(150);
             STOP();
            delay(900);
        }
         while ((SensorRight.getDistanceFloat()>15)){
            TurnRight();
            delay(150);
             STOP();
            delay(900);
        }
      if ((SensorLeft.getDistanceFloat()>50)&&(SensorRight.getDistanceFloat()>50)&&(SensorFront.getDistanceFloat()>50)){
        return;
      }
    }

}

void loop() {
    //digitalWrite(MY_BLUE_LED_PIN, HIGH);  // Turn the LED off by making the voltage HIGH

    BP32.update();
    color_sensor();
  
        
     for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
         GamepadPtr controller = myGamepads[i];
         if (controller && controller->isConnected()) {
           
             if (controller->l1() == 1) {
                 Serial.print("Servo move");
                 servo.write(1000);
             }
             if (controller->l1() == 0) {
                 Serial.print("Servo stop");
                 servo.write(1500);
             }

             if(controller->axisY() < 0) { // negative y is upward on stick
                 Serial.println(" DC motor move");
                 //digitalWrite(IN1, LOW);
                 //digitalWrite(IN2, HIGH);
             }
             if(controller->axisY() == 0) { // stop motor 1
                 Serial.println(" DC motor stop");
             //digitalWrite(IN1, LOW);
                 //digitalWrite(IN2, LOW);
             }

             // PHYSICAL BUTTON A  -> CHECKS COLOR SENSOR
             if (controller->b()) {  
                 Serial.println("button a pressed");
                    //Will_mode=1e;
                    color_sensor();
             }

             // PHYSICAL BUTTON B -> CHECKS LINE SENSOR
            if (controller->a()){
                Serial.println("button b pressed");
                for(uint8_t i = 0; i < 250; i++){
                    Serial.println("calibrating");
                    qtr.calibrate();
                    delay(20);
                    }
                line_sensor();
            }

            //  PHYSICAL BUTTON Y - > CHECKS WALL SENSOR
            if (controller->x()){
                Serial.println("button y pressed");
                wall_sensor();
            }

            //  PHYSICAL BUTTON X -> ARM MOVEMENT
            if (controller->y()){
                Serial.println("button x pressed");
            }
         }
         vTaskDelay(1);
     }
     
} 
   
        



