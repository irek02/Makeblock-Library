#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <makeblock.h>
#include 'MeCompass.h'

#include "Wire.h"
#include "I2Cdev.h"


MeCompass myCompass(PORT_4);


#define LED_PIN 13

void setup() 
{
    
    Serial.begin(115200);

    Serial.println("Initializing I2C devices...");
    myCompass.init();

    Serial.println("Testing device connections...");
    Serial.println(myCompass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

}

void loop() 
{
    int16_t head_X,head_Y,head_Z;
    double angle_number=0;

    // head_X = myCompass.getHeadingX();
    // head_Y = myCompass.getHeadingY();
    // head_Z = myCompass.getHeadingZ();

    angle_number = myCompass.getAngle();
    
    Serial.println(angle_number, 1);

    // Serial.println(sizeof(Compass_Calibration_Parameter));

    delay(500);

}
