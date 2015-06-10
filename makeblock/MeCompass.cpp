#include "MeCompass.h"
#include <EEPROM.h>


MeCompass::MeCompass():MePort()
{
    pinMode(s1, INPUT);        //KEY pin
    pinMode(s2, OUTPUT);       //LED pin
    digitalWrite(s2,HIGH);

    Device_Address = 0x00;
    Calibration_Flag=false;
}


MeCompass::MeCompass(uint8_t port):MePort(port)
{
	pinMode(s1, INPUT);        //KEY pin
	pinMode(s2, OUTPUT);       //LED pin
    digitalWrite(s2,HIGH);

    Device_Address = COMPASS_DEFAULT_ADDRESS;
    Calibration_Flag=false;
}


MeCompass::MeCompass(uint8_t port, uint8_t address):MePort(port)
{
	pinMode(s1, INPUT);        //KEY pin
	pinMode(s2, OUTPUT);       //LED pin	
    digitalWrite(s2,HIGH);

    Device_Address = address;
    Calibration_Flag=false;
}


void MeCompass::init(void)
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

 //   #ifdef COMPASS_SERIAL_DEBUG
 //       Serial.begin(115200);
 //   #endif

    // write CONFIG_A register
    I2Cdev::writeByte(Device_Address, COMPASS_RA_CONFIG_A, 
    	COMPASS_AVERAGING_8 | COMPASS_RATE_15 | COMPASS_BIAS_NORMAL);

    // write CONFIG_B register
    I2Cdev::writeByte(Device_Address, COMPASS_RA_CONFIG_B, COMPASS_GAIN_1090);
    
    // write MODE register
    Measurement_Mode = COMPASS_MODE_SINGLE;
    I2Cdev::writeByte(Device_Address, COMPASS_RA_MODE, Measurement_Mode);

    read_EEPROM_Buffer();

    deviceCalibration();

}


bool MeCompass::testConnection(void)
{
    if (I2Cdev::readBytes(Device_Address, COMPASS_RA_ID_A, 3, buffer) == 3) 
    {
        return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
    }
    return false;
}


double MeCompass::getAngle(void)
{
	double compass_angle;
	int16_t cx,cy,cz;

    deviceCalibration();

	getHeading(&cx,&cy,&cz);

    if(Calibration_Flag == true)
    {
        cx = (cx + Cal_parameter.X_excursion)*Cal_parameter.X_gain;
        cy = (cy + Cal_parameter.Y_excursion)*Cal_parameter.Y_gain;
        cz = (cz + Cal_parameter.Z_excursion)*Cal_parameter.Z_gain;

        if(Cal_parameter.Rotation_Axis == 1)  //X_Axis
        {
            compass_angle = atan2((double)cy,(double)cz);
        }
    	else if(Cal_parameter.Rotation_Axis == 2)  //Y_Axis
        {
            compass_angle = atan2((double)cx,(double)cz);
        }
        else if(Cal_parameter.Rotation_Axis == 3)  //Z_Axis
        {
            compass_angle = atan2((double)cy,(double)cx);
        }
    }
    else
    {
        compass_angle = atan2((double)cy,(double)cx);
    }

    if(compass_angle < 0)
    {
        compass_angle = (compass_angle + 2 * COMPASS_PI)*180/COMPASS_PI;
    }
    else
    {
        compass_angle = compass_angle *180/COMPASS_PI;
    }
	return compass_angle;
}


int16_t MeCompass::getHeadingX(void)
{
    deviceCalibration();

    I2Cdev::readBytes(Device_Address, COMPASS_RA_DATAX_H, 2, buffer);
    if (Measurement_Mode == COMPASS_MODE_SINGLE)
    {
    	I2Cdev::writeByte(Device_Address, COMPASS_RA_MODE, COMPASS_MODE_SINGLE);
    } 
    	
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}


int16_t MeCompass::getHeadingY(void)
{
    deviceCalibration();

    I2Cdev::readBytes(Device_Address, COMPASS_RA_DATAY_H, 2, buffer);
    if (Measurement_Mode == COMPASS_MODE_SINGLE)
    {
    	I2Cdev::writeByte(Device_Address, COMPASS_RA_MODE, COMPASS_MODE_SINGLE);
    } 
    	
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}


int16_t MeCompass::getHeadingZ(void)
{
    deviceCalibration();

    I2Cdev::readBytes(Device_Address, COMPASS_RA_DATAZ_H, 2, buffer);
    if (Measurement_Mode == COMPASS_MODE_SINGLE)
    {
    	I2Cdev::writeByte(Device_Address, COMPASS_RA_MODE, COMPASS_MODE_SINGLE);
    } 
    	
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}


void MeCompass::getHeading(int16_t *x, int16_t *y, int16_t *z)
{
    deviceCalibration();

    I2Cdev::readBytes(Device_Address, COMPASS_RA_DATAX_H, 6, buffer);
    if (Measurement_Mode == COMPASS_MODE_SINGLE)
    {
    	I2Cdev::writeByte(Device_Address, COMPASS_RA_MODE, COMPASS_MODE_SINGLE);
    } 
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[4]) << 8) | buffer[5];
    *z = (((int16_t)buffer[2]) << 8) | buffer[3];
}


void MeCompass::deviceCalibration(void)
{
    
    if(dRead1() == 0)   //check the KEY
    {
    	delay(10);
    	if(dRead1() == 0)
    	{
            if(testConnection()==false)
            {
                #ifdef COMPASS_SERIAL_DEBUG
                    Serial.println("It is not Me Compass!!!!!");
                #endif
                    return;
            }

            long time_num,cal_time;
            bool LED_state=0;
            int16_t X_num,Y_num,Z_num;
            int16_t X_max = -32768;
            int16_t X_min = 32767;
            int16_t Y_max = -32768;
            int16_t Y_min = 32767;
            int16_t Z_max = -32768;
            int16_t Z_min = 32767;
            int16_t X_abs,Y_abs,Z_abs;


            #ifdef COMPASS_SERIAL_DEBUG
                Serial.println("Compass calibration !!!");
            #endif

            time_num = millis();
            while(dRead1()==0)
            {
                if(millis()-time_num>200)   //control the LED
                {
                    time_num = millis();
                    LED_state = !LED_state;
                    dWrite2(LED_state);

                    #ifdef COMPASS_SERIAL_DEBUG
                        Serial.println("You can free the KEY now ");
                    #endif
                }
            }

            #ifdef COMPASS_SERIAL_DEBUG
                Serial.println("collecting value.....");
            #endif

            delay(100);

            cal_time = millis();
            do
            {
                if(millis()-time_num>200)   //control the LED
                {
                    time_num = millis();
                    LED_state = !LED_state;
                    dWrite2(LED_state);
                }
                if(millis()-cal_time>10)
                {
                    getHeading(&X_num,&Y_num,&Z_num);

                    if(X_num < X_min)
                    {
                        X_min = X_num;
                    }
                    else if(X_num > X_max)
                    {
                        X_max = X_num;
                    }

                    if(Y_num < Y_min)
                    {
                        Y_min = Y_num;
                    }
                    else if(Y_num > Y_max)
                    {
                        Y_max = Y_num;
                    }

                    if(Z_num < Z_min)
                    {
                        Z_min = Z_num;
                    }
                    else if(Z_num > Z_max)
                    {
                        Z_max = Z_num;
                    }
                }
            }while(dRead1() == 1);

            dWrite2(LOW);  //turn off the LED

            Cal_parameter.X_excursion = -((float)X_max + (float)X_min)/2;
            Cal_parameter.Y_excursion = -((float)Y_max + (float)Y_min)/2;
            Cal_parameter.Z_excursion = -((float)Z_max + (float)Z_min)/2;
            Cal_parameter.X_gain = 1;
            Cal_parameter.Y_gain = ((float)Y_max - (float)Y_min)/((float)X_max - (float)X_min);
            Cal_parameter.Z_gain = ((float)Z_max - (float)Z_min)/((float)X_max - (float)X_min);

            X_abs = abs(X_max-X_min);
            Y_abs = abs(Y_max-Y_min);
            Z_abs = abs(Z_max-Z_min);

            if(X_abs<=Y_abs && X_abs<=Z_abs)
            {
                Cal_parameter.Rotation_Axis=1;  //X_Axis
            }
            else if(Y_abs<=X_abs && Y_abs<=Z_abs)
            {
                Cal_parameter.Rotation_Axis=2;  //Y_Axis
            }
            else
            {
                Cal_parameter.Rotation_Axis=3;  //Z_Axis
            }


            #ifdef COMPASS_SERIAL_DEBUG
                Serial.println("Print Calibration Parameter:");
                Serial.print("X_excursion: "); Serial.print(Cal_parameter.X_excursion,1); Serial.println(" ");
                Serial.print("Y_excursion: "); Serial.print(Cal_parameter.Y_excursion,1); Serial.println(" ");
                Serial.print("Z_excursion: "); Serial.print(Cal_parameter.Z_excursion,1); Serial.println(" ");
                Serial.print("X_gain: ");      Serial.print(Cal_parameter.X_gain,1);      Serial.println(" ");
                Serial.print("Y_gain: ");      Serial.print(Cal_parameter.Y_gain,1);      Serial.println(" ");
                Serial.print("Z_gain: ");      Serial.print(Cal_parameter.Z_gain,1);      Serial.println(" "); 
                Serial.print("Axis=   ");      Serial.print(Cal_parameter.Rotation_Axis);      Serial.println(" ");
            #endif

            write_EEPROM_Buffer(&Cal_parameter);

            dWrite2(HIGH);   //turn on the LED
            while(dRead1() == 0);
            delay(100);
        }
    }
}


void MeCompass::read_EEPROM_Buffer()
{
    uint8_t parameter_buffer[sizeof(Compass_Calibration_Parameter)];
    struct Compass_Calibration_Parameter *parameter_pointer;
    uint8_t verify_number;


    for(int address =0x00; address<sizeof(Compass_Calibration_Parameter); address++)
    {
        parameter_buffer[address]=EEPROM.read(START_ADDRESS_OF_EEPROM_BUFFER + address);
    }

    parameter_pointer = (struct Compass_Calibration_Parameter *)parameter_buffer;

    verify_number =(uint8_t)( parameter_pointer -> X_excursion
                            + parameter_pointer -> Y_excursion
                            + parameter_pointer -> Z_excursion
                            + parameter_pointer -> X_gain
                            + parameter_pointer -> Y_gain
                            + parameter_pointer -> Z_gain
                            + parameter_pointer -> Rotation_Axis 
                            + 0xaa  );

    if(verify_number == parameter_pointer -> verify_flag)
    {
        #ifdef COMPASS_SERIAL_DEBUG
                Serial.println("Verify number is true!!!");
        #endif

        Cal_parameter = (*parameter_pointer);

        Calibration_Flag = true;
    }
    else
    {
        #ifdef COMPASS_SERIAL_DEBUG
                Serial.println("Verify number is false!!!");
        #endif

        Calibration_Flag = false;
    }
}


void MeCompass::write_EEPROM_Buffer(struct Compass_Calibration_Parameter *parameter_pointer)
{
    uint8_t *buffer_pointer;
    uint8_t verify_number;

    parameter_pointer -> verify_flag = (uint8_t)( parameter_pointer -> X_excursion
                                                + parameter_pointer -> Y_excursion
                                                + parameter_pointer -> Z_excursion
                                                + parameter_pointer -> X_gain
                                                + parameter_pointer -> Y_gain
                                                + parameter_pointer -> Z_gain
                                                + parameter_pointer -> Rotation_Axis 
                                                + 0xaa    );

    buffer_pointer = (uint8_t *)parameter_pointer;

    for(int address =0x00; address<sizeof(Compass_Calibration_Parameter); address++)
    {
        EEPROM.write(START_ADDRESS_OF_EEPROM_BUFFER + address, *(buffer_pointer+address));
    }

    Calibration_Flag = true;

    #ifdef COMPASS_SERIAL_DEBUG
        Serial.println("Write EEPROM Buffer Success!!!");
    #endif

}








