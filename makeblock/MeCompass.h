#ifndef _MECOMPASS_H_
#define _MECOMPASS_H_


#include <Wire.h>
#include <EEPROM.h>
#include <Makeblock.h>
#include "I2Cdev.h"

#include <SoftwareSerial.h>

#define COMPASS_SERIAL_DEBUG




// Me Compass only has one address
#define COMPASS_DEFAULT_ADDRESS    0x1E    


//Me Compass Register Address
#define COMPASS_RA_CONFIG_A        0x00
#define COMPASS_RA_CONFIG_B        0x01
#define COMPASS_RA_MODE            0x02
#define COMPASS_RA_DATAX_H         0x03
#define COMPASS_RA_DATAX_L         0x04
#define COMPASS_RA_DATAZ_H         0x05
#define COMPASS_RA_DATAZ_L         0x06
#define COMPASS_RA_DATAY_H         0x07
#define COMPASS_RA_DATAY_L         0x08
#define COMPASS_RA_STATUS          0x09
#define COMPASS_RA_ID_A            0x0A
#define COMPASS_RA_ID_B            0x0B
#define COMPASS_RA_ID_C            0x0C


//define number of samples averaged per measurement
#define COMPASS_AVERAGING_1        0x00
#define COMPASS_AVERAGING_2        0x20
#define COMPASS_AVERAGING_4        0x40
#define COMPASS_AVERAGING_8        0x60


//define data output rate value (Hz)
#define COMPASS_RATE_0P75          0x00   // 0.75 (Hz)
#define COMPASS_RATE_1P5           0x40   // 1.5  (Hz)
#define COMPASS_RATE_3             0x08   // 3    (Hz)
#define COMPASS_RATE_7P5           0x0C   // 7.5  (Hz)
#define COMPASS_RATE_15            0x10   // 15   (Hz)
#define COMPASS_RATE_30            0x14   // 30   (Hz)
#define COMPASS_RATE_75            0x18   // 75   (Hz)


//define measurement bias value
#define COMPASS_BIAS_NORMAL        0x00
#define COMPASS_BIAS_POSITIVE      0x01
#define COMPASS_BIAS_NEGATIVE      0x02


//define magnetic field gain value 
/* -+-------------+-----------------
 *  | Field Range | Gain (LSB/Gauss)
 * -+-------------+-----------------
 *  | +/- 0.88 Ga | 1370
 *  | +/- 1.3 Ga  | 1090 (Default)
 *  | +/- 1.9 Ga  | 820
 *  | +/- 2.5 Ga  | 660
 *  | +/- 4.0 Ga  | 440
 *  | +/- 4.7 Ga  | 390
 *  | +/- 5.6 Ga  | 330
 *  | +/- 8.1 Ga  | 230
 * -+-------------+-----------------*/
#define COMPASS_GAIN_1370          0x00
#define COMPASS_GAIN_1090          0x20
#define COMPASS_GAIN_820           0x40
#define COMPASS_GAIN_660           0x60
#define COMPASS_GAIN_440           0x80
#define COMPASS_GAIN_390           0xA0
#define COMPASS_GAIN_330           0xC0
#define COMPASS_GAIN_220           0xE0


//define measurement mode
#define COMPASS_MODE_CONTINUOUS    0x00
#define COMPASS_MODE_SINGLE        0x01
#define COMPASS_MODE_IDLE          0x02

//define others parameter
#define COMPASS_PI 3.14159265F
#define START_ADDRESS_OF_EEPROM_BUFFER  (int)(0x00)


struct Compass_Calibration_Parameter
{
    float X_excursion;
    float Y_excursion;
    float Z_excursion;
    float X_gain;
    float Y_gain;
    float Z_gain;
    uint8_t Rotation_Axis;   //1:X_Axis   2:Y_Axis   3:Z_Axis

    uint8_t verify_flag;
};

/*        Me Compass Module         */
class MeCompass:public MePort
{
public:
    MeCompass();
    MeCompass(uint8_t port);
    MeCompass(uint8_t port, uint8_t address);
    
    void init(void);
    bool testConnection(void);
    double getAngle(void);

    int16_t getHeadingX(void);
    int16_t getHeadingY(void);
    int16_t getHeadingZ(void);
    void getHeading(int16_t *x, int16_t *y, int16_t *z);

private:
	bool Calibration_Flag;
    uint8_t buffer[6];
	uint8_t Device_Address;
	uint8_t Measurement_Mode;
    struct Compass_Calibration_Parameter Cal_parameter;

	void deviceCalibration(void);
    void read_EEPROM_Buffer();
    void write_EEPROM_Buffer(struct Compass_Calibration_Parameter *parameter_pointer);
};





#endif   /*   _MECOMPASS_H_   */





