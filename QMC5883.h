// Credit to https://github.com/jarzebski/Arduino-HMC5883L for interface and code.
// Usees a simular interface and includes aditional functionallity from https://github.com/jarzebski/Arduino-HMC5883L examples

#ifndef QMC5883_h
#define QMC5883_h

#include "Arduino.h"
#include <math.h>
#include "Wire.h"

#define QMC5883_ADDRESS			(0x0D)

#define QMC5883_REG_OUT_X_LSB	(0x00)
#define QMC5883_REG_OUT_X_MSB	(0x01)
#define QMC5883_REG_OUT_Y_LSB	(0x02)
#define QMC5883_REG_OUT_Y_MSB	(0x03)
#define QMC5883_REG_OUT_Z_LSB	(0x04)
#define QMC5883_REG_OUT_Z_MSB	(0x05)
#define QMC5883_REG_OUT_T_LSB	(0x07)
#define QMC5883_REG_OUT_T_MSB	(0x08)

#define QMC5883_REG_STATUS		(0x06)
#define QMC5883_REG_CONTROL_1	(0x09)
#define QMC5883_REG_CONTROL_2	(0x0A)
#define QMC5883_REG_PERIOD		(0x0B)

typedef enum
{
	QMC5883_MODE_STANDBY		= 0b00,
	QMC5883_MODE_CONTINUOUS		= 0b01
} qmc5883_mode_t;

typedef enum
{
	QMC5883_DATARATE_200		= 0b11,
	QMC5883_DATARATE_100		= 0b10,
	QMC5883_DATARATE_50			= 0b01,
	QMC5883_DATARATE_10			= 0b00
} qmc5883_dataRate_t;

typedef enum
{
	QMC5883_RANGE_8				= 0b00,
	QMC5883_RANGE_2				= 0b01
} qmc5883_range_t;

typedef enum
{
	QMC5883_SAMPLES_512			= 0b00,
	QMC5883_SAMPLES_256			= 0b01,
	QMC5883_SAMPLES_128			= 0b10,
	QMC5883_SAMPLES_64			= 0b11
} qmc5883_samples_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};
#endif

class QMC5883{
	public:
		QMC5883(uint8_t addr = QMC5883_ADDRESS);
		
		void begin();
		
		Vector readRaw();
		Vector readNormalize();
		float readHeading(float offset = 0.0, bool invert = false);
		float convertHeading(float heading, float offset = 0.0, bool invert = false);
		
		void setOffset(uint16_t xo, uint16_t yo, uint16_t zo);
		
		void  setMode(qmc5883_mode_t mode);
		qmc5883_mode_t getMode();
		
		void  setDataRate(qmc5883_dataRate_t dataRate);
		qmc5883_dataRate_t getDataRate();

		void  setRange(qmc5883_range_t range);
		qmc5883_range_t getRange();
		
		void  setSamples(qmc5883_samples_t samples);
		qmc5883_samples_t getSamples();
		
		void  setInterrupt(bool interrupt);
		bool getInterrupt();
		
		void  setRollOver(bool rollover);
		bool getRollOver();
		
		void softReset();
		
	private:
		uint8_t address;
		float mgPerDigit;
		int16_t xOffset, yOffset, zOffset;
		
		uint8_t read8(uint8_t reg, uint8_t stop=true);
		void write8(uint8_t reg, uint8_t val, uint8_t stop=true);
		int16_t read16(uint8_t reg, uint8_t stop=true);
			
};

#endif
