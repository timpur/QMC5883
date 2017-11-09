
#include "QMC5883.h"

QMC5883::QMC5883(uint8_t addr){
	address = addr;
}

void QMC5883::begin(){
	setOffset(0, 0, 0);
	
	Wire.begin();
	
	softReset();
	delay(10);
	
	// Default Settings	
	write8(QMC5883_REG_PERIOD, 0x01); //Define Set/Reset period
	setMode(QMC5883_MODE_CONTINUOUS);
	setDataRate(QMC5883_DATARATE_10);
	setRange(QMC5883_RANGE_8);
	setSamples(QMC5883_SAMPLES_512);
}


Vector QMC5883::readRaw()
{
	Vector v = {};
	
	v.XAxis = read16(QMC5883_REG_OUT_X_LSB, false) - xOffset;
	v.YAxis = read16(QMC5883_REG_OUT_Y_LSB, false) - yOffset;
	v.ZAxis = read16(QMC5883_REG_OUT_Z_LSB, true) - zOffset;

	return v;
}

Vector QMC5883::readNormalize()
{
	Vector v = {};
	
	v.XAxis = (float)(read16(QMC5883_REG_OUT_X_LSB, false) - xOffset) * mgPerDigit;
	v.YAxis = (float)(read16(QMC5883_REG_OUT_Y_LSB, false) - yOffset) * mgPerDigit;
	v.ZAxis = (float)(read16(QMC5883_REG_OUT_Z_LSB, true) - zOffset) * mgPerDigit;

	return v;
}

float QMC5883::readHeading(float offset, bool invert){	
	Vector norm = readNormalize();
	
	 // Calculate heading
	float heading = atan2(norm.YAxis, norm.XAxis);
	
	// Set declination angle on your location and fix heading
	// You can find your declination on: http://magnetic-declination.com/
	// (+) Positive or (-) for negative
	// Formula: (deg + (min / 60.0)) / (180 / PI);
	//float declinationAngle = (12.0 + (34.0 / 60.0)) / (180 / PI);
	//heading += declinationAngle;

	// Convert to degrees
	float headingDegrees = heading * (180.0 / PI); 
	
	headingDegrees = convertHeading(headingDegrees, offset, invert);
	
	return headingDegrees;
}

float QMC5883::convertHeading(float heading, float offset, bool invert){
	if (invert)
		heading = 360.0 - heading;
	heading += offset;
	if (heading < 0.0)
		heading += 360.0;
	else if (heading > 360.0)
		heading -= 360.0;
	return heading;
}


void QMC5883::setOffset(uint16_t xo, uint16_t yo, uint16_t zo)
{
	xOffset = xo;
	yOffset = yo;
	zOffset = zo;
}


void QMC5883::setMode(qmc5883_mode_t mode)
{
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_1);
	value &= 0b11111100;
	value |= mode;

	write8(QMC5883_REG_CONTROL_1, value);
}
qmc5883_mode_t QMC5883::getMode(){
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_1);
	value &= 0b00000011;

	return (qmc5883_mode_t)value;
}


void QMC5883::setDataRate(qmc5883_dataRate_t dataRate)
{
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_1);
	value &= 0b11110011;
	value |= (dataRate << 2);

	write8(QMC5883_REG_CONTROL_1, value);
}
qmc5883_dataRate_t QMC5883::getDataRate(){
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_1);
	value &= 0b00001100;
	value >>= 2;

	return (qmc5883_dataRate_t)value;
}


void QMC5883::setRange(qmc5883_range_t range)
{
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_1);
	value &= 0b11001111;
	value |= (range << 4);

	write8(QMC5883_REG_CONTROL_1, value);
	
	switch(range)
	{
		case QMC5883_RANGE_8:
			mgPerDigit = 0.08330;
			break;

		case QMC5883_RANGE_2:
			mgPerDigit = 0.33332;
			break;
	}

}
qmc5883_range_t QMC5883::getRange(){
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_1);
	value &= 0b00110000;
	value >>= 4;

	return (qmc5883_range_t)value;
}


void QMC5883::setSamples(qmc5883_samples_t sample)
{
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_1);
	value &= 0b00111111;
	value |= (sample << 6);

	write8(QMC5883_REG_CONTROL_1, value);
}
qmc5883_samples_t QMC5883::getSamples(){
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_1);
	value &= 0b11000000;
	value >>= 6;

	return (qmc5883_samples_t)value;
}


void QMC5883::setInterrupt(bool interrupt){
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_2);
	value &= 0b11111110;
	value |= interrupt;

	write8(QMC5883_REG_CONTROL_2, value);
}
bool QMC5883::getInterrupt(){
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_2);
	value &= 0b00000001;

	return (bool)value;
}


void QMC5883::setRollOver(bool rollover){
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_2);
	value &= 0b10111111;
	value |= (rollover << 6);

	write8(QMC5883_REG_CONTROL_2, value);
}
bool QMC5883::getRollOver(){
	uint8_t value;

	value = read8(QMC5883_REG_CONTROL_2);
	value &= 0b01000000;
	value >>= 6;

	return (bool)value;
}


void QMC5883::softReset(){
	write8(QMC5883_REG_CONTROL_2, (true << 7));
}


uint8_t QMC5883::read8(uint8_t reg, uint8_t stop){
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(address, (uint8_t)1, stop);	
	return Wire.read();
}

void QMC5883::write8(uint8_t reg, uint8_t val, uint8_t stop){
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(val);
	Wire.endTransmission(stop);
}

int16_t QMC5883::read16(uint8_t reg, uint8_t stop){
	int16_t val;

	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(address, (uint8_t)2, stop);
	val = Wire.read(); //LSB
	val |= Wire.read() << 8; //MSB
	
	return val;
}
