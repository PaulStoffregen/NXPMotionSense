#include "NXPMotionSense.h"
#include "utility/NXPSensorRegisters.h"
#include <util/crc16.h>
#include <elapsedMillis.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

bool NXPMotionSense::begin()
{
	unsigned char buf[NXP_MOTION_CAL_SIZE];
	uint8_t i;
	uint16_t crc;

	Wire.begin();
	Wire.setClock(400000);

	memset(accel_mag_raw, 0, sizeof(accel_mag_raw));
	memset(gyro_raw, 0, sizeof(gyro_raw));

	//Serial.println("init hardware");
	while (!FXOS8700_begin()) {
		Serial.println("config error FXOS8700");
		delay(1000);
	}
	while (!FXAS21002_begin()) {
		Serial.println("config error FXAS21002");
		delay(1000);
	}
	while (!MPL3115_begin()) {
		Serial.println("config error MPL3115");
		delay(1000);
	}
	//Serial.println("init done");

	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		buf[i] = EEPROM.read(NXP_MOTION_CAL_EEADDR + i);
	}
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, buf[i]);
	}
	if (crc == 0 && buf[0] == 117 && buf[1] == 84) {
		memcpy(cal, buf+2, sizeof(cal));
	} else {
		memset(cal, 0, sizeof(cal));
		cal[9] = 50.0f;
	}
	return true;

}


void NXPMotionSense::update()
{
	static elapsedMillis msec;
	int32_t alt;

	if (FXOS8700_read(accel_mag_raw)) { // accel + mag
		//Serial.println("accel+mag");
	}
	if (MPL3115_read()) { // alt
		//Serial.println("alt");
	}
	if (FXAS21002_read(gyro_raw)) {  // gyro
		//Serial.println("gyro");
		newdata = 1;
	}
}


static bool write_reg(uint8_t i2c, uint8_t addr, uint8_t val)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	Wire.write(val);
	return Wire.endTransmission() == 0;
}

uint8_t read_reg(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  delayMicroseconds(250);
  //Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

static bool read_regs(uint8_t i2c, uint8_t addr, uint8_t *data, uint8_t num)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	if (Wire.endTransmission(false) != 0) return false;
	Wire.requestFrom(i2c, num);
	if (Wire.available() != num) return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
	return true;
}

static bool read_regs(uint8_t i2c, uint8_t *data, uint8_t num)
{
	Wire.requestFrom(i2c, num);
	if (Wire.available() != num) return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
	return true;
}

bool NXPMotionSense::FXOS8700_begin()
{
	const uint8_t i2c_addr=FXOS8700_I2C_ADDR0;
	uint8_t b;

	//Serial.println("FXOS8700_begin");
	// detect if chip is present
	if (!read_regs(i2c_addr, FXOS8700_WHO_AM_I, &b, 1)) return false;
	//Serial.printf("FXOS8700 ID = %02X\n", b);
	if (b != 0xC7) return false;
	// place into standby mode
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0)) return false;
	// configure magnetometer
	if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG1, 0x1F)) return false;
	if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG2, 0x20)) return false;
	// configure accelerometer
	if (!write_reg(i2c_addr, FXOS8700_XYZ_DATA_CFG, 0x01)) return false; // 4G range
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG2, 0x02)) return false; // hires
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0x15)) return false; // 100Hz A+M
	//Serial.println("FXOS8700 Configured");
	return true;
}

bool NXPMotionSense::FXOS8700_read(int16_t *data)  // accel + mag
{
	static elapsedMicros usec_since;
	static int32_t usec_history=5000;
	const uint8_t i2c_addr=FXOS8700_I2C_ADDR0;
	uint8_t buf[13];

	int32_t usec = usec_since;
	if (usec + 100 < usec_history) return false;

	if (!read_regs(i2c_addr, FXOS8700_STATUS, buf, 1)) return false;
	if (buf[0] == 0) return false;

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -15) diff = -15;
	else if (diff > 15) diff = 15;
	usec_history += diff;

	if (!read_regs(i2c_addr, FXOS8700_OUT_X_MSB, buf+1, 12)) return false;
	//if (!read_regs(i2c_addr, buf, 13)) return false;

	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	data[3] = (int16_t)((buf[7] << 8) | buf[8]);
	data[4] = (int16_t)((buf[9] << 8) | buf[10]);
	data[5] = (int16_t)((buf[11] << 8) | buf[12]);
	return true;
}

bool NXPMotionSense::FXAS21002_begin()
{
        const uint8_t i2c_addr=FXAS21002_I2C_ADDR0;
        uint8_t b;

	if (!read_regs(i2c_addr, FXAS21002_WHO_AM_I, &b, 1)) return false;
	//Serial.printf("FXAS21002 ID = %02X\n", b);
	if (b != 0xD7) return false;

	// place into standby mode
	if (!write_reg(i2c_addr, FXAS21002_CTRL_REG1, 0)) return false;
	// switch to active mode, 100 Hz output rate
	if (!write_reg(i2c_addr, FXAS21002_CTRL_REG0, 0x00)) return false;
	if (!write_reg(i2c_addr, FXAS21002_CTRL_REG1, 0x0E)) return false;

	//Serial.println("FXAS21002 Configured");
	return true;
}

bool NXPMotionSense::FXAS21002_read(int16_t *data) // gyro
{
	static elapsedMicros usec_since;
	static int32_t usec_history=10000;
	const uint8_t i2c_addr=FXAS21002_I2C_ADDR0;
	uint8_t buf[7];

	int32_t usec = usec_since;
	if (usec + 100 < usec_history) return false;

	if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 1)) return false;
	if (buf[0] == 0) return false;

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -15) diff = -15;
	else if (diff > 15) diff = 15;
	usec_history += diff;
	//Serial.println(usec);

	if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 7)) return false;
	//if (!read_regs(i2c_addr, buf, 7)) return false;

	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	return true;
}

bool NXPMotionSense::MPL3115_begin() // pressure
{
	MPL3115Reset();                // Start off by resetting all registers to the default
	initRealTimeMPL3115();         // initialize the altimeter for realtime data acquisition if communication is OK
	MPL3115SampleRate(SAMPLERATE); // Set oversampling ratio
	MPL3115enableEventflags();     // Set data ready enable
	delay (1000);
	//Serial.println("MPL3115 Configured");
	return true;
}

bool NXPMotionSense::MPL3115_read()
{
	static elapsedMicros usec_since;
	static int32_t usec_history=980000;
	const uint8_t i2c_addr=MPL3115_I2C_ADDR;
	uint8_t buf[6];

	int32_t usec = usec_since;
	if (usec + 500 < usec_history) return false;

	if (!read_regs(i2c_addr, FXAS21002_STATUS, buf, 1)) return false;
	if (buf[0] == 0) return false;

	if (!read_regs(i2c_addr, buf, 6)) return false;

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -1000) diff = -1000;
	else if (diff > 1000) diff = 1000;
	usec_history += diff;

    MPL3115ActiveAltimeterMode(); 
    MPL3115readAltitude();  // Read the altitude

    MPL3115ActiveBarometerMode(); 
    MPL3115readPressure();  // Read the pressure

    const int station_elevation_m = def_sea_press*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = pressure/100; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;
	
	pressure = pressure/100.0;
	//Serial.printf("%02X %d %d: ", buf[0], usec, usec_history);
	//Serial.printf("%2f,%2f, %2f", altitude, pressure, temperature);
	//Serial.println();
	
	return true;
}

bool NXPMotionSense::writeCalibration(const void *data)
{
	const uint8_t *p = (const uint8_t *)data;
	uint16_t crc;
	uint8_t i;

	if (p[0] != 117 || p[1] != 84) return false;
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, p[i]);
	}
	if (crc != 0) return false;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		EEPROM.write(NXP_MOTION_CAL_EEADDR + i, p[i]);
	}
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		if (EEPROM.read(NXP_MOTION_CAL_EEADDR + i) != p[i]) return false;
	}
	memcpy(cal, ((const uint8_t *)data)+2, sizeof(cal));
	return true;
}


//===================================================
//MPL3115 Sensor Calls
//===================================================
void NXPMotionSense::MPL3115readAltitude() // Get altitude in meters and temperature in centigrade
{
  uint8_t rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers 

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(INTP1) == LOW); // Wait for interrupt pin INTP1 to go HIGH
// digitalWrite(INTP1, LOW);  // Reset interrupt pin INTP1
// while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
 while ((read_reg(MPL3115_I2C_ADDR, MPL3115_STATUS) & 0x08) == 0);  MPL3115toggleOneShot(); 
  
  read_regs(MPL3115_I2C_ADDR, MPL3115_OUT_P_MSB, &rawData[0], 5);  // Read the five raw data registers into data array

// Altutude bytes-whole altitude contained defined by msb, csb, and first two bits of lsb, fraction by next two bits of lsb
  uint8_t msbA = rawData[0];
  uint8_t csbA = rawData[1];
  uint8_t lsbA = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4];
 
 // Calculate altitude, check for negative sign in altimeter data
 long foo = 0;
 if(msbA > 0x7F) {
   foo = ~((long)msbA << 16 | (long)csbA << 8 | (long)lsbA) + 1; // 2's complement the data
   altitude = (float) (foo >> 8) + (float) ((lsbA >> 4)/16.0); // Whole number plus fraction altitude in meters for negative altitude
   altitude *= -1.;
 }
 else {
   altitude = (float) ( (msbA << 8) | csbA) + (float) ((lsbA >> 4)/16.0);  // Whole number plus fraction altitude in meters
 }

 //Adafruit takes one more step:
 //altitude /= 65536.0;
 
// Calculate temperature, check for negative sign
if(msbT > 0x7F) {
 foo = ~(msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) (foo >> 8) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

void NXPMotionSense::MPL3115readPressure()
{
  uint8_t  rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
// while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
 while ((read_reg(MPL3115_I2C_ADDR, MPL3115_STATUS) & 0x08) == 0); MPL3115toggleOneShot(); 
 
  read_regs(MPL3115_I2C_ADDR, MPL3115_OUT_P_MSB, &rawData[0], 5);  // Read the five raw data registers into data array

// Pressure bytes
  uint8_t msbP = rawData[0];
  uint8_t csbP = rawData[1];
  uint8_t lsbP = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4]; 
 
  long pressure_whole =   ((long)msbP << 16 |  (long)csbP << 8 |  (long)lsbP) ; // Construct whole number pressure
  pressure_whole >>= 6; // Only two most significant bits of lsbP contribute to whole pressure; its an 18-bit number
 
  lsbP &= 0x30; // Keep only bits 5 and 6, the fractional pressure
  lsbP >>= 4; // Shift to get the fractional pressure in terms of quarters of a Pascal
  float pressure_frac = (float) lsbP/4.0; // Convert numbers of fractional quarters to fractional pressure n Pasacl

  pressure = (float) (pressure_whole) + pressure_frac; // Combine whole and fractional parts to get entire pressure in Pascal

// Calculate temperature, check for negative sign
long foo = 0;
if(msbT > 0x7F) { // Is the most significant bit a 1? Then its a negative number in two's complement form
 foo = ~((long) msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) ((foo >> 8) + ((lsbT >> 4)/16.0)); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

/**
 * Sets sea level pressure
 * 
*/
void NXPMotionSense::setSeaPress(uint16_t sea_press_inp) {

	def_sea_press = sea_press_inp;

  uint16_t bar = sea_press_inp / 2;
  Wire.beginTransmission(MPL3115_I2C_ADDR);
  Wire.write((uint8_t)MPL3115_BAR_IN_MSB);
  Wire.write((uint8_t)(bar >> 8));
  Wire.write((uint8_t)bar);
  Wire.endTransmission(false);
}

/*
=====================================================================================================
Define functions according to 
"Data Manipulation and Basic Settings of the MPL3115 Command Line Interface Drive Code"
by Miguel Salhuana
Freescale Semiconductor Application Note AN4519 Rev 0.1, 08/2012
=====================================================================================================
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Clears then sets OST bit which causes the sensor to immediately take another reading
void NXPMotionSense::MPL3115toggleOneShot()
{
    MPL3115Active();  // Set to active to start reading
    uint8_t c = read_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1);
    write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, c & ~(1<<1)); // Clear OST (bit 1)
    c = read_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1);
    write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, c | (1<<1)); // Set OST bit to 1
}
    
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Outputting Sample Rate
void NXPMotionSense::MPL3115SampleRate(uint8_t samplerate)
{
  MPL3115Standby();  // Must be in standby to change registers

  uint8_t c = read_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1);
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, c & ~(0x38)); // Clear OSR bits 3,4,5
  if(samplerate < 8) { // OSR between 1 and 7
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, c | (samplerate << 3));  // Write OSR to bits 3,4,5
  }
  
  MPL3115Active();  // Set to active to start reading
 }
 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115 registers for FIFO mode
void NXPMotionSense::initFIFOMPL3115()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_P_MSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_P_CSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_P_LSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_T_MSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_T_LSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_F_STATUS);
  
   MPL3115Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG4, 0x40);  // enable FIFO
  
  //  Configure INT 1 for data ready, all other (inc. FIFO) interrupts to INT2
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  write_reg(MPL3115_I2C_ADDR, MPL3115_F_SETUP, 0x00); // Clear FIFO mode
// In overflow mode, when FIFO fills up, no more data is taken until the FIFO registers are read
// In watermark mode, the oldest data is overwritten by new data until the FIFO registers are read
  write_reg(MPL3115_I2C_ADDR, MPL3115_F_SETUP, 0x80); // Set F_MODE to interrupt when overflow = 32 reached
//  writeByte(MPL3115_I2C_ADDR, F_SETUP, 0x60); // Set F_MODE to accept 32 data samples and interrupt when watermark = 32 reached

  MPL3115Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115 for realtime data collection 
void NXPMotionSense::initRealTimeMPL3115()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_P_MSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_P_CSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_P_LSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_T_MSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_OUT_T_LSB);
  temp = read_reg(MPL3115_I2C_ADDR, MPL3115_F_STATUS);
  
   MPL3115Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG4, 0x80);  
  
  //  Configure INT 1 for data ready, all other interrupts to INT2
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  write_reg(MPL3115_I2C_ADDR, MPL3115_F_SETUP, 0x00); // disable FIFO mode
  
  MPL3115Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Auto Acquisition Time Step
void NXPMotionSense::MPL3115TimeStep(uint8_t ST_Value)
{
 MPL3115Standby(); // First put device in standby mode to allow write to registers
 
 uint8_t c = read_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG2); // Read contents of register CTRL_REG2
 if (ST_Value <= 0xF) {
 write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG2, (c | ST_Value)); // Set time step n from 0x0 to 0xF (bits 0 - 3) for time intervals from 1 to 32768 (2^n) seconds
 }
 
 MPL3115Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enable the pressure and temperature event flags
 // Bit 2 is general data ready event mode on new Pressure/Altitude or temperature data
 // Bit 1 is event flag on new Pressure/Altitude data
 // Bit 0 is event flag on new Temperature data
void NXPMotionSense::MPL3115enableEventflags()
{
  MPL3115Standby();  // Must be in standby to change registers
  write_reg(MPL3115_I2C_ADDR, MPL3115_PT_DATA_CFG, 0x07); //Enable all three pressure and temperature event flags
  MPL3115Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Altimeter mode
void NXPMotionSense::MPL3115ActiveAltimeterMode()
{
 MPL3115Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = read_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1); // Read contents of register CTRL_REG1
 write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, c | (0x80)); // Set ALT (bit 7) to 1
 MPL3115Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Barometer mode
void NXPMotionSense::MPL3115ActiveBarometerMode()
{
 MPL3115Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = read_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1); // Read contents of register CTRL_REG1
 write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, c & ~(0x80)); // Set ALT (bit 7) to 0
 MPL3115Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Software resets the MPL3115.
// It must be in standby to change most register settings
void NXPMotionSense::MPL3115Reset()
{
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, 0x04); // Set RST (bit 2) to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115 to standby mode.
// It must be in standby to change most register settings
void NXPMotionSense::MPL3115Standby()
{
  uint8_t c = read_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1); // Read contents of register CTRL_REG1
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, c & ~(0x01)); // Set SBYB (bit 0) to 0
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115 to active mode.
// Needs to be in this mode to output data
void NXPMotionSense::MPL3115Active()
{
  uint8_t c = read_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1); // Read contents of register CTRL_REG1
  write_reg(MPL3115_I2C_ADDR, MPL3115_CTRL_REG1, c | 0x01); // Set SBYB (bit 0) to 1
}
