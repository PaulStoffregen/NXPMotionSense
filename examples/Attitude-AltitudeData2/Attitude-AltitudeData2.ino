// Full orientation sensing using NXP's advanced sensor fusion algorithm.
// toggleOneShot has been added to the NXPMotionSense.cpp so that you can start a altitude conversion
// on the MPLX3115, and then do other things like read all the 9dof data and calculate their fusion
// By the time you readOneShotAlt , the conversion is complete. 

// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.

#include "NXPMotionSense.h"
#include <Wire.h>
#include <EEPROM.h>
#include "kalman.h"         // https://www.andico.org/2013/06/1-dimensional-kalman-filter-arduino.html?m=1
#include <CircularBuffer.h> // https://github.com/rlogiacco/CircularBuffer

KalmanFilter kf(0, .008, 10); // This is for kalman altimeter filter tuning ... was (0, .01, 1.0)
NXPMotionSense imu;
NXPSensorFusion filter;
float minAlt = 32767;      // zero minAlt and avg, but max out minimum, so that it gives true minimum
float maxAlt = 0;          // zero maxAlt
elapsedMillis MPL = 0;
elapsedMicros MPLm = 0;
CircularBuffer<float, 20> buffer;  // setup a circular buffer that holds 20 values
// if you change the sampleHertz away from 10hz, your circular buffer will not hold 1 second worth of data 

void setup() {
  Serial.begin(9600);
  imu.begin();
  filter.begin(100); // Normal rate is 100 hertz for gathering and computing SensorFusion from the IMU
  imu.setOversampleRate(1); //MPL3115 sample rate of 2 keeps up with 200 hertz reads on a Teensy 4.0

  //sets local sea level pressure
  //you can get this from your closest airport from
  //     https://aviationweather.gov/metar
  imu.setSeaPressure(102450); // value of 101940 = 1019.40 bar
}

void loop() {
  MPLm = 0;
  imu.MPL3115_toggleOneShot(); //tells MPL3115 to start gathering altitude and temperature data
  // Do other things while MPL3115 works on altitude and temperature data
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    heading = 360-heading;
    if (heading <=180) {  // correct heading direction
      heading = heading +180;
    } else {
      heading = heading - 180;
    } 
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(-roll); //correct roll direction with -
    Serial.print(" ");

    imu.readOneShotAlt();
    //Serial.printf("Alt: %f, TempC: %f\n", imu.altitudeM, imu.temperatureC);
    float alt_kalman = kf.step(imu.altitudeM); //filter noise from altimeter sensor, and get readings to settle
    
    //Circular buffer part from https://github.com/rlogiacco/CircularBuffer/tree/master/examples/CircularBuffer 
    //I have it set for holding the last 20 readings and then we calulate maxAlt, minAlt and avg
    //in the buffer after each new reading is taken. Oldest readings are discarded each time a new one is pushed 
    buffer.push(alt_kalman); // Place new kalman filtered reading in altitude buffer
		float avg = 0;
	  using index_t = decltype(buffer)::index_t; // sets up circularbuffer
    for (index_t i = 0; i < buffer.size(); i++) {
			avg += buffer[i]; 
      if (buffer[i] > maxAlt) maxAlt = buffer[i];
      if (buffer[i] < minAlt) minAlt = buffer[i];
		}
    avg = avg / buffer.size(); //calculates new average each time a new reading is taken
    Serial.print(avg);
    Serial.print(" ");
    Serial.println(MPLm);
    //imu.readAltitude();
    //imu.readPressure();
  }
}
