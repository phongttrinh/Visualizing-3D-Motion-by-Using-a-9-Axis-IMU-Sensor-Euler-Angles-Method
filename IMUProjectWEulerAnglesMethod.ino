//Phong Thanh Trinh
//Project:Visualizing 3D Motion by Using a 9-Axis IMU Sensor Euler Angles Method
//27-05-2022
//******************************************************************************

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h> //for good measurement
#include <math.h>
float theta; //Pitch
float gama; //Roll
float mega; // Yaw
float thetaMea;
float gamaMea;
float thetaFterOld = 0;
float thetaFterNew;
float gamaFterOld = 0;
float gamaFterNew;
float thetaGyr = 0;
float gamaGyr = 0;
float dT;
unsigned long milliSecOld;
float Xm;
float Ym;
float thetaRad;
float gamaRad;

#define BNO055_SAMPLERATE_DELAY_MS (100) // Ask sensor to sample after 100 miliseconds
Adafruit_BNO055 myIMU = Adafruit_BNO055();// using adafruit libary to creat an obejct called myIMU, so when you want to interact with sensor, you interact with myImu
void setup() {
// put your setup code here, to run once:
Serial.begin(115200);
myIMU.begin(); // turn myIMU on
delay(1000);
int8_t temp = myIMU.getTemp(); // get temperture
myIMU.setExtCrystalUse(true);//dont use the crytal on the chip itseft, use crytal on the board 
//for better result, we dont want to use the crytal as a time reference which is on the actual chip itself. U want to use the want on the board
milliSecOld = millis();
}

void loop() {
// put your main code here, to run repeatedly:
uint8_t system, gyro, accel, mg = 0; // an efficent way to store data
myIMU.getCalibration(&system, &gyro, &accel, &mg);//calibration sytax --> starting and get the calibration data
imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);//standand systax in Andfruit Libary: create a acc vector with 3 demensions (x,y,z) and assigned value from myIMU to it
imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // doing same thing for gyro
imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);// doing same thing for mag

//Uusing Accelertometer: find the change in angle --> can trust for long term
thetaMea = -atan2(acc.x(),acc.z())*180/3.141582;
thetaFterNew = 0.95*thetaFterOld + 0.05*thetaMea;//Fillter --> not depend much on new measured values 
gamaMea = -atan2(acc.y(),acc.z())*180/3.141582;
gamaFterNew = 0.95*gamaFterOld + 0.05*gamaMea;//Fillter --> not depend much on new measured values 

dT = (millis() - milliSecOld)/1000.;
milliSecOld = millis();
//Complimentary Filter combine Accelertometer and Gryoscope
theta = (theta + gyr.y()*dT)*0.95 + thetaMea*0.05;
gama = (gama - gyr.x()*dT)*0.95 + gamaMea*0.05; //minus here becasue want to change the sign of gamaGyr
//Using Gryoscope to find the change in angle --> can trust for short term
thetaGyr = thetaGyr + gyr.y()*dT;
gamaGyr = gamaGyr - gyr.x()*dT; // minus here becasue want to change the sign of gamaGyr

thetaRad = theta*3.141582/180;
gamaRad = gama*3.141582/180;
Xm = mag.x()*cos(thetaRad) - mag.y()*sin(gamaRad)*sin(thetaRad) + mag.z()*cos(gamaRad)*sin(thetaRad);
Ym = mag.y()*cos(gamaRad) + mag.z()*sin(gamaRad);
mega = atan2(Ym,Xm)*180/3.141582;

//Print Accelerometer values:
Serial.print(accel);
Serial.print(",");
Serial.print(gyro);
Serial.print(",");
Serial.print(mg);
Serial.print(",");
Serial.print(system);
Serial.print(",");
Serial.print(theta);
Serial.print(",");
Serial.print(gama);
Serial.print(",");
Serial.println(mega);

thetaFterOld = thetaFterNew; 
gamaFterOld = gamaFterNew;

delay(BNO055_SAMPLERATE_DELAY_MS);
}
