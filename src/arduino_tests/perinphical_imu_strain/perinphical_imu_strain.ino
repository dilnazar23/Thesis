#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
// IMU related stuff
#include <Wire.h>
#include "CircleBuffer.h"

// Initialize the BLE service
BLEService PerinphicalService("13012F00-F8C3-4F4A-A8F4-15CD926DA146");

//*****Finger related stuff*****//
BLECharacteristic FingerPosCharac("13012F01-F8C3-4F4A-A8F4-15CD926DA146", BLENotify, 8);

CIRCLE_BUFFER<uint16_t> buffer_sens1;
CIRCLE_BUFFER<uint16_t> buffer_sens2;
CIRCLE_BUFFER<uint16_t> buffer_sens3;
CIRCLE_BUFFER<uint16_t> buffer_sens4;

const int BUFF_SIZE = 32;
uint16_t fingerPos[4];
//******************************//

//******IMU related stuff******//
BLECharacteristic IMUcharacteristic("13012F02-F8C3-4F4A-A8F4-15CD926DA146", BLENotify, 16);
float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;
// IMU data packet
typedef union {
  float anglesQuaternion[4];
  uint8_t bytes[16];
} imuunion_t;

imuunion_t _imudata;

struct Quaternion
{
  float w, x, y, z;
};

Quaternion q;
//******************************//

void setup(){
    Serial.begin(9600); 

    InitialiseBuffers();

    if (!IMU.begin()) {    
      while (1){    
        Serial.println("Failed to initialize IMU!");
      }
    }
    else{
      Serial.println("IMU initialized");
    }
    calibrateIMU(250, 250);

    if (!BLE.begin()) {    
      while (1){
        Serial.println("failed to initialize BLE!");
      }
    }
  // set advertised local name and service UUID:
    BLE.setLocalName("Arduino NANO 33 IoT");
    BLE.setAdvertisedService(PerinphicalService);    

    PerinphicalService.addCharacteristic(FingerPosCharac);
    PerinphicalService.addCharacteristic(IMUcharacteristic);
    BLE.addService(PerinphicalService);
  // set the initial value for the characeristic:
    FingerPosCharac.writeValue(fingerPos,8);
    IMUcharacteristic.writeValue(_imudata.bytes, 16);
    lastTime = micros();
    
    BLE.advertise();
    delay(100);
    Serial.println("advertising ...");
}

void loop(){
    // listen for BLE centrals to connect:
    BLEDevice central = BLE.central();

    // if a central is connected to peripheral:
    if (central) {
        Serial.print("Connected to central: ");
        // print the central's MAC address:
        Serial.println(central.address());
        /*What is this for could have done in void setup()*/
        uint16_t prev_fingerPos[4] = {0, 0, 0, 0};        
        for(int i=0; i < BUFF_SIZE; i++){
          WriteToBuffers();
        }
        
        // while the central is still connected to peripheral:
        while (central.connected()){
          if (readIMU()) {
            long currentTime = micros();
            lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
            lastTime = currentTime;

            doCalculations();
            //printCalculations();      
            
            _imudata.anglesQuaternion[0] = q.x;
            _imudata.anglesQuaternion[1] = q.y;
            _imudata.anglesQuaternion[2] = q.z;  
            _imudata.anglesQuaternion[3] = q.w;        
            IMUcharacteristic.writeValue(_imudata.bytes, 16);
          }
          WriteToBuffers();
          BufferToPos();
          bool notify_flag = false; 
          for (int i=0;i<4;i++){ 
              if (abs(fingerPos[i]-prev_fingerPos[i])>5){
                  notify_flag = true;
                  prev_fingerPos[i] = fingerPos[i];
                  break;
              }
                                         
          }
          if (notify_flag){
              FingerPosCharac.writeValue(fingerPos,8);            
          }
        }
        Serial.print(F("Disconnected from central: "));
        Serial.println(central.address());
    }
    //Serial.print(F("Not connected to any device: "));
        
}

//*****Finger related functions*****//
void InitialiseBuffers(){
    buffer_sens1.begin(BUFF_SIZE);
    buffer_sens2.begin(BUFF_SIZE);
    buffer_sens3.begin(BUFF_SIZE);
    buffer_sens4.begin(BUFF_SIZE);
}
void WriteToBuffers(){
    buffer_sens1.write(analogRead(A0));
    buffer_sens2.write(analogRead(A2));
    buffer_sens3.write(analogRead(A4));
    buffer_sens4.write(analogRead(A6));
}
void BufferToPos(){
    fingerPos[0] = buffer_sens1.readMean();
    fingerPos[1] = buffer_sens2.readMean();
    fingerPos[2] = buffer_sens3.readMean();
    fingerPos[3] = buffer_sens4.readMean();
}
//*********************************//

//******IMU related functions******//
/**
   Read accel and gyro data.
   returns true if value is 'new' and false if IMU is returning old cached data
*/
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    return true;
  }
  return false;
}
/*
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still
  we can correct the drift when doing real measurements later
*/
void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;
    calibrationCount++;
    }
  }
  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }
  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
  
}


void doCalculations() {
  accRoll = atan2(accelY, accelZ);
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));

  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
  q = ToQuaternion(complementaryRoll, complementaryPitch, complementaryYaw);
}

// This is not in game format, it is in mathematical format.
Quaternion ToQuaternion(float roll, float pitch, float yaw) // roll (x), pitch (y), yaw (z), angles are in radians
{
    // Abbreviations for the various angular functions

    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}
/**
   This comma separated format is best 'viewed' using 'serial plotter' or processing.org client 
   (see ./processing/RollPitchYaw3d.pde example)
*/
void printCalculations() {
  // Serial.print(complementaryRoll);
  // Serial.print(',');
  // Serial.print(complementaryPitch);
  // Serial.print(',');
  // Serial.print(complementaryYaw);
  // Serial.println("");
  Serial.print(q.x);
  Serial.print(',');
  Serial.print(q.y);
  Serial.print(',');
  Serial.print(q.z);
  Serial.print(q.w);
  Serial.print(',');
  Serial.println("");
}
//*********************************//