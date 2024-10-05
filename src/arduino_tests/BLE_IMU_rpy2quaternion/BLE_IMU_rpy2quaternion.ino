/*
 * Simple program to stream IMU data through BLE.
 */

#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
// IMU related stuff
#include <Wire.h>

float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;

#define DEBUG false


// These UUIDs have been randomly generated. - they must match between the Central and Peripheral devices
// Any changes you make here must be suitably made in the Python program as well

BLEService nanoIMUService("13012F00-F8C3-4F4A-A8F4-15CD926DA146"); // BLE Service
// Accelerometer and Gyroscope characteristics
BLECharacteristic IMUcharacteristic("13012F02-F8C3-4F4A-A8F4-15CD926DA146", BLENotify, 20);

// IMU data packet
typedef union {
  float anglesQuaternion[5];
  uint8_t bytes[20];
} imuunion_t;

typedef union {
    unsigned long data;
    uint8_t bytes[4];
} ulongunion_t;

ulongunion_t _time;
imuunion_t _imudata;

struct Quaternion
{
    float w, x, y, z;
};

Quaternion q;

void setup() {
  Serial.begin(9600);

  if (DEBUG) {
    while (!Serial);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  calibrateIMU(250, 250);
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Arduino NANO 33 IoT");
  BLE.setAdvertisedService(nanoIMUService);

  // add the characteristic to the service
  nanoIMUService.addCharacteristic(IMUcharacteristic);
  
  // add service
  BLE.addService(nanoIMUService);

  // set the initial value for the characeristic:
  IMUcharacteristic.writeValue(_imudata.bytes, 20);

  lastTime = micros();
  // start advertising
  BLE.advertise();
  delay(100);
  Serial.println("ProtoStax Arduino Nano BLE LED Peripheral Service Started");
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

void loop() {
  // listen for BLE centrals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
        if (readIMU()) {
          long currentTime = micros();
          lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
          lastTime = currentTime;

          doCalculations();
          printCalculations();
          _time.data = micros();
          // Update first four bytes of IMU data union.
          for (int i = 0; i < 4;i ++) {
              _imudata.bytes[i] = _time.bytes[i];
          }
          _imudata.anglesQuaternion[1] = q.x;
          _imudata.anglesQuaternion[2] = q.y;
          _imudata.anglesQuaternion[3] = q.z;  
          _imudata.anglesQuaternion[4] = q.w;        
          IMUcharacteristic.writeValue(_imudata.bytes, 20);
        }

    }
    
    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
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
   This comma separated format is best 'viewed' using 'serial plotter' or processing.org client (see ./processing/RollPitchYaw3d.pde example)
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