#include <ArduinoBLE.h>
#include "CircleBuffer.h"

BLEService FingerService("13012F00-F8C3-4F4A-A8F4-15CD926DA146");
BLECharacteristic FingerPosCharac("13012F01-F8C3-4F4A-A8F4-15CD926DA146", BLENotify, 8);

CIRCLE_BUFFER<uint16_t> buffer_sens1;
CIRCLE_BUFFER<uint16_t> buffer_sens2;
CIRCLE_BUFFER<uint16_t> buffer_sens3;
CIRCLE_BUFFER<uint16_t> buffer_sens4;

const int BUFF_SIZE = 32;

uint16_t fingerPos[4];

void setup(){
    Serial.begin(9600); 

    InitialiseBuffers();

    if (!BLE.begin()) {
    Serial.println("failed to initialize BLE!");
    while (1);
    }
  // set advertised local name and service UUID:
    BLE.setLocalName("Arduino NANO 33 IoT");
    BLE.setAdvertisedService(FingerService);

    FingerService.addCharacteristic(FingerPosCharac);
    BLE.addService(FingerService);
  // set the initial value for the characeristic:
    FingerPosCharac.writeValue(fingerPos,8);
    BLE.advertise();
    delay(100);
    Serial.println("advertising ...");
}

void loop(){
    BLEDevice central = BLE.central();

    // if a central is connected to peripheral:
    if (central) {
        Serial.print("Connected to central: ");
        // print the central's MAC address:
        Serial.println(central.address());

        uint16_t prev_fingerPos[4] = {0, 0, 0, 0};        
        for(int i=0; i < BUFF_SIZE; i++){
          WriteToBuffers();
        }
        
        // while the central is still connected to peripheral:
        while (central.connected()){
          WriteToBuffers();
          BufferToPos();
          bool notify_flag = false; 
          for (int i=0;i<4;i++){ 
              if (abs(fingerPos[i]-prev_fingerPos[i])>10){
                  notify_flag = true;
              }
              prev_fingerPos[i] = fingerPos[i];                           
          }
          if (notify_flag){
              FingerPosCharac.writeValue(fingerPos,8);            
          }
        }
        Serial.print(F("Disconnected from central: "));
        Serial.println(central.address());
    }
}
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