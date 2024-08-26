#include <ArduinoBLE.h>

BLEService FingerService("13012F00-F8C3-4F4A-A8F4-15CD926DA146");
BLECharacteristic FingerPosCharac("13012F01-F8C3-4F4A-A8F4-15CD926DA146", BLENotify, 4);

uint8_t fingerPos[4];

const int8_t FingerPins[4] = {A0, A2, A4, A6};

void setup(){
    Serial.begin(9600);   

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
    FingerPosCharac.writeValue(fingerPos,4);
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
        uint8_t prev_fingerPos[4] = {0, 0, 0, 0};
        // while the central is still connected to peripheral:
        while (central.connected()) {  
            bool notify_flag = false; 
            for (int i=0;i<4;i++){        
                fingerPos[i] = analogRead(FingerPins[i]);        
                if (abs(fingerPos[i]-prev_fingerPos[i])>5){
                    prev_fingerPos[i] = fingerPos[i];
                    notify_flag = true;
                }
                // Serial.print(F(" "));                
            }
            if (notify_flag){
                delay(10);
                FingerPosCharac.writeValue(fingerPos,4);
                //Serial.println(F("Big enough Connected and sent"));
            }

            // FingerPosCharac.writeValue(fingerPos,4);
            // delay(100);
            // Serial.println(F("Connected and sent"));
        }
        Serial.print(F("Disconnected from central: "));
        Serial.println(central.address());
    }
}