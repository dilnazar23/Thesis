// TODO: receive data from the central and control the finger
//Pin A0: Slope (k) = -1.23, Intercept (b) = 928.49
// -1.23, Intercept (b) = 928.48

#include <FingerLib_mod.h>

/*
 * WHAT IT DOES
 * Uses the FingerLib.h library to open and close a single finger every 2 seconds
 * 
 */

// uncomment one of the following to select the board
#define ALMOND_BOARD
//#define CHESTNUT_BOARD

// number of controllable fingers (number of motors)
 #if defined(ALMOND_BOARD)
#define NUM_FINGERS 4
#define MYSERIAL Serial
#elif defined(CHESTNUT_BOARD)
#define NUM_FINGERS 4
#define MYSERIAL SerialUSB
#else
#error 'No board selected'
#endif

const float SLOPE = -1.23;
const float OFFSET = 978.74;

// // initialise Finger class
// Finger fingers[NUM_FINGERS];
Finger finger;

union received_data{
    byte bytes[8];
    uint16_t val[4];
};
received_data data;

void setup()
{ 
  // MYSERIAL.print is used to allow compatability between both the Mega (Serial.print) 
  // and the Zero Native Port (SerialUSB.print), and is defined in FingerLib.h
  Serial.begin(9600);
  Serial.println("Started");

  // configure finger pins
//   fingers[0].attach(1, 2, A0);
//   fingers[1].attach(3, 4, A1);
//   fingers[2].attach(5, 6, A2);
//   fingers[3].attach(7, 8, A3);
  finger.attach(24, 25, A0);

  Serial.println("Pins configured");
}

void loop()
{
    while (Serial.available() > 8){
        if (Serial.read()=='S'){
            Serial.readBytes(data.bytes, 8);
            Serial.println();
            Serial.print("Data received:");
            // Debugging prints
            for (int i = 0; i < 4; i++){
                Serial.print(' ');
                Serial.print(data.val[i]);
            }
            int command = data.val[0]*SLOPE + OFFSET;
            Serial.println("Command: ");
            Serial.print(command);
            // TODO: control the finger
            finger.writePos(command);
        };       
        
    }

}