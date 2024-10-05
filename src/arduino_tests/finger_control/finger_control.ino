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
Finger finger[4];

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

  
  finger[0].attach(3, 2, 11, A0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
  finger[1].attach(4, 1, 3, A1);
//   finger[2].attach(6, 0, 5, A2);
//   finger[3].attach(7, 5, 6, A3);

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
            int commands[4];
            Serial.println("Command: ");
            for (int i = 0; i < 4; i++){
                commands[i] = data.val[i]*SLOPE + OFFSET;                
                Serial.print(commands[i]);
                Serial.print(' ');
            }
            Serial.println();
            finger[0].writePos(commands[0]);
            finger[1].writePos(commands[1]);
            
            // for (int i = 0; i < 4; i++){
            //     finger[i].writePos(commands[i]);                
            // }
        };       
        
    }

}