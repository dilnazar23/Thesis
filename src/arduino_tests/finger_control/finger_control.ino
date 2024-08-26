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
    if (Serial.available() > 0) {
        char start_byte = Serial.read();
        if (start_byte == 'S'){
            Serial.println(F("Receive: "));            
            Serial.read(); // remove the comma
            String income_data = "";
            int data[4];
            int i = 0;
            while(Serial.available() > 0){
                char income_byte = Serial.read();
                if (income_byte == '\n'){
                    break;
                }
                else if(income_byte == ','){
                    i = i % 4;
                    data[i] = income_data.toInt();
                    i++;
                    Serial.print("here");
                    Serial.println(income_data);
                    income_data = "";
                    continue;
                }
                else{
                    income_data += income_byte;
                }               
            }            

            // for (int i = 0; i < 1; i++){
            //     int pos = data[i]*SLOPE + OFFSET;
            //     fingers[i].writePos(pos);
            //     Serial.print(pos);
            //     delay(10);
            // }
            //finger.invertFingerDir();
            int command = data[0]*SLOPE + OFFSET;
            Serial.println("Command: ");
            Serial.print(command);
            Serial.println("Data: ");
            Serial.print(data[0]);
            finger.writePos(command);
        }
    }



}
