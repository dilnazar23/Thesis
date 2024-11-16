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
Finger finger;
void setup(){
  Serial.begin(9600);
  finger.attach(3, 2, 11, A0);
}
void loop(){
    finger.writePos(900);
    Serial.println(finger.readTargetSpeed());
    delay(1000);
    finger.writePos(300);
    delay(2000);
}