const int numReadings = 500;
const int analogPin1 = A0;
const int analogPin2 = A2;
const int analogPin3 = A4;

float x1_mean[3];
float x2_mean[3];

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    // Wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Press 'y' when you are ready to start data collection.");
  while (Serial.read() != 'y') {
    // Wait for user to press 'y'
  };

  // Collect and process first set of data
  collectData(x1_mean);
  
  Serial.println("Bend your finger and press 'y' when you are ready to start data collection.");
  while (Serial.read() != 'y') {
    // Wait for user to press 'y'
  };

  // Collect and process second set of data
  collectData(x2_mean);
  Serial.println(x2_mean[0]);

  // Calculate and print the linear relationship for each pin
  for (int i = 0; i < 3; i++) {
    float slope = 923.0 / (x2_mean[i] - x1_mean[i]);
    float intercept = 50-x1_mean[i] * slope;
    Serial.print("Pin A");
    Serial.print(i);
    Serial.print(": Slope (k) = ");
    Serial.print(slope);
    Serial.print(", Intercept (b) = ");
    Serial.println(intercept);
  }
}

void loop() {
  // Do nothing
}

void waitForUserInput() {
  while (Serial.read() != 'y') {
    // Wait for user to press 'y'
  }
}

void collectData(float *means) {
  int total[3] = {0, 0, 0};
  
  for (int i = 0; i < numReadings; i++) {
    total[0] += analogRead(analogPin1);
    total[1] += analogRead(analogPin2);
    total[2] += analogRead(analogPin3);
    delay(10); // Small delay between readings
  }
  
  for (int i = 0; i < 3; i++) {
    means[i] = total[i] / (float)numReadings;
  }
}
