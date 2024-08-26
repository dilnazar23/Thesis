const int numReadings = 10; // Number of readings for the moving average

int readings1[numReadings]; // Array to store readings for strain1
int readings2[numReadings]; // Array to store readings for strain2
int readings3[numReadings]; // Array to store readings for strain3
int readings4[numReadings]; // Array to store readings for strain4

int readIndex1 = 0; // Index of the current reading for strain1
int readIndex2 = 0; // Index of the current reading for strain2
int readIndex3 = 0; // Index of the current reading for strain3
int readIndex4 = 0; // Index of the current reading for strain4

int total1 = 0; // Sum of the readings for strain1
int total2 = 0; // Sum of the readings for strain2
int total3 = 0; // Sum of the readings for strain3
int total4 = 0; // Sum of the readings for strain4

int average1 = 0; // Average of the readings for strain1
int average2 = 0; // Average of the readings for strain2
int average3 = 0; // Average of the readings for strain3
int average4 = 0; // Average of the readings for strain4

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Initialize all the readings to 0
  for (int i = 0; i < numReadings; i++) {
    readings1[i] = 0;
    readings2[i] = 0;
    readings3[i] = 0;
    readings4[i] = 0;
  }
}

void loop() {
  // Read the input on analog pins
  int strain1 = analogRead(A0);
  int strain2 = analogRead(A2);
  int strain3 = analogRead(A4);
  int strain4 = analogRead(A6);

  // Update the total by subtracting the last reading and adding the new reading
  total1 = total1 - readings1[readIndex1] + strain1;
  total2 = total2 - readings2[readIndex2] + strain2;
  total3 = total3 - readings3[readIndex3] + strain3;
  total4 = total4 - readings4[readIndex4] + strain4;

  // Store the new reading
  readings1[readIndex1] = strain1;
  readings2[readIndex2] = strain2;
  readings3[readIndex3] = strain3;
  readings4[readIndex4] = strain4;

  // Advance to the next position in the arrays
  readIndex1 = (readIndex1 + 1) % numReadings;
  readIndex2 = (readIndex2 + 1) % numReadings;
  readIndex3 = (readIndex3 + 1) % numReadings;
  readIndex4 = (readIndex4 + 1) % numReadings;

  // Calculate the averages
  average1 = total1 / numReadings;
  average2 = total2 / numReadings;
  average3 = total3 / numReadings;
  average4 = total4 / numReadings;

  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.3V)
  float voltage1 = average1 * (3.3 / 1023.0);
  float voltage2 = average2 * (3.3 / 1023.0);
  float voltage3 = average3 * (3.3 / 1023.0);
  float voltage4 = average4 * (3.3 / 1023.0);

  // Print out the values
  Serial.print("voltage1: ");
  Serial.println(voltage1);
  Serial.print("voltage2: ");
  Serial.println(voltage2);
  Serial.print("voltage3: ");
  Serial.println(voltage3);
  Serial.print("voltage4: ");
  Serial.println(voltage4);

  delay(100); // Delay for stability
}
