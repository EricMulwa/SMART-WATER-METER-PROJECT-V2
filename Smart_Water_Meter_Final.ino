/*
THE FINAL SMART WATER METER PROJECT PROGRAM
This program integrates STM32F103C8T6 with the following components: 
- Water Flow Sensor
- Motorized Ball Valve via L298N Driver
- HT1621 LCD
- IR Sensor
- GSM SIM7600CE 4G Module

For working and documentation, please check the "SMART WATER METER PROJECT PROGRAM" Document by ERIC MULWA
Developed by ERIC MULWA

  Last modified 27 OCT 2023
  by Eric Mulwa 
*/


#include <HT1621.h> // Include our library
HT1621 lcd; // Create an "lcd" object
#define IrPin 0 // Define the IR sensor pin as PA0
#define DrivercPin 5 // PA5 for opening the Ball-valve
#define DriveraPin 6 // PA6 for closing the Ball-valve

int sensorPin = 4; // PA4
volatile long pulse;
float volume;
unsigned long lastSMSTime = 0; // Timer for the last SMS sent
const unsigned long smsInterval = 86400000; // 24 hours in milliseconds

char incomingMessage[100]; // Array to store incoming SMS
unsigned long batteryMarkerTimer = 0;
int batteryMarkerState = 3; // Initial state
bool batteryMarkersExecuted = false; // Flag to control battery markers execution

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(IrPin, INPUT); // Set the IR sensor pin as input
  pinMode(DrivercPin, OUTPUT);
  pinMode(DriveraPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), increase, RISING);

  Serial1.begin(115200); // Initialize GSM serial communication
  delay(1000); // Wait for the GSM module to start
  // Initialize the GSM module
  Serial1.println("AT");
  delay(1000);
  Serial1.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
  Serial1.println("AT+CNMI=2,2,0,0,0"); // Enable SMS notifications
  delay(1000);

  // Start the LCD:
  // CS to pin PA1
  // WR to pin PA2
  // Data to pin PA3
  lcd.begin(PA1, PA2, PA3); // (CS, WR, Data)
  lcd.clear();

  // Set the last SMS time to the current time to ensure the first SMS is sent after 24 hours
  lastSMSTime = millis();
}

void loop() {
  // Check for incoming SMS
  if (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      // Check the received message
      if (strstr(incomingMessage, "syson") != NULL) {
        digitalWrite(DrivercPin, HIGH); // Open the motorized ball-valve
        delay(3000); // 
        digitalWrite(DrivercPin, LOW);
      } else if (strstr(incomingMessage, "sysoff") != NULL) {
        digitalWrite(DriveraPin, HIGH); // Close the motorized ball-valve
        delay(3000); // Wait for 2s
        digitalWrite(DriveraPin, LOW); //
      }
      // Clear the message buffer
      memset(incomingMessage, 0, sizeof(incomingMessage));
    } else {
      // Append the received character to the message buffer
      strncat(incomingMessage, &c, 1);
    }
  }

  // Calculate and display the volume value continuously
  //volume = 2.663 * pulse; // Volume in Milliliters // for testing purposes 
  volume = 0.002663 * pulse; // Volume in Liters 

  int detect = digitalRead(IrPin); // Read IR sensor status and store
  if (detect == LOW) {
    lcd.clear(); // Clear the screen if an obstacle is detected
    delay(1000);
  } else if (detect == HIGH) {
    lcd.print(volume, 1); // Display the volume with 1 decimal place
  }

  unsigned long currentMillis = millis();
  // Setting the Battery Markers without interfering with any other code section
  if (!batteryMarkersExecuted) {
    // Display the 3rd Markers for 160 hours. That is the 3rd marker to last for 160 hours
    if (currentMillis - batteryMarkerTimer <= 576000000) {
      lcd.setBatteryLevel(3);
    }
    // Display the 2nd Marker for 320 hours. That is 2nd marker to last 160 hours after the 3rd fades out
    else if (currentMillis - batteryMarkerTimer <= 1152000000) {
      lcd.setBatteryLevel(2);
    }
    // Display the 1st Marker for 480 hours. That is 1st marker to last 160 hours after the 2nd fades out
    else if (currentMillis - batteryMarkerTimer <= 1728000000) {
      lcd.setBatteryLevel(1);
    }
    // Display no marker after the 480 hours. Battery shutting down
    else if (currentMillis - batteryMarkerTimer <= 1728002000) {
    lcd.setBatteryLevel(0);
    }
    // Stop the execution
    else {
      batteryMarkersExecuted = true;
    }
  }
  // Check if it's time to send the volume SMS
  if (currentMillis - lastSMSTime >= smsInterval) {
    // Send the SMS
    sendVolumeSMS();
    // Update the last SMS time
    lastSMSTime = currentMillis;
  }
}

void increase() {
  pulse++;
}

void sendVolumeSMS() {
  char mobileNumber[] = "+254796456877"; // Mobile Number to send Volume of water consumed.
  char ATcommand[80];
  uint8_t buffer[30] = {0};
  uint8_t ATisOK = 0;
  while (!ATisOK) {
    sprintf(ATcommand, "AT\r\n");
    Serial1.print(ATcommand);
    delay(1000);
    if (Serial1.find("OK")) {
      ATisOK = 1;
    }
    delay(1000);
  }
  sprintf(ATcommand, "AT+CMGF=1\r\n");
  Serial1.print(ATcommand);
  delay(100);
  Serial1.readBytes(buffer, sizeof(buffer));
  delay(1000);
  memset(buffer, 0, sizeof(buffer));
  sprintf(ATcommand, "AT+CMGS=\"%s\"\r\n", mobileNumber);
  Serial1.print(ATcommand);
  delay(100);
  Serial1.print("Volume of water consumed in Liters: ");
  Serial1.print(volume, 2);
  Serial1.write(0x1a); // Send the Ctrl+Z character to indicate the end of the message
  delay(4000);
}

 //--------------------@ ERIC MULWA BSc EEE ------------ ------------------//
//------------------SMART WATER METER PROJECT-----------------------------//