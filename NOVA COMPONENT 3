/*********************************************************************
 * DF Pong Controller
 * 
 * This program implements a Bluetooth Low Energy controller for Pong.
 * It sends movement data to a central device running in the browser and
 * provides audio feedback through a buzzer.
 *
 * Game Link : https://digitalfuturesocadu.github.io/df-pong/
 * 
 * Movement Values:
 * 0 = No movement / Neutral position
 * 1 = UP movement (paddle moves up)
 * 2 = DOWN movement (paddle moves down)
 * 3 = Handshake signal (used for initial connection verification)
 * 
 * Key Functions:
 * - handleInput(): Process the inputs to generate the states
 * - sendMovement(): Sends movement data over BLE (0-3)
 * - updateBLE(): Handles BLE connection management and updates
 * - updateBuzzer(): Provides different buzzer patterns for different movements
 * 
 * Key Variables:
 * - currentMovement: Stores current movement state (0-2)
 * - deviceName : GIVE YOUR DEVICE AN APPROPRIATE NAME
 * - LED_PIN : It is important to see the status of the arduino through the LED. 
      if you can see the built-in add an external one and update the pin it is connected to
 * 

 *********************************************************************/



#include <ArduinoBLE.h>
#include "ble_functions.h"
#include "buzzer_functions.h"
//Since code is split over multiple files, we have to include them here


//Name your controller!
const char* deviceName = "NOVA COMPONENT 3";

// Pin definitions buzzer/LED
const int BUZZER_PIN = 11;        // Pin for haptic feedback buzzer
const int LED_PIN = LED_BUILTIN;  // Status LED pin

// Movement state tracking
int currentMovement = 0;  // Current movement value (0=none, 1=up, 2=down, 3=handshake)

int inputState = 0;

int lightPin = A7;
const int lightAverageWindow = 10;  // Number of samples to average

// Global variables
int lightValue = 0;           // Raw value
int smoothedLightValue = 0;   // Filtered value
int startupLightValue = 0;    // Calibration value from startup
String brightnessState = "";  // Stores comparison to startup
unsigned long lastLightReadTime = 0;
unsigned int lightReadInterval = 50;  // Time between reads in milliseconds


void setup() {

  Serial.begin(9600);

  Serial.println("Light sensor");

  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize the rolling average
  initializeLightAverage();

  // Perform startup calibration
  calibrateSensor();


  // Configure LED for connection status indication
  pinMode(LED_PIN, OUTPUT);

  // Initialize Bluetooth Low Energy with device name and status LED
  setupBLE(deviceName, LED_PIN);

  // Initialize buzzer for feedback
  setupBuzzer(BUZZER_PIN);
}

void loop() {
  // Update BLE connection status and handle incoming data
  updateBLE();

  //read the inputs te determine the current state
  //results in changing the value of currentMovement
  handleInput();

  //send the movement state to P5
  sendMovement(currentMovement);

  //make the correct noise
  updateBuzzer(currentMovement);
}

//put code here that reads the sensor input
//and assigns currentMovement(0=stop, 1=up, 2=down)


// Rolling average variables
int lightReadings[lightAverageWindow];
int lightReadIndex = 0;
long lightTotalValue = 0;

// Function to initialize the rolling average array
void initializeLightAverage() {
  // Initialize all readings to 0
  for (int i = 0; i < lightAverageWindow; i++) {
    lightReadings[i] = 0;
  }
  lightTotalValue = 0;
  lightReadIndex = 0;
}

// Function to update rolling average with new value
void updateLightAverage(int newValue) {
  lightTotalValue = lightTotalValue - lightReadings[lightReadIndex];
  lightReadings[lightReadIndex] = newValue;
  lightTotalValue = lightTotalValue + newValue;
  lightReadIndex = (lightReadIndex + 1) % lightAverageWindow;
  smoothedLightValue = lightTotalValue / lightAverageWindow;
}

// Function to compare current brightness to startup
void updateBrightnessState() {
  const int threshold = 10;  // Tolerance for considering values "same"
  if (abs(smoothedLightValue - startupLightValue) <= threshold) {
    brightnessState = "same";
  } else if (smoothedLightValue > startupLightValue) {
    brightnessState = "brighter";
  } else {
    brightnessState = "darker";
  }
}

// Function to read light sensor and update the global value
void readLightSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastLightReadTime >= lightReadInterval) {
    // Read the analog value
    lightValue = analogRead(lightPin);

    // Update the rolling average
    updateLightAverage(lightValue);

    // Compare to startup value
    updateBrightnessState();

    // Print the values
    printLightValue();

    // Update the last read time
    lastLightReadTime = currentTime;
  }
}

// Function to print light sensor values
void printLightValue() {
  Serial.print("Light Raw: ");
  Serial.print(lightValue);
  Serial.print("\tLight Smoothed: ");
  Serial.print(smoothedLightValue);
  Serial.print("\tStartup Value: ");
  Serial.print(startupLightValue);
  Serial.print("\tCompared to Startup: ");
  Serial.println(brightnessState);
}

void calibrateSensor() {
  Serial.println("Calibrating sensor...");
  // Take multiple readings and average them for startup value
  long total = 0;
  for (int i = 0; i < lightAverageWindow; i++) {
    total += analogRead(lightPin);
    delay(50);  // Short delay between readings
  }
  startupLightValue = total / lightAverageWindow;
  Serial.print("Calibration complete. Startup value: ");
  Serial.println(startupLightValue);
}


void handleInput() {


  readLightSensor();
  // Determine input state
  if (brightnessState == "brighter") {
    currentMovement = 1;
  } else if (brightnessState == "darker") {
    currentMovement = 2;
  } else {
    currentMovement = 0;
  }


  // Call buzzer output function
  buzzerOutput(currentMovement);
  
}

void buzzerOutput(int state) {
  unsigned long currentMillis = millis();
  int interval;


  switch (state) {
    case 1:          // Touch 0 active
      interval = 3;  // 3ms interval
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        buzzerState = !buzzerState;  // Toggle buzzer state
        digitalWrite(BUZZER_PIN, buzzerState);
      }
      break;

    case 2:           // Touch 1 active
      interval = 20;  // 20ms interval
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        buzzerState = !buzzerState;  // Toggle buzzer state
        digitalWrite(BUZZER_PIN, buzzerState);
      }
      break;

    default:                          // No touch (state 0)
      digitalWrite(BUZZER_PIN, LOW);  // Turn off buzzer
      buzzerState = false;            // Reset buzzer state
      break;
  }
}
