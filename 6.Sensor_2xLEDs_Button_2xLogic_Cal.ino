#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_NeoPixel.h>

const int MPU_addr = 0x68;
MPU6050 mpu;

int16_t calibX, calibY, calibZ;

float initialTiltAngle = 0.0;

// Filter coefficients (adjust as needed)
const float alpha = 0.7;

// Define LED parameters for the first strip (connected to pin 6)
#define LED_PIN 8
#define NUM_LEDS 20
#define NUM_ACTIVE_LEDS 20
#define NORMAL_RIDE_LEDS 10
#define SPORT_RIDE_LEDS 5
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Define LED parameters for the second strip (connected to pin 8)
#define LED_PIN_8 6

Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_LEDS, LED_PIN_8, NEO_GRB + NEO_KHZ800);

// Function declarations
void controlLEDsPositive(float tiltAngleX);
void controlLEDsNegative(float tiltAngleX);
void updateLEDs(Adafruit_NeoPixel &strip, int activeLEDs, int red, int green, int blue);
void controlServiceModeLEDsPositive(float tiltAngleX);
void controlServiceModeLEDsNegative(float tiltAngleX);
void calibrateSensor();
void updateServiceModeLEDs(Adafruit_NeoPixel &strip, int activeLEDs, int red, int green, int blue);


// Define button parameters
const int buttonPin = 2;  // Change this to the pin connected to your button
int buttonState = 0;
int lastButtonState = 0;
unsigned long buttonPressStartTime = 0;
const int shortPressDuration = 500;  // Adjust as needed



// Define modes
enum OperatingMode {
  DrivingMode,
  ServiceMode
};

OperatingMode currentMode = DrivingMode;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed. Please check your connections.");
    while (1);
  }

  Serial.println("Calibrating...");
  calibrateSensor();
  Serial.println("Calibration Complete");

  strip.begin();
  strip.show();
  strip.setBrightness(5);
  
  strip2.begin();
  strip2.show();
  strip2.setBrightness(5);

  // Initialize button with internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
}



void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // Check button state
  buttonState = digitalRead(buttonPin);

  // Check if the button is pressed to initiate recalibration
if (buttonState == LOW && lastButtonState == HIGH) {
  Serial.println("Button pressed. Recalibrating...");
  calibrateSensor();
  Serial.println("Recalibration Complete");
  delay(1000);  // Delay to debounce the button
}

// Check if the button is pressed to switch modes
if (buttonState == LOW && lastButtonState == HIGH) {
  if (currentMode == DrivingMode) {
    currentMode = ServiceMode;
    Serial.println("Switched to Service mode");
  } else {
    currentMode = DrivingMode;
    Serial.println("Switched to Driving mode");
  }
  delay(1000);  // Delay to debounce the button
}

lastButtonState = buttonState;

// Read sensor data
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Apply filtering
  float filtered_ax = alpha * filtered_ax + (1 - alpha) * ax;
  float filtered_ay = alpha * filtered_ay + (1 - alpha) * ay;
  float filtered_az = alpha * filtered_az + (1 - alpha) * az;

  // Subtract initial angle to normalize readings
  float tiltAngleX = atan2(filtered_ay, filtered_az) * 180.0 / M_PI - initialTiltAngle;
  

  Serial.print("Tilt Angle X: ");
  Serial.print(tiltAngleX);



    if (currentMode == DrivingMode) {
    controlLEDsPositive(tiltAngleX);
    controlLEDsNegative(tiltAngleX);
  } else if (currentMode == ServiceMode) {
    controlServiceModeLEDsPositive(tiltAngleX);
    controlServiceModeLEDsNegative(tiltAngleX);
  }

  Serial.println();

  delay(100);
}



void calibrateSensor() {
  delay(2000);

  int numReadings = 300;
  calibX = calibY = calibZ = 0;

  // Capture initial angle
  int16_t initial_ax, initial_ay, initial_az;
  mpu.getAcceleration(&initial_ax, &initial_ay, &initial_az);
  initialTiltAngle = atan2(initial_ay, initial_az) * 180.0 / M_PI;

  for (int i = 0; i < numReadings; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    calibX += ax;
    calibY += ay;
    calibZ += az;

    delay(10);
  }

  calibX /= numReadings;
  calibY /= numReadings;
  calibZ /= numReadings;
}


void controlLEDsPositive(float tiltAngleX) {
  int activeLEDs = 0;

  if (tiltAngleX >= 0 && tiltAngleX <= 25) {
    //Normal ride
    activeLEDs = min(int(tiltAngleX / 2.5), NORMAL_RIDE_LEDS);
  } 
  else if (tiltAngleX > 25 && tiltAngleX <= 39) {
    //Sport ride
    activeLEDs = min(int((tiltAngleX - 25) / 2.67), SPORT_RIDE_LEDS) + NORMAL_RIDE_LEDS;
  } 
  else if (tiltAngleX > 39 && tiltAngleX <= 51) {
    //Near limit
    activeLEDs = min(int((tiltAngleX - 39) / 2.33), SPORT_RIDE_LEDS) + NORMAL_RIDE_LEDS + SPORT_RIDE_LEDS;
  } 
  else if (tiltAngleX > 51) {
    //Crash imminent
    activeLEDs = NUM_ACTIVE_LEDS;
  }

  updateLEDs(strip, activeLEDs, 0, 255, 0);
}

void controlLEDsNegative(float tiltAngleX) {
  int activeLEDs = 0;

  if (tiltAngleX <= 0 && tiltAngleX >= -25) {
    //Normal ride
    activeLEDs = min(int((-tiltAngleX) / 2.5), NORMAL_RIDE_LEDS);
  } 
  else if (tiltAngleX < -25 && tiltAngleX >= -39) {
    //Sport ride
    activeLEDs = min(int(((-tiltAngleX) - 25) / 2.67), SPORT_RIDE_LEDS) + NORMAL_RIDE_LEDS;
  } 
  else if (tiltAngleX < -39 && tiltAngleX >= -51) {
    //Near limit
    activeLEDs = min(int(((-tiltAngleX) - 39) / 2.33), SPORT_RIDE_LEDS) + NORMAL_RIDE_LEDS + SPORT_RIDE_LEDS;
  } 
  else if (tiltAngleX < -51) {
    //Crash imminent
    activeLEDs = NUM_ACTIVE_LEDS;
  }

  updateLEDs(strip2, activeLEDs, 0, 255, 0);  // Set color (green in this example) for strip2
}

void controlServiceModeLEDsPositive(float tiltAngleX) {
  int activeLEDs = 0;

  if (tiltAngleX >= 0 && tiltAngleX <= 5) {
    //Normal ride
    activeLEDs = min(int(tiltAngleX / 0.5), NORMAL_RIDE_LEDS);
  } 
  else if (tiltAngleX > 5 && tiltAngleX <= 10) {
    //Sport ride
    activeLEDs = min(int((tiltAngleX - 5) / 1), SPORT_RIDE_LEDS) + NORMAL_RIDE_LEDS;
  } 
  else if (tiltAngleX > 10 && tiltAngleX <= 15) {
    //Near limit
    activeLEDs = min(int((tiltAngleX - 10) /1), SPORT_RIDE_LEDS) + NORMAL_RIDE_LEDS + SPORT_RIDE_LEDS;
  } 
  else if (tiltAngleX > 15) {
    //Crash imminent
    activeLEDs = NUM_ACTIVE_LEDS;
  }

  updateLEDs(strip, activeLEDs, 0, 255, 0);
}

void controlServiceModeLEDsNegative(float tiltAngleX) {
  int activeLEDs = 0;

  if (tiltAngleX <= 0 && tiltAngleX >= -5) {
    //Normal ride
    activeLEDs = min(int((-tiltAngleX) / 0.5), NORMAL_RIDE_LEDS);
  } 
  else if (tiltAngleX < -5 && tiltAngleX >= -10) {
    //Sport ride
    activeLEDs = min(int(((-tiltAngleX) - 5) / 1), SPORT_RIDE_LEDS) + NORMAL_RIDE_LEDS;
  } 
  else if (tiltAngleX < -10 && tiltAngleX >= -15) {
    //Near limit
    activeLEDs = min(int(((-tiltAngleX) - 10) /1), SPORT_RIDE_LEDS) + NORMAL_RIDE_LEDS + SPORT_RIDE_LEDS;
  } 
  else if (tiltAngleX < -15) {
    //Crash imminent
    activeLEDs = NUM_ACTIVE_LEDS;
  }

  updateLEDs(strip2, activeLEDs, 0, 255, 0);  // Set color (green in this example) for strip2
}

void updateLEDs(Adafruit_NeoPixel &strip, int activeLEDs, int red, int green, int blue) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }

  if (activeLEDs > 0) {
    for (int i = 0; i < activeLEDs; i++) {
      if (i < NORMAL_RIDE_LEDS) {
        strip.setPixelColor(i, strip.Color(0, 255, 0));
      } else if (i < NORMAL_RIDE_LEDS + SPORT_RIDE_LEDS) {
        strip.setPixelColor(i, strip.Color(255, 255, 0));
      } else {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
      }
    }
  }

  strip.show();
}
