#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "person_sensor.h"

// How long to wait between reading the Person Sensor
const int32_t SAMPLE_DELAY_MS = 100;  // default 200ms

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo pulse width range
#define SERVOMIN 150  // minimum pulse length count (out of 4096)
#define SERVOMAX 600  // maximum pulse length count (out of 4096)

// Servo frequency for analog servos
#define SERVO_FREQ 50  // 50 Hz for analog servos

// Person sensor
int box_center_x;
int box_center_y;
int prev_center_x = 1;

// Servo variables
uint8_t servonum = 0;
int xval;
int yval;
int xpulse;
int ypulse;
int randNumber;

// Timer
unsigned long previousMillis = 0;
const long interval = 2000; // equal to 2000 milliseconds, which is equal to 2 seconds

// Joystick button pin
const int buttonPin = 2;  // Joystick button (SW) connected to pin 2
int switchState = 1;

void setup() {
  // Initialize I2C and serial communication
  Wire.begin();
  pinMode(buttonPin, INPUT);  // Set button pin as input
  digitalWrite(buttonPin, HIGH); // Set button pin as digital
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // Set oscillator frequency
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  Serial.println("Setup complete. Up and running.");
}

void blink() {
  Serial.println("Blink");
  // code to close eyes, index 2-5 are the ones controlling eyelids, middle servo is servo #1, then servo controlling eyes is servo #0

  delay(1000);
  // code to open eyes
}

void loop() {
  unsigned long currentMillis = millis();
  randNumber = random(1000);
  if ((currentMillis - previousMillis >= interval) & (randNumber >= 990)) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    Serial.println("Previous Millis: " + String(previousMillis));
    Serial.println("Current Millis: " + String(currentMillis));
    blink();
  }
  switchState = digitalRead(buttonPin);
  // Read the joystick button (SW) and check if it's pressed
  if (!switchState) {  // LOW means pressed (since the button might be wired to ground)
    closeEyes();  // Call closeEyes function if button is pressed
  }

  // Read the person sensor data
  person_sensor_results_t results = {};
  if (!person_sensor_read(&results)) {
    Serial.println("No person sensor results found on the I2C bus");
    return;
  }

  // Process each detected face
  for (int i = 0; i < results.num_faces; ++i) {
    const person_sensor_face_t* face = &results.faces[i];
    
    // Calculate the center of the detected face
    box_center_x = (face->box_left + ((face->box_right)-(face->box_left)) / 2);
    box_center_y = (face->box_bottom + ((face->box_top)-(face->box_bottom)) / 2);

    // Map the x and y values to control the servo
    xval = ((box_center_x * -4) + 1023) * 1.2;
    yval = ((box_center_y * -4) + 1023) * 1.2;

    // Map xval to servo PWM pulse width
    xpulse = map(xval, 0, 1023, SERVOMIN, SERVOMAX);
    ypulse = map(yval, 0, 1023, SERVOMIN, SERVOMAX);

    // Print the pulse value for debugging
    Serial.print("Moving servo to pulse: ");
    Serial.println(xpulse);

    // Gradually move the servo to the new position
    pwm.setPWM(0, 0, xpulse);  // Set PWM for the servo
    pwm.setPWM(1, 0, ypulse);
    delay(SAMPLE_DELAY_MS);  // Add a delay between each frame
  }
}

// Function to simulate "closing the eyes" by moving the servo to a closed position
void closeEyes() {
  Serial.println("Joystick button pressed. Closing eyes...");
  pwm.setPWM(0, 0, SERVOMIN);  // Move the servo to the minimum position (closed eyes)
  Serial.println("Servo moved to closed position (pulse: " + String(SERVOMIN) + ")");
}
