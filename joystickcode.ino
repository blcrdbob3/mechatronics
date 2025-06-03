#include <Servo.h>

#define VRX_PIN  A0 // Arduino pin connected to VRX pin
#define VRY_PIN  A1 // Arduino pin connected to VRY pin

int xValue = 0;
int yValue = 0;
int valX;
int valY;
Servo myservoX;
Servo myservoY;

void setup() {
  Serial.begin(9600) ;
  myservoX.attach(9);
  myservoY.attach(10);
}

void loop() {
  // read analog X and Y analog values
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);
 

  // print data to Serial Monitor on Arduino IDE
  Serial.print("x = ");
  Serial.print(xValue);
  Serial.print(", y = ");
  Serial.println(yValue);
  valX = map(xValue, 0, 1023, 0, 180);
  valY = map(yValue, 0, 1023, 0, 180);
  myservoX.write(valX);
  myservoY.write(valY);

  delay(20);
}