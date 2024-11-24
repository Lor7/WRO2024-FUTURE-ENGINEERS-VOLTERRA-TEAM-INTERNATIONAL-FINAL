#include <Servo.h>

// Create a Servo object
Servo myServo;
// Pin connected to the servo signal wire
const int servoPin = 2;
// Define offset and angle variable
const int OFFSET = 0;  // Offset angle (you can change this value as needed)
int currentAngle = OFFSET;

void setup() {
  // Attach the servo to the pin
  myServo.attach(servoPin);
  // Start Serial communication at 115200 baud rate
  Serial.begin(115200);
  //Serial.println("Servo test started");
  // Set servo to initial offset angle
  myServo.write(100);
  //Serial.print("Initial Position set to OFFSET: ");
  //Serial.println(currentAngle);
  // Set a timeout for Serial.parseInt (default is 1000ms)
  Serial.setTimeout(10); // 50ms timeout to handle invalid input quickly
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    int receivedAngle = Serial.parseInt();  // Read integer from serial
    clearSerialBuffer();               // Remove any unwanted characters
    // Validate the angle (0 to 180 degrees)
    if ((receivedAngle - 50)>= 0 && (receivedAngle - 50)<= 180) {
      currentAngle = receivedAngle + OFFSET;  // Apply offset if needed
      myServo.write(currentAngle);            // Move servo to the new angle
      // Print the new position
      //Serial.print("New Position set to: ");
      //Serial.println(currentAngle);
    } else { ;
    }
  }
}

// Function to clear unwanted characters from the serial buffer
void clearSerialBuffer() {
  while (Serial.available() > 0 && !isDigit(Serial.peek())) {
    Serial.read(); // Discard non-digit characters
  }
}
