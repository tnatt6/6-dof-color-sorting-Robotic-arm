#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150 
#define SERVOMAX 600 
#define SERVO_FREQ 50 

void setup() {
  Serial.begin(115200); 
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(100); // Allow PWM driver to stabilize
  
  // Move to rest position on startup
  moveToNone(); 
  
  Serial.println("Arduino Servo Controller Ready");
  Serial.println("Waiting for commands...");
}

void moveServo(int num, int angle) {
  // Constraints to prevent servo damage
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(num, 0, pulse);
}

// Helper function to set the resting/default state
void moveToNone() {
  Serial.println("Moving to NONE position");
  moveServo(0, 90);  // Base
  delay(200);
  moveServo(1, 135); // Shoulder
  delay(200);
  moveServo(2, 70);  // Elbow
  delay(200);
  moveServo(3, 0);   // Wrist Pitch
  delay(200);
  moveServo(4, 35);  // Wrist Roll
  delay(200);
  moveServo(5, 45);  // Gripper
  delay(200);
}

void moveToRed() {
  Serial.println("Moving to RED position");
  moveServo(0, 45);  // Move Base to Red position
  delay(300);
  moveServo(1, 90);  // Adjust height
  delay(300);
  moveServo(2, 130);
  delay(300);
  moveServo(3, 45);
  delay(300);
  // Servos 4 and 5 stay in current or default position
}

void moveToGreen() {
  Serial.println("Moving to GREEN position");
  moveServo(0, 95);  // Move Base to Green position
  delay(300);
  moveServo(1, 90);
  delay(300);
  moveServo(2, 130);
  delay(300);
  moveServo(3, 45);
  delay(300);
}

void moveToBlue() {
  Serial.println("Moving to BLUE position");
  moveServo(0, 130); // Move Base to Blue position
  delay(300);
  moveServo(1, 90);
  delay(300);
  moveServo(2, 130);
  delay(300);
  moveServo(3, 45);
  delay(300);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the complete command until newline
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any whitespace or newline characters
    
    // Debug: Print what we received
    Serial.print("Received command: '");
    Serial.print(command);
    Serial.println("'");
    
    // Execute command
    if (command == "RED") {
      moveToRed();
    } 
    else if (command == "GREEN") {
      moveToGreen();
    } 
    else if (command == "BLUE") {
      moveToBlue();
    } 
    else if (command == "NONE") {
      moveToNone();
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
    
    Serial.println("Ready for next command");
  }
}
