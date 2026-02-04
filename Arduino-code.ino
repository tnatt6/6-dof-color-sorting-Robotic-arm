#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150 
#define SERVOMAX 600 
#define SERVO_FREQ 50 
#define NUM_SERVOS 6

// Servo easing 
struct ServoState {
  int currentAngle;
  int targetAngle;
  int startAngle;
  unsigned long startTime;
  unsigned long duration;
  bool isMoving;
};

ServoState servos[NUM_SERVOS];

void setup() {
  Serial.begin(115200); 
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(100);
  
  // start servo states
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = 90;
    servos[i].targetAngle = 90;
    servos[i].startAngle = 90;
    servos[i].startTime = 0;
    servos[i].duration = 0;
    servos[i].isMoving = false;
  }
  
  // Move to rest position on start
  moveToNone(); 
  
  Serial.println("Arduino Servo Controller with Easing Ready");
  Serial.println("Waiting for commands...");
}

// Easing function - ease in-out cubic for smooth acceleration and deceleration
float easeInOutCubic(float t) {
  if (t < 0.5) {
    return 4 * t * t * t;
  } else {
    float f = ((2 * t) - 2);
    return 0.5 * f * f * f + 1;
  }
}

// Start a smooth movement for a servo
void startEaseTo(int servoNum, int targetAngle, unsigned long durationMs) {
  if (servoNum < 0 || servoNum >= NUM_SERVOS) return;
  
  targetAngle = constrain(targetAngle, 0, 180);
  
  servos[servoNum].startAngle = servos[servoNum].currentAngle;
  servos[servoNum].targetAngle = targetAngle;
  servos[servoNum].startTime = millis();
  servos[servoNum].duration = durationMs;
  servos[servoNum].isMoving = true;
}

// Update all servos - call this continuously in loop()
void updateServos() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].isMoving) {
      unsigned long elapsed = currentTime - servos[i].startTime;
      
      if (elapsed >= servos[i].duration) {
        // Movement complete
        servos[i].currentAngle = servos[i].targetAngle;
        servos[i].isMoving = false;
        setPWMAngle(i, servos[i].currentAngle);
      } else {
        // Calculate eased position
        float progress = (float)elapsed / (float)servos[i].duration;
        float easedProgress = easeInOutCubic(progress);
        
        int angleDiff = servos[i].targetAngle - servos[i].startAngle;
        servos[i].currentAngle = servos[i].startAngle + (int)(angleDiff * easedProgress);
        
        setPWMAngle(i, servos[i].currentAngle);
      }
    }
  }
}

// Direct PWM control (internal use)
void setPWMAngle(int num, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(num, 0, pulse);
}

// Check if any servos are still moving
bool areServosMoving() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].isMoving) return true;
  }
  return false;
}

// Wait for all servos to finish moving
void waitForServos() {
  while (areServosMoving()) {
    updateServos();
    delay(10);
  }
}

// default state 
void moveToNone() {
  Serial.println("Moving to NONE position");
  startEaseTo(0, 90, 1000);   // Base
  delay(100);
  startEaseTo(1, 135, 1000);  // Shoulder
  delay(100);
  startEaseTo(2, 70, 1000);   // Elbow
  delay(100);
  startEaseTo(3, 0, 1000);    // Wrist Pitch
  delay(100);
  startEaseTo(4, 35, 1000);   // Wrist Roll
  delay(100);
  startEaseTo(5, 45, 1000);   // Gripper
  waitForServos();
}

void moveToRed() {
  Serial.println("Moving to RED position");
  startEaseTo(0, 45, 1500);   
  delay(100);
  startEaseTo(1, 105, 1500);   
  delay(100);
  startEaseTo(2, 130, 1500);
  delay(150);
  startEaseTo(3, 45, 1500);
  delay(100);
  startEaseTo(5, 0, 1500);
  delay(100);
  waitForServos();
}

void moveToGreen() {
  Serial.println("Moving to GREEN position");
  startEaseTo(0, 95, 1500);
  delay(100);  
  startEaseTo(1, 105, 1500);
  delay(100);
  startEaseTo(2, 130, 1500);
  delay(100);
  startEaseTo(3, 45, 1500);
  delay(100);
  startEaseTo(5, 0, 1500);
  delay(100);
  waitForServos();
}

void moveToBlue() {
  Serial.println("Moving to BLUE position");
  startEaseTo(0, 130, 1000);  
  delay(100);
  startEaseTo(1, 105, 1000);
  delay(100);
  startEaseTo(2, 0, 1000);
  delay(100);
  startEaseTo(3, 45, 1000);
  delay(100);
  startEaseTo(5, 0, 1000);
  delay(100);
  waitForServos();
}

void loop() {
  // Update servos every command 
  updateServos();
  
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    Serial.print("Received command: '");
    Serial.print(command);
    Serial.println("'");
    
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
  
  delay(10); // Small delay for stability
}
