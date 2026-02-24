#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150 
#define SERVOMAX 600 
#define SERVO_FREQ 50 
#define NUM_SERVOS 6

struct ServoState {
  int currentAngle;
  int targetAngle;
  int startAngle;
  unsigned long startTime;
  unsigned long duration;
  bool isMoving;
};

ServoState servos[NUM_SERVOS];
String currentColor = "NONE";

void setup() {
  Serial.begin(115200); 
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].currentAngle = 90;
    servos[i].targetAngle = 90;
    servos[i].startAngle = 90;
    servos[i].startTime = 0;
    servos[i].duration = 0;
    servos[i].isMoving = false;
  }
  
  moveToNone(); 
  Serial.println("Ready - Waiting for commands...");
}

float easeInOutCubic(float t) {
  if (t < 0.5) {
    return 4 * t * t * t;
  } else {
    float f = ((2 * t) - 2);
    return 0.5 * f * f * f + 1;
  }
}

void startEaseTo(int servoNum, int targetAngle, unsigned long durationMs) {
  if (servoNum < 0 || servoNum >= NUM_SERVOS) return;
  targetAngle = constrain(targetAngle, 0, 180);
  servos[servoNum].startAngle = servos[servoNum].currentAngle;
  servos[servoNum].targetAngle = targetAngle;
  servos[servoNum].startTime = millis();
  servos[servoNum].duration = durationMs;
  servos[servoNum].isMoving = true;
}

void updateServos() {
  unsigned long currentTime = millis();
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].isMoving) {
      unsigned long elapsed = currentTime - servos[i].startTime;
      if (elapsed >= servos[i].duration) {
        servos[i].currentAngle = servos[i].targetAngle;
        servos[i].isMoving = false;
        setPWMAngle(i, servos[i].currentAngle);
      } else {
        float progress = (float)elapsed / (float)servos[i].duration;
        float easedProgress = easeInOutCubic(progress);
        int angleDiff = servos[i].targetAngle - servos[i].startAngle;
        servos[i].currentAngle = servos[i].startAngle + (int)(angleDiff * easedProgress);
        setPWMAngle(i, servos[i].currentAngle);
      }
    }
  }
}

void setPWMAngle(int num, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(num, 0, pulse);
}

bool areServosMoving() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].isMoving) return true;
  }
  return false;
}

void waitForServos() {
  while (areServosMoving()) {
    updateServos();
    delay(10);
  }
}

void moveToNone() {
  Serial.println("Moving to rest");
  startEaseTo(0, 90, 800);
  startEaseTo(1, 135, 800);
  startEaseTo(2, 70, 800);
  startEaseTo(3, 0, 800);
  startEaseTo(4, 35, 800);
  startEaseTo(5, 45, 800);
  waitForServos();
}

void moveToPick() {
  Serial.println("Moving to Pick");
  startEaseTo(0, 0, 800);
  startEaseTo(1, 85, 800);
  startEaseTo(2, 180, 800);
  startEaseTo(3, 50, 800);
  startEaseTo(4, 26, 800);
  startEaseTo(5, 0, 600);
  waitForServos();
  
  startEaseTo(5, 45, 500);    // close gripper - grab object
  waitForServos();
}

void dropDown() {
  startEaseTo(1, 85, 800);
  startEaseTo(2, 180, 800);
  startEaseTo(3, 50, 800);
  waitForServos();
  startEaseTo(5, 0, 500);     // open gripper - release object
  waitForServos();
}

void moveToRed() {
  Serial.println("Moving to RED");
  moveToNone();
  startEaseTo(0, 45, 800);
  waitForServos();
  dropDown();
}

void moveToGreen() {
  Serial.println("Moving to GREEN");
  moveToNone();
  startEaseTo(0, 95, 800);
  waitForServos();
  dropDown();
}

void moveToBlue() {
  Serial.println("Moving to BLUE");
  moveToNone();
  startEaseTo(0, 130, 800);
  waitForServos();
  dropDown();
}

void loop() {
  updateServos();
  
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    Serial.print("Received command: '");
    Serial.print(command);
    Serial.println("'");
    
    if (command == "RED" || command == "GREEN" || command == "BLUE") {
      moveToPick();
      
      if (command == "RED") moveToRed();
      else if (command == "GREEN") moveToGreen();
      else if (command == "BLUE") moveToBlue();
      
      currentColor = command;
    }
    else if (command == "NONE") {
      moveToNone();
      currentColor = "NONE";
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
    
    Serial.println("Waiting for next command...");
  }
  
  delay(10);
}
