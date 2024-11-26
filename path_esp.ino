
#include <NewPing.h>

#define TRIG_PIN 12
#define ECHO_PIN 14
#define MAX_DISTANCE 200
#define MOTOR_LEFT 5
#define MOTOR_RIGHT 4

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(115200); // UART communication with Raspberry Pi
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
}

void loop() {
  static String command = "";
  
  // Check for incoming commands from Raspberry Pi
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      executeCommand(command);
      command = "";
    } else {
      command += c;
    }
  }
  
  // Obstacle avoidance logic
  int distance = sonar.ping_cm();
  if (distance > 0 && distance < 20) {
    stopMotors();
    Serial.println("stop");
  } else {
    moveForward();
    Serial.println("forward");
  }
  delay(100);
}

void executeCommand(String cmd) {
  if (cmd == "forward") {
    moveForward();
  } else if (cmd == "left") {
    turnLeft();
  } else if (cmd == "right") {
    turnRight();
  } else if (cmd == "stop") {
    stopMotors();
  }
}

void moveForward() {
  digitalWrite(MOTOR_LEFT, HIGH);
  digitalWrite(MOTOR_RIGHT, LOW);
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT, LOW);
  digitalWrite(MOTOR_RIGHT, HIGH);
}

void turnRight() {
  digitalWrite(MOTOR_LEFT, HIGH);
  digitalWrite(MOTOR_RIGHT, HIGH);
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT, LOW);
  digitalWrite(MOTOR_RIGHT, LOW);
}

