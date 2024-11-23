#include <ESP32Servo.h>
#include <NewPing.h>

// Pin definitions
#define TRIG_PIN 18 // sets pin for TRIG of ultrasonic
#define ECHO_PIN 5 // sets pin for ECHO of ultrasonic
#define MAX_DISTANCE 200 // sets MAX distance for ultrasonic
#define MAX_SPEED 190 // sets MAX speed of DC motors
#define MAX_SPEED_OFFSET 20

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

Servo myservo;

bool goesForward = false;
int distance = 100;
int speedSet = 0;

// Motor A connections
const int enA = 13; // Enable pin for Motor A
const int in1 = 33; // Input 1 for Motor A
const int in2 = 32; // Input 2 for Motor A

// Motor B connections
const int enB = 23; // Enable pin for Motor B
const int in3 = 21; // Input 3 for Motor B
const int in4 = 22; // Input 4 for Motor B

// Servo motor connection
const int servoPin = 25;

// PWM Channels
const int PWM_CHANNEL_A = 0; // PWM channel for Motor A
const int PWM_CHANNEL_B = 1; // PWM channel for Motor B

void setup() {
    // Set pin modes for motor controls
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Configure PWM channels
    ledcSetup(PWM_CHANNEL_A, 5000, 8); // 5 kHz frequency, 8-bit resolution
    ledcAttachPin(enA, PWM_CHANNEL_A);

    ledcSetup(PWM_CHANNEL_B, 5000, 8);
    ledcAttachPin(enB, PWM_CHANNEL_B);

    // Attach servo motor and move it to the center position
    myservo.attach(servoPin);
    myservo.write(115);

    // Wait for the ultrasonic sensor to stabilize and read the initial distance
    delay(2000);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);
    distance = readPing();
    delay(100);
}

void loop() {
    int distanceR = 0;
    int distanceL = 0;
    delay(40);

    // If an obstacle is detected within 15 cm, stop and turn
    if (distance <= 15) {
        moveStop();
        delay(100);
        moveBackward();
        delay(300);
        moveStop();
        delay(200);
        distanceR = lookRight();
        delay(200);
        distanceL = lookLeft();
        delay(200);

        // Choose the direction with the greater distance and turn
        if (distanceR >= distanceL) {
            turnRight();
            moveStop();
        } else {
            turnLeft();
            moveStop();
        }
    } else {
        // Move forward
        moveForward();
    }

    // Read the current distance
    distance = readPing();
}

// Turn the servo motor to the right and return the distance to the nearest obstacle
int lookRight() {
    myservo.write(50);
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115);
    return distance;
}

// Turn the servo motor to the left and return the distance to the nearest obstacle
int lookLeft() {
    myservo.write(170);
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115);
    return distance;
}

// Send a ping to the ultrasonic sensor and return the distance to the nearest obstacle
int readPing() {
    int attempts = 5;
    int cm = 0;
    for (int i = 0; i < attempts; i++) {
        delay(70);
        cm = sonar.ping_cm();
        if (cm > 0) {
            return cm;
        }
    }
    return MAX_DISTANCE;
}

// Stop the motors
void moveStop() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    goesForward = false;
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
}

// Move forward
void moveForward() {
    if (!goesForward) {
        goesForward = true;
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        rampUpSpeed();
    }
}

// Move backward
void moveBackward() {
    if (goesForward) {
        goesForward = false;
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        rampUpSpeed();
    }
}

// Turn right
void turnRight() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    ledcWrite(PWM_CHANNEL_A, MAX_SPEED);
    ledcWrite(PWM_CHANNEL_B, MAX_SPEED);
    delay(500);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    goesForward = true;
    rampUpSpeed();
}

// Turn left
void turnLeft() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    ledcWrite(PWM_CHANNEL_A, MAX_SPEED);
    ledcWrite(PWM_CHANNEL_B, MAX_SPEED);
    delay(500);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    goesForward = true;
    rampUpSpeed();
}

// Ramp up the speed gradually to avoid loading down the batteries too quickly
void rampUpSpeed() {
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
        ledcWrite(PWM_CHANNEL_A, speedSet);
        ledcWrite(PWM_CHANNEL_B, speedSet);
        delay(5);
    }
    ledcWrite(PWM_CHANNEL_A, MAX_SPEED);
    ledcWrite(PWM_CHANNEL_B, MAX_SPEED);
}
