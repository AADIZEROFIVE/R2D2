#include <ESP32Servo.h>
#include <BluetoothSerial.h>

// Ultrasonic Sensor Pins
const int trigPin = 18;  // TRIG pin of the ultrasonic sensor
const int echoPin = 19;   // ECHO pin of the ultrasonic sensor

// Motor Driver Pins
const int enA = 13;      // Enable pin for Motor A
const int in1 = 33;      // Motor A direction control pin
const int in2 = 32;      // Motor A direction control pin
const int enB = 23;      // Enable pin for Motor B
const int in3 = 21;      // Motor B direction control pin
const int in4 = 22;      // Motor B direction control pin

// Servo Motor Pin
const int servoPin = 25;

// Define PWM channels and resolution
const int pwmChannelA = 2;
const int pwmChannelB = 3;
const int pwmFreq = 1000; // PWM frequency in Hz
const int pwmResolution = 8; // 8-bit resolution (values from 0-255)

// Define variables
long duration;
float distance;
Servo servo;  // Servo object
int scanAngle = 90; // Initial servo angle
BluetoothSerial SerialBT;  // Bluetooth object

void setup() {
    // Initialize Serial Monitor and Bluetooth
    Serial.begin(115200);
    SerialBT.begin("Obstacle_Car");  // Set Bluetooth device name

    // Setup ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Setup motor driver pins
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Configure PWM channels for motor control
    ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
    ledcAttachPin(enA, pwmChannelA);

    ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
    ledcAttachPin(enB, pwmChannelB);

    // Attach servo motor
    servo.attach(servoPin);
    servo.write(scanAngle);  // Point the servo to the front

    Serial.println("Obstacle Avoiding Car with Bluetooth Initialized");
}

void loop() {
    // Measure distance
    distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(1000);

    if (distance < 20) {  // Obstacle detected
        stopCar();
        delay(500);
        avoidObstacle();
    } else {
        moveForward();
    }
}

// Function to measure distance using ultrasonic sensor
float getDistance() {
    // Clear the TRIG pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(4);

    // Trigger the sensor
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(4);
    
    // Measure echo time
    duration = pulseIn(echoPin, HIGH);

    // Calculate distance in cm
    return duration * 0.034 / 2;
}

// Function to send direction via Bluetooth
void sendDirection(const char* direction) {
    SerialBT.println(direction);  // Send direction over Bluetooth
    Serial.println(direction);    // Display on Serial Monitor
}

// Function to move forward
void moveForward() {
    sendDirection("Forward");
    ledcWrite(pwmChannelA, 100); // Set speed (0-255)
    ledcWrite(pwmChannelB, 100); // Set speed (0-255)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

// Function to stop the car
void stopCar() {
    sendDirection("Stopped");
    ledcWrite(pwmChannelA, 0);
    ledcWrite(pwmChannelB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

// Function to move backward
void moveBackward() {
    sendDirection("Backward");
    ledcWrite(pwmChannelA, 100);
    ledcWrite(pwmChannelB, 100);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

// Function to turn left
void turnLeft() {
    sendDirection("Left");
    ledcWrite(pwmChannelA, 100);
    ledcWrite(pwmChannelB, 100);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(500);
    stopCar();
}

// Function to turn right
void turnRight() {
    sendDirection("Right");
    ledcWrite(pwmChannelA, 100);
    ledcWrite(pwmChannelB, 100);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(500);
    stopCar();
}

// Function to avoid obstacle
void avoidObstacle() {
    int leftDistance, rightDistance;

    // Scan right
    servo.write(135);
    delay(500);
    rightDistance = getDistance();

    // Scan left
    servo.write(45);
    delay(500);
    leftDistance = getDistance();

    // Reset to center
    servo.write(90);
    delay(500);

    // Decide direction
    if (rightDistance > leftDistance) {
        turnRight();
    } else {
        turnLeft();
    }
}
