#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>

// Access Point setup
const char* ssid = "ESP32_Hotspot";  // Name of the hotspot
const char* password = "12345678";   // Password for the hotspot (8+ characters)

// Ultrasonic sensor setup
const int trigPin = 12;  // Trigger pin
const int echoPin = 13;  // Echo pin

// Servo setup
Servo myservo;
int servoPin = 14;  // PWM pin to control the servo

// MPU setup
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Set up ESP32 as an Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP: ");
  Serial.println(IP);

  // Initialize the MPU
  if (!mpu.begin()) {
    Serial.println("Could not find a valid MPU6050 sensor!");
    while (1);
  }

  // Initialize servo
  myservo.attach(servoPin);
  myservo.write(0);  // Set initial position

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Read MPU data (orientation)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  // Start the 180-degree scanning with the ultrasonic sensor
  for (int angle = 0; angle <= 180; angle += 10) {  // Sweep in 10-degree increments
    myservo.write(angle);  // Move the servo to the current angle
    delay(500);            // Wait for the servo to move and stabilize

    // Measure distance
    long distance = measureDistance();

    // Send the sensor data to Python server
    String jsonData = "{\"angle\": " + String(angle) + ", \"distance\": " + String(distance) + ", \"roll\": " + String(roll) + ", \"pitch\": " + String(pitch) + "}";
    sendDataToPython(jsonData);
  }

  delay(1000);  // Delay before the next scan
}

long measureDistance() {
  // Send a 10-microsecond pulse to the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  long distance = duration * 0.034 / 2;  // Speed of sound: 343 m/s
  return distance;
}

void sendDataToPython(String jsonData) {
  HTTPClient http;
  http.begin("http://192.168.4.2:5000/receive");  // Default IP for ESP32 AP client
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(jsonData);
  if (httpCode > 0) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error in sending data");
  }
  http.end();
}
