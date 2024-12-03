#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Ultrasonic.h>
#include <ESP32Servo.h>

// WiFi setup
const char* ssid = "your_wifi_ssid";
const char* password = "12345678";

// Ultrasonic sensor setup
Ultrasonic ultrasonic(12, 13);  // Trigger pin, Echo pin

// Servo setup
Servo myservo;
int servoPin = 14;  // PWM pin to control the servo

// MPU setup
Adafruit_MPU6050 mpu;

void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize the MPU
  if (!mpu.begin()) {
    Serial.println("Could not find a valid MPU6050 sensor");
    while (1);
  }

  // Initialize servo
  myservo.attach(servoPin);
  myservo.write(0);  // Set initial position

  // Initialize sensor values
  ultrasonic.setTimeout(4000);
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
    delay(500);  // Wait for the servo to move and stabilize

    // Read ultrasonic sensor data
    long distance = ultrasonic.read();

    // Send the sensor data to Python server
    String jsonData = "{\"angle\": " + String(angle) + ", \"distance\": " + String(distance) + ", \"roll\": " + String(roll) + ", \"pitch\": " + String(pitch) + "}";
    sendDataToPython(jsonData);
  }
  
  delay(1000);  // Delay before next scan
}

void sendDataToPython(String jsonData) {
  HTTPClient http;
  http.begin("http://your_computer_ip:port/receive");  // Change this to your computer's IP and endpoint
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(jsonData);
  if (httpCode > 0) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error in sending data");
  }
  http.end();
}
