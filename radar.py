import RPi.GPIO as GPIO
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Thread, Lock

class UltrasonicRadar:
    def __init__(self):
        # GPIO Pin configuration
        self.SERVO_PIN = 18  # PWM pin for servo
        self.TRIG_PIN = 23   # Trigger pin for ultrasonic
        self.ECHO_PIN = 24   # Echo pin for ultrasonic

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)

        # Setup PWM for servo
        self.servo_pwm = GPIO.PWM(self.SERVO_PIN, 50)  # 50Hz frequency
        self.servo_pwm.start(0)

        # Initialize data storage
        self.angles = np.linspace(0, 180, 181)
        self.distances = [0] * 181
        self.max_distance = 400  # Maximum distance in cm
        self.current_angle = 0
        self.direction = 1

        # Threading
        self.data_lock = Lock()
        self.running = True

        # Setup plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = plt.subplot(111, projection='polar')
        self.line, = self.ax.plot([], [], 'b-', linewidth=2)
        self.points, = self.ax.plot([], [], 'ro', markersize=8)

        # Configure plot
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_rlim(0, self.max_distance)
        self.ax.set_rticks(np.linspace(0, self.max_distance, 5))
        self.ax.set_thetamin(0)
        self.ax.set_thetamax(180)
        self.ax.grid(True)
        plt.title("Ultrasonic Radar - Raspberry Pi 5")

    def angle_to_duty_cycle(self, angle):
        """Convert angle (0-180) to duty cycle (2-12)"""
        return 2 + (angle/180) * 10

    def measure_distance(self):
        """Measure distance using ultrasonic sensor"""
        # Send trigger pulse
        GPIO.output(self.TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.TRIG_PIN, GPIO.LOW)

        # Wait for echo
        pulse_start = time.time()
        pulse_timeout = pulse_start + 1  # 1 second timeout

        # Wait for echo to start
        while GPIO.input(self.ECHO_PIN) == GPIO.LOW:
            pulse_start = time.time()
            if pulse_start > pulse_timeout:
                return self.max_distance

        # Wait for echo to end
        pulse_end = time.time()
        pulse_timeout = pulse_end + 1

        while GPIO.input(self.ECHO_PIN) == GPIO.HIGH:
            pulse_end = time.time()
            if pulse_end > pulse_timeout:
                return self.max_distance

        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound / 2

        return min(distance, self.max_distance)

    def radar_scan(self):
        """Continuously scan and measure distances"""
        while self.running:
            # Update servo position
            duty_cycle = self.angle_to_duty_cycle(self.current_angle)
            self.servo_pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.03)  # Allow servo to reach position

            # Measure distance
            distance = self.measure_distance()

            # Update data with thread safety
            with self.data_lock:
                self.distances[self.current_angle] = distance

            # Update angle
            self.current_angle += self.direction
            if self.current_angle >= 180:
                self.direction = -1
            elif self.current_angle <= 0:
                self.direction = 1

    def update_plot(self, frame):
        """Update the radar plot"""
        with self.data_lock:
            theta = np.radians(self.angles)
            self.line.set_data(theta, self.distances)

            current_theta = np.radians([self.current_angle])
            current_distance = [self.distances[self.current_angle]]
            self.points.set_data(current_theta, current_distance)

        return self.line, self.points

    def run(self):
        """Start the radar system"""
        # Start scanning thread
        scan_thread = Thread(target=self.radar_scan)
        scan_thread.daemon = True
        scan_thread.start()

        # Start animation
        ani = FuncAnimation(self.fig, self.update_plot, frames=None,
                          interval=50, blit=True)
        plt.show()

    def cleanup(self):
        """Clean up GPIO and close plot"""
        self.running = False
        time.sleep(0.5)  # Allow thread to complete
        self.servo_pwm.stop()
        GPIO.cleanup()
        plt.close()

if __name__ == "__main__":
    try:
        radar = UltrasonicRadar()
        radar.run()
    except KeyboardInterrupt:
        print("\nStopping radar...")
    finally:
        radar.cleanup()
