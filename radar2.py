import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import math
from collections import deque
import json

class RCRadarMapper:
    def __init__(self):
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)

        # Radar pins
        self.SERVO_PIN = 18
        self.TRIG_PIN = 23
        self.ECHO_PIN = 24

        # RC Car motor control pins
        self.MOTOR_A1 = 5  # Left motor
        self.MOTOR_A2 = 6
        self.MOTOR_B1 = 19  # Right motor
        self.MOTOR_B2 = 26
        self.MOTOR_ENABLE_A = 13  # PWM speed control
        self.MOTOR_ENABLE_B = 12

        # Initialize all GPIO pins
        self.setup_gpio()

        # Map parameters
        self.map_size = (1000, 1000)  # cm
        self.resolution = 5  # cm per pixel
        self.map_array = np.zeros((self.map_size[0] // self.resolution,
                                 self.map_size[1] // self.resolution))

        # Vehicle state
        self.position = [self.map_size[0]//2, self.map_size[1]//2]  # Start at center
        self.heading = 0  # Degrees, 0 is forward
        self.speed = 0
        self.turn_angle = 0

        # Radar parameters
        self.current_radar_angle = 0
        self.radar_direction = 1
        self.max_distance = 400  # cm
        self.scan_history = deque(maxlen=1000)  # Store recent scans

        # Threading control
        self.running = True
        self.lock = threading.Lock()

        # Setup visualization
        self.setup_visualization()

    def setup_gpio(self):
        # Radar GPIO setup
        GPIO.setup(self.TRIG_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.SERVO_PIN, 50)
        self.servo_pwm.start(0)

        # Motor GPIO setup
        gpio_pins = [self.MOTOR_A1, self.MOTOR_A2, self.MOTOR_B1, self.MOTOR_B2,
                    self.MOTOR_ENABLE_A, self.MOTOR_ENABLE_B]
        for pin in gpio_pins:
            GPIO.setup(pin, GPIO.OUT)

        self.motor_pwm_a = GPIO.PWM(self.MOTOR_ENABLE_A, 1000)
        self.motor_pwm_b = GPIO.PWM(self.MOTOR_ENABLE_B, 1000)
        self.motor_pwm_a.start(0)
        self.motor_pwm_b.start(0)

    def setup_visualization(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(15, 7))

        # Radar view (polar plot)
        self.ax1.set_title("Radar View")
        self.radar_line, = self.ax1.plot([], [], 'b-', linewidth=2)
        self.radar_point, = self.ax1.plot([], [], 'ro', markersize=8)
        self.ax1.set_xlim(-self.max_distance, self.max_distance)
        self.ax1.set_ylim(-self.max_distance, self.max_distance)
        self.ax1.grid(True)

        # Map view
        self.ax2.set_title("Floor Map")
        self.map_image = self.ax2.imshow(self.map_array, origin='lower',
                                       extent=(0, self.map_size[0],
                                              0, self.map_size[1]))
        self.car_point, = self.ax2.plot([], [], 'r^', markersize=10)
        self.ax2.grid(True)

    def angle_to_duty_cycle(self, angle):
        return 2 + (angle/180) * 10

    def measure_distance(self):
        GPIO.output(self.TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.TRIG_PIN, GPIO.LOW)

        pulse_start = time.time()
        pulse_timeout = pulse_start + 1

        while GPIO.input(self.ECHO_PIN) == GPIO.LOW:
            pulse_start = time.time()
            if pulse_start > pulse_timeout:
                return self.max_distance

        while GPIO.input(self.ECHO_PIN) == GPIO.HIGH:
            pulse_end = time.time()
            if pulse_end > pulse_timeout:
                return self.max_distance

        distance = (pulse_end - pulse_start) * 17150
        return min(distance, self.max_distance)

    def control_motors(self, left_speed, right_speed):
        # Convert -100 to 100 speed range to PWM duty cycle
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        # Set motor directions
        GPIO.output(self.MOTOR_A1, left_speed > 0)
        GPIO.output(self.MOTOR_A2, left_speed < 0)
        GPIO.output(self.MOTOR_B1, right_speed > 0)
        GPIO.output(self.MOTOR_B2, right_speed < 0)

        # Set motor speeds
        self.motor_pwm_a.ChangeDutyCycle(abs(left_speed))
        self.motor_pwm_b.ChangeDutyCycle(abs(right_speed))

    def update_position(self):
        # Update position based on speed and heading
        dx = self.speed * math.cos(math.radians(self.heading))
        dy = self.speed * math.sin(math.radians(self.heading))

        with self.lock:
            self.position[0] += dx
            self.position[1] += dy
            self.heading += self.turn_angle

            # Keep heading in 0-360 range
            self.heading = self.heading % 360

    def scan_and_map(self):
        while self.running:
            # Move radar
            self.current_radar_angle += self.radar_direction
            if self.current_radar_angle >= 180:
                self.radar_direction = -1
            elif self.current_radar_angle <= 0:
                self.radar_direction = 1

            # Control servo
            duty_cycle = self.angle_to_duty_cycle(self.current_radar_angle)
            self.servo_pwm.ChangeDutyCycle(duty_cycle)

            # Measure distance
            distance = self.measure_distance()

            # Calculate absolute position of detected obstacle
            absolute_angle = (self.heading + self.current_radar_angle) % 360
            rad_angle = math.radians(absolute_angle)

            obstacle_x = self.position[0] + distance * math.cos(rad_angle)
            obstacle_y = self.position[1] + distance * math.sin(rad_angle)

            # Update map
            if 0 <= obstacle_x < self.map_size[0] and 0 <= obstacle_y < self.map_size[1]:
                map_x = int(obstacle_x / self.resolution)
                map_y = int(obstacle_y / self.resolution)
                with self.lock:
                    self.map_array[map_y, map_x] = 1

            # Store scan data
            self.scan_history.append({
                'angle': self.current_radar_angle,
                'distance': distance,
                'position': self.position.copy(),
                'heading': self.heading
            })

            time.sleep(0.03)  # Delay for servo movement

    def update_visualization(self, frame):
        with self.lock:
            # Update radar view
            if self.scan_history:
                angles = [math.radians(s['angle']) for s in self.scan_history]
                distances = [s['distance'] for s in self.scan_history]
                x = [d * math.cos(a) for d, a in zip(distances, angles)]
                y = [d * math.sin(a) for d, a in zip(distances, angles)]
                self.radar_line.set_data(x, y)

                # Current scan point
                current_x = distances[-1] * math.cos(angles[-1])
                current_y = distances[-1] * math.sin(angles[-1])
                self.radar_point.set_data([current_x], [current_y])

            # Update map view
            self.map_image.set_array(self.map_array)
            self.car_point.set_data([self.position[0]], [self.position[1]])

            # Update car heading indicator
            heading_length = 30
            dx = heading_length * math.cos(math.radians(self.heading))
            dy = heading_length * math.sin(math.radians(self.heading))
            self.car_point.set_data([self.position[0]], [self.position[1]])

        return self.radar_line, self.radar_point, self.map_image, self.car_point

    def save_map(self, filename):
        """Save the generated map to a file"""
        map_data = {
            'map_array': self.map_array.tolist(),
            'resolution': self.resolution,
            'map_size': self.map_size
        }
        with open(filename, 'w') as f:
            json.dump(map_data, f)

    def load_map(self, filename):
        """Load a previously saved map"""
        with open(filename, 'r') as f:
            map_data = json.load(f)
        self.map_array = np.array(map_data['map_array'])
        self.resolution = map_data['resolution']
        self.map_size = tuple(map_data['map_size'])

    def run(self):
        # Start scanning thread
        scan_thread = threading.Thread(target=self.scan_and_map)
        scan_thread.daemon = True
        scan_thread.start()

        # Start position update thread
        position_thread = threading.Thread(target=self.update_position)
        position_thread.daemon = True
        position_thread.start()

        # Start visualization
        ani = FuncAnimation(self.fig, self.update_visualization,
                          interval=50, blit=True)
        plt.show()

    def cleanup(self):
        self.running = False
        time.sleep(0.5)
        self.servo_pwm.stop()
        self.motor_pwm_a.stop()
        self.motor_pwm_b.stop()
        GPIO.cleanup()
        plt.close()

# Example usage with keyboard control
from pynput import keyboard

def main():
    mapper = RCRadarMapper()

    def on_press(key):
        try:
            if key == keyboard.Key.up:
                mapper.speed = 5
            elif key == keyboard.Key.down:
                mapper.speed = -5
            elif key == keyboard.Key.left:
                mapper.turn_angle = -2
            elif key == keyboard.Key.right:
                mapper.turn_angle = 2
            elif key == keyboard.Key.space:
                mapper.speed = 0
                mapper.turn_angle = 0
            elif key == keyboard.Key.esc:
                mapper.running = False
                return False
        except AttributeError:
            pass

    def on_release(key):
        if key in [keyboard.Key.left, keyboard.Key.right]:
            mapper.turn_angle = 0
        elif key in [keyboard.Key.up, keyboard.Key.down]:
            mapper.speed = 0

    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        mapper.run()
    except KeyboardInterrupt:
        print("\nStopping mapper...")
    finally:
        mapper.cleanup()

if __name__ == "__main__":
    main()
