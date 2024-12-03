from flask import Flask, request, jsonify
import json
import matplotlib.pyplot as plt
import numpy as np

app = Flask(__name__)

# Initialize map storage
robot_positions = []

# Flask route to receive data from ESP32
@app.route('/receive', methods=['POST'])
def receive_data():
    data = request.get_json()
    angle = data['angle']
    distance = data['distance']
    roll = data['roll']
    pitch = data['pitch']
    
    # Convert polar coordinates (angle, distance) to Cartesian coordinates (x, y)
    x, y = polar_to_cartesian(angle, distance)
    
    # Store robot position and angle
    robot_positions.append((x, y, angle, distance))
    
    # Save the data (map) after receiving each update
    save_map(robot_positions)
    
    return jsonify({"status": "success"})

def polar_to_cartesian(angle, distance):
    """ Convert polar coordinates (angle, distance) to Cartesian coordinates (x, y). """
    # Convert angle to radians
    angle_rad = np.radians(angle)
    x = distance * np.cos(angle_rad)
    y = distance * np.sin(angle_rad)
    return x, y

def save_map(robot_positions):
    # Save map as JSON file
    with open("map.json", "w") as f:
        json.dump(robot_positions, f)

def plot_map():
    # Visualize the map
    x_vals, y_vals = zip(*[(pos[0], pos[1]) for pos in robot_positions])
    plt.scatter(x_vals, y_vals)
    plt.title("Floor Map")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

if __name__ == '__main__':
    app.run(host='192.168.1.15', port=5000)
