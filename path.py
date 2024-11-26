import serial
import time

# Initialize serial communication with ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Path data storage
path_data = []

def record_path():
    """
    Record the car's path while performing obstacle avoidance.
    """
    print("Recording path. Press Ctrl+C to stop.")
    try:
        while True:
            # Read data from ESP32 (e.g., obstacle avoidance status and action)
            data = ser.readline().decode().strip()
            print(f"Action: {data}")
            
            # Store the action in the path_data list
            path_data.append(data)
            
            # Add a delay for better readability
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Path recording stopped.")
        save_path()

def save_path():
    """
    Save the recorded path to a file.
    """
    with open('path_data.txt', 'w') as f:
        for entry in path_data:
            f.write(f"{entry}\n")
    print("Path saved to 'path_data.txt'.")

def trace_path():
    """
    Replay the recorded path.
    """
    try:
        with open('path_data.txt', 'r') as f:
            print("Tracing path...")
            for command in f:
                # Send each recorded command back to the ESP32
                ser.write(command.strip().encode())
                ser.write(b'\n')  # Ensure proper formatting for ESP32
                print(f"Sent command: {command.strip()}")
                time.sleep(0.5)  # Adjust delay based on action duration
        print("Path tracing completed.")
    except FileNotFoundError:
        print("No path data found. Please record a path first.")

# Main program menu
if __name__ == "__main__":
    while True:
        print("\n1. Record Path")
        print("2. Trace Path")
        print("3. Exit")
        choice = input("Select an option: ")
        
        if choice == "1":
            record_path()
        elif choice == "2":
            trace_path()
        elif choice == "3":
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please try again.")

