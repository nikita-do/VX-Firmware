# Description: This script subscribes to a MQTT broker and receives sensor data from a device.
#              The script processes the data and stores it in a CSV file.
#              It also displays a real-time plot of the PPG data using Matplotlib.

import paho.mqtt.client as mqtt

from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation  

import time
import csv
import json

import numpy as np

# --------------- High-pass Filter --------------- #
fc = 0.5  # Cutoff frequency in Hz
fs = 50    # Sampling frequency in Hz (must be > 2 * fc)
T = 1 / fs  # Sampling interval
tau = 1 / (2 * np.pi * fc)  # Time constant
alpha = tau / (tau + T)  # Filter coefficient

def high_pass_filter(x, x_prev, y_prev):
    """High-pass filter to remove DC component."""
    return alpha * y_prev + alpha * (x - x_prev)

# --------------- CSV File --------------- #
filename = time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv" # Generate a safe filename with timestamp
f = open(filename, "w", newline="") # Open the CSV file
output_writer = csv.writer(f) # Define CSV writer
output_writer.writerow(["time","gsr","ecg","ir", "red", "ppg"]) # Write the header
print(f"CSV file created: {filename}")


# --------------- MQTT and Data processing --------------- #
# HiveMQ Cloud Credentials
BROKER = "700be638167b43289186dff783367cc3.s1.eu.hivemq.cloud"
PORT = 8883  # Secure MQTT port
USERNAME = "ngocdo"
PASSWORD = "Ng19102002"
TOPICS_SUBSCRIBE = [("VitalX_001/status", 0), ("VitalX_001/data", 1), ("VitalX_001/cmd", 1)]  # List of (topic, QoS)

# Data storage (deque for smooth animation)
max_length = 100  # Store last 100 data points
time_series = deque([0], maxlen=max_length)
ir_series = deque([0], maxlen=max_length)
red_series = deque([0], maxlen=max_length)
average_series = deque([0], maxlen=max_length)
processed_series = deque([0], maxlen=max_length)

# Data processing function
def process_data(msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        # Extract sensor data from the JSON message
        ir = data.get("ppg_ir")
        red = data.get("ppg_red")
        ecg = data.get("ecg")
        gsr = data.get("gsr")
        time = data.get("time")

        if not isinstance(data, dict):  # Ensure it's a dictionary
            raise ValueError("Received data is not a dictionary")
        
        if ir is not None and red is not None:
            ppg_avg = (-1)*(ir + red) / 2  # Compute average from inverted PPG signals

            # Apply high-pass filter to remove DC component
            filter_ppg = high_pass_filter(ppg_avg, average_series[-1], processed_series[-1])
            processed_series.append(filter_ppg)

            time_series.append(time_series[-1]+1)
            ir_series.append(ir)
            red_series.append(red)
            average_series.append(ppg_avg)

            output_writer.writerow([time, gsr, ecg, ir, red, filter_ppg])

        else:    
            print("Invalid JSON format: Missing 'ir' or 'red'")

    except json.JSONDecodeError as e:
        print(f"Invalid JSON format: {msg.payload.decode('utf-8')} - Error: {e}")

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to HiveMQ Cloud!")
        client.subscribe(TOPICS_SUBSCRIBE)
    else:
        print(f"Connection failed with code {rc}")

# Callback when a message is received
def on_message(client, userdata, msg):
    if msg.topic == "VitalX_001/data":  # Process only messages from "data" topic
        process_data(msg)
    else:
        print(f"Received message: {msg.payload.decode()} on topic {msg.topic}")

# Update plot function
def update_plot(frame):
    """Update the plot dynamically with new data."""
    line_ir.set_data(time_series, ir_series)
    line_red.set_data(time_series, red_series)
    line_avg.set_data(time_series, processed_series)

    ax.relim()  # Recalculate limits
    ax.autoscale_view()  # Autoscale

    return line_ir, line_red, line_avg


# --------------- Application --------------- #
# Setup MQTT client
client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)
client.tls_set()  # Enables TLS encryption
client.on_connect = on_connect
client.on_message = on_message

# Matplotlib real-time plot
fig, ax = plt.subplots()
ax.set_title("Real-Time Sensor Data")
ax.set_xlabel("Time")
ax.set_ylabel("Sensor Values")
line_ir, = ax.plot([], [], "r-", label="IR")  # Red line
line_red, = ax.plot([], [], "b-", label="Red")  # Blue line
line_avg, = ax.plot([], [], "g-", label="Average")  # Green line
ax.legend()

# Animation function
ani = FuncAnimation(fig, update_plot, interval=20)

# Connect and start the loop
try:
    client.connect(BROKER, PORT, 60)
    client.loop_start()  # Non-blocking loop
    # Show the real-time plot
    plt.show()
    
    while True:
        time.sleep(1)  # Keep the script running

# Ctrl+C to stop the script
except KeyboardInterrupt:
    print("\nDisconnected by user")
    client.loop_stop()  # Stop the MQTT loop
    client.disconnect()  # Disconnect from the broker

except Exception as e:
    print(f"Error occurred: {e}")

finally:
    try:
        f.close()  # Ensure file is closed safely
        print("CSV file closed")
    except NameError:
        pass  # If 'f' was never created, avoid an error