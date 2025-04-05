# TODO: Add a high-pass filter to the PPG data
import json
import numpy as np
import pyqtgraph as pg
import paho.mqtt.client as mqtt
import time
import csv
from collections import deque

# --------------- High-pass Filter --------------- #
def high_pass_filter(x, x_prev, y_prev):
    fc = 0.5  # Cutoff frequency in Hz
    fs = 50    # Sampling frequency in Hz (must be > 2 * fc)
    T = 1 / fs  # Sampling interval
    tau = 1 / (2 * np.pi * fc)  # Time constant
    alpha = tau / (tau + T)  # Filter coefficient

    """High-pass filter to remove DC component."""
    return alpha * y_prev + alpha * (x - x_prev)

# --------------- CSV File --------------- #
filename = time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv" # Generate a safe filename with timestamp
f = open(filename, "a", newline="") # Open the CSV file
output_writer = csv.writer(f) # Define CSV writer
output_writer.writerow(["Time(ms)","GSR","ECG","IR Channel", "Red Channel", "PPG"]) # Write the header
print(f"CSV file created: {filename}")

def write_batch_to_csv(time_list, ir_list, red_list, ecg_list, gsr_list, ppg_avg_list):
    """ Writes a batch of sensor readings to a CSV file. """

    # Write all rows at once
    rows = zip(time_list, ir_list, red_list, ecg_list, gsr_list, ppg_avg_list)
    output_writer.writerows(rows)

    print(f"✅ Successfully saved {len(time_list)} samples to {filename}")


# --------------- PyQTGraph --------------- #
app = pg.mkQApp()
win = pg.GraphicsLayoutWidget()
plot = win.addPlot()
curve = plot.plot()

# Data storage (deque for smooth animation)
max_length = 1000  # Store last 100 data points
time_series = deque([0], maxlen=max_length)
ir_series = deque([0], maxlen=max_length)
red_series = deque([0], maxlen=max_length)
average_series = deque([0], maxlen=max_length)
processed_series = deque([0], maxlen=max_length)

# --------------- MQTT and Data processing --------------- #
# HiveMQ Cloud Credentials
BROKER = "700be638167b43289186dff783367cc3.s1.eu.hivemq.cloud"
PORT = 8883  # Secure MQTT port
USERNAME = "ngocdo"
PASSWORD = "Ng19102002"
TOPICS_SUBSCRIBE = [("VitalX_001/status", 0), ("VitalX_001/data", 1), ("VitalX_001/cmd", 1)]  # List of (topic, QoS)

def plot_data(time_series, ir_series):
    if len(time_series) != len(ir_series):
        print(f"Mismatch in time and sample lengths: {len(time_series)} vs {len(ir_series)}")
        return

    # Update PyQtGraph plot
    curve.setData(np.array(time_series), np.array(ir_series))

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to HiveMQ Cloud!")
        client.subscribe(TOPICS_SUBSCRIBE)
    else:
        print(f"Connection failed with code {rc}")
        
def on_message(client, userdata, msg):
    if msg.topic == "VitalX_001/data":  # Process only messages from "data" topic
        try:
            data = json.loads(msg.payload.decode("utf-8"))

            if not isinstance(data, dict):  # Ensure it's a dictionary
                raise ValueError("Received data is not a dictionary")
            
            # Extract sensor data from the JSON message
            time_list = data.get("time", [])
            ir_list = data.get("ir", [])
            red_list = data.get("red", [])
            ecg_list = data.get("ecg", [])
            gsr_list = data.get("gsr", [])

            # Ensure they are lists, not single values
            if not (isinstance(time_list, list) and isinstance(ir_list, list) and isinstance(red_list, list) and isinstance(ecg_list, list) and isinstance(gsr_list, list)):
                # If any of the lists are not present or not lists, print an error message
                print("❌ Invalid JSON format: 'time', 'ir', 'red', 'ecg', or 'gsr' is missing or not a list.")
                return
            
            # Ensure all lists have the same length
            batch_size = len(time_list)
            if not all(len(lst) == batch_size for lst in [ir_list, red_list, ecg_list, gsr_list]):
                print("❌ Error: Mismatch in batch sizes of sensor data.")
                print(f"Batch sizes: time={len(time_list)}, ir={len(ir_list)}, red={len(red_list)}, ecg={len(ecg_list)}, gsr={len(gsr_list)}")
                return
            
            # Compute PPG Avg for the entire batch
            ppg_avg_list = [-1 * (ir + red) / 2 for ir, red in zip(ir_list, red_list)]

            # Write to CSV
            write_batch_to_csv(time_list, ir_list, red_list, ecg_list, gsr_list, ppg_avg_list)

            # Plot the last batch of PPG Avg values
            plot_data(time_list, ppg_avg_list)

        except json.JSONDecodeError as e:
            print(f"Invalid JSON format: {msg.payload.decode('utf-8')} - Error: {e}")

        except Exception as e:
            print(f"❌ Unexpected error: {e}")
    else:
        print(f"Received message: {msg.payload.decode()} on topic {msg.topic}")

# --------------- Application --------------- #
# Setup MQTT client
client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)
client.tls_set()  # Enables TLS encryption
client.on_connect = on_connect
client.on_message = on_message

# Connect and start the loop
try:
    client.connect(BROKER, PORT, 60)
    client.loop_start()  # Non-blocking loop
    # Show the real-time plot
    win.show()
    app.exec()
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