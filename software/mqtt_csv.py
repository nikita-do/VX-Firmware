import paho.mqtt.client as mqtt
# import ast
import time
import json
from csv import DictWriter

# Generate a safe filename with timestamp
filename = time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"

# Open the CSV file
f = open(filename, "w", newline="")

# Define CSV writer with field names
output_writer = DictWriter(f, fieldnames=["time", "gsr", "ecg", "ppg_ir", "ppg_red"])
output_writer.writeheader()

print(f"CSV file created: {filename}")

# HiveMQ Cloud Credentials
BROKER = "700be638167b43289186dff783367cc3.s1.eu.hivemq.cloud"
PORT = 8883  # Secure MQTT port
USERNAME = "ngocdo"
PASSWORD = "Ng19102002"
TOPICS = [("VitalX_001/status", 0), ("VitalX_001/data", 1), ("VitalX_001/cmd", 1)]  # List of (topic, QoS)

def process_data(msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))

        if not isinstance(data, dict):  # Ensure it's a dictionary
            raise ValueError("Received data is not a dictionary")
        
        output_writer.writerow(data)

    except json.JSONDecodeError as e:
        print(f"Invalid JSON format: {msg.payload.decode('utf-8')} - Error: {e}")

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to HiveMQ Cloud!")
        client.subscribe(TOPICS)
    else:
        print(f"Connection failed with code {rc}")

# Callback when a message is received
def on_message(client, userdata, msg):
    if msg.topic == "VitalX_001/data":  # Process only messages from "data" topic
        process_data(msg)
        print("Data recording...")
    else:
        print(f"Received message: {msg.payload.decode()} on topic {msg.topic}")

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
