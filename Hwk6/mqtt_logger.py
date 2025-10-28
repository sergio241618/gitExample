#!/usr/bin/env python3

"""
MQTT v5 Logger Script (hwk6.pdf)

This script:
1. Connects to the local Linux machine via SSH to verify the
   Mosquitto service is running using Paramiko.
2. Subscribes to all 4 topics on the LOCAL broker using Paho-MQTT v5.
3. Logs all received messages into a pandas DataFrame.
4. Saves the data to 'mqtt_capture.csv' on exit.
"""

import paho.mqtt.client as mqtt
import paho.mqtt.properties as props
from paho.mqtt.packettypes import PacketTypes
import pandas as pd
import paramiko
import time
import sys
import os
import signal

# --- Configuration Section ---

# MQTT Broker settings (local)
BROKER_HOST = "127.0.0.1"  # Running on the same machine
BROKER_PORT = 1883

# Subscribes to all 4 topics from the PDF
TOPICS = [
    "u77/sensors/temperature",  
    "u77/sensors/gps",
    "u77/data/status",
    "u77/data/uptime"
]

# SSH settings to check Mosquitto service
SSH_HOST = "127.0.0.1"      # Local machine
SSH_PORT = 22
SSH_USER = "sergio2416" 
SSH_PASS = "Sergio16" 

# CSV output file
CSV_FILE = "mqtt_capture.csv"

# --- Global Variables ---
g_data_rows = []
g_columns = ['timestamp', 'topic', 'message', 'qos']
g_client = None
g_exit_flag = False

# --- SSH Service Check ---
def check_mosquitto_service():
    """Uses Paramiko to SSH and check 'systemctl is-active mosquitto'."""
    print(f"[SSH] Connecting to {SSH_HOST} as {SSH_USER}...")
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(SSH_HOST, port=SSH_PORT, username=SSH_USER, password=SSH_PASS, timeout=5)
        
        print("[SSH] Connected. Checking Mosquitto service...")
        stdin, stdout, stderr = ssh.exec_command("systemctl is-active mosquitto")
        
        service_state = stdout.read().decode('utf-8').strip()
        print(f"[SSH] Service state: {service_state}")
        
        if service_state != "active":
            print(f"[SSH] Error: Mosquitto service is not 'active' (state: {service_state}).")
            return False
            
        print("[SSH] Check successful.")
        return True
        
    except Exception as e:
        print(f"[SSH] Error checking service: {e}")
        return False
    finally:
        if 'ssh' in locals():
            ssh.close()

# --- MQTT Handlers ---
def on_connect(client, userdata, flags, rc, v5data=None):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        print(f"[MQTT] Connected to broker at {BROKER_HOST}")
        # Subscribe to all topics in the list
        for topic in TOPICS:
            print(f"[MQTT] Subscribing to: {topic}")
            client.subscribe(topic, qos=1)
    else:
        print(f"[MQTT] Connection failed with code: {rc}")

def on_message(client, userdata, msg, v5data=None):
    """Callback for when a message is received."""
    try:
        payload = msg.payload.decode('utf-8')
        print(f"[MQTT] Received: Topic='{msg.topic}', Message='{payload}'")
        
        # Append data for the CSV
        g_data_rows.append({
            'timestamp': pd.Timestamp.now(),
            'topic': msg.topic,
            'message': payload,
            'qos': msg.qos
        })
    except Exception as e:
        print(f"[MQTT] Error processing message: {e}")

def on_subscribe(client, userdata, mid, granted_qos, v5data=None):
    print(f"[MQTT] Subscribed with MID {mid}")

# --- Data Saving ---
def save_data_to_csv():
    """Saves the collected data to a CSV file."""
    if not g_data_rows:
        print("[CSV] No data collected, nothing to save.")
        return

    print(f"\n[CSV] Saving {len(g_data_rows)} new messages to {CSV_FILE}...")
    new_df = pd.DataFrame(g_data_rows, columns=g_columns)
    
    try:
        if os.path.exists(CSV_FILE):
            # Append to existing file (without header)
            new_df.to_csv(CSV_FILE, mode='a', header=False, index=False)
            print(f"[CSV] Appended data to {CSV_FILE}")
        else:
            # Create new file (with header)
            new_df.to_csv(CSV_FILE, mode='w', header=True, index=False)
            print(f"[CSV] Created new file {CSV_FILE}")
            
    except Exception as e:
        print(f"[CSV] Error saving data: {e}")

# --- Graceful Exit ---
def signal_handler(sig, frame):
    """Handles Ctrl+C to exit gracefully."""
    global g_exit_flag
    if not g_exit_flag:
        g_exit_flag = True
        print("\n[INFO] Ctrl+C pressed. Shutting down...")
        if g_client:
            g_client.disconnect()
            g_client.loop_stop()
        save_data_to_csv()
        sys.exit(0)

# --- Main ---
def main():
    global g_client
    
    # Register Ctrl+C handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # 1. Check if Mosquitto is running
    if not check_mosquitto_service():
        print("[MAIN] Aborting due to SSH service check failure.")
        sys.exit(1)

    # 2. Set up MQTT client
    g_client = mqtt.Client(client_id="", protocol=mqtt.MQTTv5)
    
    # Assign callbacks
    g_client.on_connect = on_connect
    g_client.on_message = on_message
    g_client.on_subscribe = on_subscribe

    print(f"[MAIN] Connecting to MQTT broker at {BROKER_HOST}...")
    
    try:
        g_client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    except Exception as e:
        print(f"[MAIN] Error connecting to broker: {e}")
        sys.exit(1)

    # 3. Start the network loop
    g_client.loop_start()

    # Keep the script running
    print("[MAIN] Logger is running. Press Ctrl+C to stop.")
    while not g_exit_flag:
        time.sleep(1)

if __name__ == "__main__":
    main()