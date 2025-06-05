#!/usr/bin/env python3
"""
Version 7: Binary Protocol
==========================

This version implements binary data serialization for efficient
communication between the Webots client and ESP32 server.

New Features:
- Struct-based binary data packing
- Sensor data packet construction
- Binary communication protocol
- Efficient data transmission format

Author: N. Wolfs
Version: 7.0
"""

from controller import Robot
import socket
import struct

# Network configuration
ESP32_SERVER_IP = '192.168.1.21'
TCP_SERVER_PORT = 65432

# Create primary robot controller instance
robot = Robot()

# Get simulation timestep and convert to seconds
timestep = int(robot.getBasicTimeStep())
timestep_seconds = timestep / 1000.0

# Initialize differential drive motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize wheel encoder sensors
encoders = []
encoder_names = ['left wheel sensor', 'right wheel sensor']

for i in range(2):
    encoder = robot.getDevice(encoder_names[i])
    encoder.enable(timestep)
    encoders.append(encoder)

previous_encoder_values = [0.0, 0.0]

# Initialize ground sensor array
ground_sensors = []
ground_sensor_names = ['gs0', 'gs1', 'gs2']

for i in range(3):
    sensor = robot.getDevice(ground_sensor_names[i])
    sensor.enable(timestep)
    ground_sensors.append(sensor)

# Initialize TCP client connection
print(f"Connecting to ESP32 server at {ESP32_SERVER_IP}:{TCP_SERVER_PORT}...")

try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ESP32_SERVER_IP, TCP_SERVER_PORT))
    print("Connection established successfully!")
    
except socket.error as e:
    print(f"Connection failed: {e}")
    robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)
    exit()

print("Binary protocol communication initialized")
print("Data format: [gs0, gs1, gs2, wheel_vel_left, wheel_vel_right, delta_time]")
print("Starting binary protocol control loop...")

# Main control loop with binary protocol
while robot.step(timestep) != -1:
    
    # ================================================================
    # SENSOR DATA COLLECTION
    # ================================================================
    
    current_encoder_values = [encoder.getValue() for encoder in encoders]
    ground_sensor_values = [sensor.getValue() for sensor in ground_sensors]
    
    # ================================================================
    # VELOCITY CALCULATION
    # ================================================================
    
    if previous_encoder_values[0] == 0.0 and previous_encoder_values[1] == 0.0:
        left_velocity = 0.0
        right_velocity = 0.0
    else:
        left_velocity = (current_encoder_values[0] - previous_encoder_values[0]) / timestep_seconds
        right_velocity = (current_encoder_values[1] - previous_encoder_values[1]) / timestep_seconds
    
    # ================================================================
    # BINARY DATA PACKET CONSTRUCTION
    # ================================================================
    
    # Create binary data packet with sensor information
    # Format: 6 floats packed as little-endian 32-bit values
    sensor_data_packet = struct.pack('<6f', 
                                   ground_sensor_values[0], 
                                   ground_sensor_values[1], 
                                   ground_sensor_values[2],
                                   left_velocity, 
                                   right_velocity, 
                                   timestep_seconds)
    
    # ================================================================
    # BINARY PROTOCOL TESTING
    # ================================================================
    
    # For testing, we'll just verify packet construction
    # Actual transmission will be implemented in next version
    packet_size = len(sensor_data_packet)
    
    # Verify packet can be unpacked correctly
    unpacked_data = struct.unpack('<6f', sensor_data_packet)
    
    # Apply default motor commands (stationary)
    left_motor_command = 0.0
    right_motor_command = 0.0
    
    left_motor.setVelocity(left_motor_command)
    right_motor.setVelocity(right_motor_command)
    
    # ================================================================
    # PREPARE FOR NEXT ITERATION
    # ================================================================
    
    previous_encoder_values = current_encoder_values[:]
    
    # Debug output for binary protocol verification
    # print(f"Packet size: {packet_size} bytes | Data: {unpacked_data}")

# Cleanup
print("Binary protocol execution completed.")
client_socket.close()