
"""
Version 8: Remote Control Integration
====================================

This version implements complete client-server architecture with
bidirectional communication between Webots and ESP32.

New Features:
- Send sensor data to ESP32 server
- Receive motor commands from ESP32
- Bidirectional binary communication
- Remote control execution

Author: N. Wolfs
Version: 8.0
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

print("Remote control system initialized")
print("Starting remote control loop...")

# Main control loop with remote control
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
    # SENSOR DATA PACKET CONSTRUCTION
    # ================================================================
    
    sensor_data_packet = struct.pack('<6f', 
                                   ground_sensor_values[0], 
                                   ground_sensor_values[1], 
                                   ground_sensor_values[2],
                                   left_velocity, 
                                   right_velocity, 
                                   timestep_seconds)
    
    try:
        # ============================================================
        # SEND SENSOR DATA TO ESP32
        # ============================================================
        
        client_socket.sendall(sensor_data_packet)
        
        # ============================================================
        # RECEIVE MOTOR COMMANDS FROM ESP32
        # ============================================================
        
        # Wait for motor command response (2 floats = 8 bytes)
        motor_command_response = client_socket.recv(8)
        
        if not motor_command_response:
            print("ESP32 server disconnected.")
            break
            
        # Unpack motor commands
        left_motor_command, right_motor_command = struct.unpack('<2f', motor_command_response)
        
        # ============================================================
        # EXECUTE MOTOR COMMANDS
        # ============================================================
        
        left_motor.setVelocity(left_motor_command)
        right_motor.setVelocity(right_motor_command)
        
    except socket.error as e:
        print(f"Communication error: {e}")
        # Emergency stop on communication failure
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break
    
    # ================================================================
    # PREPARE FOR NEXT ITERATION
    # ================================================================
    
    previous_encoder_values = current_encoder_values[:]
    
    # Debug output (uncomment for testing)
    # print(f"Sent: GS=[{ground_sensor_values[0]:.0f},{ground_sensor_values[1]:.0f},{ground_sensor_values[2]:.0f}] "
    #       f"Vel=[{left_velocity:.2f},{right_velocity:.2f}] | "
    #       f"Received: Motors=[{left_motor_command:.2f},{right_motor_command:.2f}]")

# Cleanup
print("Remote control execution completed.")
client_socket.close()