#!/usr/bin/env python3
"""
Distributed Robot Control Framework with Hardware-in-Loop Integration
===================================================================

This system implements a client-server architecture for robotic control where
the simulation environment acts as a sensor/actuator interface while delegating
decision-making to an external embedded processor. The design enables real-world
hardware testing using simulated sensor inputs.

System Architecture:
- Simulation Client: Collects sensor data, executes motor commands
- Embedded Server: Processes navigation logic, sends control decisions
- Communication Protocol: Binary TCP socket data exchange

Target Hardware: E-puck differential drive robot platform
External Processor: ESP32 microcontroller with MicroPython runtime
Simulation Environment: Webots robotics development platform

Technical Specifications:
- Webots Version: R2023a
- Host Platform: Windows 11 Professional x64
- Python Runtime: 3.10.5 64-bit
- Remote System: ESP32 + MicroPython v1.25.0
- Network Protocol: TCP/IP over local network

Development Credits:
Author: N. Wolfs
Initial Version: 20 May 2025
Latest Revision: 6 June 2025
Production Version: 10.0
"""

# =====================================================================
# REQUIRED LIBRARY IMPORTS
# =====================================================================

from controller import Robot  # Webots simulation framework interface
import numpy as np           # Mathematical operations and array handling
import struct               # Binary data serialization utilities
import socket              # TCP/IP network communication primitives

# =====================================================================
# WEBOTS SIMULATION INTERFACE SETUP
# =====================================================================

# Create primary robot controller instance for simulation interaction
# This establishes the connection between Python code and Webots environment
webots_robot_instance = Robot()

# Extract simulation timing parameters
# The timestep defines how frequently the control loop executes
simulation_timestep = int(webots_robot_instance.getBasicTimeStep())

# Convert timestep to seconds for mathematical calculations
# This conversion is essential for velocity and acceleration computations
timestep_in_seconds = simulation_timestep / 1000.0

# =====================================================================
# DIFFERENTIAL DRIVE MOTOR INITIALIZATION
# =====================================================================

# Obtain references to the left and right drive motors
# E-puck uses differential steering with two independent wheels
motor_left_wheel = webots_robot_instance.getDevice('left wheel motor')
motor_right_wheel = webots_robot_instance.getDevice('right wheel motor')

# Configure motors for continuous velocity control
# Setting position to infinity switches from position to velocity control mode
motor_left_wheel.setPosition(float('inf'))
motor_right_wheel.setPosition(float('inf'))

# Initialize motors in stopped state for safety
# Robot remains stationary until external controller provides commands
motor_left_wheel.setVelocity(0.0)
motor_right_wheel.setVelocity(0.0)

# =====================================================================
# WHEEL ENCODER SENSOR CONFIGURATION
# =====================================================================

# Initialize encoder sensor array for odometry measurements
wheel_encoder_devices = []
encoder_device_names = ['left wheel sensor', 'right wheel sensor']

# Configure each encoder sensor for continuous position monitoring
for encoder_index in range(2):
    current_encoder = webots_robot_instance.getDevice(encoder_device_names[encoder_index])
    current_encoder.enable(simulation_timestep)
    wheel_encoder_devices.append(current_encoder)

# Initialize previous encoder values to prevent calculation errors
# First iteration needs baseline values for velocity computation
encoder_values_previous = [0.0, 0.0]

# =====================================================================
# GROUND SENSOR ARRAY INITIALIZATION
# =====================================================================

# Create ground sensor collection for line detection and navigation
ground_sensor_array = []
ground_sensor_identifiers = ['gs0', 'gs1', 'gs2']

# Enable each ground sensor for surface reflectance measurement
# These sensors detect dark lines on light surfaces for path following
for sensor_index in range(3):
    current_ground_sensor = webots_robot_instance.getDevice(ground_sensor_identifiers[sensor_index])
    current_ground_sensor.enable(simulation_timestep)
    ground_sensor_array.append(current_ground_sensor)

# =====================================================================
# TCP CLIENT SOCKET COMMUNICATION SETUP
# =====================================================================

# Network configuration for ESP32 server connection
ESP32_SERVER_IP = '192.168.1.21'    # IP address of ESP32 device
TCP_SERVER_PORT = 65432              # Communication port number

print(f"Establishing connection to ESP32 server at {ESP32_SERVER_IP}:{TCP_SERVER_PORT}...")

try:
    # Create TCP socket and establish connection to ESP32 server
    tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_client_socket.connect((ESP32_SERVER_IP, TCP_SERVER_PORT))
    print("Connection established successfully with ESP32 server!")
    
except socket.error as socket_exception:
    print(f"Connection failed with error: {socket_exception}")
    print("Troubleshooting checklist:")
    print("- Verify ESP32 server application is running")
    print("- Check network connectivity and IP address")
    print("- Ensure firewall permits the connection")
    
    # Pause simulation when communication fails
    webots_robot_instance.simulationSetMode(webots_robot_instance.SIMULATION_MODE_PAUSE)
    exit()

# =====================================================================
# ROBOT POSE TRACKING VARIABLES
# =====================================================================

# Initialize robot position coordinates (maintained for reference)
# Note: These variables are not used for control, ESP32 handles navigation
robot_position_x = 0.0
robot_position_y = 0.0
robot_heading_phi = 0

# =====================================================================
# CONFIGURATION PARAMETERS
# =====================================================================

# Safety parameters
MAX_MOTOR_VELOCITY = 10.0  # Maximum safe motor velocity (rad/s)
DEBUG_OUTPUT_ENABLED = False  # Set to True for development debugging

print("Production control system initialized")
print(f"Timestep: {simulation_timestep}ms ({timestep_in_seconds:.3f}s)")
print(f"Safety limit: ±{MAX_MOTOR_VELOCITY} rad/s")
print("Starting production control loop...")

# =====================================================================
# PRIMARY CONTROL LOOP EXECUTION
# =====================================================================

# Main control loop - executes until simulation termination signal
while webots_robot_instance.step(simulation_timestep) != -1:
    
    # =================================================================
    # SENSOR DATA COLLECTION PHASE
    # =================================================================
    
    # Read current encoder positions for wheel odometry
    current_encoder_readings = [encoder.getValue() for encoder in wheel_encoder_devices]
    
    # Sample ground sensors for line detection data
    current_ground_sensor_values = [sensor.getValue() for sensor in ground_sensor_array]

    # =================================================================
    # WHEEL VELOCITY CALCULATION
    # =================================================================
    
    # Compute wheel angular velocities from encoder position changes
    # Special handling for first iteration when no previous data exists
    if encoder_values_previous[0] == 0.0 and encoder_values_previous[1] == 0.0:
        # First loop iteration - no velocity calculation possible
        left_wheel_velocity = 0.0
        right_wheel_velocity = 0.0
    else:
        # Calculate velocities using finite difference method
        left_wheel_velocity = (current_encoder_readings[0] - encoder_values_previous[0]) / timestep_in_seconds
        right_wheel_velocity = (current_encoder_readings[1] - encoder_values_previous[1]) / timestep_in_seconds

    # =================================================================
    # SENSOR DATA PACKET CONSTRUCTION
    # =================================================================
    
    # Create binary data packet containing all sensor information
    # Format: [gs0, gs1, gs2, wheel_vel_left, wheel_vel_right, delta_time]
    # Each value packed as 32-bit little-endian float
    sensor_data_packet = struct.pack('<6f', 
                                   current_ground_sensor_values[0], 
                                   current_ground_sensor_values[1], 
                                   current_ground_sensor_values[2],
                                   left_wheel_velocity, 
                                   right_wheel_velocity, 
                                   timestep_in_seconds)

    try:
        # =============================================================
        # TRANSMIT SENSOR DATA TO ESP32
        # =============================================================
        
        # Send complete sensor package to ESP32 server
        tcp_client_socket.sendall(sensor_data_packet)
        
        # =============================================================
        # RECEIVE MOTOR COMMANDS FROM ESP32
        # =============================================================
        
        # Wait for motor command response from ESP32
        # Expected: 2 float values (left_motor_speed, right_motor_speed)
        motor_command_response = tcp_client_socket.recv(8)  # 2 floats × 4 bytes = 8 bytes
        
        # Check for connection termination or empty response
        if not motor_command_response:
            print("ESP32 server disconnected or sent empty response.")
            break
            
        # Verify response length for data integrity
        if len(motor_command_response) != 8:
            print(f"Invalid response length: expected 8 bytes, received {len(motor_command_response)}")
            continue
            
        # Unpack binary motor commands
        left_motor_command, right_motor_command = struct.unpack('<2f', motor_command_response)
        
        # Apply safety limits to motor commands
        if abs(left_motor_command) > MAX_MOTOR_VELOCITY or abs(right_motor_command) > MAX_MOTOR_VELOCITY:
            print(f"Warning: Motor commands exceed safety limits. L={left_motor_command:.2f}, R={right_motor_command:.2f}")
            left_motor_command = max(-MAX_MOTOR_VELOCITY, min(MAX_MOTOR_VELOCITY, left_motor_command))
            right_motor_command = max(-MAX_MOTOR_VELOCITY, min(MAX_MOTOR_VELOCITY, right_motor_command))

        # =============================================================
        # EXECUTE MOTOR COMMANDS
        # =============================================================
        
        # Apply received velocity commands to physical motors
        motor_left_wheel.setVelocity(left_motor_command)
        motor_right_wheel.setVelocity(right_motor_command)

    except socket.error as communication_error:
        print(f"Socket communication error occurred: {communication_error}")
        print("Initiating emergency stop and simulation pause.")
        
        # Emergency stop - set all motors to zero velocity
        motor_left_wheel.setVelocity(0.0)
        motor_right_wheel.setVelocity(0.0)
        
        # Pause simulation for safety
        webots_robot_instance.simulationSetMode(webots_robot_instance.SIMULATION_MODE_PAUSE)
        break
        
    except struct.error as unpacking_error:
        print(f"Data unpacking error: {unpacking_error}")
        print("Skipping this iteration and continuing...")
        continue
        
    except Exception as unexpected_error:
        print(f"Unexpected error occurred: {unexpected_error}")
        print("Implementing emergency stop...")
        
        # Emergency stop for any unexpected error
        motor_left_wheel.setVelocity(0.0)
        motor_right_wheel.setVelocity(0.0)
        break

    # =================================================================
    # PREPARE FOR NEXT ITERATION
    # =================================================================
    
    # Store current encoder values for next velocity calculation
    encoder_values_previous = current_encoder_readings[:]

    # Optional debug output for development
    if DEBUG_OUTPUT_ENABLED:
        print(f"Webots: GS: {current_ground_sensor_values[0]:.0f},{current_ground_sensor_values[1]:.0f},{current_ground_sensor_values[2]:.0f} | "
              f"Sent: Wl:{left_wheel_velocity:.2f}, Wr:{right_wheel_velocity:.2f} | "
              f"Rcvd: L:{left_motor_command:.2f}, R:{right_motor_command:.2f}")

# =====================================================================
# CLEANUP AND SHUTDOWN
# =====================================================================

print("Webots controller execution completed.")

# Close TCP socket connection gracefully
try:
    tcp_client_socket.close()
    print("Network connection closed successfully.")
except Exception as cleanup_error:
    print(f"Note: Network cleanup encountered: {cleanup_error}")