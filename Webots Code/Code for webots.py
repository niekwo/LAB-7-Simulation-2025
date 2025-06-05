#!/usr/bin/env python3
"""
Version 4: Ground Sensors
=========================

This version adds ground sensor array for line detection and navigation,
enabling the robot to detect dark lines on light surfaces.

New Features:
- Ground sensor array initialization
- Surface reflectance measurement
- Line detection capabilities
- Multi-sensor data collection

Author: N. Wolfs
Version: 4.0
"""

from controller import Robot

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

print("Ground sensor array initialized")
print(f"Number of ground sensors: {len(ground_sensors)}")
print("Sensors enabled for line detection")

# Control loop with ground sensor reading
while robot.step(timestep) != -1:
    # Read encoder data
    current_encoder_values = [encoder.getValue() for encoder in encoders]
    
    # Calculate wheel velocities
    if previous_encoder_values[0] == 0.0 and previous_encoder_values[1] == 0.0:
        left_velocity = 0.0
        right_velocity = 0.0
    else:
        left_velocity = (current_encoder_values[0] - previous_encoder_values[0]) / timestep_seconds
        right_velocity = (current_encoder_values[1] - previous_encoder_values[1]) / timestep_seconds
    
    # Read ground sensor data
    ground_sensor_values = [sensor.getValue() for sensor in ground_sensors]
    
    # Store current encoder values for next iteration
    previous_encoder_values = current_encoder_values[:]
    
    # Debug output (uncomment for testing)
    # print(f"Ground sensors: {ground_sensor_values[0]:.0f}, {ground_sensor_values[1]:.0f}, {ground_sensor_values[2]:.0f}")

print("Ground sensor integration execution completed.")