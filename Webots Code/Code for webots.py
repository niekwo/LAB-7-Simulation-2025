#!/usr/bin/env python3
"""
Version 3: Encoder Integration
==============================

This version adds wheel encoder sensors for odometry measurements,
enabling velocity calculation and position tracking.

New Features:
- Wheel encoder sensor initialization
- Position reading capabilities
- Velocity calculation from encoder data
- Previous value tracking for differential calculations

Author: N. Wolfs
Version: 3.0
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

# Configure motors for velocity control
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

# Initialize previous encoder values for velocity calculation
previous_encoder_values = [0.0, 0.0]

print("Encoder sensors initialized and enabled")
print(f"Timestep: {timestep}ms ({timestep_seconds}s)")

# Control loop with encoder reading
while robot.step(timestep) != -1:
    # Read current encoder positions
    current_encoder_values = [encoder.getValue() for encoder in encoders]
    
    # Calculate wheel velocities
    if previous_encoder_values[0] == 0.0 and previous_encoder_values[1] == 0.0:
        # First iteration - no velocity calculation possible
        left_velocity = 0.0
        right_velocity = 0.0
    else:
        # Calculate velocities using finite difference
        left_velocity = (current_encoder_values[0] - previous_encoder_values[0]) / timestep_seconds
        right_velocity = (current_encoder_values[1] - previous_encoder_values[1]) / timestep_seconds
    
    # Store current values for next iteration
    previous_encoder_values = current_encoder_values[:]
    
    # Debug output (uncomment for testing)
    # print(f"Encoder positions: L={current_encoder_values[0]:.2f}, R={current_encoder_values[1]:.2f}")
    # print(f"Wheel velocities: L={left_velocity:.2f}, R={right_velocity:.2f}")

print("Encoder integration execution completed.")