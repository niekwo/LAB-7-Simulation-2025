#!/usr/bin/env python3
"""
Version 2: Motor Control System
===============================

This version adds differential drive motor control functionality,
enabling basic movement capabilities for the E-puck robot.

New Features:
- Left and right wheel motor initialization
- Velocity-based motor control
- Motor safety (initial stop state)

Author: N. Wolfs
Version: 2.0
"""

from controller import Robot

# Create primary robot controller instance
robot = Robot()

# Get simulation timestep
timestep = int(robot.getBasicTimeStep())

# Initialize differential drive motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Configure motors for velocity control
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Initialize motors in stopped state for safety
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

print("Motor control system initialized")
print("Left and right wheel motors configured for velocity control")

# Basic control loop with motor control
while robot.step(timestep) != -1:
    # Motors remain stopped until external control is added
    # This ensures safe operation
    pass

print("Motor control system execution completed.")