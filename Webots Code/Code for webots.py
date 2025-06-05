#!/usr/bin/env python3
"""
Version 1: Basic Robot Setup
============================

This version establishes the foundation by initializing the Webots robot
controller and setting up basic timing parameters.

Features:
- Robot instance creation
- Timestep configuration
- Basic simulation interface

Author: N. Wolfs
Version: 1.0
"""

from controller import Robot

# Create primary robot controller instance
robot = Robot()

# Get simulation timestep
timestep = int(robot.getBasicTimeStep())

print(f"Robot controller initialized with timestep: {timestep}ms")

# Basic control loop
while robot.step(timestep) != -1:
    # Placeholder for future functionality
    pass

print("Basic robot controller execution completed.")