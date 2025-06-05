
"""
Version 5: Local Control Loop
=============================

This version implements a complete local robot control system with
sensor data collection and basic processing capabilities.

New Features:
- Complete sensor data collection
- Structured control loop
- Data processing framework
- Local autonomous operation capability

Author: N. Wolfs
Version: 5.0
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

# Initialize robot state tracking
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

print("Local control system initialized")
print("All sensors enabled and ready")
print("Starting main control loop...")

# Main control loop
while robot.step(timestep) != -1:
    
    # ================================================================
    # SENSOR DATA COLLECTION
    # ================================================================
    
    # Read encoder positions
    current_encoder_values = [encoder.getValue() for encoder in encoders]
    
    # Read ground sensor values
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
    # LOCAL CONTROL LOGIC (PLACEHOLDER)
    # ================================================================
    
    # This is where local control algorithms would be implemented
    # For now, robot remains stationary
    left_motor_command = 0.0
    right_motor_command = 0.0
    
    # Apply motor commands
    left_motor.setVelocity(left_motor_command)
    right_motor.setVelocity(right_motor_command)
    
    # ================================================================
    # PREPARE FOR NEXT ITERATION
    # ================================================================
    
    previous_encoder_values = current_encoder_values[:]
    
    # Optional debug output
    # print(f"GS: [{ground_sensor_values[0]:.0f},{ground_sensor_values[1]:.0f},{ground_sensor_values[2]:.0f}] | "
    #       f"Vel: L={left_velocity:.2f}, R={right_velocity:.2f}")

print("Local control execution completed.")