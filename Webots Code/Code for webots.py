
"""
Version 9: Error Handling & Robustness
======================================

This version adds comprehensive error handling and robustness features
for reliable operation in real-world conditions.

New Features:
- Comprehensive error handling
- Connection failure recovery
- Emergency stop functionality
- Graceful shutdown procedures
- Detailed error reporting

Author: N. Wolfs
Version: 9.0
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

# Initialize robot state tracking (for reference)
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

# Initialize TCP client connection with robust error handling
print(f"Establishing connection to ESP32 server at {ESP32_SERVER_IP}:{TCP_SERVER_PORT}...")

try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ESP32_SERVER_IP, TCP_SERVER_PORT))
    print("Connection established successfully with ESP32 server!")
    
except socket.error as socket_exception:
    print(f"Connection failed with error: {socket_exception}")
    print("Troubleshooting checklist:")
    print("- Verify ESP32 server application is running")
    print("- Check network connectivity and IP address")
    print("- Ensure firewall permits the connection")
    
    # Pause simulation when communication fails
    robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)
    exit()

print("Robust control system initialized")
print("Starting error-handled control loop...")

# Main control loop with comprehensive error handling
while robot.step(timestep) != -1:
    
    # ================================================================
    # SENSOR DATA COLLECTION PHASE
    # ================================================================
    
    current_encoder_values = [encoder.getValue() for encoder in encoders]
    ground_sensor_values = [sensor.getValue() for sensor in ground_sensors]
    
    # ================================================================
    # WHEEL VELOCITY CALCULATION
    # ================================================================
    
    if previous_encoder_values[0] == 0.0 and previous_encoder_values[1] == 0.0:
        # First loop iteration - no velocity calculation possible
        left_velocity = 0.0
        right_velocity = 0.0
    else:
        # Calculate velocities using finite difference method
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
        # TRANSMIT SENSOR DATA TO ESP32
        # ============================================================
        
        client_socket.sendall(sensor_data_packet)
        
        # ============================================================
        # RECEIVE MOTOR COMMANDS FROM ESP32
        # ============================================================
        
        motor_command_response = client_socket.recv(8)  # 2 floats × 4 bytes = 8 bytes
        
        # Check for connection termination or empty response
        if not motor_command_response:
            print("ESP32 server disconnected or sent empty response.")
            break
            
        # Verify response length
        if len(motor_command_response) != 8:
            print(f"Invalid response length: expected 8 bytes, received {len(motor_command_response)}")
            continue
            
        # Unpack binary motor commands
        left_motor_command, right_motor_command = struct.unpack('<2f', motor_command_response)
        
        # Validate motor command values (safety check)
        MAX_VELOCITY = 10.0  # Maximum safe velocity
        if abs(left_motor_command) > MAX_VELOCITY or abs(right_motor_command) > MAX_VELOCITY:
            print(f"Warning: Motor commands exceed safety limits. L={left_motor_command:.2f}, R={right_motor_command:.2f}")
            left_motor_command = max(-MAX_VELOCITY, min(MAX_VELOCITY, left_motor_command))
            right_motor_command = max(-MAX_VELOCITY, min(MAX_VELOCITY, right_motor_command))
        
        # ============================================================
        # EXECUTE MOTOR COMMANDS
        # ============================================================
        
        left_motor.setVelocity(left_motor_command)
        right_motor.setVelocity(right_motor_command)
        
    except socket.error as communication_error:
        print(f"Socket communication error occurred: {communication_error}")
        print("Initiating emergency stop and simulation pause.")
        
        # Emergency stop - set all motors to zero velocity
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        # Pause simulation for safety
        robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)
        break
        
    except struct.error as unpacking_error:
        print(f"Data unpacking error: {unpacking_error}")
        print("Skipping this iteration and continuing...")
        continue
        
    except Exception as unexpected_error:
        print(f"Unexpected error occurred: {unexpected_error}")
        print("Implementing emergency stop...")
        
        # Emergency stop for any unexpected error
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break
    
    # ================================================================
    # PREPARE FOR NEXT ITERATION
    # ================================================================
    
    previous_encoder_values = current_encoder_values[:]
    
    # Optional debug output (commented out for performance)
    # print(f"GS: {ground_sensor_values[0]:.0f},{ground_sensor_values[1]:.0f},{ground_sensor_values[2]:.0f} | "
    #       f"Sent: Wl:{left_velocity:.2f}, Wr:{right_velocity:.2f} | "
    #       f"Rcvd: L:{left_motor_command:.2f}, R:{right_motor_command:.2f}")

# =====================================================================
# CLEANUP AND SHUTDOWN
# =====================================================================

print("Robust controller execution completed.")

# Close TCP socket connection gracefully
try:
    client_socket.close()
    print("Network connection closed successfully.")
except:
    print("Note: Network connection was already closed.")