"""
ESP32 Robot Navigation and Control System
========================================
This module provides comprehensive functionality for autonomous robot navigation including:
- Line following sensor reading
- LED status indication
- PID-based motion control
- A* pathfinding algorithm
- Angle normalization utilities

Author: Niek & Remco
Version: 1.0
"""

from config import led_yellow, led_blue, led_green, led_red
import math # type: ignore
import heapq

# ============================================================================
# SENSOR READING SECTION
# ============================================================================
# Functions for interpreting sensor data from the robot's line-following sensors

def read_Sensor_Status_OuterLine(msg_bytes) -> tuple[bool, bool, bool]:
    """
    Reads the sensor status from the received message bytes.
    
    This function decodes sensor data from a byte message to determine which
    line-following sensors are currently detecting a line. The message format
    expects the last 4 characters to contain sensor status information.
    
    Parameters:
    -----------
    msg_bytes : bytes
        Raw message bytes containing sensor status information
        Expected format: "...xyz1" where x,y,z are sensor states
    
    Returns:
    --------
    tuple[bool, bool, bool]
        A tuple containing (line_left, line_center, line_right)
        True means line detected, False means no line detected
    
    Message Format:
    ---------------
    The message string is parsed as follows:
    - Position [-4:-3]: Left sensor status ('1' = line detected)
    - Position [-3:-2]: Center sensor status ('1' = line detected)  
    - Position [-2:-1]: Right sensor status ('1' = line detected)
    """
    # Convert received bytes to UTF-8 string for parsing
    msg_str = str(msg_bytes, 'UTF-8')
    
    # Extract individual sensor states from the message string
    # Each sensor returns '1' when a line is detected, '0' when no line
    line_left = msg_str[-4:-3] == '1'    # Left sensor status
    line_center = msg_str[-3:-2] == '1'  # Center sensor status  
    line_right = msg_str[-2:-1] == '1'   # Right sensor status
    
    return (line_left, line_center, line_right)


# ============================================================================
# LED CONTROL SECTION  
# ============================================================================
# Functions for controlling status indication LEDs on the robot

def led_Control(yellow, blue, green, red) -> None:
    """
    Controls the state of all four status LEDs on the robot.
    
    This function provides centralized control for the robot's status LEDs,
    typically used to indicate current operating mode or system status.
    
    Parameters:
    -----------
    yellow : bool
        State for yellow LED (typically indicates forward movement)
    blue : bool  
        State for blue LED (typically indicates turning left)
    green : bool
        State for green LED (typically indicates turning right)
    red : bool
        State for red LED (typically indicates stopping/error)
    
    LED Usage Convention:
    -------------------
    - Yellow: Forward movement or normal operation
    - Blue: Left turn or left sensor active
    - Green: Right turn or right sensor active  
    - Red: Stop condition, error state, or reverse
    """
    led_yellow.value(yellow)  # Control yellow LED state
    led_blue.value(blue)      # Control blue LED state
    led_green.value(green)    # Control green LED state
    led_red.value(red)        # Control red LED state


# ============================================================================
# MATHEMATICAL UTILITIES SECTION
# ============================================================================
# Helper functions for angle calculations and value constraints

def normalize_angle(angle_rad):
    """
    Normalize an angle to the standard range [-π, π].
    
    This function ensures that angle values are always within the standard
    mathematical range, preventing issues with angle wrapping in navigation
    calculations. This is crucial for PID controllers and heading calculations.
    
    Parameters:
    -----------
    angle_rad : float
        Input angle in radians (can be any value)
    
    Returns:
    --------
    float
        Normalized angle in the range [-π, π] radians
    
    Algorithm:
    ----------
    - Subtracts 2π for angles > π until they're within range
    - Adds 2π for angles < -π until they're within range
    """
    # Reduce angles greater than π by subtracting 2π
    while angle_rad > math.pi: 
        angle_rad -= 2 * math.pi
    
    # Increase angles less than -π by adding 2π    
    while angle_rad < -math.pi: 
        angle_rad += 2 * math.pi
        
    return angle_rad


def clip_value(value, min_val, max_val):
    """
    Clamp a value between specified minimum and maximum bounds.
    
    This utility function ensures that values stay within safe operating
    ranges, preventing motor speeds or control outputs from exceeding
    hardware limitations.
    
    Parameters:
    -----------
    value : float
        The input value to be constrained
    min_val : float  
        Minimum allowed value (lower bound)
    max_val : float
        Maximum allowed value (upper bound)
    
    Returns:
    --------
    float
        Constrained value within [min_val, max_val] range
    """
    return max(min_val, min(value, max_val))


# ============================================================================
# NAVIGATION CONSTANTS SECTION
# ============================================================================
# Configuration parameters for robot navigation and control systems

# Waypoint Navigation Constants
WAYPOINT_REACHED_THRESHOLD_ESP = 0.005          # Distance threshold to consider waypoint reached (meters)
INTERSECTION_APPROACH_OFFSET_ESP = 0.005        # Offset distance when approaching intersections (meters)

# Line Following Constants  
LINE_FOLLOW_SPEED_FACTOR_ESP = 0.8              # Speed reduction factor during line following (0.0-1.0)
LINE_FOLLOW_COUNTER_MAX_ESP = 3                 # Maximum iterations for line following attempts

# Turning and Orientation Constants
TURN_COMPLETION_THRESHOLD_ESP = math.radians(10)           # Angle threshold to consider turn complete
ORIENTATION_CORRECTION_THRESHOLD_ERROR_ESP = math.radians(20)  # Max angle error before correction needed
TURN_EARLY_EXIT_THRESHOLD_ESP = math.radians(15)          # Early exit threshold for turns (not used)

# PID Controller Constants for Turning
KP_TURN_ESP = 2.0                               # Proportional gain for turn control
KI_TURN_ESP = 0.0                               # Integral gain for turn control  
KD_TURN_ESP = 0.0                               # Derivative gain for turn control

# Speed Limits
MAX_SPEED_ESP = 6.28                            # Maximum robot speed (rad/s or m/s)


# ============================================================================
# PATHFINDING SECTION - A* ALGORITHM
# ============================================================================
# Implementation of A* pathfinding algorithm for autonomous navigation

def heuristic_esp(node_key, goal_key, coords):
    """
    Heuristic function for A* pathfinding using Euclidean distance.
    
    This function calculates the straight-line distance between two nodes,
    providing an optimistic estimate of the actual path cost. The Euclidean
    distance is admissible (never overestimates) making it suitable for A*.
    
    Parameters:
    -----------
    node_key : str/int
        Identifier for the starting node
    goal_key : str/int  
        Identifier for the destination node
    coords : dict
        Dictionary mapping node keys to (x,y) coordinate tuples
    
    Returns:
    --------
    float
        Euclidean distance between the two nodes
    
    Formula:
    --------
    distance = √[(x₂-x₁)² + (y₂-y₁)²]
    """
    # Get coordinate tuples for both nodes
    n1 = coords[node_key]  # Starting node coordinates (x1, y1)
    n2 = coords[goal_key]  # Goal node coordinates (x2, y2)
    
    # Calculate and return Euclidean distance
    return math.sqrt((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2)


def a_star_search_esp(start_key, goal_key, coords, connections):
    """
    A* pathfinding algorithm for optimal route planning.
    
    This implementation of the A* algorithm finds the shortest path between
    two nodes in a graph. It uses a combination of actual distance traveled
    (g-score) and heuristic estimate to goal (h-score) to efficiently
    explore the most promising paths first.
    
    Parameters:
    -----------
    start_key : str/int
        Identifier for the starting node
    goal_key : str/int
        Identifier for the destination node  
    coords : dict
        Dictionary mapping node keys to (x,y) coordinate tuples
    connections : dict
        Dictionary mapping each node to list of connected neighbor nodes
    
    Returns:
    --------
    list or None
        List of node keys representing the optimal path from start to goal
        Returns None if no path exists or invalid start/goal nodes
    
    Algorithm Overview:
    -------------------
    1. Initialize open set with start node
    2. For each node, maintain g-score (actual cost) and f-score (total estimated cost)
    3. Always expand the node with lowest f-score
    4. Update neighbor costs if better path found
    5. Reconstruct path when goal reached
    """
    
    # Validate that start and goal nodes exist in coordinates
    if start_key not in coords or goal_key not in coords:
        print(f"A* Error: Start ({start_key}) or Goal ({goal_key}) not in coords.")
        return None
    
    # Initialize A* data structures
    open_set = []  # Priority queue of nodes to explore (f-score, node_key)
    heapq.heappush(open_set, (0, start_key))
    
    came_from = {}  # Track parent nodes for path reconstruction
    
    # g_score: actual cost from start to each node
    g_score = {node: float('inf') for node in coords}
    g_score[start_key] = 0
    
    # f_score: estimated total cost (g_score + heuristic)
    f_score = {node: float('inf') for node in coords}
    f_score[start_key] = heuristic_esp(start_key, goal_key, coords)
    
    # Hash set for efficient open set membership checking
    open_set_hash = {start_key}
    
    # Main A* loop - continue until open set is empty
    while open_set:
        # Get node with lowest f-score
        _, current_key = heapq.heappop(open_set)
        
        # Skip if node already processed (duplicate in heap)
        if current_key not in open_set_hash: 
            continue
        open_set_hash.remove(current_key)
        
        # Check if goal reached
        if current_key == goal_key:
            # Reconstruct path by following parent pointers
            path = []
            temp_key = current_key
            while temp_key in came_from:
                path.append(temp_key)
                temp_key = came_from[temp_key]
            path.append(start_key)
            return path[::-1]  # Reverse to get start->goal order
        
        # Skip nodes with no connections
        if current_key not in connections: 
            continue
        
        # Examine each neighbor of current node
        for neighbor_key in connections[current_key]:
            # Validate neighbor exists in coordinate system
            if neighbor_key not in coords:
                print(f"A* Warning: Neighbor {neighbor_key} of {current_key} not in coords.")
                continue
            
            # Calculate cost to reach this neighbor via current node
            cost = heuristic_esp(current_key, neighbor_key, coords)
            tentative_g_score = g_score[current_key] + cost
            
            # If this path to neighbor is better than previous paths
            if tentative_g_score < g_score.get(neighbor_key, float('inf')):
                # Update path tracking and costs
                came_from[neighbor_key] = current_key
                g_score[neighbor_key] = tentative_g_score
                new_f_score = tentative_g_score + heuristic_esp(neighbor_key, goal_key, coords)
                f_score[neighbor_key] = new_f_score
                
                # Add neighbor to open set for future exploration
                heapq.heappush(open_set, (new_f_score, neighbor_key))
                open_set_hash.add(neighbor_key)
    
    # No path found after exploring all reachable nodes
    print(f"A* Path from {start_key} to {goal_key} not found.")
    return None


# ============================================================================
# PID CONTROLLER SECTION
# ============================================================================
# Advanced PID controller implementation for precise robot motion control

class PIDController_ESP:
    """
    Advanced PID Controller for robot heading and speed control.
    
    This controller implements a Proportional-Integral-Derivative (PID) control
    algorithm specifically designed for robot navigation. It includes features
    like integral windup protection, output limiting, and angle normalization
    for heading control applications.
    
    PID Theory:
    -----------
    - Proportional (P): Responds to current error magnitude
    - Integral (I): Eliminates steady-state error by accumulating past errors  
    - Derivative (D): Predicts future error trends to reduce overshoot
    
    Output = Kp*error + Ki*∫error*dt + Kd*d(error)/dt
    
    Features:
    ---------
    - Automatic angle normalization for heading control
    - Integral windup prevention with configurable limits
    - Output saturation limits for hardware protection
    - Reset functionality for mode changes
    - Configurable gains for different control scenarios
    """
    
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-MAX_SPEED_ESP, MAX_SPEED_ESP), integral_limits=(-100, 100)):
        """
        Initialize PID controller with specified parameters.
        
        Parameters:
        -----------
        Kp : float
            Proportional gain - how strongly to react to current error
        Ki : float  
            Integral gain - how strongly to react to accumulated error
        Kd : float
            Derivative gain - how strongly to react to error rate of change
        setpoint : float
            Target value the controller should achieve (normalized for angles)
        output_limits : tuple
            (min, max) limits for controller output to prevent actuator damage
        integral_limits : tuple  
            (min, max) limits for integral term to prevent windup
        """
        # Store PID gains
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain  
        self.Kd = Kd  # Derivative gain
        
        # Set and normalize target setpoint
        self.setpoint = normalize_angle(setpoint)
        
        # Initialize internal state variables
        self.prev_error = 0.0    # Previous error for derivative calculation
        self.integral = 0.0      # Accumulated error for integral term
        
        # Store constraint limits
        self.output_limits = output_limits      # Output saturation limits
        self.integral_limits = integral_limits  # Anti-windup limits

    def update(self, measured_value, dt):
        """
        Update PID controller with new measurement and calculate control output.
        
        This method performs the core PID calculation using the current
        measurement and time step. It includes error normalization for
        angle control and implements integral windup prevention.
        
        Parameters:
        -----------
        measured_value : float
            Current measured value (e.g., current heading angle)
        dt : float
            Time step since last update (seconds)
            
        Returns:
        --------
        float
            Control output value (constrained by output_limits)
            
        Algorithm Steps:
        ----------------
        1. Calculate current error (setpoint - measurement)
        2. Normalize error for angle calculations  
        3. Update integral with windup protection
        4. Calculate derivative from error change rate
        5. Combine P, I, D terms with respective gains
        6. Apply output limits and return result
        """
        # Prevent division by zero in derivative calculation
        if dt == 0: 
            return 0
        
        # Calculate current error (desired - actual)
        error = self.setpoint - measured_value
        error = normalize_angle(error)  # Normalize for angle control
        
        # Update integral term with accumulated error
        self.integral += error * dt
        # Prevent integral windup by constraining integral term
        self.integral = clip_value(self.integral, self.integral_limits[0], self.integral_limits[1])
        
        # Calculate derivative term (rate of error change)
        derivative = (error - self.prev_error) / dt
        
        # Combine PID terms to calculate control output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Store current error for next derivative calculation
        self.prev_error = error
        
        # Apply output limits to protect actuators
        if self.output_limits:
            output = clip_value(output, self.output_limits[0], self.output_limits[1])
            
        return output

    def reset(self):
        """
        Reset PID controller's internal state.
        
        This method clears the integral accumulator and previous error,
        effectively giving the controller a fresh start. Use this when
        changing operating modes or after significant disturbances.
        
        When to use:
        ------------
        - Switching between different control modes
        - After manual robot repositioning  
        - When setpoint changes significantly
        - To clear accumulated integral windup
        """
        self.prev_error = 0.0  # Clear previous error memory
        self.integral = 0.0    # Clear integral accumulator

    def set_setpoint(self, setpoint):
        """
        Update controller setpoint and reset internal state.
        
        This method changes the target value and automatically resets
        the controller state to prevent incorrect integral and derivative
        calculations based on the old setpoint.
        
        Parameters:
        -----------
        setpoint : float
            New target value (will be normalized for angle control)
            
        Usage Example:
        --------------
        # Change robot heading target
        heading_controller.set_setpoint(math.radians(90))  # Turn to 90 degrees
        """
        self.setpoint = normalize_angle(setpoint)  # Set and normalize new target
        self.reset()  # Clear state to prevent erroneous calculations


# ============================================================================
# END OF ROBOT NAVIGATION AND CONTROL SYSTEM
# ============================================================================
