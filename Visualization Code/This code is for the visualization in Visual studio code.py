#!/usr/bin/env python3
"""
Real-Time Autonomous Robot Trajectory Monitoring Framework
=========================================================

This application provides a comprehensive visualization dashboard for monitoring
autonomous robot navigation in structured grid environments. The system establishes
real-time communication with robotic platforms to display current position, planned
trajectories, and environmental obstacles in an interactive graphical interface.

Core Capabilities:
- Live robot position tracking with grid-based coordinate system
- Dynamic path planning visualization with waypoint display
- Interactive obstacle mapping and grid environment representation
- Real-time TCP/IP communication with autonomous navigation systems
- Threaded data processing for responsive user interface updates

Technical Architecture:
- Frontend: matplotlib-based graphical visualization engine
- Backend: Multi-threaded TCP socket communication handler
- Data Format: JSON-based message protocol for robot state exchange
- Grid System: Alphanumeric node identification with obstacle mapping

Target Applications:
- Educational robotics demonstration and debugging
- Research prototype monitoring and analysis
- Industrial automation path verification
- Multi-robot coordination visualization

Development Environment:
- Python 3.x with matplotlib, numpy, and threading libraries
- TCP/IP network communication over local area network
- JSON message serialization for cross-platform compatibility

Author:Niek Wolfs 
Creation Date: 21 May 2025
Last Updated: 6 June 2025
"""

# =====================================================================
# ESSENTIAL LIBRARY IMPORTS AND DEPENDENCIES
# =====================================================================

import socket              # TCP/IP network communication primitives
import json               # JavaScript Object Notation data serialization
import numpy as np        # Numerical computation and array operations
import matplotlib.pyplot as plt    # Scientific plotting and visualization
from matplotlib.animation import FuncAnimation  # Real-time plot animation
import time               # System timing and delay functions
import threading          # Concurrent execution and background processing
import queue             # Thread-safe data exchange mechanisms

# =====================================================================
# NAVIGATION ENVIRONMENT TOPOLOGY DEFINITION
# =====================================================================

# Comprehensive grid map defining the robot's operational environment
# Legend: 0 = navigable path, 1 = obstacle/wall, 'XX' = named waypoint nodes
OPERATIONAL_ENVIRONMENT_MAP = [
    ['A0', 1, 'A1', 1, 'A2', 1, 'A3', 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    ['B0', 0, 0, 0, 0, 0, 0, 0, 'B4', 0, 0, 0, 0, 0, 0, 0, 'B8'],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 'C4', 0, 0, 0, 0, 0, 0, 0, 'C8'],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    ['D0', 0, 0, 0, 0, 0, 0, 0, 'D4', 0, 0, 0, 0, 0, 0, 0, 'D8'],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    ['E0', 0, 0, 0, 0, 0, 0, 0, 'E4', 1, 1, 1, 1, 1, 1, 1, 'E8'],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    ['F0', 0, 0, 0, 0, 0, 0, 0, 'F4', 0, 'F5', 0, 'F6', 0, 'F7', 0, 'F8'],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 'G5', 1, 'G6', 1, 'G7', 1, 'G8']
]

# =====================================================================
# PRIMARY VISUALIZATION AND MONITORING CLASS
# =====================================================================

class AutonomousRobotMonitoringDashboard:
    """
    Advanced real-time robot trajectory visualization and monitoring system.
    
    This class provides comprehensive functionality for visualizing robot navigation
    including position tracking, path planning display, obstacle mapping, and
    real-time communication with autonomous navigation controllers.
    """
    
    def __init__(self, network_host='10.149.34.27', communication_port=65433):
        """
        Initialize the robot monitoring dashboard with network communication setup.
        
        Parameters:
        -----------
        network_host : str
            IP address of the robot's communication interface
        communication_port : int  
            TCP port number for robot data exchange
        """
        
        # =============================================================
        # GRAPHICAL INTERFACE INITIALIZATION
        # =============================================================
        
        # Create primary visualization canvas with appropriate dimensions
        self.visualization_figure, self.plot_axes = plt.subplots(figsize=(12, 8))
        
        # Initialize robot state tracking variables
        self.current_robot_location = None     # Active robot position node identifier
        self.computed_trajectory_sequence = [] # Planned navigation waypoint sequence
        self.robot_position_indicator = None   # Visual marker for robot location
        self.trajectory_visualization_line = None  # Path visualization graphic element
        self.network_connection_active = False # TCP connection status flag
        
        # Visualization state management
        self.displayed_path_waypoints = []     # Currently rendered path nodes
        self.displayed_robot_node = None       # Currently rendered robot position
        
        # =============================================================
        # NETWORK COMMUNICATION INFRASTRUCTURE
        # =============================================================
        
        # Configure TCP socket for robot communication
        self.communication_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.communication_socket.settimeout(0.1)
        self.target_host_address = network_host
        self.target_communication_port = communication_port
        
        # Thread-safe data exchange mechanisms
        self.incoming_data_queue = queue.Queue()
        self.thread_termination_signal = threading.Event()
        self.message_reception_buffer = ""
        self.background_communication_thread = None
        
        # Initiate initial connection attempt
        self.establish_robot_connection()
        
        # Configure visualization environment
        self.initialize_grid_visualization()

    def establish_robot_connection(self):
        """
        Attempt to establish TCP connection with the autonomous robot system.
        Handles connection failures gracefully and provides status feedback.
        """
        if not self.network_connection_active:
            try:
                # Clean up any existing socket connections
                if self.communication_socket:
                    self.communication_socket.close()
                    
                # Create fresh socket with connection timeout
                self.communication_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.communication_socket.settimeout(2.0)
                self.communication_socket.connect((self.target_host_address, self.target_communication_port))
                
                # Connection successful - update status and start background thread
                self.network_connection_active = True
                print(f"✓ Robot communication link established: {self.target_host_address}:{self.target_communication_port}")
                
                # Initialize background data reception thread
                self.thread_termination_signal.clear()
                self.background_communication_thread = threading.Thread(
                    target=self.continuous_data_reception_handler, 
                    daemon=True
                )
                self.background_communication_thread.start()
                
            except socket.error:
                # Connection failed - update status and schedule retry
                self.network_connection_active = False
                print(f"⚠ Awaiting robot connection at {self.target_host_address}:{self.target_communication_port}")
                time.sleep(1)  # Prevent excessive retry attempts

    def continuous_data_reception_handler(self):
        """
        Background thread function for continuous robot data reception.
        
        Handles TCP data reception, message buffering, JSON parsing, and
        thread-safe data queuing for the main visualization thread.
        """
        while not self.thread_termination_signal.is_set() and self.network_connection_active:
            try:
                # Receive data packet from robot
                incoming_data_packet = self.communication_socket.recv(1024)
                
                # Check for connection termination
                if not incoming_data_packet:
                    self.network_connection_active = False
                    break
                
                # Append to message buffer for complete message assembly
                self.message_reception_buffer += incoming_data_packet.decode('utf-8')
                
                # Process complete messages delimited by newline characters
                while '\n' in self.message_reception_buffer:
                    complete_message, self.message_reception_buffer = self.message_reception_buffer.split('\n', 1)
                    
                    if complete_message.strip():
                        try:
                            # Parse JSON message and queue for main thread processing
                            parsed_robot_data = json.loads(complete_message)
                            self.incoming_data_queue.put(parsed_robot_data)
                        except Exception as json_parsing_error:
                            print(f"⚠ Message parsing error: {json_parsing_error}")
                            continue
                            
            except socket.timeout:
                # Normal timeout - continue reception loop
                continue
            except Exception as communication_error:
                print(f"✗ Communication handler error: {communication_error}")
                self.network_connection_active = False
                break

    def resolve_waypoint_coordinates(self, waypoint_identifier):
        """
        Convert alphanumeric waypoint identifier to grid coordinates.
        
        Parameters:
        -----------
        waypoint_identifier : str
            Waypoint name (e.g., 'E4', 'B0', 'G7')
            
        Returns:
        --------
        tuple or None
            (x, y) coordinates for plotting, or None if waypoint not found
        """
        for grid_row_index, grid_row_data in enumerate(OPERATIONAL_ENVIRONMENT_MAP):
            for grid_column_index, grid_cell_content in enumerate(grid_row_data):
                if grid_cell_content == waypoint_identifier:
                    # Convert grid indices to plot coordinates (y-axis inversion)
                    return grid_column_index, 14 - grid_row_index
        return None

    def initialize_grid_visualization(self):
        """
        Render the complete operational environment including obstacles,
        navigable paths, and waypoint locations with appropriate visual styling.
        """
        # Clear any existing visualization elements
        self.plot_axes.clear()

        # Iterate through grid map and render appropriate visual elements
        for row_index, row_data in enumerate(OPERATIONAL_ENVIRONMENT_MAP):
            for column_index, cell_content in enumerate(row_data):
                
                # Calculate plot coordinates (invert y-axis for standard orientation)
                plot_x_coordinate = column_index
                plot_y_coordinate = 14 - row_index
                
                if cell_content == 1:  # Obstacle/wall cell
                    # Render obstacle as filled gray rectangle
                    obstacle_patch = plt.Rectangle(
                        (plot_x_coordinate - 0.5, plot_y_coordinate - 0.5), 
                        1, 1, 
                        fill=True, 
                        facecolor='lightgray',
                        edgecolor='black',
                        linewidth=0.5
                    )
                    self.plot_axes.add_patch(obstacle_patch)
                    
                elif cell_content == 0 or isinstance(cell_content, str):  # Navigable path or waypoint
                    # Render navigable area as outlined rectangle
                    path_patch = plt.Rectangle(
                        (plot_x_coordinate - 0.5, plot_y_coordinate - 0.5), 
                        1, 1, 
                        fill=False, 
                        edgecolor='black',
                        linewidth=0.5
                    )
                    self.plot_axes.add_patch(path_patch)
                    
                    if isinstance(cell_content, str):  # Named waypoint location
                        # Add waypoint marker and label
                        self.plot_axes.scatter(
                            plot_x_coordinate, plot_y_coordinate, 
                            c='blue', 
                            s=50, 
                            marker='s',
                            alpha=0.7
                        )
                        self.plot_axes.annotate(
                            cell_content, 
                            (plot_x_coordinate, plot_y_coordinate), 
                            xytext=(5, 5), 
                            textcoords='offset points',
                            fontsize=8,
                            fontweight='bold'
                        )

        # Configure plot appearance and properties
        self.plot_axes.set_title('Autonomous Robot Navigation Dashboard', fontsize=14, fontweight='bold')
        self.plot_axes.grid(True, alpha=0.3)
        self.plot_axes.set_aspect('equal')
        self.plot_axes.set_xlim(-1, 17)
        self.plot_axes.set_ylim(-1, 15)
        self.plot_axes.set_xlabel('Grid X Coordinate', fontsize=12)
        self.plot_axes.set_ylabel('Grid Y Coordinate', fontsize=12)

    def refresh_visualization_display(self, animation_frame_number):
        """
        Update visualization with latest robot data from communication queue.
        
        This function is called periodically by the matplotlib animation system
        to ensure real-time display updates as new robot data arrives.
        
        Parameters:
        -----------
        animation_frame_number : int
            Frame number from matplotlib animation system (unused)
        """
        # Attempt connection if not currently active
        if not self.network_connection_active:
            self.establish_robot_connection()
            return

        # Process all available data from the communication queue
        visualization_updated = False
        while not self.incoming_data_queue.empty():
            current_robot_data = self.incoming_data_queue.get()
            visualization_updated = True
            
            # ==========================================================
            # TRAJECTORY VISUALIZATION UPDATE
            # ==========================================================
            
            # Update planned path visualization if trajectory data is available
            if 'planned_path' in current_robot_data and current_robot_data['planned_path']:
                self.displayed_path_waypoints = current_robot_data['planned_path']
                
                # Convert waypoint names to plot coordinates
                trajectory_coordinates = [
                    self.resolve_waypoint_coordinates(waypoint) 
                    for waypoint in self.displayed_path_waypoints
                ]
                valid_coordinates = [coord for coord in trajectory_coordinates if coord is not None]
                
                # Render trajectory path if valid coordinates exist
                if valid_coordinates:
                    trajectory_x_values = [coordinate[0] for coordinate in valid_coordinates]
                    trajectory_y_values = [coordinate[1] for coordinate in valid_coordinates]
                    
                    if self.trajectory_visualization_line is None:
                        # Create new trajectory line visualization
                        self.trajectory_visualization_line, = self.plot_axes.plot(
                            trajectory_x_values, trajectory_y_values, 
                            'b:', 
                            alpha=0.7, 
                            linewidth=2, 
                            marker='o', 
                            markersize=6,
                            label='Planned Trajectory'
                        )
                    else:
                        # Update existing trajectory line
                        self.trajectory_visualization_line.set_data(trajectory_x_values, trajectory_y_values)
            
            # ==========================================================
            # ROBOT POSITION UPDATE
            # ==========================================================
            
            # Update robot position marker regardless of trajectory availability
            if 'current_node' in current_robot_data and current_robot_data['current_node']:
                current_waypoint = current_robot_data['current_node']
                waypoint_coordinates = self.resolve_waypoint_coordinates(current_waypoint)
                
                if waypoint_coordinates:
                    self.displayed_robot_node = current_waypoint
                    
                    # Remove previous robot position marker
                    if self.robot_position_indicator is not None:
                        self.robot_position_indicator.remove()
                        self.robot_position_indicator = None
                    
                    # Create new robot position marker
                    self.robot_position_indicator = self.plot_axes.scatter(
                        *waypoint_coordinates, 
                        c='red', 
                        s=120, 
                        marker='o', 
                        zorder=10,
                        edgecolors='darkred',
                        linewidth=2,
                        label='Robot Position'
                    )
        
        # Refresh display if visualization was updated
        if visualization_updated:
            self.visualization_figure.canvas.draw_idle()

    def shutdown_monitoring_system(self):
        """
        Perform clean shutdown of all system resources including network
        connections, background threads, and visualization components.
        """
        # Signal background threads to terminate
        self.thread_termination_signal.set()
        
        # Close network socket connection
        if self.communication_socket:
            try:
                self.communication_socket.close()
            except:
                pass  # Ignore errors during shutdown
        
        # Wait for background thread termination
        if (self.background_communication_thread and 
            self.background_communication_thread.is_alive()):
            self.background_communication_thread.join(timeout=1)

    def execute_monitoring_dashboard(self):
        """
        Launch the interactive monitoring dashboard with real-time updates.
        
        This method starts the matplotlib animation loop and displays the
        visualization window until manually closed by the user.
        """
        # Create matplotlib animation for real-time updates
        dashboard_animation = FuncAnimation(
            self.visualization_figure, 
            self.refresh_visualization_display,
            interval=300,  # Update interval optimized for robot communication rate
            cache_frame_data=False,
            save_count=1000
        )
        
        # Display interactive visualization window
        plt.show()

# =====================================================================
# MAIN PROGRAM EXECUTION ENTRY POINT
# =====================================================================

if __name__ == "__main__":
    """
    Primary execution entry point for the robot monitoring application.
    
    Creates monitoring dashboard instance and handles graceful shutdown
    when the visualization window is closed or program is interrupted.
    """
    
    print("="*60)
    print("AUTONOMOUS ROBOT MONITORING DASHBOARD")
    print("="*60)
    print("Initializing real-time visualization system...")
    
    # Create and configure monitoring dashboard instance
    robot_monitoring_dashboard = AutonomousRobotMonitoringDashboard()
    
    try:
        print("✓ Dashboard initialization complete")
        print("✓ Starting interactive visualization...")
        print("✓ Close the visualization window to exit")
        print("-" * 60)
        
        # Launch interactive monitoring dashboard
        robot_monitoring_dashboard.execute_monitoring_dashboard()
        
    except KeyboardInterrupt:
        print("\n⚠ Program interrupted by user")
    except Exception as system_error:
        print(f"✗ System error occurred: {system_error}")
    finally:
        print("\nInitiating system shutdown...")
        robot_monitoring_dashboard.shutdown_monitoring_system()
        print("✓ All resources released successfully")
        print("✓ Program terminated cleanly")

# =====================================================================
# END OF MONITORING SYSTEM
# =====================================================================