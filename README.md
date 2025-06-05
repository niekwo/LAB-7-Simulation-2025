# Autonomous Robot Navigation and Control System

This repository contains the software components for an autonomous robot navigation and control system, designed for an ESP32 microcontroller and a simulated environment (Webots), complemented by a real-time visualization dashboard. The system integrates modules for network connectivity, low-level sensor processing and motor control, advanced pathfinding, and a visualization interface for monitoring robot operations.

## Repository Structure

The repository is organized into several key Python modules, each responsible for a specific aspect of the robot's functionality:

* `AStar.py`
* `OuterLine.py`
* `boot.py`
* `Code for webots.py`
* `This code is for the visualization in Visual studio code.py`
* `config.py`
* `utils.py`

---

### `AStar.py`

This module implements the core navigation logic for the ESP32 robot. It utilizes the A\* search algorithm for pathfinding within a defined grid environment. Key functionalities include:

* **Robot Pose Management**: Tracks the robot's `(x, y, phi)` position.
* **Line Following PID**: Integrates a PID controller for precise line following.
* **Waypoint Navigation**: Manages the planned path and transitions between waypoints.
* **Communication**: Handles data exchange with the visualization system.

---

### `OuterLine.py`

This module is dedicated to real-time line following and ground sensor data processing on the ESP32. It is responsible for:

* **Line Following State Determination**: Analyzes ground sensor values to determine the robot's state (`'forward'`, `'turn_left'`, `'turn_right'`, `'lost'`).
* **Speed Computation**: Calculates appropriate left and right wheel speeds based on the detected line-following state.
* **TCP Server**: Establishes a TCP server for communication, likely to receive commands or send sensor data.

---

### `boot.py`

This module manages the network connectivity for the ESP32 microcontroller at startup. Its primary functions include:

* **Network Initialization**: Automatically establishes a Wi-Fi connection upon boot.
* **Authentication Parameters**: Configures the network SSID and password.
* **Connection Retries**: Implements retry mechanisms to ensure robust network establishment.

---

### `Code for webots.py`

This script is specifically designed for the Webots simulation environment. It enables the simulation of the robot's hardware and basic movements:

* **Robot Initialization**: Sets up the robot instance within the simulation.
* **Motor Control**: Configures differential drive motors for velocity control.
* **Encoder Integration**: Initializes and enables wheel encoder sensors for odometry measurements and velocity calculation.

---

### `This code is for the visualization in Visual studio code.py`

This Python application serves as a real-time monitoring framework for the autonomous robot's trajectory, intended to run in Visual Studio Code. It offers a comprehensive visualization dashboard with the following capabilities:

* **Live Position Tracking**: Displays the robot's current position within a grid-based coordinate system.
* **Path Planning Visualization**: Visualizes planned trajectories and waypoints dynamically.
* **Obstacle Mapping**: Represents environmental obstacles and the grid layout.
* **Real-time Communication**: Uses TCP/IP for communication with the robot system.
* **Threaded Processing**: Ensures a responsive user interface through multi-threaded data handling.

---

### `config.py`

This file centralizes the configuration parameters for the entire ESP32 autonomous navigation control system. It defines critical operational parameters and hardware mappings:

* **Operational Thresholds**: Sets limits for processing iterations.
* **Hardware Pin Assignments**: Configures physical pin assignments for LEDs and user input buttons on the ESP32.
* **LED Indicators**: Defines LEDs for various system status displays (e.g., onboard status, warning, navigation, success, error).
* **User Input**: Configures buttons for manual control (left and right navigation).
* **Navigation Waypoints**: Specifies strategic starting and destination points for pathfinding, including `INTERSECTION_COORDS_ESP` and `VALID_CONNECTIONS_ESP` for grid mapping and connectivity.

---

### `utils.py`

This module provides a collection of essential utility functions and constants that support the ESP32 robot's navigation and control system. Key utilities include:

* **Sensor Reading**: Functions for interpreting data from line-following sensors.
* **LED Control**: Centralized function to control the state of the robot's status LEDs (yellow, blue, green, red).
* **Mathematical Utilities**: Helper functions for angle normalization (`normalize_angle`) and value clamping (`clip_value`).
* **Navigation Constants**: Defines parameters such as waypoint reached thresholds and line-following speed factors.

---

## System Requirements

The following software versions are recommended for optimal performance:

* **Operating System**: Windows 11
* **Thonny**: 4.1.6
    * **Python**: 3.10.11
    * **Tk**: 8.6.13
* **Webots**: R2025a (released February 4, 2025)
* **Visual Studio Code**: Version 1.100.3 (windows\_nt x64 10.0.26100)

**Note on Libraries**: The Python libraries required for the visualization code are typically standard libraries (`matplotlib`, `numpy`, `threading`, `json`, `socket`, `time`). For MicroPython on the ESP32, the modules imported (`machine`, `network`, `usocket`, `ustruct`, `time`, `math`, `heapq`) are part of the MicroPython core or standard libraries.

## Installation and Setup

### 1. ESP32 Setup (using Thonny)

1.  **Install MicroPython Firmware**:
    * Open Thonny (version 4.1.6).
    * Go to `Tools` > `Options` > `Interpreter` tab.
    * Select "MicroPython (ESP32)" as the interpreter.
    * Click "Install or update MicroPython" and install **MicroPython version 1.25.0** onto your ESP32.
2.  **Upload Files to ESP32**:
    * After MicroPython is installed, upload the following files from this repository to your ESP32's filesystem via Thonny:
        * `AStar.py`
        * `OuterLine.py`
        * `boot.py`
        * `config2.py`
        * `utils.py`

### 2. Visualization Setup (using Visual Studio Code)

1.  Open Visual Studio Code (version 1.100.3).
2.  Add the file `This code is for the visualization in Visual studio code.py` to your Visual Studio Code workspace.

### 3. Webots Simulation Setup

1.  **Open World**:
    * Locate the Webots world file (e.g., `Webots_RaFLite_HiL 2025.wbt`).
    * Open Webots (R2025a) and load this world file by selecting `File` > `Open World`.
2.  **Create New Controller**:
    * In Webots, go to `File` > `New Robot Controller`.
    * Select Python as the programming language for the new controller.
    * Copy the entire content of the `Code for webots.py` file into this newly created controller file and save it.

## Usage Instructions

Once the installation and setup are complete, follow these steps to operate the robot and its simulation:

### 1. Boot and Wi-Fi Connection (ESP32)

1.  **Boot ESP32**: Power on your ESP32, or if already powered, press the `boot` button.
2.  **Monitor Connection**: Observe the Thonny terminal. It will display the Wi-Fi connection progress. Wait until the ESP32 successfully connects to your network.
3.  **Get IP Address**: The Thonny terminal will print the assigned IP address of the ESP32.
4.  **Update IP Address in Code**: **Crucially**, take the IP address displayed in the Thonny terminal and update the IP address placeholders within `Code for webots.py` (for the Webots simulation) and `This code is for the visualization in Visual studio code.py` (for the Visual Studio Code visualization).
5.  **Ready Indicator**: Once the ESP32 is fully ready and connected, the blue LED on the board will start blinking.

### 2. Choosing an Algorithm (ESP32)

After the ESP32 is connected and ready, you have two main operational choices for the robot's behavior:

* **Outer Line Follower**: Press the **left button** on the ESP32 to initiate the `OuterLine.py` algorithm. This will make the robot follow the outer line.
* **A\* Path Planner**: Press the **right button** on the ESP32 to start the `AStar.py` algorithm. This will engage the A\* pathfinding and navigation.
* **Stop/REPL**: You can always click `STOP` in Thonny to halt the current execution on the ESP32 and return to the MicroPython REPL (Read-Eval-Print Loop).

### 3. Running with A\* Path Planner (and Visualization)

If you choose the A\* Path Planner (by pressing the **right button** on the ESP32):

1.  **Start Visualization Promptly**: Within **10 seconds** of pressing the right button on the ESP32, switch to Visual Studio Code and run the `This code is for the visualization in Visual studio code.py` script. Click "Play" or execute the script to start the visualization.
    * **Important**: If the visualization is not started within this 10-second window, it will not be able to connect to the ESP32 for the current run, and you may need to restart the ESP32 process.
2.  **Visualization Connection**: Once the visualization script connects to the ESP32, it will begin to display the robot's planned path and real-time position.
3.  **Visualization Ready Indicator**: After the 10-second window, if the connection is successful, the green LED on the ESP32 will start shining, indicating that the visualization is successfully connected and synchronized.
4.  **Start Webots**: Once the visualization is active and the green LED is shining, you can start the simulation in Webots. The robot in Webots will then execute its job according to the A\* path planned by the ESP32.

### 4. Running with Outer Line Follower (Webots only)

If you choose the "in the loop" mode (Outer Line Follower) by pressing the **left button** on the ESP32:

* The external visualization in Visual Studio Code is not typically used for this mode.
* After pressing the left button on the ESP32, you can immediately start the simulation in Webots. The robot will then begin to execute the line-following algorithm in the simulated environment.
