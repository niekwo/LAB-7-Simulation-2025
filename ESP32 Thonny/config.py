# =====================================================
# ESP32 AUTONOMOUS NAVIGATION CONTROL SYSTEM
# =====================================================
# Advanced pathfinding and hardware interface module
# for robotic navigation and user interaction control
# =====================================================

from machine import Pin  # Hardware abstraction layer interface

# =====================================================
# OPERATIONAL THRESHOLD PARAMETERS
# =====================================================
# Critical system limits for safety and performance
# These values control maximum operational cycles
# =====================================================

MINIMUM_CYCLE_THRESHOLD = 5   # Lower boundary for processing iterations
MAXIMUM_CYCLE_BOUNDARY = 50   # Upper safety limit for operational loops

# =====================================================
# HARDWARE INTERFACE CONFIGURATION
# =====================================================
# Physical pin assignments for visual indicators
# and user input devices on ESP32 microcontroller
# =====================================================

# ==========================================
# VISUAL FEEDBACK SUBSYSTEM
# ==========================================
# LED indicators for system status display
# Each LED serves a specific operational purpose
# ==========================================

onboard_status_indicator = Pin(2, Pin.OUT)   # Internal ESP32 status LED - system heartbeat
amber_warning_light = Pin(4, Pin.OUT)        # Amber LED - caution/warning states
azure_navigation_light = Pin(23, Pin.OUT)    # Blue LED - navigation mode indicator
emerald_success_light = Pin(22, Pin.OUT)     # Green LED - successful operation confirmation
crimson_error_light = Pin(21, Pin.OUT)       # Red LED - error condition alert

# ==========================================
# USER INPUT INTERFACE SUBSYSTEM
# ==========================================
# Physical buttons for manual system control
# Pull-down resistors ensure stable digital readings
# ==========================================

left_control_switch = Pin(34, Pin.IN, Pin.PULL_DOWN)   # Left navigation button
right_control_switch = Pin(35, Pin.IN, Pin.PULL_DOWN)  # Right navigation button

# =====================================================
# NAVIGATION WAYPOINT DEFINITIONS
# =====================================================
# Strategic starting and destination points for
# autonomous pathfinding algorithm execution
# =====================================================

ORIGIN_WAYPOINT_IDENTIFIER = 'A0'      # Initial position marker in grid system
DESTINATION_WAYPOINT_IDENTIFIER = 'G8'  # Target endpoint for navigation sequence

# =====================================================
# SPATIAL COORDINATE MAPPING SYSTEM
# =====================================================
# Precise cartesian coordinates for each intersection
# in the navigation grid - enables accurate positioning
# =====================================================

INTERSECTION_POSITION_DATABASE = {
    # ==========================================
    # SECTOR A - NORTHERN BOUNDARY REGION
    # ==========================================
    # Top row intersection coordinates
    'A0': (-0.495148, +0.361619), 'A1': (-0.392342, +0.361619), 
    'A2': (-0.292898, +0.361619), 'A3': (-0.190444, +0.361631),
    
    # ==========================================
    # SECTOR B - UPPER CENTRAL CORRIDOR
    # ==========================================
    # Primary horizontal navigation pathway
    'B0': (-0.495148, +0.247345), 'B1': (-0.392342, +0.247345), 
    'B2': (-0.292898, +0.247345), 'B3': (-0.190444, +0.247345), 
    'B4': (0, +0.247345), 'B8': (+0.495148, +0.247345),
    
    # ==========================================
    # SECTOR C - EASTERN TRANSITION ZONE
    # ==========================================
    # Right-side vertical connector points
    'C4': (0, +0.1), 'C8': (+0.495148, +0.1),
    
    # ==========================================
    # SECTOR D - CENTRAL HUB REGION
    # ==========================================
    # Main intersection cluster - highest connectivity
    'D0': (-0.495148, 0), 'D4': (0, 0), 'D8': (+0.495148, 0),
    
    # ==========================================
    # SECTOR E - WESTERN TRANSITION ZONE
    # ==========================================
    # Left-side vertical connector points
    'E0': (-0.495148, -0.1), 'E4': (0, -0.1),
    
    # ==========================================
    # SECTOR F - LOWER CENTRAL CORRIDOR
    # ==========================================
    # Secondary horizontal navigation pathway
    'F0': (-0.495148, -0.247345), 'F4': (0, -0.247345), 
    'F5': (+0.190444, -0.247345), 'F6': (+0.292898, -0.247345), 
    'F7': (+0.392342, -0.247345), 'F8': (+0.495148, -0.247345),
    
    # ==========================================
    # SECTOR G - SOUTHERN BOUNDARY REGION
    # ==========================================
    # Bottom row intersection coordinates
    'G5': (+0.190444, -0.361619), 'G6': (+0.292898, -0.361619), 
    'G7': (+0.392342, -0.361619), 'G8': (+0.495148, -0.361619),
}

# =====================================================
# PATHFINDING CONNECTIVITY MATRIX
# =====================================================
# Defines legal movement paths between intersections
# Used by navigation algorithms for route planning
# =====================================================

NAVIGATION_ADJACENCY_GRAPH = {
    # ==========================================
    # SECTOR A CONNECTIONS - NORTHERN ENDPOINTS
    # ==========================================
    # Limited connectivity - mostly downward paths
    'A0': ['B0'], 'A1': ['B1'], 'A2': ['B2'], 'A3': ['B3'],
    
    # ==========================================
    # SECTOR B CONNECTIONS - UPPER CORRIDOR
    # ==========================================
    # High horizontal connectivity with vertical branches
    'B0': ['A0', 'B1', 'D0'], 'B1': ['A1', 'B0', 'B2'], 
    'B2': ['A2', 'B1', 'B3'], 'B3': ['A3', 'B2', 'B4'], 
    'B4': ['B3', 'B8', 'C4'], 'B8': ['B4', 'C8'],
    
    # ==========================================
    # SECTOR C CONNECTIONS - EASTERN TRANSITIONS
    # ==========================================
    # Vertical pathway connectors
    'C4': ['B4', 'D4'], 'C8': ['B8', 'D8'],
    
    # ==========================================
    # SECTOR D CONNECTIONS - CENTRAL HUB
    # ==========================================
    # Maximum connectivity hub - critical routing nodes
    'D0': ['B0', 'E0', 'D4'], 'D4': ['C4', 'D0', 'D8', 'E4'], 
    'D8': ['C8', 'D4', 'F8'],
    
    # ==========================================
    # SECTOR E CONNECTIONS - WESTERN TRANSITIONS
    # ==========================================
    # Left-side vertical pathway management
    'E0': ['D0', 'E4', 'F0'], 'E4': ['C4', 'D4', 'E0', 'F4'],
    
    # ==========================================
    # SECTOR F CONNECTIONS - LOWER CORRIDOR
    # ==========================================
    # Secondary horizontal pathway with eastern branch
    'F0': ['E0', 'F4'], 'F4': ['E4', 'F0', 'F5'], 
    'F5': ['F4', 'F6', 'G5'], 'F6': ['F5', 'F7', 'G6'], 
    'F7': ['F6', 'F8', 'G7'], 'F8': ['D8', 'F7', 'G8'],
    
    # ==========================================
    # SECTOR G CONNECTIONS - SOUTHERN ENDPOINTS
    # ==========================================
    # Terminal nodes - limited upward connectivity only
    'G5': ['F5'], 'G6': ['F6'], 'G7': ['F7'], 'G8': ['F8']
}