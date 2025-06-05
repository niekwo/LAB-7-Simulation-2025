import network
import machine
import usocket as socket
import ustruct as struct
import time
from utils import led_Control  # <-- import the real led_Control

# --- Wi-Fi Setup (assume already connected) ---
wlan = network.WLAN(network.STA_IF)
ESP32_IP_ADDRESS = wlan.ifconfig()[0] if wlan.isconnected() else "0.0.0.0"

# --- Constants ---
MAX_SPEED = 6.28

def determine_line_following_state(gs_values):
    """
    Determine the robot's line following state based on ground sensor values.
    Returns one of: 'forward', 'turn_left', 'turn_right', 'lost'.
    """
    left_on = gs_values[2] < 500
    center_on = gs_values[1] < 500
    right_on = gs_values[0] < 500

    if not left_on and not center_on and not right_on:
        return 'lost'
    elif center_on:
        return 'forward'
    elif left_on:
        return 'turn_right'
    elif right_on:
        return 'turn_left'
    else:
        return 'forward'

def determine_line_following_speeds(gs_values):
    """
    Compute left and right wheel speeds based on the current line following state.
    """
    base_speed = MAX_SPEED * 1
    state = determine_line_following_state(gs_values)
    if state == 'forward':
        return base_speed, base_speed
    elif state == 'turn_left':
        return 0.2 * base_speed, base_speed
    elif state == 'turn_right':
        return base_speed, 0.2 * base_speed
    elif state == 'lost':
        return -0.2 * base_speed, 0.2 * base_speed
    else:
        return 0.0, 0.0

def set_led_by_state(state):
    """
    Set the LEDs to indicate the current robot state.
    """
    if state == 'forward':
        led_Control(1, 0, 0, 0)  # Yellow
    elif state == 'turn_right':
        led_Control(0, 1, 0, 0)  # Blue
    elif state == 'turn_left':
        led_Control(0, 0, 1, 0)  # Green
    elif state == 'lost':
        led_Control(0, 0, 0, 1)  # Red
    else:
        led_Control(0, 0, 0, 0)  # All off

def process_robot_data_tcp(client_socket):
    """
    Handle a single TCP message from Webots, process sensor data, and send wheel speeds.
    """
    try:
        expected_bytes = 24
        received = b''
        while len(received) < expected_bytes:
            packet = client_socket.recv(expected_bytes - len(received))
            if not packet:
                return False
            received += packet
        gs0, gs1, gs2, wl, wr, delta_t = struct.unpack('<6f', received)
        gs_values = [gs0, gs1, gs2]
        state = determine_line_following_state(gs_values)
        set_led_by_state(state)
        leftSpeed, rightSpeed = determine_line_following_speeds(gs_values)
        response = struct.pack('<2f', leftSpeed, rightSpeed)
        client_socket.sendall(response)
        return True
    except Exception as e:
        try:
            client_socket.sendall(struct.pack('<2f', 0.0, 0.0))
        except:
            pass
        return False

def start_server(host='0.0.0.0', port=65432):
    """
    Start the TCP server for Webots communication (OuterLine mode).
    """
    print("ESP32 OuterLine: Waiting for Webots connection...")
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"ESP32 OuterLine: TCP server listening on {ESP32_IP_ADDRESS}:{port}")
    while True:
        client_sock = None
        try:
            client_sock, client_addr = server_socket.accept()
            print(f"ESP32 OuterLine: Client connected from {client_addr}")
            client_sock.settimeout(5.0)
            while True:
                if not process_robot_data_tcp(client_sock):
                    print("ESP32 OuterLine: Client disconnected or error.")
                    break
        except Exception as e:
            print("ESP32 OuterLine: Socket error:", e)
        finally:
            if client_sock:
                client_sock.close()
                print("ESP32 OuterLine: Client socket closed.")
                machine.reset()
    server_socket.close()

def run():
    """
    Entry point: start the OuterLine TCP server.
    """
    start_server()

if __name__ == "__main__":
    run()

# Add these LED control calls in your state machine or wherever you handle robot states:
# - led_Control(1, 0, 0, 0)  # Yellow: moving forward
# - led_Control(0, 1, 0, 0)  # Blue: turning right
# - led_Control(0, 0, 1, 0)  # Green: turning left
# - led_Control(0, 0, 0, 1)  # Red: stopped

# Example usage:
# if current_state == 'forward':
#     led_Control(1, 0, 0, 0)
# elif current_state == 'turn_right':
#     led_Control(0, 1, 0, 0)
# elif current_state == 'turn_left':
#     led_Control(0, 0, 1, 0)
# elif current_state == 'stop':
#     led_Control(0, 0, 0, 1)
