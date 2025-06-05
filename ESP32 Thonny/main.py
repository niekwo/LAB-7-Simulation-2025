# Implements communication with Webots via Serial over USB.
# Works with the "line_following_with_HIL" controller in Webots.

# Tested with MicroPython v1.25.0 on ESP32 WROOM 32 dev kit board
# communicating with Webots R2023a, on Windows 11 running Python 3.10.5 64-bit

# To use ulab, you need to install a micropython interpreter that contains this

# Author: N Wolfs  
# Date: 1 June 2025
# Last update: 1 June 2025


###### Close Thonny after start running this code #######
# This is necessary because the serial port cannot be used by
# two different programs (Thonny and Webots) at the same time.

import usocket as socket
import ustruct as struct
import network
import uheapq as heapq # MicroPython's heap queue
from time import sleep
from config import led_board, button_left, button_right
import OuterLine as OuterLine # type: ignore
import AStar as AStar # type: ignore


# --- Check Wi-Fi Connection ---
wlan = network.WLAN(network.STA_IF)
if not wlan.isconnected():
    print("WiFi not connected. Please ensure boot.py ran successfully and connected to WiFi.")
    print("You might need to reset the device.")
else:
    ESP32_IP_ADDRESS = wlan.ifconfig()[0]
    print(f"ESP32 is connected to WiFi. IP Address: {ESP32_IP_ADDRESS}")


# Wait for the button click before starting the corresponding algorithm.
# During the wait period, the program can be stopped using the STOP button in Thonny.

print("""
Press the left button on the esp32 to start the Outer Line Follower algorithm.
Press the right button on the esp32 to start the A* Path Planner algorithm.
Or click STOP in Thonny to return to the REPL.
""")
while True:
    if button_left() == True and button_right() == False:
        print("Button left pressed. Starting Outer Line Follower...")
        led_board.value(0)
        OuterLine.run()
        break
    elif button_right() == True and button_left() == False:
        print("Button right pressed. Starting A* Path Planner...")
        led_board.value(1)
        AStar.run()
        break
    else:
        # Blink the LED board to indicate waiting for button press
        sleep(0.25)
        led_board.value(not led_board())

# for debugging purposes, this place should not be reached
led_board.value(0)  # Turn off the LED board
print("Program finished. Returning to REPL.")

