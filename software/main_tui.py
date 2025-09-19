import serial
from CalcLidarData import CalcLidarData
import math
import curses
import numpy as np
import time

# Initialize serial connection to LiDAR
ser = serial.Serial(port='/dev/ttyUSB0',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

# Initialize curses for TUI
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.keypad(True)
stdscr.nodelay(True)  # Make getch() non-blocking

# Create a color pair for the radar display
curses.start_color()
curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)

# Radar display parameters
RADAR_WIDTH = 60
RADAR_HEIGHT = 20
RADAR_CENTER_X = RADAR_WIDTH // 2
RADAR_CENTER_Y = RADAR_HEIGHT // 2
MAX_DISPLAY_DISTANCE = 4000  # mm

def draw_radar(angles, distances):
    """Draw a radar-like display in the terminal"""
    stdscr.clear()
    height, width = stdscr.getmaxyx()
    
    # Create a radar grid
    radar = [[' ' for _ in range(RADAR_WIDTH)] for _ in range(RADAR_HEIGHT)]
    
    # Draw radar circles
    for r in range(1, 6):
        radius = r * MAX_DISPLAY_DISTANCE / 5
        circle_radius_pixels = int((r * RADAR_HEIGHT) / 5)
        for angle in np.linspace(0, 2 * np.pi, 100):
            x = int(RADAR_CENTER_X + circle_radius_pixels * np.cos(angle))
            y = int(RADAR_CENTER_Y + circle_radius_pixels * np.sin(angle))
            if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                radar[y][x] = '.'
    
    # Draw axes
    for i in range(RADAR_WIDTH):
        radar[RADAR_CENTER_Y][i] = '-'
    for i in range(RADAR_HEIGHT):
        radar[i][RADAR_CENTER_X] = '|'
    radar[RADAR_CENTER_Y][RADAR_CENTER_X] = '+'
    
    # Plot LiDAR points
    for angle, distance in zip(angles, distances):
        if distance > MAX_DISPLAY_DISTANCE:
            continue
            
        # Convert polar to Cartesian coordinates
        x = int(RADAR_CENTER_X + (distance / MAX_DISPLAY_DISTANCE) * RADAR_HEIGHT * np.cos(angle))
        y = int(RADAR_CENTER_Y + (distance / MAX_DISPLAY_DISTANCE) * RADAR_HEIGHT * np.sin(angle))
        
        if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
            # Color code based on distance
            if distance < 1000:  # Close distance
                radar[y][x] = 'X'  # Very close
            elif distance < 2000:  # Medium distance
                radar[y][x] = 'o'  # Medium
            else:
                radar[y][x] = '*'  # Far
    
    # Draw the radar
    for y in range(RADAR_HEIGHT):
        if y < height - 2:
            try:
                stdscr.addstr(y, 0, ''.join(radar[y])[:width-1])
            except curses.error:
                pass
    
    # Display information
    if height > RADAR_HEIGHT + 5:
        stdscr.addstr(RADAR_HEIGHT + 1, 0, f"Max Distance: {MAX_DISPLAY_DISTANCE}mm")
        stdscr.addstr(RADAR_HEIGHT + 2, 0, f"Points: {len(angles)}")
        stdscr.addstr(RADAR_HEIGHT + 3, 0, "Press 'q' to quit")
    
    stdscr.refresh()

# Cleanup function
def cleanup():
    ser.close()
    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()

# Main loop
try:
    tmpString = ""
    angles = list()
    distances = list()
    flag2c = False
    
    while True:
        # Check for quit command
        key = stdscr.getch()
        if key == ord('q'):
            break
            
        # Read LiDAR data
        all_b = ser.read_all()
        for b in all_b:
            tmpInt = int(b)
            b = bytes([b])
            if tmpInt == 0x54:
                tmpString += b.hex() + " "
                flag2c = True
                continue
            elif tmpInt == 0x2c and flag2c:
                tmpString += b.hex()
                if not len(tmpString[0:-5].replace(' ', '')) == 90:
                    tmpString = ""
                    flag2c = False
                    continue
                
                lidarData = CalcLidarData(tmpString[0:-5])
                angles.extend(lidarData.Angle_i)
                distances.extend(lidarData.Distance_i)
                
                if len(angles) > 50 * 12:
                    split = [i for i in range(len(angles)-1) if angles[i+1] < angles[i]]
                    if split:
                        first = angles[:split[0]+1]
                        angles = angles[split[0]+1:]
                        firstDist = distances[:split[0]+1]
                        distances = distances[split[0]+1:]
                        
                        # Draw the radar display
                        draw_radar(first, firstDist)
                
                tmpString = ""
            else:
                tmpString += b.hex() + " "
            
            flag2c = False

except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print(f"Error: {e}")
finally:
    cleanup()