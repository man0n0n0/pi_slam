import serial
from CalcLidarData import CalcLidarData
import math
import RPi.GPIO as GPIO
import time
import curses
import numpy as np

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin definitions
SERVO_PIN = 18    # GPIO pin for servo motor
ESC_PIN = 17      # GPIO pin for ESC (Electronic Speed Controller)

# Setup PWM pins
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Initialize PWM
servo_pwm = GPIO.PWM(SERVO_PIN, 50)   # 50Hz for servo
esc_pwm = GPIO.PWM(ESC_PIN, 50)       # 50Hz for ESC

# Start PWM with neutral signals
servo_pwm.start(7.5)  # Neutral position for servo (90 degrees)
esc_pwm.start(7.5)    # Neutral position for ESC (stop)

# Function to set speed using PWM (ESC)
def set_speed(speed_percent):
    """
    Set robot speed using PWM ESC
    speed_percent: -100 (full reverse) to 100 (full forward)
    """
    # Limit speed between -100 and 100
    speed_percent = max(-100, min(100, speed_percent))
    
    # Convert percentage to duty cycle (typically 5-10% for ESCs)
    # Neutral at 7.5%, full forward at 10%, full reverse at 5%
    duty_cycle = 7.5 + (speed_percent / 100) * 2.5
    esc_pwm.ChangeDutyCycle(duty_cycle)

# Function to set steering angle (servo motor)
def set_steering(angle):
    """
    Set steering angle using servo motor
    angle: -90 (full left) to 90 (full right)
    """
    # Limit angle between -90 and 90
    angle = max(-90, min(90, angle))
    
    # Convert angle to duty cycle (typically 5-10% for servos)
    # 0 degrees at 7.5%, -90 at 5%, 90 at 10%
    duty_cycle = 7.5 + (angle / 90) * 2.5
    servo_pwm.ChangeDutyCycle(duty_cycle)

# Obstacle avoidance parameters
SAFE_DISTANCE = 1000  # mm (1 meter)
SLOW_DOWN_DISTANCE = 1500  # mm (1.5 meters)
TURN_ANGLE = 45       # degrees to turn when avoiding obstacles

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

def draw_radar(angles, distances, status):
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
            if distance < SAFE_DISTANCE:
                radar[y][x] = 'X'  # Very close - danger
            elif distance < SLOW_DOWN_DISTANCE:
                radar[y][x] = 'o'  # Close - warning
            else:
                radar[y][x] = '*'  # Far - safe
    
    # Draw the radar
    for y in range(RADAR_HEIGHT):
        if y < height - 2:
            try:
                stdscr.addstr(y, 0, ''.join(radar[y])[:width-1])
            except curses.error:
                pass
    
    # Display status information
    if height > RADAR_HEIGHT + 5:
        stdscr.addstr(RADAR_HEIGHT + 1, 0, f"Status: {status}")
        stdscr.addstr(RADAR_HEIGHT + 2, 0, f"Safe Distance: {SAFE_DISTANCE}mm")
        stdscr.addstr(RADAR_HEIGHT + 3, 0, f"Press 'q' to quit")
    
    stdscr.refresh()

def obstacle_avoidance(angles, distances):
    """Implement obstacle avoidance logic"""
    front_obstacle = False
    left_clear = True
    right_clear = True
    status = "No obstacles detected"
    
    # Check for obstacles in front
    for angle, distance in zip(angles, distances):
        # Convert angle to degrees and normalize to -180 to 180
        angle_deg = math.degrees(angle) % 360
        if angle_deg > 180:
            angle_deg -= 360
            
        # Check front sector (-30 to 30 degrees)
        if abs(angle_deg) < 30:
            if distance < SAFE_DISTANCE:
                front_obstacle = True
                status = "Obstacle detected in front!"
                
        # Check left sector (30 to 90 degrees)
        if 30 < angle_deg < 90:
            if distance < SAFE_DISTANCE:
                left_clear = False
                
        # Check right sector (-90 to -30 degrees)
        if -90 < angle_deg < -30:
            if distance < SAFE_DISTANCE:
                right_clear = False
    
    # Decision making for obstacle avoidance
    if front_obstacle:
        # Stop or slow down
        set_speed(20)
        
        # Decide which way to turn
        if left_clear and right_clear:
            # Both sides clear, turn to the side with more space
            set_steering(TURN_ANGLE)  # Default to right turn
            status = "Turning right to avoid obstacle"
        elif left_clear:
            set_steering(TURN_ANGLE)  # Turn right
            status = "Turning right to avoid obstacle"
        elif right_clear:
            set_steering(-TURN_ANGLE)  # Turn left
            status = "Turning left to avoid obstacle"
        else:
            # Both sides blocked, reverse
            set_speed(-30)
            set_steering(0)
            status = "Both sides blocked, reversing"
            time.sleep(1)
    else:
        # No obstacles, move forward
        set_speed(50)
        set_steering(0)
        status = "Moving forward"
    
    return status

# Cleanup function
def cleanup():
    set_speed(0)
    set_steering(0)
    time.sleep(1)
    servo_pwm.stop()
    esc_pwm.stop()
    GPIO.cleanup()
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
                        
                        # Process the data for obstacle avoidance
                        status = obstacle_avoidance(first, firstDist)
                        
                        # Draw the radar display
                        draw_radar(first, firstDist, status)
                
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