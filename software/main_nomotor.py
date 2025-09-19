import serial
import math
import time
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
from CalcLidarData import CalcLidarData


# ==============================
# GPIO Setup
# ==============================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin definitions
SERVO_PIN = 22   # GPIO pin for servo motor
ESC_PIN = 23     # GPIO pin for ESC (Electronic Speed Controller)

# Setup PWM pins
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(ESC_PIN, GPIO.OUT)

# Initialize PWM
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for servo
esc_pwm = GPIO.PWM(ESC_PIN, 50)      # 50Hz for ESC

# Start PWM with neutral signals
servo_pwm.start(7.5)  # Neutral position for servo (90 degrees)
esc_pwm.start(7.5)    # Neutral position for ESC (stop)
esc_pwm.ChangeDutyCycle(5.0)    # Send minimum throttle
time.sleep(2)
esc_pwm.ChangeDutyCycle(7.5)    # Move to neutral
time.sleep(2)


# ==============================
# Motor Control Functions
# ==============================
def set_speed(speed_percent: float):
    """
    Set robot speed using PWM ESC.
    :param speed_percent: -100 (full reverse) to 100 (full forward)
    """
    speed_percent = max(-100, min(100, speed_percent))  # clamp

    # Convert percentage to duty cycle (typically 5-10% for ESCs)
    # Neutral at 7.5%, full forward at 10%, full reverse at 5%
    duty_cycle = 7.5 + (speed_percent / 100) * 2.5
    esc_pwm.ChangeDutyCycle(duty_cycle)


def set_steering(angle: float):
    """
    Set steering angle using servo motor.
    :param angle: -90 (full left) to 90 (full right)
    """
    angle = max(-90, min(90, angle))  # clamp

    # Convert angle to duty cycle (typically 5-10% for servos)
    # 0 degrees at 7.5%, -90 at 5%, 90 at 10%
    duty_cycle = 7.5 + (angle / 90) * 2.5
    servo_pwm.ChangeDutyCycle(duty_cycle)


# ==============================
# Obstacle Avoidance Parameters
# ==============================
SAFE_DISTANCE = 0.10      # in m (0.1 meter) 
SLOW_DOWN_DISTANCE = 0.15  # in m (0.15 meters)
TURN_ANGLE = 45            # degrees to turn when avoiding obstacles

# ==============================
# Serial Connection to LiDAR
# ==============================
ser = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=230400,
    timeout=5.0,
    bytesize=8,
    parity="N",
    stopbits=1,
)


tmpString = ""
angles = []
distances = []
prevLine = None


# ==============================
# Callback for Processing LiDAR Data
# ==============================
# Global variables for smoothing
obstacle_history = []
HISTORY_SIZE = 5  # Number of recent readings to consider
OBSTACLE_THRESHOLD = 0.6  # 60% of readings must detect obstacle

def the_callback(angles, distances):
    global prevLine, obstacle_history
    
    # Obstacle avoidance logic
    front_obstacle_raw = False
    left_clear = True
    right_clear = True
    
    # Check for obstacles
    for angle, distance in zip(angles, distances):
        angle_deg = math.degrees(angle) % 360
        if angle_deg > 180:
            angle_deg -= 360
        
        # Front sector (-30 to 30 degrees)
        if abs(angle_deg) < 30 and distance < SAFE_DISTANCE:
            front_obstacle_raw = True
        
        # Left sector (30 to 90 degrees)
        if 30 < angle_deg < 90 and distance < SAFE_DISTANCE:
            left_clear = False
        
        # Right sector (-90 to -30 degrees)
        if -90 < angle_deg < -30 and distance < SAFE_DISTANCE:
            right_clear = False
    
    # Add current reading to history and maintain size
    obstacle_history.append(front_obstacle_raw)
    if len(obstacle_history) > HISTORY_SIZE:
        obstacle_history.pop(0)
    
    # Apply smoothing filter - obstacle confirmed if threshold percentage of recent readings detect it
    obstacle_ratio = sum(obstacle_history) / len(obstacle_history)
    front_obstacle = obstacle_ratio >= OBSTACLE_THRESHOLD
    
    # Decision making
    if front_obstacle:
        set_speed(0)  # slow down
        print(f"front obstacle (smoothed: {obstacle_ratio:.2f}) \n angle : {angle_deg} \n distance : {distance}")
        
        if left_clear and right_clear:
            set_steering(TURN_ANGLE)  # default right
        elif left_clear:
            set_steering(TURN_ANGLE)
        elif right_clear:
            set_steering(-TURN_ANGLE)
        else:
            set_speed(0)  # reverse
            set_steering(0)
            time.sleep(1)
    else:
        set_speed(0)  # move forward
        set_steering(0)

# ==============================
# Cleanup Function
# ==============================
def cleanup():
    set_speed(0)
    set_steering(0)
    time.sleep(1)
    servo_pwm.stop()
    esc_pwm.stop()
    GPIO.cleanup()
    ser.close()


# ==============================
# Main Loop
# ==============================
try:
    while True:
        all_b = ser.read_all()
        for b in all_b:
            tmpInt = int(b)
            b = bytes([b])

            if tmpInt == 0x54:
                tmpString += b.hex() + " "
                flag2c = True
                continue

            elif tmpInt == 0x2C and flag2c:
                tmpString += b.hex()
                if not len(tmpString[0:-5].replace(" ", "")) == 90:
                    tmpString = ""
                    loopFlag = False
                    flag2c = False
                    continue

                lidarData = CalcLidarData(tmpString[0:-5])
                angles.extend(lidarData.Angle_i)
                distances.extend(lidarData.Distance_i)

                if len(angles) > 50 * 12:
                    split = [i for i in range(len(angles) - 1) if angles[i + 1] < angles[i]]
                    first = angles[: split[0] + 1]
                    firstDist = distances[: split[0] + 1]
                    angles = angles[split[0] + 1 :]
                    distances = distances[split[0] + 1 :]/10
                    the_callback(first, firstDist)

                tmpString = ""

            else:
                tmpString += b.hex() + " "

            flag2c = False

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    cleanup()
