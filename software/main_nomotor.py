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
SAFE_DISTANCE = 1.0      # in dm (0.1 meter) 
SLOW_DOWN_DISTANCE = 1.5  # in dm (0.15 meters)
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
# Global variables for smoothing and artifact filtering
# Global variables for artifact filtering
front_distances = []
MIN_READINGS_FRONT = 3  # Minimum readings in front sector to be valid
MAX_DISTANCE_VARIANCE = 0.5  # Maximum allowed variance in distance readings (meters)

def the_callback(angles, distances):
    global prevLine, front_distances
    
    # Obstacle avoidance logic
    front_obstacle_raw = False
    left_clear = True
    right_clear = True
    
    # Collect front sector distances for artifact filtering
    current_front_distances = []
    
    # Check for obstacles
    for angle, distance in zip(angles, distances):
        # Normalize angle to [-π, π] range
        angle_norm = angle % (2 * math.pi)
        if angle_norm > math.pi:
            angle_norm -= 2 * math.pi
        
        # Front sector (-π/6 to π/6 radians, i.e., -30 to 30 degrees)
        if abs(angle_norm) < math.pi/6:
            current_front_distances.append(distance)
            if distance < SAFE_DISTANCE:
                front_obstacle_raw = True
        
        # Left sector (π/6 to π/2 radians, i.e., 30 to 90 degrees)
        if math.pi/6 < angle_norm < math.pi/2 and distance < SAFE_DISTANCE:
            left_clear = False
        
        # Right sector (-π/2 to -π/6 radians, i.e., -90 to -30 degrees)
        if -math.pi/2 < angle_norm < -math.pi/6 and distance < SAFE_DISTANCE:
            right_clear = False
    
    # Artifact filtering for front obstacle detection
    if front_obstacle_raw and len(current_front_distances) >= MIN_READINGS_FRONT:
        # Check for consistent readings (filter out single-point artifacts)
        min_dist = min(current_front_distances)
        max_dist = max(current_front_distances)
        distance_variance = max_dist - min_dist
        
        # Filter out if readings are too inconsistent (likely artifacts)
        if distance_variance > MAX_DISTANCE_VARIANCE:
            front_obstacle_raw = False
            print(f"Artifact filtered: distance variance {distance_variance:.2f}m too high")
        
        # Additional check: require multiple close readings
        close_readings = sum(1 for d in current_front_distances if d < SAFE_DISTANCE)
        if close_readings < 2:  # Need at least 2 close readings
            front_obstacle_raw = False
            print(f"Artifact filtered: only {close_readings} close readings")
    
    elif front_obstacle_raw and len(current_front_distances) < MIN_READINGS_FRONT:
        # Not enough readings in front sector - likely artifact
        front_obstacle_raw = False
        print(f"Artifact filtered: insufficient front readings ({len(current_front_distances)})")
    
    # Store current front distances for potential future use
    front_distances = current_front_distances
    
    # Use filtered result directly (no smoothing)
    front_obstacle = front_obstacle_raw
    
    # Decision making
    if front_obstacle:
        set_speed(0)  # slow down
        print(f"front obstacle detected \n distance readings: {len(current_front_distances)} \n min distance: {min(current_front_distances):.2f}m")
        
        if left_clear and right_clear:
            set_steering(TURN_ANGLE)  # default right
        elif left_clear:
            set_steering(TURN_ANGLE)
        elif right_clear:
            set_steering(-TURN_ANGLE)
        else:
            set_speed(0)  # reverse
            set_steering(0)
            #time.sleep(1)
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
                    distances = distances[split[0] + 1 :]
                    the_callback(first, firstDist)

                tmpString = ""

            else:
                tmpString += b.hex() + " "

            flag2c = False

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    cleanup()
