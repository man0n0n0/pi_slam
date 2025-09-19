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
    angle = max(-45, min(45, angle)) * -1 # clamp and revert

    # Convert angle to pulse width (1.0ms to 2.0ms)
    # Formula: pulse_width = 1.5ms + (angle/90) * 0.5ms
    pulse_width_ms = 1.5 + (angle / 45.0) * 0.5
    
    # Convert pulse width to duty cycle percentage
    # Duty cycle = (pulse_width / period) * 100
    # Period = 20ms for 50Hz
    duty_cycle = (pulse_width_ms / 20.0) * 100.0

    servo_pwm.ChangeDutyCycle(duty_cycle)


# ==============================
# Obstacle Avoidance Parameters
# ==============================
SLOW_DOWN_DISTANCE = 10  # in dm
SAFE_DISTANCE = 2    # in dm 

# Global variables for artifact filtering
front_distances = []
MIN_READINGS_FRONT = 10  # Minimum readings in front sector to be valid

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
def the_callback(angles, distances):
    global prevLine, front_distances
    
    # Obstacle avoidance logic
    front_obstacle_raw = False
    slow_down = False
    # left_clear = True
    # right_clear = True
    
    # Collect front sector distances for artifact filtering
    current_front_distances = []
    
    max_distance = 0

    # Check for obstacles
    for angle, distance in zip(angles, distances):
            
        # Front sector: 0° ± 30° (considering wraparound)
        # This covers [330°-360°] and [0°-30°] in degrees
        # Or [11π/6 - 2π] and [0 - π/6] in radians
        front_condition = (angle<= math.pi/6) or (angle >= 11*math.pi/6)
        
        if front_condition:
            current_front_distances.append(distance)

            if distance < SAFE_DISTANCE:
                front_obstacle_raw = True

            if distance < SLOW_DOWN_DISTANCE:
                slow_down = True

            # Direction determination: keep the angle of the most distant point
            if distance > max_distance:
                max_distance = distance

                if angle <= math.pi:
                    TURN_ANGLE = math.degrees(angle)
                else:
                    TURN_ANGLE = math.degrees(angle - 2*math.pi)  # Convert to negative for left side
        
    # Artifact filtering for front obstacle detection
    if front_obstacle_raw and len(current_front_distances) < MIN_READINGS_FRONT:
        # Not enough readings in front sector - likely artifact
        front_obstacle_raw = False
    
    # Store current front distances for potential future use
    front_distances = current_front_distances
    
    # Use filtered result directly (no smoothing)
    front_obstacle = front_obstacle_raw
    
    # Decision making
    if front_obstacle:
        print(f"front obstacle detected \n distance readings: {current_front_distances} \n min distance: {min(current_front_distances):.2f}m")
        set_speed(-20)  # move backward

    elif slow_down:
        print(f"far front_obstacle : slowing down to {TURN_ANGLE} deg")
        set_speed(0)  # move forward
        set_steering(TURN_ANGLE)

    else:
        print(f"no obstacle detected : turning to {TURN_ANGLE} deg")
        set_speed(0)  # move forward
        set_steering(TURN_ANGLE)

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
