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
# def set_speed(speed_percent: float):
#     """
#     Set robot speed using PWM ESC.
#     :param speed_percent: -100 (full reverse) to 100 (full forward)
#     """
#     speed_percent = max(-100, min(100, speed_percent))  # clamp

#     # Convert percentage to duty cycle (typically 5-10% for ESCs)
#     # Neutral at 7.5%, full forward at 10%, full reverse at 5%
#     duty_cycle = 7.5 + (speed_percent / 100) * 2.5
#     esc_pwm.ChangeDutyCycle(duty_cycle)

current_speed = 0
speed_change_time = 0
MIN_SPEED_INTERVAL = 0.1  # Minimum time between speed changes

def set_speed(speed_percent: float):
    """
    Improved speed setting with rate limiting and proper reverse handling
    """
    global current_speed, speed_change_time
    
    current_time = time.time()
    
    # Rate limiting - don't change speed too frequently
    if current_time - speed_change_time < MIN_SPEED_INTERVAL:
        return
    
    speed_percent = max(-100, min(100, speed_percent))  # clamp
    
    # If changing direction (crossing zero), pause at neutral first
    if (current_speed > 0 and speed_percent < 0) or (current_speed < 0 and speed_percent > 0):
        print(f"Direction change: {current_speed} -> {speed_percent}, pausing at neutral")
        esc_pwm.ChangeDutyCycle(7.5)  # Neutral
        time.sleep(0.2)  # Brief pause for ESC to register
    
    # Convert percentage to duty cycle
    duty_cycle = 7.5 + (speed_percent / 100) * 2.5
    esc_pwm.ChangeDutyCycle(duty_cycle)
    
    current_speed = speed_percent
    speed_change_time = current_time
    
    print(f"Speed set to: {speed_percent}% (duty: {duty_cycle}%)")

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
SAFE_DISTANCE = 5   # in dm 
K_SPEED = 30 # max speed for exponential function
BACKWARD_SPEED = -20
STEEPNESS_SPEED = 10 # Smaller steepness (e.g., 5) = faster acceleration, reaches max speed sooner // larger steepness gentler acceleration, more gradual speed increase
MIN_READINGS_FRONT = 30  # Minimum readings in front sector to be valid ([# )Global variables for artifact filtering)

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
TURN_ANGLE = 0
prevLine = None

# ==============================
# Callback for Processing LiDAR Data
# ==============================
def the_callback(angles, distances):
    global prevLine, TURN_ANGLE
    # Local variable init
    MAX_DISTANCE = 0
    FRONT_READINGS = 0    # Collect front sector distances for artifact filtering
    FRONT_OBJECT = False
    current_steering_angle = None

    #bundaries = [math.pi/6,11*math.pi/6] # +- 30deg to avoid computation in the iterative loop
    boundaries = [math.pi/4, 7*math.pi/4]  # ±45° (45° and 315°)

    # Check for obstacles
    for angle, distance in zip(angles, distances):
            
        # Front sector:        
        if (angle<= boundaries[0]) or (angle >= boundaries[1]):

            # Safety mesure : emergency stop with safe distance
            if distance < SAFE_DISTANCE :
                FRONT_READINGS += 1 #to avoid artifacts
                if FRONT_READINGS >= MIN_READINGS_FRONT :
                    FRONT_OBJECT = True

            # Direction determination: keep the angle of the most distant point
            if distance > MAX_DISTANCE :
                MAX_DISTANCE = distance
                current_steering_angle = angle

    # Set direction
    if current_steering_angle is not None:

        # convert rad to degree
        if current_steering_angle <= math.pi:
            steering_degrees = math.degrees(current_steering_angle)
        else:
            steering_degrees = math.degrees(current_steering_angle - 2*math.pi)
        
        # exponential filter to smooth direction
        TURN_ANGLE = TURN_ANGLE*0.7 + steering_degrees*0.3

    set_steering(TURN_ANGLE)

    if FRONT_OBJECT:
        set_speed(BACKWARD_SPEED)
    else :
        # Speed based on a exponential functin that tend to max speed (k_value)
        set_speed(K_SPEED * (1 - math.exp(-MAX_DISTANCE/STEEPNESS_SPEED)))
        
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
