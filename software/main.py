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
SAFE_DISTANCE = 5    # in dm 
MAX_MESURED_DISTANCE = 40   # in dm 

K_SPEED = 30 # proportional quotient (speed for 1m distance object)

# Global variables for artifact filtering
MIN_READINGS_FRONT = 5  # Minimum readings in front sector to be valid

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
    global prevLine
    
    # Obstacle avoidance logic
    front_obstacle_raw = False
    
    # Collect front sector distances for artifact filtering
    FRONT_READINGS = []

    # Local variable init
    MAX_DISTANCE = 0
    TURN_ANGLE = 0


    print(zip(angles, distances))
    
    # Check for obstacles
    for angle, distance in zip(angles, distances):
            
        # Front sector: 0° ± 30° (considering wraparound) so [11π/6 - 2π] and [0 - π/6] in radians
        front_condition = (angle<= math.pi/6) or (angle >= 11*math.pi/6)
        
        if front_condition and distance < MAX_MESURED_DISTANCE:

            # Safety mesure : emergency stop with safe distance
            if distance < SAFE_DISTANCE :
                FRONT_READINGS += 1
                if FRONT_READINGS >= MIN_READINGS_FRONT :
                    front_obstacle_raw = True
                    break

            # Direction determination: keep the angle of the most distant point
            if distance > MAX_DISTANCE :
                MAX_DISTANCE = distance
                if angle <= math.pi:
                    current_angle = math.degrees(angle)
                else:
                    current_angle = math.degrees(angle - 2*math.pi)  # Convert to negative for left side
                TURN_ANGLE = TURN_ANGLE*0.7 + current_angle*0.3 # exponential filter to smooth direction
    
    # Decision making
    if front_obstacle_raw:
        set_speed(-20)  # move backward
        set_steering(TURN_ANGLE)

    else:
        #print(f"NO obstacle detected : {TURN_ANGLE} deg")
        #consider the proportionnal speed base on a k ration (speed at 10dm = 1m distance)
        set_speed(MAX_DISTANCE*K_SPEED/10) #speed depends of max distance
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
