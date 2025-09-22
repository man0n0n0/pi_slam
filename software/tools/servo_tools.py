import serial
import math
import time
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO

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


# ==============================
# Motor Control Functions
# ==============================

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

    
# =============================================
# for v in range (0,-50,-1):
#     print(v)
#     set_speed(v)
#     time.sleep(0.2)

for v in range (-45,45):
    print(v)
    set_steering(v)
    time.sleep(0.2)


