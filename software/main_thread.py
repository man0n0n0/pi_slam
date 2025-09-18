#!/usr/bin/env python3
"""
Multithreaded LiDAR reader + obstacle avoidance + plotting for Raspberry Pi.

Threads:
 - reader_thread: reads serial bytes, parses LiDAR frames using CalcLidarData,
                  pushes (angles, distances) snapshots into lidar_queue.
 - control_thread: consumes lidar snapshots, runs obstacle avoidance logic and
                   updates PWM (servo/esc). Also forwards data to plot_queue.
 - main thread: runs matplotlib plotting loop (required by many backends to be main thread).

Make sure CalcLidarData is in PYTHONPATH and returns Angle_i (rad) and Distance_i (same unit).
"""
import serial
import math
import time
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import threading
import queue
from collections import deque
from CalcLidarData import CalcLidarData

# ==============================
# Config / GPIO Setup
# ==============================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

SERVO_PIN = 22   # GPIO pin for servo motor
ESC_PIN = 23     # GPIO pin for ESC (Electronic Speed Controller)

GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(ESC_PIN, GPIO.OUT)

servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for servo
esc_pwm = GPIO.PWM(ESC_PIN, 50)      # 50Hz for ESC

# Start PWM (neutral)
servo_pwm.start(7.5)
esc_pwm.start(7.5)
time.sleep(0.5)
esc_pwm.ChangeDutyCycle(5.0)    # send minimum throttle for many ESCs
time.sleep(1)
esc_pwm.ChangeDutyCycle(7.5)    # neutral
time.sleep(1)

# Motor helpers
def set_speed(speed_percent: float):
    """
    Set robot speed using PWM ESC.
    :param speed_percent: -100 (full reverse) to 100 (full forward)
    """
    speed_percent = max(-100.0, min(100.0, float(speed_percent)))
    # Many ESCs: 5% => full reverse, 7.5% => neutral, 10% => full forward
    duty_cycle = 7.5 + (speed_percent / 100.0) * 2.5
    esc_pwm.ChangeDutyCycle(duty_cycle)

def set_steering(angle: float):
    """
    Set steering angle using servo motor.
    :param angle: -90 (full left) to 90 (full right)
    """
    angle = max(-90.0, min(90.0, float(angle)))
    duty_cycle = 7.5 + (angle / 90.0) * 2.5
    servo_pwm.ChangeDutyCycle(duty_cycle)

# ==============================
# Obstacle Avoidance Parameters (adjust units as needed)
# ==============================
# NOTE: check what unit CalcLidarData returns for Distance_i.
# If it's meters, use meters here. Your original used "mm" but values looked like 0.1, 0.15 which
# suggests meters; adapt as necessary.
SAFE_DISTANCE = 0.10       # meters (stop / consider obstacle)
SLOW_DOWN_DISTANCE = 0.15  # meters (begin slowing)
TURN_ANGLE = 45            # degrees to turn when avoiding obstacles

# ==============================
# Threading Queues & Control
# ==============================
lidar_queue = queue.Queue(maxsize=10)   # (angles, distances) snapshots from reader
plot_queue = queue.Queue(maxsize=2)     # latest snapshot for plotting
stop_event = threading.Event()          # signal threads to stop

# ==============================
# Matplotlib setup (main thread)
# ==============================
plt.ion()
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='polar')
ax.set_title("LiDAR (exit: Key 'E' or Ctrl-C)", fontsize=14)
prev_scatter = None
# ensure theta orientation same as before
ax.set_theta_offset(math.pi / 2)

# ==============================
# Serial connection
# ==============================
ser = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=230400,
    timeout=0.1,     # non-blocking-ish reads
    bytesize=8,
    parity="N",
    stopbits=1,
)

# ==============================
# Reader thread: read serial and build lidar frames using your pattern
# ==============================
def reader_thread_fn(serial_port, out_queue: queue.Queue):
    """
    Read bytes from serial, detect frames that start with 0x54 and have trailing 0x2C sequence
    according to your original parsing. When a full frame is detected, CalcLidarData is called
    and the resulting angles/distances are pushed to out_queue.
    """
    tmpString = ""
    flag2c = False
    try:
        while not stop_event.is_set():
            all_b = serial_port.read_all()
            if not all_b:
                # small sleep to avoid busy loop when no data
                time.sleep(0.01)
                continue

            for b in all_b:
                tmpInt = int(b)
                b_hex = bytes([b]).hex()

                if tmpInt == 0x54:
                    tmpString = b_hex + " "
                    flag2c = True
                    continue

                elif tmpInt == 0x2C and flag2c:
                    tmpString += b_hex
                    # check length / format - original checked specific length condition
                    # keep original semantics but guard against malformed frames
                    compact = tmpString[0:-5].replace(" ", "")
                    if not len(compact) == 90:
                        # malformed: reset and continue
                        tmpString = ""
                        flag2c = False
                        continue

                    # parse frame with CalcLidarData (may raise on bad data)
                    try:
                        lidarData = CalcLidarData(tmpString[0:-5])
                        angles = list(lidarData.Angle_i)     # radians (presumed)
                        distances = list(lidarData.Distance_i)  # meters? verify
                        # push to queue (non-blocking drop oldest if full)
                        try:
                            out_queue.put_nowait((angles, distances))
                        except queue.Full:
                            # drop oldest and put newest
                            try:
                                _ = out_queue.get_nowait()
                                out_queue.put_nowait((angles, distances))
                            except queue.Empty:
                                pass
                    except Exception as e:
                        print("CalcLidarData parse error:", e)
                    # reset
                    tmpString = ""
                    flag2c = False
                else:
                    tmpString += b_hex + " "
                    flag2c = False
    except Exception as e:
        print("Reader thread exception:", e)
    finally:
        # signal main to stop if reader dies unexpectedly
        stop_event.set()

# ==============================
# Control thread: process latest snapshots and drive motors + forward to plotting
# ==============================
def control_thread_fn(in_queue: queue.Queue, plot_queue: queue.Queue):
    """
    Consume lidar snapshots, run obstacle avoidance, update motors,
    and forward a copy of the snapshot to plot_queue.
    """
    try:
        while not stop_event.is_set():
            try:
                angles, distances = in_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            # Basic obstacle avoidance logic (based on your original)
            front_obstacle = False
            left_clear = True
            right_clear = True

            for angle, distance in zip(angles, distances):
                # angle: assumed radians (original used math.degrees(angle))
                angle_deg = math.degrees(angle) % 360
                if angle_deg > 180:
                    angle_deg -= 360

                # If distance is zero or None, skip
                if distance is None or distance <= 0:
                    continue

                if abs(angle_deg) < 30 and distance < SAFE_DISTANCE:
                    front_obstacle = True
                if 30 < angle_deg < 90 and distance < SAFE_DISTANCE:
                    left_clear = False
                if -90 < angle_deg < -30 and distance < SAFE_DISTANCE:
                    right_clear = False

            # Decision making
            if front_obstacle:
                # slow down
                set_speed(10)
                # choose turn based on availability
                if left_clear and not right_clear:
                    set_steering(TURN_ANGLE)
                elif right_clear and not left_clear:
                    set_steering(-TURN_ANGLE)
                elif left_clear and right_clear:
                    set_steering(TURN_ANGLE)  # default right
                else:
                    # stuck - reverse a bit
                    set_speed(-10)
                    set_steering(0)
                    time.sleep(1)
            else:
                # forward
                set_speed(10)
                set_steering(0)

            # forward to plot queue (non-blocking)
            try:
                plot_queue.put_nowait((angles, distances))
            except queue.Full:
                # drop the oldest then enqueue
                try:
                    _ = plot_queue.get_nowait()
                    plot_queue.put_nowait((angles, distances))
                except queue.Empty:
                    pass

    except Exception as e:
        print("Control thread exception:", e)
    finally:
        stop_event.set()

# ==============================
# Cleanup function
# ==============================
def cleanup():
    try:
        set_speed(0)
        set_steering(0)
        time.sleep(0.5)
    except Exception:
        pass
    try:
        servo_pwm.stop()
    except Exception:
        pass
    try:
        esc_pwm.stop()
    except Exception:
        pass
    try:
        GPIO.cleanup()
    except Exception:
        pass
    try:
        ser.close()
    except Exception:
        pass

# ==============================
# Start threads
# ==============================
reader_t = threading.Thread(target=reader_thread_fn, args=(ser, lidar_queue), daemon=True)
control_t = threading.Thread(target=control_thread_fn, args=(lidar_queue, plot_queue), daemon=True)

reader_t.start()
control_t.start()

# ==============================
# Main: plotting loop (must run in main thread for many backends)
# ==============================
try:
    print("Running. Press Ctrl-C to stop, or close the plot window.")
    while not stop_event.is_set():
        # check for keyboard event to exit (E key)
        # Matplotlib key event handling can be used, but here we'll also accept Ctrl-C
        try:
            snapshot = plot_queue.get(timeout=0.2)
        except queue.Empty:
            # if nothing to plot, continue loop and allow GUI events
            plt.pause(0.01)
            continue

        angles, distances = snapshot

        # Prepare polar plotting points: convert angles from radians to polar theta
        thetas = [ -a for a in angles ]  # your original inverted angles
        rs = distances

        # update scatter plot
        if prev_scatter is not None:
            try:
                prev_scatter.remove()
            except Exception:
                pass

        prev_scatter = ax.scatter(thetas, rs, s=6)
        ax.relim()
        ax.autoscale_view()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("Interrupted by user (Ctrl-C)")

finally:
    print("Stopping threads...")
    stop_event.set()
    # allow threads to finish
    reader_t.join(timeout=1.0)
    control_t.join(timeout=1.0)
    cleanup()
    print("Cleaned up. Exiting.")
