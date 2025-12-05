# L3_color_tracking.py
# This program was designed to have SCUTTLE following a target using a USB camera input

import cv2              # For image capture and processing
import numpy as np
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import netifaces as ni
from time import sleep
from math import radians, pi

# ---- Shooter motor + servo imports ----
import gpiozero
from gpiozero import PWMOutputDevice as pwm
from gpiozero import Servo
import time

# Gets IP to grab MJPG stream
def getIp():
    for interface in ni.interfaces()[1:]:   #For interfaces eth0 and wlan0
        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip
        except KeyError:                    #We get a KeyError if the interface does not have the info
            continue                        #Try the next interface since this one has no IPv4
    return 0

# Camera
stream_ip = getIp()
if not stream_ip:
    print("Failed to get IP for camera stream")
    exit()

camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

size_w  = 240   # Resized image width. This is the image width in pixels.
size_h = 160    # Resized image height. This is the image height in pixels.

fov = 1         # Camera field of view in rad (estimate)

# ---------- Red HSV ----------
R1_min = 0      # Minimum H value
R2_min = 85     # Minimum S value
R3_min = 115    # Minimum V value

R1_max = 30     # Maximum H value
R2_max = 255    # Maximum S value
R3_max = 255    # Maximum V value

# ---------- Green HSV ----------
G1_min = 0
G2_min = 110
G3_min = 50

G1_max = 255
G2_max = 255
G3_max = 100
# --------------------------------------

target_width = 100      # Target pixel width of tracked object
angle_margin = 0.2      # Radians object can be from image center to be considered "centered"
width_margin = 10       # Minimum width error to drive forward/back

# ---------- Shooter motor setup (pins 35,36,37,38) ----------
# Using BCM numbering:
# Physical 35 -> GPIO19
# Physical 36 -> GPIO16
# Physical 37 -> GPIO26
# Physical 38 -> GPIO20
frq = 150

shoot1A = pwm(19, frequency=frq, initial_value=0)   # Pin 35
shoot1B = pwm(16, frequency=frq, initial_value=0)   # Pin 36
shoot2A = pwm(26, frequency=frq, initial_value=0)   # Pin 37
shoot2B = pwm(20, frequency=frq, initial_value=0)   # Pin 38

def computePWM(speed):   # speed in [-1,1]
    if speed == 0:
        return np.array([0, 0])
    x = speed + 1.0              # [0,2]
    chA = 0.5 * x                # [0,1]
    chB = 1 - 0.5 * x            # [1,0]
    return np.round(np.array([chA, chB]), 2)

def sendShooter(speed):
    """Drive both shooter motors with signed speed in [-1,1]."""
    shoot1A.value = 0.0
    shoot1B.value = 1.0
    shoot2A.value = 0.0
    shoot2B.value = 1.0

# ---------- Servo setup ----------
SERVO_PIN = 13
servo = Servo(SERVO_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

# Positions (tune these if needed)
SERVO_LOAD = 1.0   # ball waiting position
SERVO_FIRE = -0.3    # pushes ball into flywheel

def servo_load():
    servo.value = SERVO_LOAD
    time.sleep(0.3)

def servo_fire():
    # push ball
    servo.value = SERVO_FIRE
    time.sleep(0.3)
    # return to load position
    servo_load()

# minimum color pixels inside the bounding box to consider it that color
COLOR_AREA_MIN = 100

def main():
    # move servo to load at start
    servo_load()

    # Try opening camera with default method
    try:
        camera = cv2.VideoCapture(0)
    except:
        camera = None

    # Try opening camera stream if default method failed
    if (camera is None) or (not camera.isOpened()):
        camera = cv2.VideoCapture(camera_input)

    camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    camera.set(4, size_h)                       # Set height of images that will be retrived from camera

    try:
        while True:
            sleep(.05)

            ret, image = camera.read()  # Get image from camera

            # Make sure image was grabbed
            if not ret:
                print("Failed to retrieve image!")
                break

            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)              # Convert image to HSV

            height, width, channels = image.shape                       # Get shape of image

            # --------- Two color thresholds ----------
            threshred = cv2.inRange(image,
                                     (R1_min, R2_min, R3_min),
                                     (R1_max, R2_max, R3_max))
            threshgreen = cv2.inRange(image,
                                       (G1_min, G2_min, G3_min),
                                       (G1_max, G2_max, G3_max))
            # Combined for tracking (same as before)
            thresh  = cv2.bitwise_or(threshred, threshgreen)
            # ----------------------------------------

            kernel = np.ones((5,5),np.uint8)                            # Set kernel size

            # Clean masks
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            red_mask = cv2.morphologyEx(threshred, cv2.MORPH_OPEN, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

            green_mask = cv2.morphologyEx(threshgreen, cv2.MORPH_OPEN, kernel)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image

            if len(cnts) > 0:                             # If more than 0 closed shapes exist

                c = max(cnts, key=cv2.contourArea)                      # return the largest target area
                x,y,w,h = cv2.boundingRect(c)                           # Get bounding rectangle (x,y,w,h) of the largest contour
                center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                wheel_measured = kin.getPdCurrent()                     # Wheel speed measurements

                # If robot is facing target
                if abs(angle) < angle_margin:
                    e_width = target_width - w                          # Find error in target width and measured width

                    # If error width is within acceptable margin -> ALIGNED
                    if abs(e_width) < width_margin:
                        sc.driveOpenLoop(np.array([0.,0.]))             # Stop when centered and aligned

                        # --- Determine color of aligned target ---
                        roi_red   = red_mask[y:y+h,   x:x+w]
                        roi_green = green_mask[y:y+h, x:x+w]

                        red_pixels   = cv2.countNonZero(roi_red)
                        green_pixels = cv2.countNonZero(roi_green)

                        print("Aligned! width =", w,
                              "| red_px =", red_pixels,
                              "| green_px =", green_pixels)

                        # Logic: if green -> do NOT shoot; if red -> shoot with servo
                        if green_pixels > COLOR_AREA_MIN and green_pixels >= red_pixels:
                            # green dominant -> no shooting
                            sendShooter(0)
                            print("Aligned on GREEN -> DO NOT SHOOT")

                        elif red_pixels > COLOR_AREA_MIN and red_pixels > green_pixels:
                            # red dominant -> spin flywheel and fire one ball
                            print("Aligned on RED -> SPIN + FIRE")
                            servo_load()
                            sendShooter(1.0)          # spin shooter motors
                            time.sleep(0.5)           # spin-up time (tune if needed)
                            servo_fire()              # push ball into wheel

                        else:
                            # unknown / not enough color pixels -> safe = no shoot
                            sendShooter(0)
                            print("Aligned but color unclear -> DO NOT SHOOT")

                        continue

                    # Not at correct distance yet -> drive, BUT DON'T SHOOT
                    sendShooter(0)

                    fwd_effort = e_width/target_width

                    wheel_speed = ik.getPdTargets(np.array([0.8*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                    sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                    print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\\R: ", *wheel_measured)
                    continue

                # Not facing target -> turning only, shooter OFF
                sendShooter(0)

                wheel_speed = ik.getPdTargets(np.array([0, -1.1*angle]))    # Find wheel speeds for only turning

                sc.driveClosedLoop(wheel_speed, wheel_measured, 0)          # Drive robot
                print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\\R: ", *wheel_measured)

            else:
                print("No targets")
                sc.driveOpenLoop(np.array([0.,0.]))         # stop if no targets detected
                sendShooter(0)                              # make sure shooter is off

    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        pass

    finally:
        print("Exiting Color Tracking.")
        sendShooter(0)   # ensure shooter is off on exit
        servo_load()     # park servo in load position

if __name__ == '__main__':
    main()
