import time
import numpy as np
from L1_lidar import Lidar
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import Final_color_tracking as shooter


'''
STATES:
0) NAVIGATE: follow hallway, stay centered, look for doors
1) PAUSE_FOR_OBSTACLE: stop while something is in front, resume when clear
2) TURN_TO_DOOR: rotate toward detectd doorway (left or right)
3) ENGAGE_ROOM: face doorway, run shooter/camera
4) RETURN_TO_HALLWAY: rotate back to original heading, resume navigation
5) END_OF_HALL: dead end detected, stop and stay stopped
'''


# CONSTANTS

NUM_POINTS = 108 # L1_lidar.get() output size
MIN_VALID_DIST = 0.10 # m
MAX_VALID_DIST = 3.00 # m

# Angular sectors

LEFT_SECTOR_DEG = (60.0, 120.0)
RIGHT_SECTOR_DEG = (-120.0,-60.0)
FRONT_SECTOR_DEG = (-20.0,20.0)

# Hallway-centering control

FORWARD_SPEED = 0.20 # m/s
KP_WALL = 1.0 # gain
MAX_TURN_RATE = 1.0 # rad/s

# Moving obstacle stopping logic

OBSTACLE_STOP_DIST = 0.60 # m: if anything this close in front, stop
OBSTACLE_RESUME_DIST = 0.80 # m: resume when farhter than this

# Doorway detection parameters

BASELINE_ALPHA = 0.05  # was 0.1, slower baseline so gaps stand out longer
DOOR_DELTA = 0.20      # was 0.40 m: side distance must jump this much above reference
DOOR_MIN_RANGE = 0.80  # was 1.20 m: side distance must be at least this to be considered door

# Extra: drive a bit past the gap before turning
DOOR_ADVANCE_TIME = 2.8   # seconds to continue NAVIGATE after first detecting door

# Turning to/from doorway

TURN_RATE = 1.3 # rad/s
DESIRED_ANGLE = np.pi/4 # 90 degree turn
DOOR_TURN_DURATION = DESIRED_ANGLE / TURN_RATE
RETURN_TURN_DURATION = DESIRED_ANGLE / TURN_RATE # time to swing back
LOOP_DT = 0.1 # control loop period

# End of hallway detection

HALL_END_FRONT_DIST = 1.0 # m: front must be closer than this
HALL_END_SIDE_DIST = 1.0 # m: both sides must be closer than this
HALL_END_CONFIRM_LOOPS = 15

# States

NAVIGATE = 0
PAUSE_FOR_OBSTACLE = 1
TURN_TO_DOOR = 2
ENGAGE_ROOM = 3
RETURN_TO_HALLWAY = 4
END_OF_HALL = 5


# Functions

def sector_average(distances, angles, sector):
    # Return average distance within angular sector
    a_min, a_max = sector

    # Angles have to be within range
    mask = (angles >= a_min) & (angles <= a_max)
    if not np.any(mask):
        return None
   
    d = distances[mask]

    # Only count distances within range
    valid = (d>MIN_VALID_DIST) & (d<MAX_VALID_DIST)

    # Update d
    d = d[valid]
    if d.size == 0:
        return None
   
    # Return average distance
    return float(np.mean(d))


def center(left_d, right_d, front_d):

    # If front obstacle too close -> stop
    if (front_d is not None) and (front_d < OBSTACLE_STOP_DIST):
        return 0.0, 0.0
   
    x_dot = FORWARD_SPEED
    theta_dot = 0.0

    # If both walls visible -> use difference to find turn rate
    if (left_d is not None) and (right_d is not None):
        error = right_d - left_d
        theta_dot = -KP_WALL * error
       
        # -MAX_TURN_RATE  <=  theta_dot  <=  MAX_TURN_RATE to prevent too large error
        if theta_dot > MAX_TURN_RATE:
            theta_dot = MAX_TURN_RATE
        elif theta_dot < -MAX_TURN_RATE:
            theta_dot = -MAX_TURN_RATE

    return x_dot, theta_dot


def detect_door(left_d, right_d, base_left, base_right):

    # If the doorway is on the left, return L
    if (left_d is not None) and (base_left is not None):
        delta_L = left_d - base_left
        print(f"[DOOR DEBUG] Left_d={left_d:.2f}, base_left={base_left:.2f}, delta_L={delta_L:.2f}")
        if (delta_L > DOOR_DELTA) and (left_d > DOOR_MIN_RANGE):
            return 'L'
   
    # If the doorway is on the right, return R
    if (right_d is not None) and (base_right is not None):
        delta_R = right_d - base_right
        print(f"[DOOR DEBUG] Right_d={right_d:.2f}, base_right={base_right:.2f}, delta_R={delta_R:.2f}")
        if (delta_R > DOOR_DELTA) and (right_d > DOOR_MIN_RANGE):
            return 'R'
       
    return None


def engage_targets():
    

    time.sleep(5.0)
    print("Finished engagement.")


def main():
    # Initialize LIDAR
    lidar = Lidar()
    lidar.connect()
    lidar_thread = lidar.run()
    time.sleep(0.5)

    # Baseline distances to walls
    baseline_left = None
    baseline_right = None

    # State machine variables
    state = NAVIGATE # basically the default state
    door_side = None # Can be L or R
    state_start_time = time.time()
    engaged_once = False

    # Remember when we first saw a door, so we can advance a bit
    door_detected_time = None

    # Counter for detecting end of hallway
    hall_end_counter = 0

    try:
        while True:
            scan = lidar.get(NUM_POINTS)
            if scan is None:
                time.sleep(LOOP_DT)
                continue

            distances = scan[:,0]
            angles = scan[:,1]

            # Defining angle sectors for sides
            left_d = sector_average(distances, angles, LEFT_SECTOR_DEG)
            right_d = sector_average(distances, angles, RIGHT_SECTOR_DEG)
            front_d = sector_average(distances, angles, FRONT_SECTOR_DEG)

            # End of hallway detection while navigating
            if state in (NAVIGATE, RETURN_TO_HALLWAY, TURN_TO_DOOR):
                if (front_d != None) and (front_d < OBSTACLE_STOP_DIST):
                    state = PAUSE_FOR_OBSTACLE
                    state_start_time = time.time()
                    print("Entering PAUSE_FOR_OBSTACLE")

            # Define PAUSE_FOR_OBSTACLE
            if state == PAUSE_FOR_OBSTACLE:
                x_dot, theta_dot = 0.0, 0.0
                if (front_d is None) or (front_d > OBSTACLE_RESUME_DIST):
                    state = NAVIGATE
                    state_start_time = time.time()
                    print("Obstacle cleared, resuming NAVIGATE")

            # Define NAVIGATE

            elif state == NAVIGATE:

                # Update baselines for hallway walls when both sides look reasonable
                # These baselines are used later for doorway detection
                if (left_d is not None) and (right_d is not None):
                    if baseline_left is None:
                        baseline_left = left_d
                        baseline_right = right_d

                    else:
                        # new_baseline = (1 - alpha) * old_baseline + alpha * new_measurement (used for noisy data)
                        baseline_left = (1.0 - BASELINE_ALPHA) * baseline_left + BASELINE_ALPHA * left_d
                        baseline_right = (1.0 - BASELINE_ALPHA) * baseline_right + BASELINE_ALPHA * right_d

                # If we already decided we're at the end, don't look for doors
                if state == END_OF_HALL:
                    x_dot, theta_dot = 0.0, 0.0
                else:
                    # Detect doorway (only for first detection)
                    door = detect_door(left_d, right_d, baseline_left, baseline_right)

                    print(f"[STATE NAVIGATE] Door check -> {door}, "
                          f"L={left_d}, R={right_d}, base_L={baseline_left}, base_R={baseline_right}, "
                          f"door_detected_time={door_detected_time}")

                    # If we see a door for the first time, start the advance timer
                    if (door is not None) and (door_detected_time is None):
                        door_side = door
                        door_detected_time = time.time()
                        print(f"Door detected on {door_side}, starting DOOR_ADVANCE_TIME timer.")

                    # If we already saw a door and have advanced long enough, now turn
                    if (door_detected_time is not None) and \
                       ((time.time() - door_detected_time) >= DOOR_ADVANCE_TIME):
                        state = TURN_TO_DOOR
                        state_start_time = time.time()
                        print(f"DOOR_ADVANCE_TIME elapsed, entering TURN_TO_DOOR towards {door_side}")
                        # Reset timer so the next door in the hallway can be detected later
                        door_detected_time = None
                        x_dot, theta_dot = 0.0, 0.0
                    else:
                        # Normal centering behavior while we advance
                        x_dot, theta_dot = center(left_d, right_d, front_d)

            # Define TURN_TO_DOOR
            elif state == TURN_TO_DOOR:
                x_dot = 0.0

                # Door on left side -> turn left
                if door_side == 'L':
                    theta_dot = TURN_RATE
               
                # Door on right side -> turn right
                else:
                    theta_dot = -TURN_RATE

                # Calculate how long we've been in TURN_TO_DOOR state
                if time.time() - state_start_time >= DOOR_TURN_DURATION:
                    state = ENGAGE_ROOM
                    engaged_once = False # lets you know you haven't run the shooter code yet
                    state_start_time = time.time()
                    print("Completed turn toward door, entering ENGAGE_ROOM")

            # Define ENGAGE_ROOM
            elif state == ENGAGE_ROOM:
                x_dot, theta_dot = 0.0, 0.0 # completely stopped
                sc.driveOpenLoop(np.array([0.0,0.0]))

                shooter.engage()
                state = RETURN_TO_HALLWAY
                state_start_time = time.time()
                #if not engaged_once: # shooter code hasn't run yet
                    #engage_targets() # Ricky's code activates
                    #engaged_once = True  # fixed name
                    #state = RETURN_TO_HALLWAY
                    #state_start_time = time.time()
                    #print("Entering RETURN_TO_HALLWAY")
                   
            # Define RETURN_TO_HALLWAY
            elif state == RETURN_TO_HALLWAY:
                x_dot = 0.0

                # Basically reversing back into original position
                if door_side == 'L':
                    theta_dot = -TURN_RATE   # fixed: assignment, not comparison
                else:
                    theta_dot = TURN_RATE    # fixed: assignment, not comparison

                if time.time() - state_start_time >= RETURN_TURN_DURATION:
                    state = NAVIGATE
                    state_start_time = time.time()
                    print("Back to hallway heading, entering NAVIGATE")
               
            # Define END_OF_HALL
            elif state == END_OF_HALL:
                # Hard stop at the end of hallway
                x_dot, theta_dot = 0.0, 0.0

            else:
                x_dot, theta_dot = 0.0, 0.0

            # Speed commands from inverse kinematics
            pd_targets = ik.getPdTargets(np.array([x_dot,theta_dot]))
            sc.driveOpenLoop(pd_targets)

            # Debug prints
            print(f"State:{state}  L:{left_d}  R:{right_d}  F:{front_d}  "
                  f"xd:{x_dot:.2f}  td:{theta_dot:.2f}  "
                  f"pdl:{pd_targets[0]:.2f}  pdr:{pd_targets[1]:.2f}")
           
            time.sleep(LOOP_DT)
   
    except KeyboardInterrupt:
        print("Stopping navigation...")
   
    finally:
        sc.driveOpenLoop(np.array([0.0,0.0]))
        lidar.kill(lidar_thread)


if __name__ == "__main__":
    main()


