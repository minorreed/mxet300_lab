import L2_compass_heading as cmph
import L1_log as log
import numpy as np 
import time




def cardinal_direction (angle):
    a = angle
    if -22.5 <= a < 22.5:
        print("North")
        return "North"
    elif 22.5 <= a < 67.5:
        print("NorthEast")
        return "NorthEast"
    elif 67.5 <= a < 112.5:
        print("East")
        return "East"
    elif 112.5 <= a < 157.5:
        print("SouthEast")
        return "SouthEast"
    elif a >= 157.5 or a < -157.5:  # includes 180 and -180
        print("South")
        return "South"
    elif -157.5 <= a < -112.5:
        print("SouthWest")
        return "SouthWest"
    elif -112.5 <= a < -67.5:
        print("West")
        return "West"
    else:  # -67.5 <= a < -22.5
        print("Northwest")
        return "NorthWest"

def to_360(theta):
    return (theta + 360) % 360

while True:
    angle = cmph.get_heading()
    angle_360 = to_360(angle)
    print("Heading: {:6.3f}".format(angle)) 

    log.tmpFile(angle_360, "L2_compass_heading")
    cardir = cardinal_direction(angle)
    log.stringTmpFile(cardir, "L2_cardinal_direction")


    time.sleep(0.2)