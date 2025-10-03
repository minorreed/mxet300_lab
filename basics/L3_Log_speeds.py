import L2_kinematics as km 
import numpy as np
import L1_log as log 
import time

while True:
    xdot = km.getMotion()[0]
    thetadot = km.getMotion()[1]
    LW = km.getPdCurrent()[0]
    RW = km.getPdCurrent()[1]
    
    log.tmpFile(xdot, "L5_xdot")
    log.tmpFile(thetadot, "L5_thetadot")

    log.tmpFile(LW, "L5_LeftWheelSpeed")
    log.tmpFile(RW, "L5_RightWheelSpeed")
    time.sleep(0.2)
    
