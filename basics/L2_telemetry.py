import L1_ina as ina
import L1_log as log
import numpy as np 
import time

while True:
    BV = ina.readVolts()
    print("Voltage", BV)
    log.tmpFile(BV, "L1_ina")

    time.sleep(0.2)
