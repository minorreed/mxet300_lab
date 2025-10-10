import L1_log as log
import numpy as np
import time
import L2_vector as vector
import L1_lidar as Lid

# Instantiate the Lidar object
lidarsensor = Lid.Lidar()

# Connect to the Lidar device
lidarsensor.connect()

# Start the Lidar thread
processor = lidarsensor.run()
time.sleep(1)  # Allow some time for the thread to start

try:
    while True:
        
        scan = lidarsensor.get()
        if scan is None:
            continue  # skip if data not ready yet

        # Get nearest valid object (r, angle)
        nearest = vector.getNearest(scan)  # [distance, angle]
        distance = nearest[0]
        angle = nearest[1]

        # Convert to Cartesian coordinates (x, y)
        cart = vector.polar2cart(distance, angle)
        distx = cart[0]
        disty = cart[1]

        # Log the results
        log.tmpFile(distx, "L3_Lidar_Distance_X")
        log.tmpFile(disty, "L3_Lidar_Distance_Y")
        log.tmpFile(distance, "L3_Lidar_Distance")
        log.tmpFile(angle, "L3_Lidar_Angle")

        print(f"Nearest object: Distance={distance:.3f} m, Angle={angle:.2f}°, X={distx:.3f}, Y={disty:.3f}")

        time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopping Lidar...")
finally:
    lidarsensor.kill(processor)



