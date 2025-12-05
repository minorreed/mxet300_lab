# motor_channel_test.py
import time
from gpiozero import PWMOutputDevice as pwm
from gpiozero import Servo

frq = 150

# BCM pins for SHOOTER MOTORS (35,36,37,38)
shoot1A = pwm(19, frequency=frq, initial_value=0)   # IN1
shoot1B = pwm(16, frequency=frq, initial_value=0)   # IN2
shoot2A = pwm(26, frequency=frq, initial_value=0)   # IN3
shoot2B = pwm(20, frequency=frq, initial_value=0)   # IN4

def all_off():
    shoot1A.value = shoot1B.value = 0.0
    shoot2A.value = shoot2B.value = 0.0

# ---------- SERVO SETUP ----------
SERVO_PIN = 13   # CHANGE THIS IF YOUR SERVO SIGNAL IS ON ANOTHER PIN
servo = Servo(SERVO_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

# Servo positions
POS_A = -0.3
POS_B = 1.0

# move servo to starting position


print("Starting motor + servo test...\n")

try:
    # Test motors (same as before)
    print("Channel 1 (IN1/IN2) forward 3s...")
    shoot1A.value = 0.0
    shoot1B.value = 1.0
    shoot2A.value = 0.0
    shoot2B.value = 1.0
    time.sleep(3)
    servo.value = POS_A
    time.sleep(0.1)
    servo.value = POS_B
    time.sleep(3)
    all_off()
    time.sleep(1)
    

    print("Channel 2 (IN3/IN4) forward 3s...")
    shoot1A.value = 0.0
    shoot1B.value = 1.0
    shoot2A.value = 0.0
    shoot2B.value = 1.0
    time.sleep(0.5)
    servo.value = POS_A
    time.sleep(3)
    servo.value = POS_B
    time.sleep(0.5)
    all_off()


except KeyboardInterrupt:
    print("Stopped.")
    all_off()
    servo.value = POS_A   # Park servo
