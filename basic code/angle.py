from pymycobot import Mercury
import time
ml = Mercury("/dev/left_arm", 115200)
mr = Mercury("/dev/right_arm", 115200)
# Robot powered on
ml.power_on()
mr.power_on()
# Single angle control
ml.send_angle(1, 90, 40)
mr.send_angle(1, 90, 40)
time.sleep(3)
ml.send_angle(1, 0, 40)
mr.send_angle(1, 0, 40)
time.sleep(3)

# All angle controls
ml.send_angles([0, 0, 90, 0, 0, 90, 0], 40)
mr.send_angles([0, 0, 90, 0, 0, 90, 0], 40)
time.sleep(3)
ml.send_angles([0, 0, 0, 0, 0, 90, 0], 40)
mr.send_angles([0, 0, 0, 0, 0, 90, 0], 40)
