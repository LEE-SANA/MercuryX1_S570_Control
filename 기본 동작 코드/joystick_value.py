from pymycobot import ExoskeletonSocket
import time

obj = ExoskeletonSocket()

while True:
    arm_data = obj.get_arm_data(2)  # 오른팔
    print("arm_data:", arm_data)
    time.sleep(0.2)
