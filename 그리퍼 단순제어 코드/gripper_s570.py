#!/usr/bin/env python
import time
from pymycobot import Mercury
from pymycobot import ExoskeletonSocket

# Mercury 오른팔에 연결된 로봇
mr = Mercury('/dev/right_arm', 115200)
mr.power_on()

# 외골격 소켓 연결
obj = ExoskeletonSocket()

# 그리퍼 제어 함수
def gripper_open():
    mr.set_pro_gripper_angle(14, 80)
    print("[INFO] Gripper opened.")

def gripper_close():
    mr.set_pro_gripper_angle(14, 0)
    print("[INFO] Gripper closed.")

def main():
    print("[START] S570 그리퍼 제어 시작 (오른손 버튼 기반)")
    prev_open = 1
    prev_close = 1

    while True:
        try:
            arm_data = obj.get_arm_data(2)  # 오른팔

            now_close = arm_data[10]  # 닫기 버튼
            now_open = arm_data[9]  # 열기 버튼

            # 버튼 상태 변화 감지 (눌렀을 때만 작동)
            if now_close == 0 and prev_close == 1:
                gripper_close()

            if now_open == 0 and prev_open == 1:
                gripper_open()

            prev_close = now_close
            prev_open = now_open

            time.sleep(0.05)

        except Exception as e:
            print("[ERROR]", e)
            time.sleep(0.1)

if __name__ == '__main__':
    main()
