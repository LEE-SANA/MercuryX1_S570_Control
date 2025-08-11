#!/usr/bin/env python
import time
from pymycobot import Mercury
import keyboard  # 키보드 입력 감지용 라이브러리

# Mercury 오른팔에 연결된 로봇
ml = Mercury('/dev/left_arm', 115200)
ml.power_on()

# 그리퍼 제어 함수
def gripper_open():
    ml.set_pro_gripper_angle(14, 80)
    print("[INFO] Gripper opened.")

def gripper_close():
    ml.set_pro_gripper_angle(14, 0)
    print("[INFO] Gripper closed.")

def main():
    print("[START] Press 'A' to OPEN gripper, 'B' to CLOSE gripper.")
    while True:
        try:
            # A 키 누르면 그리퍼 닫기
            if keyboard.is_pressed('a'):
                gripper_open()
                while keyboard.is_pressed('a'):  # 길게 누를 경우 중복 방지
                    time.sleep(0.05)

            # B 키 누르면 그리퍼 열기
            if keyboard.is_pressed('b'):
                gripper_close()
                while keyboard.is_pressed('b'):
                    time.sleep(0.05)

            time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n[EXIT] Program stopped.")
            break

        except Exception as e:
            print("[ERROR]", e)
            time.sleep(0.1)

if __name__ == '__main__':
    main()
