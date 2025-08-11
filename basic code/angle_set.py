#!/usr/bin/env python
# coding=utf-8

from pymycobot import Mercury
import time

# 좌/우 로봇팔 연결
ml = Mercury("/dev/left_arm", 115200)
mr = Mercury("/dev/right_arm", 115200)

print("[INFO] 전원 켜는 중...")
ml.power_on()
mr.power_on()

# 초기 위치로 이동
print("[INFO] 초기 위치로 로봇팔 이동 중...")
ml.send_angle(1, 90, 40)
mr.send_angle(1, 90, 40)
time.sleep(3)

ml.send_angle(1, 0, 40)
mr.send_angle(1, 0, 40)
time.sleep(3)

ml.send_angles([0, 0, 90, 0, 0, 90, 0], 40)
mr.send_angles([0, 0, 90, 0, 0, 90, 0], 40)
time.sleep(3)

ml.send_angles([0, 0, 0, 0, 0, 90, 0], 40)
mr.send_angles([0, 0, 0, 0, 0, 90, 0], 40)
time.sleep(3)

# 현재 각도 확인
print("[INFO] 현재 각도:")
print("LEFT:", ml.get_angles())
print("RIGHT:", mr.get_angles())

# 각 조인트 영점 보정 (조심해서 실행)
print("[INFO] 각 조인트에 대해 영점 보정 수행 중...")
for i in range(1, 8):
    ml.set_servo_calibration(i)
    mr.set_servo_calibration(i)
    time.sleep(0.5)

# 확인용 동작
print("[INFO] 테스트 동작 수행 중...")
ml.send_angles([0, 0, -90, 0, 90, 0, 0], 20)
mr.send_angles([0, 0, -90, 0, 90, 0, 0], 20)
time.sleep(3)

print("[INFO] 영점 보정 완료!")
