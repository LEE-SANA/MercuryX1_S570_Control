#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
import threading
import time
import sys
import termios
import tty
from pymycobot import Mercury
from pymycobot import ExoskeletonSocket

# Mercury 로봇팔 좌우 연결
ml = Mercury('/dev/left_arm', 115200)
mr = Mercury('/dev/right_arm', 115200)

# 외골격 블루투스 통신 연결
obj = ExoskeletonSocket()

# 전원 켜기
ml.power_on()
mr.power_on()

# X1 영점 조정
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

# VR 모드 및 움직임 타입 설정
ml.set_movement_type(0)
mr.set_movement_type(0)
ml.set_vr_mode(1)
mr.set_vr_mode(1)

# 센서값 저장 (thread-safe)
last_arm_data = {1: None, 2: None}
data_lock = threading.Lock()

# 중립값 보정 완료 신호
calibration_done = threading.Event()

# 센서 일시정지 플래그
pause_flag = threading.Event()
pause_flag.set()  # 기본은 작동 상태

# 관절값 제한 함수
def jointlimit(angles):
    max_val = [165, 120, 175, 5, 175, 180, 175]
    min_val = [-165, -50, -175, -170, -175, 60, -175]
    for i in range(7):
        angles[i] = max(min(angles[i], max_val[i]), min_val[i])

# 그리퍼 제어 함수
def gripper_control_open(mc):
    mc.set_pro_gripper_angle(14, 80)

def gripper_control_close(mc):
    mc.set_pro_gripper_angle(14, 0)

# 이전 관절값 저장용
prev_angles = {1: [0]*7, 2: [0]*7}

# 팔 제어 루프 (센서값 수신 및 로봇 제어)
def control_arm(arm):
    calibration_done.wait()
    while True:
        pause_flag.wait()
        try:
            arm_data = obj.get_arm_data(arm)
            with data_lock:
                last_arm_data[arm] = arm_data

            mc = ml if arm == 1 else mr

            mercury_list = [
                arm_data[0] * 0.9,
                -arm_data[1] * 0.9,
                arm_data[2] * 0.9,
                -arm_data[3] * 0.9,
                arm_data[4] * 0.9,
                135 + arm_data[5] * 0.8,
                arm_data[6] * 0.8 - 30
            ]
            jointlimit(mercury_list)

            TI = 40

            if arm == 2:
                if arm_data[9] == 0 and arm_data[10] == 1:
                    threading.Thread(target=gripper_control_close, args=(mc,), daemon=True).start()
                elif arm_data[10] == 0 and arm_data[9] == 1:
                    threading.Thread(target=gripper_control_open, args=(mc,), daemon=True).start()

            with data_lock:
                if all(abs(a - b) < 1.0 for a, b in zip(prev_angles[arm], mercury_list)):
                    time.sleep(0.02)
                    continue
                prev_angles[arm] = mercury_list[:]

            mc.send_angles(mercury_list, TI, _async=True)
            time.sleep(0.03)

        except Exception:
            time.sleep(0.05)

# 3초마다 센서값 출력
def print_loop():
    while True:
        pause_flag.wait()
        time.sleep(3.0)
        with data_lock:
            l_data = last_arm_data[1]
            r_data = last_arm_data[2]
        if l_data is not None and r_data is not None:
            l_str = ', '.join([f'{v:>6.1f}' if isinstance(v, (int, float)) else str(v) for v in l_data])
            r_str = ', '.join([f'{v:>6.1f}' if isinstance(v, (int, float)) else str(v) for v in r_data])
            print(f"L: [{l_str}]\nR: [{r_str}]\n")

# ────────────────────── Joystick 기반 X1 이동 제어 ──────────────────────

SPEED_SCALE = 0.001
TURN_SCALE = 0.002
THRESHOLD = 7
JOY_X_OFFSET = 0
JOY_Y_OFFSET = 0

def calibrate_joystick(samples=50, delay=0.05):
    global JOY_X_OFFSET, JOY_Y_OFFSET
    print("[INFO] 조이스틱 중립값 자동 측정 중... 조이스틱을 2초간 건들지 마세요.")
    x_vals = []
    y_vals = []

    for _ in range(samples):
        data = obj.get_arm_data(2)
        x_vals.append(data[11])
        y_vals.append(data[12])
        time.sleep(delay)

    avg_x = sum(x_vals) / len(x_vals)
    avg_y = sum(y_vals) / len(y_vals)
    JOY_X_OFFSET = int(avg_x)
    JOY_Y_OFFSET = int(avg_y)
    print(f"[INFO] 중립값 보정 완료 → X={JOY_X_OFFSET}, Y={JOY_Y_OFFSET}")
    calibration_done.set()

def joystick_drive():
    calibration_done.wait()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        pause_flag.wait()
        try:
            arm_data = obj.get_arm_data(2)
            raw_x = arm_data[11]
            raw_y = arm_data[12]

            joy_x = raw_x - JOY_X_OFFSET
            joy_y = raw_y - JOY_Y_OFFSET

            if abs(joy_x) < THRESHOLD:
                joy_x = 0
            if abs(joy_y) < THRESHOLD:
                joy_y = 0

            twist = Twist()
            twist.linear.x = -joy_y * SPEED_SCALE
            twist.angular.z = joy_x * TURN_SCALE

            pub.publish(twist)
            rate.sleep()

        except Exception as e:
            rospy.logwarn(f"[ERROR] {e}")
            time.sleep(0.1)

# 키 입력 처리 루프
def key_control_loop():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("[INFO] 키보드 입력 대기 중 (Q/q/ESC: 정지, G: 다시 시작)")
    try:
        while True:
            ch = sys.stdin.read(1)
            if ch in ['q', 'Q', '\x1b']:
                pause_flag.clear()
                print("[INFO] 센서 정지됨 (Q/q/ESC)")
            elif ch in ['g', 'G']:
                pause_flag.set()
                print("[INFO] 센서 재개됨 (G)")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ────────────────────── 메인 쓰레드 실행 ──────────────────────

if __name__ == '__main__':
    rospy.init_node('s570_joystick_to_x1_base')
    threading.Thread(target=calibrate_joystick, daemon=True).start()
    threading.Thread(target=control_arm, args=(1,), daemon=True).start()
    threading.Thread(target=control_arm, args=(2,), daemon=True).start()
    threading.Thread(target=print_loop, daemon=True).start()
    threading.Thread(target=joystick_drive, daemon=True).start()
    threading.Thread(target=key_control_loop, daemon=True).start()

    while True:
        time.sleep(1)
