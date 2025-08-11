#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
from pymycobot import ExoskeletonSocket
import time

# 외골격 연결
obj = ExoskeletonSocket()

# 민감도 및 속도 계수
SPEED_SCALE = 0.001
TURN_SCALE = 0.002
THRESHOLD = 7

# 중립값 저장용 변수
JOY_X_OFFSET = 0
JOY_Y_OFFSET = 0

def calibrate_joystick(samples=50, delay=0.05):
    """조이스틱 중립값 자동 보정"""
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
    print(f"[INFO] 중립값 보정 완료 → X={int(avg_x)}, Y={int(avg_y)}")
    return int(avg_x), int(avg_y)

def main():
    global JOY_X_OFFSET, JOY_Y_OFFSET

    rospy.init_node('s570_joystick_to_x1_base')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(20)

    # 조이스틱 중립값 자동 보정 수행
    JOY_X_OFFSET, JOY_Y_OFFSET = calibrate_joystick()

    while not rospy.is_shutdown():
        try:
            arm_data = obj.get_arm_data(2)
            raw_x = arm_data[11]
            raw_y = arm_data[12]

            # 보정 적용
            joy_x = raw_x - JOY_X_OFFSET
            joy_y = raw_y - JOY_Y_OFFSET

            # 임계값 적용
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

if __name__ == '__main__':
    main()
