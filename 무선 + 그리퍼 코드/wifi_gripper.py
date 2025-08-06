import threading
import time
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
ml.set_movement_type(3)
mr.set_movement_type(3)
ml.set_vr_mode(1)
mr.set_vr_mode(1)

# 실시간 센서값 저장용 변수 (thread-safe)
last_arm_data = {1: None, 2: None}
data_lock = threading.Lock()

# 관절값 제한 함수
def jointlimit(angles):
    max_val = [165, 120, 175, 0, 175, 180, 175]
    min_val = [-165, -50, -175, -175, -175, 60, -175]
    for i in range(7):
        angles[i] = max(min(angles[i], max_val[i]), min_val[i])

# 그리퍼 제어 함수
def gripper_control_open(mc):
    mc.set_pro_gripper_angle(14, 80)

def gripper_control_close(mc):
    mc.set_pro_gripper_angle(14, 0)

# 팔 제어 루프 (센서값 수신 및 로봇 제어)
def control_arm(arm):
    while True:
        try:
            arm_data = obj.get_arm_data(arm)

            # 센서값 저장
            with data_lock:
                last_arm_data[arm] = arm_data

            mc = ml if arm == 1 else mr

            # 센서 → 로봇 관절값 변환 및 보정
            mercury_list = [
                arm_data[0], -arm_data[1], arm_data[2], -arm_data[3],
                arm_data[4], 135 + arm_data[5], arm_data[6] - 30
            ]
            jointlimit(mercury_list)

            # 속도 고정
            TI = 5

            # 그리퍼 제어 (오른팔만)
            if arm == 2:
                if arm_data[9] == 0:
                    threading.Thread(target=gripper_control_close, args=(mc,), daemon=True).start()
                elif arm_data[10] == 0:
                    threading.Thread(target=gripper_control_open, args=(mc,), daemon=True).start()

                # 버튼 동시 눌림 시 무시
                if arm_data[9] == 0 and arm_data[10] == 0:
                    time.sleep(0.01)
                    continue

            # 로봇에 관절값 전송
            mc.send_angles(mercury_list, TI, _async=True)
            time.sleep(0.01)

        except Exception:
            time.sleep(0.01)  # 에러 무시 후 재시도

# 3초마다 L/R 센서값 출력
def print_loop():
    while True:
        time.sleep(3.0)
        with data_lock:
            l_data = last_arm_data[1]
            r_data = last_arm_data[2]
        if l_data is not None and r_data is not None:
            l_str = ', '.join([f'{v:>6.1f}' if isinstance(v, (int, float)) else str(v) for v in l_data])
            r_str = ', '.join([f'{v:>6.1f}' if isinstance(v, (int, float)) else str(v) for v in r_data])
            print(f"L: [{l_str}]\nR: [{r_str}]\n")

# 쓰레드 실행
threading.Thread(target=control_arm, args=(1,), daemon=True).start()  # 왼팔
threading.Thread(target=control_arm, args=(2,), daemon=True).start()  # 오른팔
threading.Thread(target=print_loop, daemon=True).start()             # 출력 루프

# 메인 스레드 유지
while True:
    time.sleep(1)


'''
L: [   2.0,  -1.0,  35.0,   0.0,  -5.0,   2.0,   0.0, 1, 1, 1, 1]
      │      │      │      │      │      │       │    │  │  │  └─ [10] → 그리퍼 열기 버튼 (오른팔만 사용)
      │      │      │      │      │      │       │    │  │  └─── [9]  → 그리퍼 닫기 버튼 (오른팔만 사용)
      │      │      │      │      │      │       │    │  └───── [8]  → (미사용 또는 추가 버튼)
      │      │      │      │      │      │       │    └───────── [7]  → 모드 전환 버튼
      │      │      │      │      │      │       └──────────── [6]  → Wrist Yaw (손목 회전)
      │      │      │      │      │      └────────────────── [5]  → Wrist Pitch (손목 숙임)
      │      │      │      │      └───────────────────────── [4]  → Elbow (팔꿈치 굽힘)
      │      │      │      └──────────────────────────────── [3]  → Shoulder Roll (어깨 옆으로)
      │      │      └─────────────────────────────────────── [2]  → Shoulder Pitch (어깨 앞뒤)
      │      └────────────────────────────────────────────── [1]  → Shoulder Yaw (어깨 좌우)
      └───────────────────────────────────────────────────── [0]  → Clavicle (쇄골: 어깨 들기/내리기)
'''