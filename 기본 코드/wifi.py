import threading
import time
from pymycobot import Mercury
from pymycobot import ExoskeletonSocket

# Mercury 로봇팔 좌우 연결
ml = Mercury('/dev/left_arm', 115200)
mr = Mercury('/dev/right_arm', 115200)

# 외골격 블루투스 통신 연결 (Socket 기반)
obj = ExoskeletonSocket()  # 내부적으로 BLE 통신을 사용


ml.power_on()
mr.power_on()

l_control_mode = 1
r_control_mode = 1

l_last_mode = 0
r_last_mode = 0

ml.set_movement_type(3)
mr.set_movement_type(3)

ml.set_vr_mode(1)
mr.set_vr_mode(1)


def jointlimit(angles):
    max = [165, 120, 175, 0, 175, 180, 175]
    min = [-165, -50, -175, -175, -175, 60, -175]
    for i in range(7):
        if angles[i] > max[i]:
            angles[i] = max[i]
        if angles[i] < min[i]:
            angles[i] = min[i]


def gripper_control_open(mc):
    mc.set_pro_gripper_angle(14, 80)

	
def gripper_control_close(mc):
    mc.set_pro_gripper_angle(14, 0)


def control_arm(arm):
    global l_control_mode, l_last_mode, r_control_mode, r_last_mode
    st = 0
    while True:
        try:
            if arm == 1:
                arm_data = obj.get_arm_data(1)
                print("L: ", arm_data)
                mc = ml
            elif arm == 2:
                arm_data = obj.get_arm_data(2)
                print("R: ", arm_data)
                mc = mr
            else:
                raise ValueError("error arm")
            mercury_list = [
                arm_data[0], -arm_data[1], arm_data[2], -arm_data[3], arm_data[4],
                135 + arm_data[5], arm_data[6] - 30
            ]
            jointlimit(mercury_list)
            time_err = 1000 * (time.time() - st)
            st = time.time()
            if arm == 1:
                if arm_data[7] == 0 and l_last_mode == 0:
                    print(6)
                    l_last_mode = 1
                    l_control_mode += 1
                    if l_control_mode > 3:
                        l_control_mode = 1

                    if l_control_mode == 1:
                        obj.set_color(arm, 0, 255, 0)
                    elif l_control_mode == 2:
                        obj.set_color(arm, 0, 0, 255)
                    else:
                        obj.set_color(arm, 255, 0, 0)

                if arm_data[7] == 1:
                    l_last_mode = 0

                if l_control_mode == 1:
                    TI = 10
                elif l_control_mode == 2:
                    TI = 5
                else:
                    TI = 3
            else:
                if arm_data[7] == 0 and r_last_mode == 0:
                    print(6)
                    r_last_mode = 1
                    r_control_mode += 1
                    if r_control_mode > 3:
                        r_control_mode = 1
                    if r_control_mode == 1:
                        obj.set_color(arm, 0, 255, 0)
                    elif r_control_mode == 2:
                        obj.set_color(arm, 0, 0, 255)
                    else:
                        obj.set_color(arm, 255, 0, 0)

                if arm_data[7] == 1:
                    r_last_mode = 0
                if r_control_mode == 1:
                    TI = 10
                elif r_control_mode == 2:
                    TI = 5
                else:
                    TI = 3

                if arm_data[9] == 0:
                    threading.Thread(target=gripper_control_close, args=(mc,)).start()

                elif arm_data[10] == 0:
                    threading.Thread(target=gripper_control_open, args=(mc,)).start()

                if arm_data[9] == 0 and arm_data[10] == 0:
                    time.sleep(0.01)
                    continue

            mc.send_angles(mercury_list, TI, _async=True)
            time.sleep(0.01)

        except Exception as e:
            print(e)
            pass

threading.Thread(target=control_arm, args=(1,)).start()
threading.Thread(target=control_arm, args=(2,)).start()
