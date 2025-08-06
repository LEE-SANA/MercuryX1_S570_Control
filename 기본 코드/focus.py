from pymycobot import Mercury

ml = Mercury('/dev/left_arm', 115200)
mr = Mercury('/dev/right_arm', 115200)

ml.focus_all_servos()
mr.focus_all_servos()

