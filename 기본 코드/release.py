from pymycobot import Mercury

ml = Mercury('/dev/left_arm', 115200)
mr = Mercury('/dev/right_arm', 115200)
# Robot powered on
ml.power_on()
mr.power_on()

ml.release_all_servos()
mr.release_all_servos()

