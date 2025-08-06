
from pymycobot import Mercury
ml = Mercury('/dev/left_arm', 115200)
mr = Mercury('/dev/right_arm', 115200)
# Robot powered on
mr.power_off()
ml.power_off()
