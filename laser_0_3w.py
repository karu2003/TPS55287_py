# test for laser PLT5 520B
# Operating current 0.225 max 0.3A
# Forward voltage 6.1V  max 7.5V

from time import sleep
from tps55287 import TSP55287

BB = TSP55287()
vout = 6.1
iout = 0.225

BB.set_iout_limit_from_current(1, iout)  # Set IOUT_LIMIT to 2.5A
BB.set_vout(vout)
BB.set_mode(oe=1)
sleep(0.5)
BB.set_mode(oe=0)