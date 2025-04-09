# test for laser PLPT9 450LB_E 
# Operating current 3A max 3.8A
# Forward voltage 4.3V  max 5.0V

from time import sleep
from tps55287 import TSP55287

BB = TSP55287()
vout = 5.0
iout = 3.0

BB.set_iout_limit_from_current(1, iout)  # Set IOUT_LIMIT to 2.5A
BB.set_vout(vout)
BB.set_mode(oe=1)
sleep(0.5)
BB.set_mode(oe=0)