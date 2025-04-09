# test for laser PLPT5 447KA 
# Operating current 1.2A max 1.4A
# Forward voltage 4.6V max  5.9V

from time import sleep
from tps55287 import TSP55287

BB = TSP55287()
vout = 4.6
iout = 1.2

BB.set_iout_limit_from_current(1, iout)  
BB.set_vout(vout)
BB.set_mode(oe=1)
sleep(0.5)
BB.set_mode(oe=0)
