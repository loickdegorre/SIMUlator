import simu_lib as sl
from boat_class import Boat
from sailboat_class import Sailboat
from floater_class import Floater

from roblib import *


t0 = 0
dt = 0.05#0.1
tf = 100


xmin, xmax, ymin, ymax = -10, 50, -10, 50 



xmin, xmax, ymin, ymax = -10, 100, -10, 100 # SB 
# xmin, xmax, ymin, ymax = -1, 2, -10, 0 #Float

names = ["USV", "dubins", "sailboat", "float"]
cmd = ["PID1", "sailboat", "sailboat_obstacle", "HEOL"]
missions = ["setpoint", "line", "traj1", "traj2"]



if __name__ == "__main__":
    boat_list = []
    sailboat = Sailboat(t0, dt, tf, xmin, xmax, ymin, ymax, "sailboat_obstacle")
    sailboat.loop()
    boat_list.append(sailboat)

    # flap = Floater(t0, dt, tf, xmin, xmax, ymin, ymax, "float_3", "HEOL_float", "sine", False, 'b')
    # flap.loop()

    # boat_list.append(flap)

    # flap2 = Floater(t0, dt, tf, xmin, xmax, ymin, ymax, "float", "MFC_float", "sine", False, 'r')
    # flap2.loop()

    # boat_list.append(flap2)

    # USV = Boat(t0, dt, tf, xmin, xmax, ymin, ymax, "dubins", "PID1", "traj2")
    # USV.loop()
    # boat_list.append(USV)

    show()
