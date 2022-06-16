from matplotlib.pyplot import connect
from run_mission import run_mission
from dronekit import connect
from pymavlink import mavutil
import numpy as np

port = mavutil.auto_detect_serial(preferred_list=['*CubeOrange*if00'])
vehicle = connect(str(port[0]), wait_ready=True, baud=921600)

gps_coord = np.array([30.3247721,-97.6028609])   # Example
run_mission(gps_coord[0], gps_coord[1], vehicle)