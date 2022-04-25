import numpy as np

# Constants
A = 0.0324300504528       # projected area of payload (m^2)
ALT = 1.                  # flying altitude (m)
ALT_GROUND = 0.           # ground altitude (m)
APPROACH_ANGLE_N = 89*np.pi/180   # rad
C_D = 0.7                 # drap coefficient of payload
D_APPROACH = 25.          # approach distance (m)
DROP_DELAY_TIME = 1.13    # delay from drop signal to servo activation
G = 9.81                  # gravitational constant (m/s^2)
M = 0.113398              # mass of the payload (kg)    
M_TO_FT = 3.28084         # m to ft conversion factor
PI = np.pi                # pi
PPR = 8                   # waypoints per revolution 
R_EARTH = 6378*10**3      # radius of earth (m)
R_LOITER = 10.            # turn radius (m)
RHO = 1.225               # fluid density of air (kg/m^3)        
V_UAV = 17                # speed of UAV (m/s)