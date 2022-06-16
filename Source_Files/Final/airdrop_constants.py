'''Description: This module contains important constants used in evaluating 
and storing an optimized waypoint path in the functions in the module 
"airdrop_functions.py".'''

import numpy as np

# Constants
A = 0.0324300504528       # projected area of payload (m^2)
ALT = 60.69               # flying altitude (m)
ALT_GROUND = 0.8382       # ground altitude (m)
APPROACH_ANGLE = 110*np.pi/180   # rad
C_D = 0.25                # drap coefficient of payload
D_APPROACH = 100.         # approach distance (m)
DROP_DELAY_TIME = 1.13    # delay from drop signal to servo activation
G = 9.81                  # gravitational constant (m/s^2)
M = 0.113398              # mass of the payload (kg)    
PI = np.pi                # pi
PPR = 8                   # waypoints per revolution 
R_EARTH = 6378*10**3      # radius of earth (m)
R_LOITER = 60.            # turn radius (m)
RHO = 1.225               # fluid density of air (kg/m^3)
SERVO_CHANNEL = 9         # Servo channel
SERVO_OPEN = 1100         # Servo open PWM
SERVO_CLOSE = 1900        # Servo closed PWM
V_UAV = 17.               # Airspeed of UAV (m/s)