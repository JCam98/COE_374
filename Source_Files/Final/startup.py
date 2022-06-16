'''Description: This python module is designated as the startup script that
runs the "main.py" script once necessary start conditions have been met. In particular, 
this script is invoked by a "startup.sh" bash script once the script pulls
in an altitude reading exceeding 46 meters or once the bash script has been
running for 5 minutes (average time taken by Aerospace Design Team's Ground Crew
to perform pre-flight tests between the time that power is supplied to the software 
system thus running the startup bash script, and the time that the UAS takes off). Important Note:
Modifications were made to this startup script that were only saved on the team's Raspberry Pi's
local file system and are not currently reflected in this file. Thus,
this script is not fully functional.'''


print("Loading time module")
import time
time.sleep(5)

from dronekit import connect, VehicleMode
print("dronekit module loaded")
import argparse


print("Running python script")
