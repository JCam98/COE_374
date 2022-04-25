# Group 2 - payload drop team
# 4/24/2022
# executable for walking flight test on 

import airdrop_functions as adp
from airdrop_constants import *
from pymavlink import mavutil
from pymavlink import mavwp
import time

def run_mission(target_coord, master):

    # CARP = Target for walk
    carp_coord = target_coord # bc walking, drop on the target

    # Generate waypoints
    uav_xy = -D_APPROACH*np.array([np.cos(APPROACH_ANGLE_N), np.sin(APPROACH_ANGLE_N)])
    carp_xy = adp.convert_coord_to_vector(carp_coord, target_coord)
    first_pass_waypoints_xy = adp.calc_approach_waypoints_cw(uav_xy, carp_xy, APPROACH_ANGLE_N)
    turnaround_points_xy = adp.turnaround_cw(carp_xy, APPROACH_ANGLE_N)
    next_pass_waypoints_xy = adp.calc_approach_waypoints_cw(turnaround_points_xy[-1], carp_xy, APPROACH_ANGLE_N)
    next_pass_waypoints_xy = np.vstack((turnaround_points_xy, next_pass_waypoints_xy))

    first_pass_waypoints = adp.convert_vector_to_coord(first_pass_waypoints_xy, target_coord)
    next_pass_waypoints = [adp.convert_vector_to_coord(wp, target_coord) for wp in next_pass_waypoints_xy]
    next_pass_waypoints = np.array(next_pass_waypoints)

    # Write to file
    mission_file_name = "payload_mission.waypoints"
    adp.write_mission_file(mission_file_name, first_pass_waypoints, next_pass_waypoints)

    # read the waypoints and send to mission planner!
    wp = mavwp.MAVWPLoader()

    with open(mission_file_name) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:   
                linearray=line.split('\t')
                ln_seq = int(linearray[0])
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=(float(linearray[8]))
                ln_y=(float(linearray[9]))
                ln_z=float(linearray[10])
                ln_autocontinue = int(float(linearray[11].strip()))
                if(i == 1):
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,
                                                                ln_command,
                                                                ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wp.add(p)
                
                    
    #cmd_set_home(home_location,home_altitude)
    msg = master.recv_match(type = ['COMMAND_ACK'],blocking = True)
    # print(msg)
    # print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    time.sleep(1)
    
    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())
    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        # print(msg)
        master.mav.send(wp.wp(msg.seq))
        #print(wp.wp(msg.seq))
        # print('Sending waypoint {0}'.format(msg.seq))
