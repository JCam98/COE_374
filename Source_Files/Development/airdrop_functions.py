import numpy as np
from numpy import sin, cos, arctan2, arcsin, ceil
from numpy.linalg import norm
from pymavlink import mavutil, mavwp

from airdrop_constants import *

# Functions
def convert_coord_to_vector(coord, coord_ref):
    """
    convert_coord_to_vector(coord, coord_ref)\n
    Convert `coord` to xy vector with origin `coord_ref` (unit meters)

    Parameters
    ----------
        coord : array-like       
            (lat, lon) of converted vector. Shape=(2,) or (2,n)
        coord_ref : array-like 
            (lat, lon) of origin. Shape must be (2,)

    Returns
    ----------    
        xy : array-like
            xy vector of point (m/s). Shape matches `coord`
    """
    coord_arr = np.array(coord, copy=True)
    
    if coord_arr.ndim==1:
        lat_0 = coord_ref[0]
        lon_0 = coord_ref[1]
        lat_1 = coord[0]
        lon_1 = coord[1]
        x = (lon_1 - lon_0)*R_EARTH*(PI/180)*cos(lat_0 * PI/180)
        y = (lat_1 - lat_0)*R_EARTH*PI/180
        xy = np.array([x,y])
    elif coord_arr.ndim==2:
        lat_0 = coord_ref[:,0]
        lon_0 = coord_ref[:,1]
        lat_1 = coord[0]
        lon_1 = coord[1]
        x = (lon_1 - lon_0)*R_EARTH*(PI/180)*cos(lat_0 * PI/180)
        y = (lat_1 - lat_0)*R_EARTH*PI/180
        xy = np.column_stack((x,y))
    else:
        # dimenion error
        return    
    return xy

def convert_vector_to_coord(xy, coord_ref):
    """
    convert_vector_to_coord(xy, coord_ref)\n
    Convert `xy` vector with origin `coord_ref` to (lat, lon)

    Parameters
    ----------
        xy : array-like
            xy vector of point (m). Shape must be (2,)
        coord_ref : array-like 
            (lat, lon) of origin. Shape must be (2,)

    Returns
    ----------    
        coord : array-like       
            (lat, lon) of converted vector. Shape=(2,)
    """

    xy_arr = np.array(xy)
    if np.ndim(xy_arr)==1:
        lat_0 = coord_ref[0]
        lon_0 = coord_ref[1]
        lat_1 = lat_0 + (xy[1]/R_EARTH)*(180 / PI)
        lon_1 = lon_0 + (xy[0]/R_EARTH)*(180 / PI) / cos(lat_0 * PI/180)
        coord = np.array([lat_1, lon_1])
    elif np.ndim(xy_arr)==2:
        lat_0 = coord_ref[0]
        lon_0 = coord_ref[1]
        lat_1 = lat_0 + (xy[:,1]/R_EARTH)*(180 / PI)
        lon_1 = lon_0 + (xy[:,0]/R_EARTH)*(180 / PI) / cos(lat_0 * PI/180)
        coord = np.column_stack((lat_1, lon_1))
    else:
        # dimension error
        return
    return coord


def caclulate_CARP(v_wind, approach_angle):
    """
    caclulate_CARP(v_wind, approach_angle)\n
    Calculate optimal release point for payload drop.

    Parameters
    ----------
        v_wind : array-like
            Wind velocity (m/s). Shape must be (3,)
        approach_angle : float 
            Heading angle from which UAV will approach target, measured CCW from x-axis (rad)

    Returns
    ----------    
        carp_xy : array-like       
            XY-location of CARP relative to target (m). Shape=(2,)
    """
    theta = approach_angle
    e_approach = np.array([cos(theta), sin(theta)])

    # State vector
    x = 0.           # initial location of the payload
    y = 0.           # initial location of the payload
    z = ALT          # release height in meter(200 feet)
    # vx = V_UAV*cos(theta)      # x component of the payload velocity 
    # vy = V_UAV*sin(theta)      # y component of the payload velocity
    # vz = 0.                        # z component of the payload velocity/ initially 0

    vx = V_UAV*cos(theta)+v_wind[0]      # x component of the payload velocity 
    vy = V_UAV*sin(theta)+v_wind[1]      # y component of the payload velocity
    vz = v_wind[2]                       # z component of the payload velocity/ initially 0

    # Initial condition
    psi = np.array([x, y, z, vx, vy, vz])

    # Simulate Drop
    timestep = 0.01
    while psi[2] > 0:
        # Update wind estimate
        w = v_wind
        # w = np.zeros(3)

        # Update airspeed v_r
        v_r = np.sqrt((psi[3]-w[0])**2 + (psi[4]-w[1])**2 + (psi[5]-w[2])**2) 

        # Time derivate of state vector
        psi_dot = np.zeros(6)
        psi_dot[0] = psi[3]  # v_x
        psi_dot[1] = psi[4]  # v_y
        psi_dot[2] = psi[5]  # v_z
        psi_dot[3] = -1/(2*M)*C_D*A*RHO*v_r*(psi[3]-w[0])   # a_x
        psi_dot[4] = -1/(2*M)*C_D*A*RHO*v_r*(psi[4]-w[1])   # a_y
        psi_dot[5] = -G-1/(2*M)*C_D*A*RHO*v_r*(psi[5]-w[2]) # a_z 

        # Increment one timestep
        psi += psi_dot*timestep   

    # Shift CARP to account for drop signal delay
    carp_xy = np.array([-psi[0], -psi[1]])
    delay_distance = V_UAV*DROP_DELAY_TIME
    carp_xy-=delay_distance*e_approach

    # return CARP (displacement from target in xy)
    return carp_xy


def calc_approach_waypoints_cw(x_curr, x_carp, approach_angle):
    """
    calc_approach_waypoints_cw(x_curr, x_carp, approach_angle)\n
    Calculate approach waypoints for a clockwise approach.
    Parameters
    ----------
        x_curr : array-like
            Current XY position of UAV (m). Shape must be (2,)
        x_carp : array-like
            XY position of CARP (m). Shape must be (2,)
        approach_angle : float 
            Heading angle from which UAV will approach target, measured CCW from x-axis (rad)
    Returns
    ----------    
        waypoints : array-like       
            XY positions of approach waypoints (m). Shape=(n,2)
    """
    # Heading vector
    theta = approach_angle
    e_approach = np.array([cos(theta), sin(theta)])

    # Final approach point
    p = x_carp - e_approach*D_APPROACH                # final point in loiter
    e_ps = np.array([e_approach[1], -e_approach[0]])  # orthogonal to approach vector

    # Choose loiter circle to travel clockwise
    if np.cross(e_approach, e_ps) < 0:
        pass
    else:
        e_ps *= -1

    s = p + e_ps*R_LOITER    # Center of loiter circle

    # Check if UAV is near circle
    if norm(x_curr-s) <= 2*R_LOITER:
        p -= D_APPROACH*e_approach
        s -= D_APPROACH*e_approach

    theta = arctan2(s[1]-x_curr[1], s[0]-x_curr[0])  # angle to center of circle
    alpha = arcsin(R_LOITER/norm(x_curr-s))         # angle between center and tangent 
    D = np.abs(norm(x_curr-s)*cos(alpha))           # Distance to tangent
    x_1 = x_curr + np.array([D*cos(theta+alpha),D*sin(theta+alpha)])   # tangent point 1
    x_2 = x_curr + np.array([D*cos(theta-alpha),D*sin(theta-alpha)])   # tangent point 2

    # Choose point to travel clockwise
    if np.cross(s-x_1, x_curr-x_1) < 0:
        x_t = x_1
    else:
        x_t = x_2

    # Discrete points along circle
    # Measure angles CCW from x axis
    beta_1 = arctan2(x_t[1]-s[1], x_t[0]-s[0]) % (2*PI)
    beta_2 = arctan2(p[1]-s[1], p[0]-s[0]) % (2*PI)
    
    if beta_2-beta_1 > 0:
        arc_angle = 2*PI-(beta_2-beta_1)
        npoints = int(ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = beta_1 - np.linspace(0, arc_angle, num=npoints, endpoint=True)
    else:
        arc_angle = beta_1 - beta_2
        npoints = int(ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = beta_1 - np.linspace(0, arc_angle, num=npoints)

    waypoints = np.zeros((npoints,2))
    waypoints[:,0] += (s[0] + R_LOITER*cos(angles))
    waypoints[:,1] += (s[1] + R_LOITER*sin(angles))

    # Add midway approach point and CARP
    mid_point = p + 0.5*e_approach*D_APPROACH
    waypoints = np.vstack((waypoints, mid_point))   # append carp
    waypoints = np.vstack((waypoints, x_carp))   # append carp
    return waypoints

def calc_approach_waypoints_ccw(x_curr, x_carp, approach_angle):
    """
    calc_approach_waypoints_ccw(x_curr, x_carp, approach_angle)\n
    Calculate approach waypoints for a counter-clockwise approach.
    Parameters
    ----------
        x_curr : array-like
            Current XY position of UAV (m). Shape must be (2,)
        x_carp : array-like
            XY position of CARP (m). Shape must be (2,)
        approach_angle : float 
            Heading angle from which UAV will approach target, measured CCW from x-axis (rad)
    Returns
    ----------    
        waypoints : array-like       
            XY positions of approach waypoints (m). Shape=(n,2)
    """

    # Heading vector
    theta = approach_angle
    e_approach = np.array([cos(theta), sin(theta)])

    # Final approach point
    p = x_carp - e_approach*D_APPROACH                # final point in loiter
    e_ps = np.array([e_approach[1], -e_approach[0]])  # orthogonal to approach vector

    # Choose loiter circle to travel clockwise
    if np.cross(e_approach, e_ps) > 0:
        pass
    else:
        e_ps *= -1

    s = p + e_ps*R_LOITER    # Center of loiter circle

    # Check if UAV is near circle
    if norm(x_curr-s) <= 2*R_LOITER:
        p -= D_APPROACH*e_approach
        s -= D_APPROACH*e_approach

    theta = arctan2(s[1]-x_curr[1], s[0]-x_curr[0])  # angle to center of circle
    alpha = arcsin(R_LOITER/norm(x_curr-s))         # angle between center and tangent 
    D = np.abs(norm(x_curr-s)*cos(alpha))           # Distance to tangent
    x_1 = x_curr + np.array([D*cos(theta+alpha),D*sin(theta+alpha)])   # tangent point 1
    x_2 = x_curr + np.array([D*cos(theta-alpha),D*sin(theta-alpha)])   # tangent point 2

    # Choose point to travel clockwise
    if np.cross(s-x_1, x_curr-x_1) > 0:
        x_t = x_1
    else:
        x_t = x_2

    # Discrete points along circle
    # Measure angles CCW from x axis
    beta_1 = arctan2(x_t[1]-s[1], x_t[0]-s[0]) % (2*PI)
    beta_2 = arctan2(p[1]-s[1], p[0]-s[0]) % (2*PI)
    
    if beta_2-beta_1 > 0:
        arc_angle = beta_2-beta_1
        npoints = int(ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = beta_2 - np.linspace(0, arc_angle, num=npoints, endpoint=True)
        angles = np.flip(angles)
    else:
        arc_angle = 2*PI-(beta_1-beta_2)
        npoints = int(ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = beta_2 - np.linspace(0, arc_angle, num=npoints, endpoint=True)
        angles = np.flip(angles)

    waypoints = np.zeros((npoints,2))
    waypoints[:,0] += (s[0] + R_LOITER*cos(angles))
    waypoints[:,1] += (s[1] + R_LOITER*sin(angles))

    # Add midway approach point and CARP
    mid_point = p + 0.5*e_approach*D_APPROACH
    waypoints = np.vstack((waypoints, mid_point))   # append carp
    waypoints = np.vstack((waypoints, x_carp))   # append carp
    return waypoints

def turnaround_cw(x_curr, heading_angle):
    """
    turnaround_cw(x_curr, heading_angle)\n
    Calculate waypoints for a clockwise turnaround (180 deg)
    
    Parameters
    ----------
        x_curr : array-like
            Current XY position of UAV (m). Shape must be (2,)
        heading_angle : float 
            Heading angle of UAV, measured CCW from x-axis (rad)

    Returns
    ----------    
        waypoints : array-like       
            XY positions of turnaround waypoints (m). Shape=(n,2)
    """

    e_heading = np.array([cos(heading_angle), sin(heading_angle)])
    e_ortho = np.array([e_heading[1], -e_heading[0]])  # possible orthogonal to heading vector

    # Find orthogonal vector
    if np.cross(e_heading, e_ortho) < 0:
        pass
    else:
        e_ortho*=-1

    # Turnaround points
    x_t = x_curr + R_LOITER*e_heading
    s = x_t + R_LOITER*e_ortho  # circle center
    p = x_t + 2*R_LOITER*e_ortho  # final point

    # Discrete points along circle
    # Measure angles CCW from x axis
    beta_1 = arctan2(x_t[1]-s[1], x_t[0]-s[0]) % (2*PI)
    beta_2 = arctan2(p[1]-s[1], p[0]-s[0]) % (2*PI)
    
    if beta_2-beta_1 > 0:
        arc_angle = 2*PI-(beta_2-beta_1)
        npoints = int(ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = beta_1 - np.linspace(0, arc_angle, num=npoints, endpoint=True)
    else:
        arc_angle = beta_1 - beta_2
        npoints = int(ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = beta_1 - np.linspace(0, arc_angle, num=npoints, endpoint=True)

    waypoints = np.zeros((npoints,2))
    waypoints[:,0] += (s[0] + R_LOITER*cos(angles))
    waypoints[:,1] += (s[1] + R_LOITER*sin(angles))

    return waypoints

def turnaround_ccw(x_curr, heading_angle):
    """
    turnaround_ccw(x_curr, heading_angle)\n
    Calculate waypoints for a clockwise turnaround (180 deg)
    
    Parameters
    ----------
        x_curr : array-like
            Current XY position of UAV (m). Shape must be (2,)
        heading_angle : float 
            Heading angle of UAV, measured CCW from x-axis (rad)

    Returns
    ----------    
        waypoints : array-like       
            XY positions of turnaround waypoints (m). Shape=(n,2)
    """

    e_heading = np.array([cos(heading_angle), sin(heading_angle)])
    e_ortho = np.array([e_heading[1], -e_heading[0]])  # possible orthogonal to heading vector

    # Find orthogonal vector
    if np.cross(e_heading, e_ortho) < 0:
        e_ortho*=-1
    else:
        pass

    # Turnaround points
    x_t = x_curr + R_LOITER*e_heading
    s = x_t + R_LOITER*e_ortho  # circle center
    p = x_t + 2*R_LOITER*e_ortho  # final point

    # Discrete points along circle
    # Measure angles CCW from x axis
    beta_1 = arctan2(x_t[1]-s[1], x_t[0]-s[0]) % (2*PI)
    beta_2 = arctan2(p[1]-s[1], p[0]-s[0]) % (2*PI)
    
    if beta_2-beta_1 > 0:
        arc_angle = beta_2-beta_1
        npoints = int(ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = beta_2 - np.linspace(0, arc_angle, num=npoints, endpoint=True)
        angles = np.flip(angles)
    else:
        arc_angle = 2*PI-(beta_1-beta_2)
        npoints = int(ceil(abs(arc_angle)*PPR/(2*PI)))
        angles = beta_2 - np.linspace(0, arc_angle, num=npoints, endpoint=True)
        angles = np.flip(angles)

    waypoints = np.zeros((npoints,2))
    waypoints[:,0] += (s[0] + R_LOITER*cos(angles))
    waypoints[:,1] += (s[1] + R_LOITER*sin(angles))

    return waypoints

def create_mission(first_pass_waypoints, next_pass_waypoints):
    """
    create_mission(first_pass_waypoints, next_pass_waypoints)\n
    Prepares mission commands (i.e. waypoints) for export to autopilot.

    Parameters:
    ----------
        first_pass_waypoints : array-like
            Waypoints for first-pass airdrop. Shape must be (n,2)
        next_pass_waypoints : array-like
            Waypoints for subsequent airdrops. Shape must be (n,2)
    Returns:
        mission_items : ndarray
            Waypoints for entire mission
    ---------- 
    """

    mission = []
    item_count = 0

    # Home Location
    # home_coord = [30.291466, -97.738195]
    # ofile.write('%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%d\n' % (item_count,1,0,16,0,0,0,0,home_coord[0], home_coord[1],ALT,1))
    # mission.append([item_count,1,0,16,0,0,0,0,home_coord[0], home_coord[1],ALT,1])
    # item_count+=1

    # First pass
    num_wp = np.shape(first_pass_waypoints)[0]
    for i, wp in enumerate(first_pass_waypoints):
        if i==num_wp-1:
            # Release point is final point in pass
            mission.append([item_count,0,3,16,0,5,0,0,wp[0],wp[1],ALT,1])
        else:
            # Approach point
            mission.append([item_count,0,3,16,0,0,0,0,wp[0],wp[1],ALT,1])        
        item_count+=1

    # Release payload at release point
    mission.append([item_count,0,0,183,SERVO_CHANNEL,SERVO_OPEN,0,0,0,0,0,1])
    item_count+=1

    # Next pass
    repeat_start = item_count
    num_wp = np.shape(next_pass_waypoints)[0]
    for i, wp in enumerate(next_pass_waypoints):
        if i==1:
            # Close payload doors after first waypoint following drop
            mission.append([item_count,0,3,16,0,0,0,0,wp[0],wp[1],ALT,1])
            item_count+=1
            mission.append([item_count,0,0,183,SERVO_CHANNEL,SERVO_CLOSE,0,0,0,0,0,1])
        elif i==num_wp-1:
            # Release point is final point in pass
            mission.append([item_count,0,3,16,0,5,0,0,wp[0],wp[1],ALT,1])
        else:
            # Approach point
            mission.append([item_count,0,3,16,0,0,0,0,wp[0],wp[1],ALT,1])
        item_count+=1

    # Release payload at release point
    mission.append([item_count,0,0,183,SERVO_CHANNEL,SERVO_OPEN,0,0,0,0,0,1])
    item_count+=1

    # Repeat passes
    mission.append([item_count,0,3,177,repeat_start,2,0,0,0,0,0,1])

    return(np.array(mission))

def write_mission_file(fname, first_pass_waypoints, next_pass_waypoints):
    """
    write_mission_file(first_pass_waypoints, next_pass_waypoints)\n
    Write mission commands to waypoint file for Mission Planner sim.

    Parameters:
    ----------
        first_pass_waypoints : array-like
            Waypoints for first-pass airdrop. Shape must be (n,2)
        next_pass_waypoints : array-like
            Waypoints for subsequent airdrops. Shape must be (n,2)
            
    File Format:
    ----------
    QGC WPL <VERSION>
    <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>
    ----------
    """
    # Write to file
    with open(fname, "w+") as ofile:
        ofile.write('QGC WPL 110\n')
        item_count = 0

        ## Home Location (exclude)
        # home_coord = [30.291466, -97.738195]
        # ofile.write('%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%d\n' % (item_count,1,0,16,0,0,0,0,home_coord[0], home_coord[1],ALT,1))
        # item_count+=1

        # First pass
        num_wp = np.shape(first_pass_waypoints)[0]
        for i, wp in enumerate(first_pass_waypoints):
            if i==num_wp-1:
                # Release point is final point in pass
                ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,3,16,0,5,0,0,wp[0],wp[1],ALT,1))
            else:
                # Approach point
                ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,3,16,0,0,0,0,wp[0],wp[1],ALT,1))
            item_count+=1

        # Release payload at release point
        ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,0,183,9,1100,0,0,0,0,0,1))
        item_count+=1

        # Next pass
        repeat_start = item_count
        num_wp = np.shape(next_pass_waypoints)[0]
        for i, wp in enumerate(next_pass_waypoints):
            if i==1:
                # Close payload doors after first waypoint following drop
                ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,3,16,0,0,0,0,wp[0],wp[1],ALT,1))
                item_count+=1
                ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,0,183,9,1900,0,0,0,0,0,1))
            elif i==num_wp-1:
                # Release point is final point in pass
                ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,3,16,0,5,0,0,wp[0],wp[1],ALT,1))
            else:
                # Approach point
                ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,3,16,0,0,0,0,wp[0],wp[1],ALT,1))
            item_count+=1

        # Release payload at release point
        ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,0,183,9,1100,0,0,0,0,0,1))
        item_count+=1

        # Repeat passes
        ofile.write('%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n' % (item_count,0,3,177,repeat_start,2,0,0,0,0,0,1))