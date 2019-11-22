import numpy as np
import math

# MODE NUMBERS
STANDBY_MODE = 0
FREEDRIVE_MODE = 1
WAYPOINT_JOINT_MODE = 2
WAYPOINT_CARTESIAN_MODE = 3 # THIS MODE IS VERY UNSTABLE RIGHT NOW. RECOMMEND NOT USING UNLESS WAYPOINT IS RELATIVELY CLOSE TO CURRENT POSITION. FOR FAR AWAY WAYPOINTS, ROBOT WILL LIKELY RAISE ERROR AND STOP MOTION
POSITION_CONTROL_JOINT_MODE = 4
POSITION_CONTROL_CARTESIAN_MODE = 5
VELOCITY_CONTROL_JOINT_MODE = 6
VELOCITY_CONTROL_CARTESIAN_MODE = 7
FORCE_CONTROL_MODE = 8 # Not implemented in UR Script
IMPEDANCE_CONTROL_MODE = 9

MODE_OUTPUT_INT_REGISTER = 0
PING_OUTPUT_INT_REGISTER = 1

# REGISTER INDICES/OFFSETS
MODE_INT_REGISTER = 0
ENABLE_INT_REGISTER = 1
PING_INT_REGISTER = 2

SETP_DOUBLE_REGISTER_OFFSET = 0
AVTR_DOUBLE_REGISTER_OFFSET = 6
SERVOJ_PARAMS_DOUBLE_REGISTER_OFFSET = 10
SETP2_DOUBLE_REGISTER_OFFSET = 13

UR_RUNNING_FLAG = 2

# CONTROL MODE NUMBERS
DEFAULT_CONTROL_MODE = 0
SETPOINT_CONTROL_MODE = 1
TRAJECTORY_CONTROL_MODE = 2

JOINT_POSITION_MIN_LIMITS = [-2*math.pi,-2*math.pi,-2*math.pi,-2*math.pi,-2*math.pi,-2*math.pi]
JOINT_POSITION_MAX_LIMITS = [ 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi]
JOINT_VELOCITY_LIMITS = [math.radians(131),math.radians(131),math.radians(191),math.radians(191),math.radians(191),math.radians(191)]

# CARTESIAN_POSITION_MIN_LIMITS = [-1200.0, -600.0, -200.0, math.pi/2-0.1, -0.1,math.pi/2-0.1]
# CARTESIAN_POSITION_MAX_LIMITS = [0.0, 600.0, 1000.0, math.pi+0.1, 0.1, math.pi/2+0.1]

CARTESIAN_POSITION_MIN_LIMITS = [-1200.0, -600.0, -200.0, 0.9, -0.1 , -2.5]
CARTESIAN_POSITION_MAX_LIMITS = [0.0    , 600.0 , 1000.0, 4.0,  1.5 , 2.5]

CARTESIAN_VELOCITY_LIMITS = [300.0,300.0,300.0,math.radians(191),math.radians(191),math.radians(191)]

def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

def input_to_list(input_, n=6, offset=0, dtype="double"):
    list = []
    for i in range(0,n):
        list.append(input_.__dict__["input_"+ dtype +"_register_%i" % (i+offsets)])
    return list

def list_to_input(input_, list_, n=6, offset=0, dtype="double"):
    if n == 1 and not isinstance(list_, list):
        list_ = [list_]
    for i in range (0,n):
        input_.__dict__["input_"+ dtype + "_register_%i" % (i+offset)] = list_[i]
    return input_

def diff_norm_within_tol(vec1, vec2, tol):
    # Returns true if Euclidian norm of difference between list1 and list2 is <= tolerance "tol"
    # Used mostly to determine if two 6-DOF vectors/lists are "close enough"
    return np.linalg.norm(np.subtract(vec1,vec2)) <= tol

def rotvec_to_quat(rotvec):
    # Converts rotation vector to a quaternion.
    # Rotation vector is a unit vector in the direction of rotation multiplied by the magnitude of the rotation
    # Quaternion in form [x,y,z,w]
    rotvec = np.array(rotvec)
    rot = np.linalg.norm(rotvec)
    vec = rotvec/rot
    return list(vec*math.sin(rot/2))+[math.cos(rot/2)] # Quaternion [x,y,z,w]

def quat_to_rotvec(quat):
    # Converts quaternion to a rotation vector.
    # Quaternion in form [x,y,z,w]
    # Rotation vector is a unit vector in the direction of rotation multiplied by the magnitude of the rotation
    quat = np.array(quat)/np.linalg.norm(quat)
    rot = 2*math.acos(quat[3])
    vec = np.array(quat[0:3])/math.sin(rot/2) if math.sin(rot/2) != 0 else np.array([0.0,0.0,0.0])
    return list(rot*vec)