#!/usr/bin/python
# Driver for Universal Robot using RTDE communication protocol
#
# Authors:  Filippos Sotiropoulos (fes@mit.edu)
#           Ryan Sandzimier (rsandz@mit.edu)

# Outstanding things to do (not complete)
# -- Decide on and write custom messages

import sys
sys.path.append('..')
import logging
import threading

import copy
import rospy
import math
import numpy as np
import tf

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import helper as hp

from ur_rtde.msg import Command, ToolState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import UInt32
from std_msgs.msg import Bool

#logging.basicConfig(level=logging.INFO)

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

ROBOT_PORT = 30004
ROBOT_HOST = '169.254.157.0'
#ROBOT_HOST = '127.0.0.1'

#Driver Class
class UrRtdeDriver:

    def __init__(self):
        self.keep_running       = True
        self.loop_frequency     = 500 # Hz
        self.ros_rate           = rospy.Rate(self.loop_frequency)

        # ROS PUBLISHER SETUP
        self.pub_joint_states   = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.pub_tool_state   = rospy.Publisher('tool_state', ToolState, queue_size=1)
        self.pub_wrench         = rospy.Publisher('wrench', WrenchStamped, queue_size=1)
        self.pub_robot_ready    = rospy.Publisher('robot_ready', Bool, queue_size=1)

        # ROS SUBSCRIBER SETUP
        self.sub_command        = rospy.Subscriber('command',Command, self.cb_command)

        self.whole_message      = 0

        self.command_lock       = threading.Lock()
        self.state_lock         = threading.Lock()

        self.program_runtime_state = []
        self.robot_ready        = False
        self.ping_id            = []

        self.time_start_update  = 0

        self.mode               = 0
        self.enable             = 0 
        self.setp               = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        self.avtr               = [0.0,0.0,0.0,0.0]
        self.setp2              = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        self.servoj_params      = [0.01,0.05,2000]

        self.mode_prev = 0

    def cb_command(self, msg):

        self.command_lock.acquire()
        self.mode   = msg.mode
        self.setp   = msg.setpoint
        self.avtr   = msg.avtr
        self.setp2   = msg.setpoint2
        self.enable = msg.enable
        self.command_lock.release()

        #print(rospy.get_rostime().to_sec()-self.time_start_update)
        self.time_start_update = rospy.get_rostime().to_sec()

    def cb_state(self, event):
        state_obj = self.con.receive()

        self.state_lock.acquire()
        self.state_obj = copy.copy(state_obj)
        self.state_lock.release()

    def connect_rtde(self):
        self.config_filename = rospy.get_param("/ur_rtde_driver/config_file", 'src/ur_rtde/src/control_loop_configuration.xml') # Get param, otherwise default

        self.logger = logging.getLogger().setLevel(logging.INFO)

        #Load and parse recipe
        self.conf = rtde_config.ConfigFile(self.config_filename)
        self.state_names, self.state_types = self.conf.get_recipe('state')
        self.setp_names, self.setp_types = self.conf.get_recipe('setp')
        self.mode_names, self.mode_types = self.conf.get_recipe('mode')
        self.enable_names, self.enable_types = self.conf.get_recipe('enable')
        self.ping_names, self.ping_types = self.conf.get_recipe('ping')

        self.avtr_names, self.avtr_types = self.conf.get_recipe('avtr')
        self.servoj_params_names, self.servoj_params_types = self.conf.get_recipe('servoj_params')
        self.setp2_names, self.setp2_types = self.conf.get_recipe('setp2')


        #Define connection and connect to robot
        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.con.connect()
        
        # get controller version
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(self.state_names, self.state_types)
        self.setp_obj = self.con.send_input_setup(self.setp_names, self.setp_types)
        self.mode_obj = self.con.send_input_setup(self.mode_names, self.mode_types)
        self.enable_obj = self.con.send_input_setup(self.enable_names, self.enable_types)
        self.avtr_obj = self.con.send_input_setup(self.avtr_names, self.avtr_types)
        self.servoj_params_obj = self.con.send_input_setup(self.servoj_params_names, self.servoj_params_types)
        self.setp2_obj = self.con.send_input_setup(self.setp2_names, self.setp2_types)
        self.ping_obj = self.con.send_input_setup(self.ping_names, self.ping_types)


        # RTDE State RECEIVE THREAD

        self.cb_state(None)
        rospy.Timer(rospy.Duration(0.002), self.cb_state)

        self.initialize_variables()
    
        self.con.send(self.mode_obj)
        self.con.send(self.setp_obj)
        self.con.send(self.servoj_params_obj)
        self.con.send(self.avtr_obj)
        self.con.send(self.setp2_obj)
        self.con.send(self.enable_obj)
        self.con.send(self.ping_obj)

        #start data synchronization or exit
        if not self.con.send_start():
            sys.exit()

    def initialize_variables(self):
        hp.list_to_input(self.setp_obj, [0.0,0.0,0.0,0.0,0.0,0.0], 6, hp.SETP_DOUBLE_REGISTER_OFFSET, "double")
        hp.list_to_input(self.avtr_obj, [0.0,0.0,0.0,0.0], 4, hp.AVTR_DOUBLE_REGISTER_OFFSET, "double")
        hp.list_to_input(self.servoj_params_obj, [0.0,0.0,0.0], 3, hp.SERVOJ_PARAMS_DOUBLE_REGISTER_OFFSET, "double")
        hp.list_to_input(self.setp2_obj, [0.0,0.0,0.0,0.0,0.0,0.0], 6, hp.SETP2_DOUBLE_REGISTER_OFFSET, "double")
        hp.list_to_input(self.mode_obj, 0, 1, hp.MODE_INT_REGISTER, "int")
        hp.list_to_input(self.enable_obj, 0, 1, hp.ENABLE_INT_REGISTER, "int")
        hp.list_to_input(self.ping_obj, 0, 1, hp.PING_INT_REGISTER, "int")

    def parse_and_publish_state(self, state_object):
        time_now = rospy.get_rostime()
        
        # Joint state (position and velocity) ROS publisher
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = time_now
        joint_state_msg.header.frame_id = "From real-time state data"
        joint_state_msg.name = JOINT_NAMES
        joint_state_msg.position = state_object.actual_q
        joint_state_msg.velocity  = state_object.actual_qd

        # Tool state (position and velocity) ROS publisher
        tool_state_msg = ToolState()
        tool_state_msg.header.stamp = time_now
        tool_state_msg.header.frame_id = "From real-time state data"
        tool_pos = state_object.actual_TCP_pose
        tool_quat = hp.rotvec_to_quat(tool_pos[3:6])
        tool_state_msg.position.position.x = tool_pos[0]
        tool_state_msg.position.position.y = tool_pos[1]
        tool_state_msg.position.position.z = tool_pos[2]
        tool_state_msg.position.orientation.x = tool_quat[0]
        tool_state_msg.position.orientation.y = tool_quat[1]
        tool_state_msg.position.orientation.z = tool_quat[2]
        tool_state_msg.position.orientation.w = tool_quat[3]
        tool_vel = state_object.actual_TCP_speed
        tool_state_msg.velocity.linear.x  = tool_vel[0]
        tool_state_msg.velocity.linear.y  = tool_vel[1]
        tool_state_msg.velocity.linear.z  = tool_vel[2]
        tool_state_msg.velocity.angular.x  = tool_vel[3]
        tool_state_msg.velocity.angular.y  = tool_vel[4]
        tool_state_msg.velocity.angular.z  = tool_vel[5]

        # End effector 6Dof Force 
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = time_now
        wrench_msg.wrench.force.x = state_object.actual_TCP_force[0]
        wrench_msg.wrench.force.y = state_object.actual_TCP_force[1]
        wrench_msg.wrench.force.z = state_object.actual_TCP_force[2]
        wrench_msg.wrench.torque.x = state_object.actual_TCP_force[3]
        wrench_msg.wrench.torque.y = state_object.actual_TCP_force[4]
        wrench_msg.wrench.torque.z = state_object.actual_TCP_force[5]

        runtime_state_msg = UInt32()
        runtime_state_msg.data = state_object.runtime_state

        self.pub_joint_states.publish(joint_state_msg)
        self.pub_tool_state.publish(tool_state_msg)
        self.pub_wrench.publish(wrench_msg)


    def start_new_mode(self, mode):
        
        if mode == hp.STANDBY_MODE:
            # STANDBY MODE
            print "STANDBY MODE"

        elif mode == hp.FREEDRIVE_MODE:
            # FREEDRIVE MODE
            print "FREEDRIVE MODE"

        elif mode == hp.WAYPOINT_JOINT_MODE:
            # WAYPOINT JOINT MODE
            print "WAYPOINT JOINT MODE"

        elif mode == hp.POSITION_CONTROL_JOINT_MODE:
            # POSITION CONTROL JOINT MODE
            print "POSITION JOINT MODE"

        elif mode == hp.VELOCITY_CONTROL_JOINT_MODE:    
            #SPEED CONTROL JOINT MODE
            print "SPEED JOINT MODE"

        elif mode == hp.POSITION_CONTROL_CARTESIAN_MODE:    
            #POSITION CONTROL CARTESIAN MODE
            print "POSITION CARTESIAN MODE"

        elif mode == hp.IMPEDANCE_CONTROL_MODE:    
            #IMPEDANCE CONTROL MODE
            print "IMPEDANCE MODE"

    def update(self):
        self.state_lock.acquire()
        state = copy.copy(self.state_obj)
        self.state_lock.release()

        if state is None:
            return

        self.program_runtime_state = state.runtime_state
        
        self.parse_and_publish_state(state)

        if self.program_runtime_state != hp.UR_RUNNING_FLAG:
            self.robot_ready = False
            self.pub_robot_ready.publish(Bool(False))
            self.ping_id = []
            self.ros_rate.sleep()
            return

        if not self.robot_ready and self.ping_id == []:
            self.ping_id = int(state.output_int_register_1 != 1)
            hp.list_to_input(self.ping_obj, self.ping_id, 1, hp.PING_INT_REGISTER, "int")
            self.con.send(self.ping_obj)
        elif not self.robot_ready and self.ping_id == state.output_int_register_1:
            self.robot_ready = True
            self.pub_robot_ready.publish(Bool(True))
            print "ROBOT READY"

        self.command_lock.acquire()
        mode   = self.mode
        setp   = self.setp
        avtr   = self.avtr
        setp2  = self.setp2
        enable = self.enable
        self.command_lock.release()
        
        if self.mode_prev != mode:
            self.start_new_mode(mode)
        self.mode_prev = mode
        # do the update for each mode
        if mode == hp.STANDBY_MODE:      # STANDBY MODE
            hp.list_to_input(self.mode_obj, mode, 1, hp.MODE_INT_REGISTER, "int")
            hp.list_to_input(self.enable_obj, enable, 1, hp.ENABLE_INT_REGISTER, "int")
            self.con.send(self.mode_obj)
            self.con.send(self.enable_obj) 

        elif mode == hp.FREEDRIVE_MODE:    # FREEDRIVE MODE
            hp.list_to_input(self.mode_obj, mode, 1, hp.MODE_INT_REGISTER, "int")
            hp.list_to_input(self.enable_obj, enable, 1, hp.ENABLE_INT_REGISTER, "int")
            self.con.send(self.mode_obj)
            self.con.send(self.enable_obj)
        
        elif mode == hp.WAYPOINT_JOINT_MODE:    # WAYPOINT JOINT MODE

            hp.list_to_input(self.mode_obj, mode, 1, hp.MODE_INT_REGISTER, "int")
            hp.list_to_input(self.setp_obj, setp, 6, hp.SETP_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.avtr_obj, avtr, 4, hp.AVTR_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.enable_obj, enable, 1, hp.ENABLE_INT_REGISTER, "int")
            
            # send new setpoint        
            self.con.send(self.mode_obj)
            self.con.send(self.setp_obj)
            self.con.send(self.avtr_obj)
            self.con.send(self.enable_obj)

        elif mode == hp.POSITION_CONTROL_JOINT_MODE:    # JOINT POSITION CONTROL MODE

            hp.list_to_input(self.mode_obj, mode, 1, hp.MODE_INT_REGISTER, "int")
            hp.list_to_input(self.setp_obj, setp, 6, hp.SETP_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.servoj_params_obj, self.servoj_params, 3, hp.SERVOJ_PARAMS_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.enable_obj, enable, 1, hp.ENABLE_INT_REGISTER, "int")

            # send new setpoint        
            self.con.send(self.mode_obj)
            self.con.send(self.setp_obj)
            self.con.send(self.servoj_params_obj)
            self.con.send(self.enable_obj)

        elif mode == hp.VELOCITY_CONTROL_JOINT_MODE:    # JOINT SPEED CONTROL MODE
            hp.list_to_input(self.mode_obj, mode, 1, hp.MODE_INT_REGISTER, "int")
            hp.list_to_input(self.setp_obj, setp, 6, hp.SETP_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.avtr_obj, avtr, 4, hp.AVTR_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.enable_obj, enable, 1, hp.ENABLE_INT_REGISTER, "int")

            # send new setpoint        
            self.con.send(self.mode_obj)
            self.con.send(self.setp_obj)
            self.con.send(self.avtr_obj)
            self.con.send(self.enable_obj)

        elif mode == hp.POSITION_CONTROL_CARTESIAN_MODE:    # CARTESIAN POSITION CONTROL MODE
            hp.list_to_input(self.mode_obj, mode, 1, hp.MODE_INT_REGISTER, "int")
            hp.list_to_input(self.setp_obj, setp, 6, hp.SETP_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.servoj_params_obj, self.servoj_params, 3, hp.SERVOJ_PARAMS_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.enable_obj, enable, 1, hp.ENABLE_INT_REGISTER, "int")

            # send new setpoint        
            self.con.send(self.mode_obj)
            self.con.send(self.setp_obj)
            self.con.send(self.servoj_params_obj)
            self.con.send(self.enable_obj)


        elif mode == hp.IMPEDANCE_CONTROL_MODE:    # IMPEDANCE CONTROL MODE

            hp.list_to_input(self.mode_obj, mode, 1, hp.MODE_INT_REGISTER, "int")
            hp.list_to_input(self.setp_obj, setp, 6, hp.SETP_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.setp2_obj, setp2, 6, hp.SETP2_DOUBLE_REGISTER_OFFSET, "double")
            hp.list_to_input(self.enable_obj, enable, 1, hp.ENABLE_INT_REGISTER, "int")

            # send new setpoint        
            self.con.send(self.mode_obj)
            self.con.send(self.setp_obj)
            self.con.send(self.setp2_obj)
            self.con.send(self.enable_obj)

        self.ros_rate.sleep()

#########################################################################################
def main():
    rospy.init_node('controller', anonymous=True)

    urdriver = UrRtdeDriver()
    urdriver.connect_rtde()
    rospy.sleep(0.2)

    while not rospy.is_shutdown():
        urdriver.update()

    con.send_pause()
    con.disconnect()

if __name__ == '__main__':
    main()