#!/usr/bin/python

# Authors:  Filippos Sotiropoulos (fes@mit.edu)
#            Ryan Sandzimier (rsandz@mit.edu)

import rospy
import sensor_msgs.msg as smsg
from ur_lightweight_driver.msg import Command, Mode, Setpoint, Trajectory, ToolState
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import helper as hp
import math
import numpy as np
import threading
import tf

class CommandManager:
    def __init__(self):
        rospy.init_node('command_manager', anonymous=True)
        
        self.setpoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.setpoint2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


        self.mode           = hp.STANDBY_MODE # actual robot mode
        self.control_mode   = hp.DEFAULT_CONTROL_MODE # this nodes control mode

        self.trajectory  = []
        self.trajectory_lock = threading.Lock()

        self.enable         = 0 
        self.robot_ready  = False

        self.t = 0  

        self.actual_joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0]   
        self.actual_tool_pos = [0.0,0.0,0.0,0.0,0.0,0.0]    
 

        rospy.Subscriber('robot_ready', Bool, self.cb_robot_ready)
        rospy.Subscriber('trajectory', Trajectory, self.cb_trajectory)
        rospy.Subscriber('joint_states', JointState, self.cb_joint_states)
        rospy.Subscriber('tool_state', ToolState, self.cb_tool_state)

        rospy.Subscriber('mode', Mode, self.cb_mode)
        rospy.Subscriber('setpoint', Setpoint, self.cb_setpoint)
        
        #publishers command to the driver
        self.pub_command   = rospy.Publisher('command', Command, queue_size= 1)

        self.pub_trajectory_finished   = rospy.Publisher('trajectory_finished', Bool, queue_size= 1)

        rospy.Timer(rospy.Duration(0.002), self.cb_publish) 

    def cb_robot_ready(self, robot_ready_msg):
        self.robot_ready = robot_ready_msg.data 
        if not self.robot_ready and (self.mode != 0 or self.enable != 0):
            print "STANDBY mode engaged from program halt"
            self.enter_standby()

    def cb_joint_states(self, joint_states_msg):
        self.actual_joint_pos = joint_states_msg.position

    def cb_tool_state(self, tool_state_msg):
         pos = tool_state_msg.position
         self.actual_tool_pos = [pos.position.x,pos.position.y,pos.position.z,pos.orientation.x,pos.orientation.y,pos.orientation.z,pos.orientation.w]

    def cb_mode(self, mode_msg):
        if not self.robot_ready:
            print "Cannot process mode command while robot is not ready"
            return
        if mode_msg.mode == Mode.MODE_STANDBY:
            self.enter_standby()
        elif mode_msg.mode == Mode.MODE_FREEDRIVE:
            self.mode = hp.FREEDRIVE_MODE
            self.control_mode = hp.DEFAULT_CONTROL_MODE
            self.enable = 1
        else:
            self.enter_standby()

    def cb_setpoint(self, setpoint_msg):
        if not self.robot_ready:
            print "Cannot process setpoint command while robot is not ready"
            return
        # Need to check that setpoint is close enough to current position
        if setpoint_msg.type == Setpoint.TYPE_JOINT_POSITION:
            self.setpoint = setpoint_msg.setpoint
            self.mode = hp.POSITION_CONTROL_JOINT_MODE
            self.control_mode = hp.SETPOINT_CONTROL_MODE
            self.enable = 1
        elif setpoint_msg.type == Setpoint.TYPE_JOINT_VELOCITY:
            self.setpoint = setpoint_msg.setpoint
            self.mode = hp.VELOCITY_CONTROL_JOINT_MODE
            self.control_mode = hp.SETPOINT_CONTROL_MODE
            self.enable = 1
        elif setpoint_msg.type == Setpoint.TYPE_CARTESIAN_POSITION_IMPEDANCE:
            self.setpoint = setpoint_msg.setpoint
            self.setpoint2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.mode = hp.IMPEDANCE_CONTROL_MODE
            self.control_mode = hp.SETPOINT_CONTROL_MODE
            self.enable = 1
        else: 
            print "Setpoint type not supported right now. Entering standby mode"
            self.enter_standby()

    def cb_trajectory(self, trajectory_msg):
        if not self.robot_ready:
            print "Cannot process trajectory command while robot is not ready"
            return
        # Need to add case where first setpoint is at time 0.0
        trajectory = trajectory_msg.setpoints
        current_pos = Setpoint()
        current_pos.time = 0.0
        if len(trajectory) == 0 or trajectory[0].type == Setpoint.TYPE_JOINT_POSITION:
            current_pos.setpoint = self.actual_joint_pos
            current_pos.type = Setpoint.TYPE_JOINT_POSITION
        elif trajectory[0].type == Setpoint.TYPE_CARTESIAN_POSITION or trajectory[0].type == Setpoint.TYPE_CARTESIAN_POSITION_IMPEDANCE:
            tool_rpy = tf.transformations.euler_from_quaternion(self.actual_tool_pos[3:7])
            # Correct tool_rpy so that it is closest to first setpoint in trajectory
            # i.e. if current tool roll is -pi and first setpoint in trajectory has roll +pi/2, should correct roll to be pi instead of -pi (equivalent, but "closer")
            diff = np.array(trajectory[0].setpoint[3:7])-np.array(tool_rpy)
            tool_rpy = np.array(tool_rpy) + np.array((diff + np.sign(diff)*math.pi)/(2*math.pi),dtype=int)*2*math.pi
            current_pos.setpoint = self.actual_tool_pos[0:3] + list(tool_rpy)
            current_pos.type = trajectory[0].type
        else:
            print "Setpoint type not supported right now"
            return

        trajectory = [current_pos] + trajectory
        for i in range(0,len(trajectory)-1):
            if not self.checkContinuity(trajectory[i],trajectory[i+1]): 
                print "Trajectory not continuous. Entering standby mode"
                self.enter_standby()
                return
        if len(trajectory) > 0 and trajectory[0].type == Setpoint.TYPE_JOINT_POSITION:
            self.trajectory_lock.acquire()
            self.t = 0
            self.trajectory = trajectory
            self.mode = hp.POSITION_CONTROL_JOINT_MODE
            self.control_mode = hp.TRAJECTORY_CONTROL_MODE
            self.enable = 1
            self.time_start_trajectory = rospy.get_rostime().to_sec()
            self.trajectory_lock.release()
        elif len(trajectory) > 0 and (trajectory[0].type == Setpoint.TYPE_CARTESIAN_POSITION or trajectory[0].type == Setpoint.TYPE_CARTESIAN_POSITION_IMPEDANCE):
            for i in range(0,len(trajectory)): # Convert euler angles to rotation vectors
                rpy = trajectory[i].setpoint[3:6]
                trajectory[i].setpoint = list(trajectory[i].setpoint[0:3]) + hp.quat_to_rotvec(tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2]))
            self.trajectory_lock.acquire()
            self.t = 0
            self.trajectory = trajectory
            if trajectory[0].type == Setpoint.TYPE_CARTESIAN_POSITION:
                self.mode = hp.POSITION_CONTROL_CARTESIAN_MODE
            else:
                self.mode = hp.IMPEDANCE_CONTROL_MODE

            self.control_mode = hp.TRAJECTORY_CONTROL_MODE
            self.enable = 1
            self.time_start_trajectory = rospy.get_rostime().to_sec()
            self.trajectory_lock.release()

    def cb_publish(self, event):
        command_msg = Command()

        # Trajectory following mode
        if (self.mode == hp.POSITION_CONTROL_JOINT_MODE or self.mode == hp.POSITION_CONTROL_CARTESIAN_MODE or self.mode == hp.IMPEDANCE_CONTROL_MODE) and (self.control_mode == hp.TRAJECTORY_CONTROL_MODE):
            self.t = rospy.get_rostime().to_sec() - self.time_start_trajectory 
            self.trajectory_lock.acquire()
            while len(self.trajectory) > 1 and self.t >= self.trajectory[1].time:
                self.trajectory.pop(0)
            if (len(self.trajectory) == 1 and self.t >= self.trajectory[0].time) or len(self.trajectory) > 1:
                self.setpoint = self.next_setp()
                if self.mode == hp.IMPEDANCE_CONTROL_MODE:
                    self.setpoint2 = self.next_setp_velocity()
            if self.mode == hp.POSITION_CONTROL_CARTESIAN_MODE or self.mode == hp.IMPEDANCE_CONTROL_MODE:
                actual_val = self.actual_tool_pos[0:3] + hp.quat_to_rotvec(self.actual_tool_pos[3:7])
            else:
                actual_val = self.actual_joint_pos
            if len(self.trajectory) == 1 and (hp.diff_norm_within_tol(self.trajectory[0].setpoint, actual_val, 0.001) or (self.mode == hp.IMPEDANCE_CONTROL_MODE and self.t - self.trajectory[0].time > 1.0)):
                self.finish_condition_duration = self.finish_condition_duration + 0.002
            else:
                self.finish_condition_duration = 0.0
            if self.finish_condition_duration > 0.05:
                print "Finished trajectory. Switching to STANDBY MODE"
                self.finish_condition_duration = 0.0
                self.pub_trajectory_finished.publish(True)
                self.enter_standby()
            self.trajectory_lock.release()
        if self.setpoint is None:
            print "Setpoint is None. Entering standby"
            self.enter_standby()

        command_msg.mode = self.mode
        command_msg.enable = self.enable

        command_msg.setpoint = self.setpoint

        if self.mode == hp.VELOCITY_CONTROL_JOINT_MODE:
            command_msg.avtr    = [5.0,0.0,0.01,0.0]
        else:
            command_msg.avtr    = [1.4,1.05,0.0,0.0]

        if self.mode == hp.IMPEDANCE_CONTROL_MODE:
            command_msg.setpoint2 = self.setpoint2

        self.pub_command.publish(command_msg)

    def enter_standby(self):
        self.mode = hp.STANDBY_MODE
        self.control_mode = hp.DEFAULT_CONTROL_MODE
        self.enable = 0

    def next_setp(self):
        if len(self.trajectory) == 0:
            return None
        if len(self.trajectory) == 1:
            return self.trajectory[0].setpoint
        return self.interpolate_linear(self.trajectory[0], self.trajectory[1], self.t)

    def next_setp_velocity(self):
        if len(self.trajectory) == 0:
            return None
        if len(self.trajectory) == 1:
            return [0.0]*6
        sp1 = self.trajectory[0]
        sp2 = self.trajectory[1]
        if sp1.type == Setpoint.TYPE_CARTESIAN_POSITION or sp1.type == Setpoint.TYPE_CARTESIAN_POSITION_IMPEDANCE:
            q1 = hp.rotvec_to_quat(sp1.setpoint[3:6])
            q2 = hp.rotvec_to_quat(sp2.setpoint[3:6])
            diff = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(q1),q2)
            rv = np.array(hp.quat_to_rotvec(diff))/(sp2.time - sp1.time)
            rv = np.matmul(tf.transformations.quaternion_matrix(q1)[0:3,0:3],rv)
            p = (np.array(sp2.setpoint[0:3]) - np.array(sp1.setpoint[0:3]))/(sp2.time-sp1.time)
            return list(np.concatenate((p,rv)))            
        else:   
            return list((np.array(sp2.setpoint)-np.array(sp1.setpoint))/(sp2.time-sp1.time))

    def interpolate_linear(self, sp1, sp2, t):
        if sp1.time >= sp2.time or t < sp1.time or t > sp2.time:
            print "Cannot interpolate between waypoints"
            return None
        if sp1.type == Setpoint.TYPE_CARTESIAN_POSITION or sp1.type == Setpoint.TYPE_CARTESIAN_POSITION_IMPEDANCE:
            q1 = hp.rotvec_to_quat(sp1.setpoint[3:6])
            q2 = hp.rotvec_to_quat(sp2.setpoint[3:6])
            q = tf.transformations.quaternion_slerp(q1, q2, (t - sp1.time)/(sp2.time - sp1.time))
            rv = hp.quat_to_rotvec(q)
            p = np.array(sp1.setpoint[0:3]) + (t - sp1.time)*(np.array(sp2.setpoint[0:3]) - np.array(sp1.setpoint[0:3]))/(sp2.time-sp1.time)
            return list(np.concatenate((p,rv)))
        else:
            return list(np.array(sp1.setpoint) + (t - sp1.time)*(np.array(sp2.setpoint) - np.array(sp1.setpoint))/(sp2.time-sp1.time))
  
    def checkContinuity(self, sp1, sp2):
        if sp1.time >= sp2.time:
            print "Time continuity error"
            return False
        if sp1.type != sp2.type:
            print "Type continuity error"
            return False
        if len(sp1.setpoint)!=6 or len(sp2.setpoint)!=6:
            print "Setpoints not correct format"
            return False
        if sp1.type == Setpoint.TYPE_JOINT_POSITION:
            p1 = np.array(sp1.setpoint)
            p2 = np.array(sp2.setpoint)
            if not np.all((p1 >= hp.JOINT_POSITION_MIN_LIMITS)&(p1 <= hp.JOINT_POSITION_MAX_LIMITS)):
                print "Setpoint out of range"
                return False
            if not np.all((p2 >= hp.JOINT_POSITION_MIN_LIMITS)&(p2 <= hp.JOINT_POSITION_MAX_LIMITS)):
                print "Setpoint out of range"
                return False
            if not np.all(np.abs(p2-p1)/(sp2.time-sp1.time) <= hp.JOINT_VELOCITY_LIMITS):
                print "Setpoint velocity out of range"
                return False
        elif sp1.type == Setpoint.TYPE_CARTESIAN_POSITION or sp1.type == Setpoint.TYPE_CARTESIAN_POSITION_IMPEDANCE:
            p1 = np.array(sp1.setpoint)
            p2 = np.array(sp2.setpoint)
            if not np.all((p1 >= hp.CARTESIAN_POSITION_MIN_LIMITS)&(p1 <= hp.CARTESIAN_POSITION_MAX_LIMITS)):
                print "Setpoint out of range"
                return False
            if not np.all((p2 >= hp.CARTESIAN_POSITION_MIN_LIMITS)&(p2 <= hp.CARTESIAN_POSITION_MAX_LIMITS)):
                print "Setpoint out of range"
                return False
            if not np.all(np.abs(p2-p1)/(sp2.time-sp1.time) <= hp.CARTESIAN_VELOCITY_LIMITS):
                print "Setpoint velocity out of range"
                return False
        else:
            print "Setpoint type not yet supported for trajectories"
            return False

        return True

if __name__ == '__main__':
    cm = CommandManager()
    
    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass