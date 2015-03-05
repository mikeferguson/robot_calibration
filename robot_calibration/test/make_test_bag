#!/usr/bin/env python

from __future__ import print_function

import string

import rospy
import rosbag
import tf

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from calibration_sandbox.msg import CalibrationData

class GenerateTestBag:
    camera_frame_ = 'head_camera_rgb_optical_frame'
    last_state_ = None # last joint states
    last_point_ = None # last observed point, transformed to camera_frame_

    def __init__(self):
        rospy.init_node('generate_test_bag')
        rospy.Subscriber("joint_states", JointState, self.stateCb)
        self.listener = tf.TransformListener()

        # bag to write data to
        bag = rosbag.Bag('calibration_data.bag', 'w')

        # write the URDF
        description = String()
        description.data = rospy.get_param('robot_description')
        bag.write('robot_description', description )

        # put samples in
        while not rospy.is_shutdown():
            print('Move arm/head to a new sample position:')
            resp = raw_input('press <enter>')
            if string.upper(resp) == 'EXIT':
                break
            else:
                if self.last_state_ == None or self.last_point_ == None:
                    print('Cannot save state')
                    continue
                # save a sample
                msg = CalibrationData()
                msg.joint_states = self.last_state_
                msg.rgbd_observations.append(self.last_point_)
                print(msg.joint_states, msg.rgbd_observations[0])
                msg.header.stamp = msg.joint_states.header.stamp
                msg.header.frame_id = self.camera_frame_
                bag.write('calibration_data', msg)
        bag.close()
            

    def stateCb(self, msg):
        """ Callback for joint_states messages """
        p_in = PointStamped()
        p_in.header.frame_id = "gripper_link"
        p_in.header.stamp = rospy.Time(0)

        try:
            p_out = self.listener.transformPoint(self.camera_frame_, p_in)
            self.last_point_ = p_out.point
            self.last_state_ = msg
        except:
            pass


if __name__=='__main__':
    GenerateTestBag()
