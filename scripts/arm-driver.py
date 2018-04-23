#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
from roboclaw_interface import RoboClaw

class ArmDriver:

    def __init__(self):
        self.joint_names = ["shoulder",
                              "elbow"]
                              # "turret",
                              # "wrist_pitch",
                              # "wrist_yaw",
                              # "wrist_roll",
                              # "grip"]

        self.convert = { "shoulder" : ( 3 ,4.6 , 150 ,1650),
                       "elbow" : (3.5 , 5.2 , 60, 1400)}

        self.rc = RoboClaw(self.find_serial_port(), names = self.joint_names) # addresses = [128, 129, 130])

        rospy.init_node('arm-driver', anonymous=True)

        rospy.Subscriber("/joint_states", JointState, lambda x: self.callback(x) )

        rospy.spin()

    def callback(self, data):

        for joint in self.joint_names:

            if joint not in ['shoulder', 'elbow']:
                continue

            try:
                ind = data.name.index(joint)
            except ValueError:
                continue

            length = data.position[ind]
            position = self.scale(length, joint)
            self.rc.drive_position(joint, position)
            
    def scale(self, x, joint):
        return int((x - self.convert[joint][0] ) * (self.convert[joint][3] - self.convert[joint][2]) 
                            / (self.convert[joint][1] - self.convert[joint][0]) + self.convert[joint][2] )


    def find_serial_port(self):
        return '/dev/ttyACM0'
        return '/dev/tty.usbmodem1141'


if __name__ == '__main__':
    ad = ArmDriver()
