#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
from roboclaw_interface import RoboClaw
import math

class ArmDriver:

    def __init__(self):
        self.joint_names = ["shoulder",
                              "elbow",
                              "wrist_yaw",
                              "wrist_roll"]
                              # "turret",
                              # "wrist_pitch",
                              # "grip"]

        self.motor_names = ["shoulder",
                              "elbow",
                              "wrist_L",
                              "wrist_R"]
                              # "turret",
                              # "wrist_pitch",
                              # "grip"]

        self.manual_names = ["shoulder",
                              "elbow"]
                              # "turret",
                              # "wrist_pitch",
                              # "grip"]

        self.pos = {'shoulder': 4, 
                        'elbow': 4}

        self.convert = { "shoulder" : ( 3 ,4.6 , 150 ,1650),
                       "elbow" : (3.5 , 5.2 , 60, 1400),
                       "wrists" : (-math.pi , math.pi , -8000 , 8000),
                       }

        self.rc = RoboClaw(self.find_serial_port(), names = self.joint_names) # addresses = [128, 129, 130])

        rospy.init_node('arm-driver', anonymous=True)

        rospy.Subscriber("/joint_states", JointState, lambda x: self.callback(x) )


        r = rospy.rate(10)
        while not rospy.is_shutdown():
            self.write_to_roboclaw()
            r.sleep()
            # rospy.spinOnce()


    def callback(self, data):

        for joint in self.joint_names:

            if joint not in ['shoulder', 'elbow']:
                continue
            try:
                ind = data.name.index(joint)
            except ValueError:
                continue

            self.pos[joint] = data.position[ind]

    def write_to_roboclaw(self):
        for joint in self.manual_names:
            position = self.scale(self.pos.[joint], joint)
            self.rc.drive_position(joint, position)


        wrist_L_pos = self.pos[wrist_pitch] + self.pos[wrist_yaw] - math.pi
        wrist_R_pos = self.pos[wrist_pitch] - self.pos[wrist_yaw]

        self.rc.drive_position(joint, position)
        self.rc.drive_position(joint, position)

            
    def scale(self, x, joint):
        return int((x - self.convert[joint][0] ) * (self.convert[joint][3] - self.convert[joint][2]) 
                            / (self.convert[joint][1] - self.convert[joint][0]) + self.convert[joint][2] )


    def find_serial_port(self):
        return '/dev/ttyACM0'
        return '/dev/tty.usbmodem1141'


if __name__ == '__main__':
    ad = ArmDriver()
