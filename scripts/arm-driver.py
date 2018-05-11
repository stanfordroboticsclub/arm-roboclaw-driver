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
                              "wrist_pitch"]
                              # "wrist_roll",
                              # "turret",
                              # "wrist_pitch",
                              # "grip"]

        self.motor_names = ["wrist_L",
                              "wrist_R"]

        # self.motor_names = ["shoulder",
        #                       "elbow",
        #                       "wrist_L",
        #                       "wrist_R"]
                              # "turret",
                              # "wrist_pitch",
                              # "grip"]

        self.manual_names = []
        # self.manual_names = ["shoulder",
        #                       "elbow"]
                              # "turret",
                              # "wrist_pitch",
                              # "grip"]

        self.pos = {'shoulder': 4, 
                        'elbow': 4,
                    'wrist_pitch': math.pi/2,
                    'wrist_yaw': math.pi/2}
                   

        self.convert = { "shoulder" : ( 3 ,4.6 , 150 ,1650),
                       "elbow" : (3.5 , 5.2 , 60, 1400),
                       "wrist_pitch" : (0 , math.pi , -10000 , 10000),
                       "wrist_yaw" : (0 , math.pi , -10000 , 10000),
                       }

        # self.rc = RoboClaw(self.find_serial_port(), names = self.motor_names) # addresses = [128, 129, 130])
        self.rc = RoboClaw(self.find_serial_port(), names = self.motor_names,addresses = [129] ) # addresses = [128, 129, 130])

        self.rc.speed['wrist_L'] = 5000
        self.rc.speed['wrist_R'] = 5000

        rospy.init_node('arm-driver', anonymous=True)
        rospy.Subscriber("/joint_states", JointState, lambda x: self.callback(x) )


        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo('new loop')
        # while 1:
            self.write_to_roboclaw()
            r.sleep()
            # rospy.spinOnce()


    def callback(self, data):
        rospy.loginfo('callback')
        for joint in self.joint_names:

            try:
                ind = data.name.index(joint)
            except ValueError:
                continue

            self.pos[joint] = data.position[ind]

        
        rospy.loginfo('call '+str(self.pos['wrist_pitch']) )

    def write_to_roboclaw(self):
        rospy.loginfo('writing')
        for joint in self.manual_names:
            position = self.scale(self.pos[joint], joint)
            self.rc.drive_position(joint, position)

        pulse_pitch = self.scale(self.pos['wrist_pitch'], 'wrist_pitch')
        pulse_yaw = self.scale(self.pos['wrist_yaw'], 'wrist_yaw')

        wrist_L_pulse = pulse_pitch + pulse_yaw
        wrist_R_pulse = pulse_pitch - pulse_yaw

        wrist_L_pulse = self.clamp(wrist_L_pulse, -10000,10000)
        wrist_R_pulse = self.clamp(wrist_R_pulse, -10000,10000)

        rospy.loginfo('writ '+str(self.pos['wrist_pitch']) )
        rospy.loginfo('wL')
        self.rc.drive_position('wrist_L', wrist_L_pulse)
        rospy.loginfo('wR')
        self.rc.drive_position('wrist_R', wrist_R_pulse)
        rospy.loginfo('done')

    def clamp(self,val, mi, ma):
        return min( [ max( [val,mi] ), ma])

            
    def scale(self, x, joint):
        return int((x - self.convert[joint][0] ) * (self.convert[joint][3] - self.convert[joint][2]) 
                            / (self.convert[joint][1] - self.convert[joint][0]) + self.convert[joint][2] )


    def find_serial_port(self):
        return '/dev/ttyACM0'
        return '/dev/tty.usbmodem1141'


if __name__ == '__main__':
    ad = ArmDriver()
