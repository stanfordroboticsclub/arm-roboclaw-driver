#!/usr/bin/env python
from __future__ import division

import rospy
from sensor_msgs.msg import JointState
from roboclaw_interface import RoboClaw
import math
import time


class ArmDriver:

    def __init__(self):
        self.on_time = time.time()
        self.joint_names = ["shoulder",
                              "elbow",
                              "wrist_yaw",
                              "wrist_pitch",
                              "wrist_roll",
                              "turret",
                              "grip"]

        self.motor_names = ["shoulder",
                              "elbow",
                              "wrist_L",
                              "wrist_R",
                              "wrist_roll",
                              "turret",
                              "grip",
                               'extra']

        self.manual_names = ["wrist_roll",
                              "turret"]

        self.pos = {'shoulder': 4, 
                        'elbow': 4,
                    'wrist_pitch': math.pi/2,
                    'wrist_yaw': math.pi/2,
                    'grip':1,
                    'turret': 0,
                    'wrist_roll':0}

        self.offset = {'shoulder': 0, 
                        'elbow': 0,
                    'wrist_pitch': 0,
                    'wrist_yaw': 0,
                    'turret': 0,
                       'grip':0}
        
        self.last_grip = 0

        self.last_shoulder_command = 0
        self.last_elbow_command = 0
        self.shoulder_done = True
        self.elbow_done = True

        self.convert = { "shoulder" : ( 2.93 ,4.54 , 150 ,1580),
                       "elbow" : (3.52 , 5.12 , 60, 1500),
                       "wrist_pitch" : (0 , math.pi , -10382 , 10382),
                       "wrist_yaw" : (0 , math.pi , -10382 , 10382),
                       "grip" : (0.5 , 1 , 50 , 120),
                       "wrist_roll" : (-math.pi , math.pi, -10000 , 10000),
                       "turret" : (math.pi/2 , -math.pi/2, -8651 , 8651)
                       }

        self.rc = RoboClaw(self.find_serial_port(), names = self.motor_names,addresses = [128,129,130,131] ) # addresses = [128, 129, 130])

        self.rc.speed['wrist_L'] = 3000
        self.rc.speed['wrist_R'] = 3000

        self.rc.speed['shoulder'] = 80
        self.rc.speed['elbow'] = 120

        rospy.init_node('arm-driver', anonymous=True)
        rospy.Subscriber("/joint_states", JointState, lambda x: self.callback(x) )


        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            rospy.loginfo('new loop')
        # while 1:
            if( time.time() - self.on_time > 5):
                self.write_to_roboclaw()
            r.sleep()
            # rospy.spinOnce()


    def callback(self, data):
        # rospy.loginfo('callback')
        for joint in self.joint_names:

            try:
                ind = data.name.index(joint)
            except ValueError:
                continue

            self.pos[joint] = data.position[ind]
            self.offset[joint] = int(data.effort[ind])

        
        # rospy.loginfo('call '+str(self.pos['wrist_pitch']) )

    def write_to_roboclaw(self):
        rospy.loginfo('writing')
        for joint in self.manual_names:
            position = self.scale(self.pos[joint], joint)
            offset = int(self.offset[joint])
            self.rc.drive_position(joint, position + offset)

        # do manual calc for shoulder
        position_shoulder = self.scale(self.pos['shoulder'], 'shoulder')
        offset_shoulder = self.offset['shoulder']
        shoulder_setpoint = position_shoulder + offset_shoulder

        if math.fabs(self.rc.read_encoder('shoulder')[1] - shoulder_setpoint) < 3:
            self.shoulder_done = True

        if shoulder_setpoint != self.last_shoulder_command:
            self.shoulder_done = False

        if self.shoulder_done:
            self.rc.drive_duty('shoulder',0)
        else:
            self.rc.drive_position('shoulder', position_shoulder + offset_shoulder)

        self.last_shoulder_command = shoulder_setpoint


        # do manual calc for elbow
        position_elbow = self.scale(self.pos['elbow'], 'elbow')
        offset_elbow = self.offset['elbow']
        elbow_setpoint = position_elbow + offset_elbow

        if math.fabs(self.rc.read_encoder('elbow')[1] - elbow_setpoint) < 3:
            self.elbow_done = True

        if elbow_setpoint != self.last_elbow_command:
            self.elbow_done = False

        if self.elbow_done:
            self.rc.drive_duty('elbow',0)
        else:
            self.rc.drive_position('elbow', position_elbow + offset_elbow)

        self.last_elbow_command = elbow_setpoint

        # position_elbow = self.scale(self.pos['elbow'], 'elbow')
        # offset_elbow = self.offset['elbow']
        # self.rc.drive_position('elbow', position_elbow + offset_elbow)


        #Do Manual calc for wrist
        pulse_pitch = self.scale(self.pos['wrist_pitch'], 'wrist_pitch')
        pulse_yaw = self.scale(self.pos['wrist_yaw'], 'wrist_yaw')

        wrist_L_pulse =  - pulse_pitch + pulse_yaw
        wrist_R_pulse =  - pulse_pitch - pulse_yaw

        wrist_L_pulse = self.clamp(wrist_L_pulse, -10000,10000) \
                            - self.offset[ 'wrist_pitch'] + self.offset[ 'wrist_yaw']
        wrist_R_pulse = self.clamp(wrist_R_pulse, -10000,10000) \
                            - self.offset[ 'wrist_pitch'] - self.offset[ 'wrist_yaw']


        print "WRIST L", wrist_L_pulse
        print "WRIST R", wrist_R_pulse

        out = self.rc.drive_position('wrist_L', wrist_L_pulse)
        out2 = self.rc.drive_position('wrist_R', wrist_R_pulse)



        #Do Manual calc for gripper
        position = self.scale(self.pos['grip'], 'grip')
        offset = self.offset['grip']
        print "GRIP", position + offset
        self.rc.drive_duty('grip', position + offset)


        rospy.loginfo(str(out))

    def clamp(self,val, mi, ma):
        return min( [ max( [val,mi] ), ma])

            
    def scale(self, x, joint):
        out =  int((x - self.convert[joint][0] ) * (self.convert[joint][3] - self.convert[joint][2]) 
                            / (self.convert[joint][1] - self.convert[joint][0]) + self.convert[joint][2] )

        out = self.clamp(out,self.convert[joint][2], self.convert[joint][3])
        return out


    def find_serial_port(self):
        return '/dev/ttyUSB0'
        return '/dev/ttyACM0'
        return '/dev/tty.usbmodem1141'


if __name__ == '__main__':
    ad = ArmDriver()
