#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Time

class imu_sensor():
    def __init__(self, *args):
        if not args:
            self.delimiter = ';'
        else:
            self.delimiter = args[0]
        
        self.acceleration = Vector3()
        self.velocity = Vector3()
        self.time = rospy.Time()
        self.new_flag = True

    def update(self, msg_list):
        flag = False
        if(len(msg_list)==5 and msg_list[4] != ''):
            if msg_list[0] == 'a':
                try: 
                    self.acceleration.x = float(msg_list[2])
                    self.acceleration.y = float(msg_list[3])
                    self.acceleration.z = float(msg_list[4])
                    self.time = rospy.Time.from_sec(float(msg_list[1])*10e-9)
                    flag = True
                except ValueError:
                    pass
            elif msg_list[0] == 'y':
                try:
                    self.velocity.x = float(msg_list[2])
                    self.velocity.y = float(msg_list[3])
                    self.velocity.z = float(msg_list[4])
                    self.time = rospy.Time.from_sec(float(msg_list[1])*10e-9)
                    flag = True
                except ValueError:
                    pass
        self.new_flag = flag
        return flag


    def receiver(self, data):
        batch = data.split("\n")
        acc_flag = True
        vell_flag = True
        for msg in reversed(batch):
            msg_list = msg.split(self.delimiter)
            if (msg_list[0] == 'a' and acc_flag):
                acc_flag = not(self.update(msg_list))
            elif (msg_list[0] == 'y' and vell_flag):
                vell_flag = not(self.update(msg_list))   
            elif (not(acc_flag) and not(vell_flag)):
                break
