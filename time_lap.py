#!/usr/bin/env python3

import rospy
import math
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import subprocess

start_x = 0.1
start_y = 0.0
prev_x = 0.0
prev_y = 0.0
prev_time = 0.0

class LapTime:
    def __init__(self):
        self.start_x = 0.1
        self.start_y = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_time = 0.0


        self.nb_lap = 0

        self.listener = rospy.Subscriber("/gt_pose", PoseStamped, self.callback)

        self.scheduler = rospy.Publisher('scheduler', String, queue_size=10)

        self.creation_time = time.time()

    def callback(self, data):
        cur_x = data.pose.position.x
        cur_y = data.pose.position.y
        t = rospy.Time.from_sec(time.time())
        cur_time = t.to_sec() #floating point
        dist = math.sqrt((cur_x-self.start_x)**2 + (cur_y-self.start_y)**2)

        if (dist < 1):
            if (self.prev_x < self.start_x) and (cur_x > self.start_x):
                print("New Lap")
                if (self.prev_time != 0.0 or (time.time() - self.creation_time) > 10):
                    print(f"Lap_time: {cur_time - self.prev_time:.3f}s")
                    self.nb_lap += 1
                self.prev_time = cur_time

                msg = String()
                msg.data = "TIME_LAP" + str(self.nb_lap)
                self.scheduler.publish(msg)
               
        self.prev_x  = cur_x
        self.prev_y = cur_y

        

if __name__ == '__main__':
    rospy.init_node('laptime', anonymous=True)
    laptime = LapTime()
    rospy.spin()

