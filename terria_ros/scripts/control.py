#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import serial
import sys
import threading
import time

from std_msgs.msg import Int8, Int32

class Control:
    def __init__(self):

        self.port = None 
        self.init_port()

        self.flag = False
        self.button = None

        self.button_sub = rospy.Subscriber('/button', Int8, self.button_callback)

        self.depth = 0
        self.depth_sub = rospy.Subscriber('/front_depth', Int32, self.depth_callback)

    def button_callback(self, data):
        self.button = data.data
        """
        if data.data == 0:
            self.flag = True
            self.port.write(str(data.data).encode())
        elif data.data == 1:
            self.flag = False
            self.port.write(str(self.button).encode())
        else:
            self.button = data.data
            print(self.button)
        """

    def depth_callback(self, data):
        self.depth = data.data

    def init_port(self):
        self.port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        if not self.port.is_open:
            print('fail to open port')
            sys.exit(1)
    
    def main_loop(self):
        while True:
            if 10 < self.depth < 250:
                self.port.write(str(5).encode())
                time.sleep(1)
                print('stop')
            else:
                if self.button is not None:
                    print(self.button)
                    self.port.write(str(self.button).encode())
                    self.button = None

if __name__=='__main__':
    rospy.init_node('control', anonymous=False)

    c = Control()

    t1 = threading.Thread(target=c.main_loop, daemon=True)
    t1.start()
    
    while not rospy.is_shutdown():
        rospy.spin()
