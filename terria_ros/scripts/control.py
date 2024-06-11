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
        self.mode = 'manual'
        self.button = None

        self.button_sub = rospy.Subscriber('/button', Int8, self.button_callback)

        self.depth = 0
        self.depth_sub = rospy.Subscriber('/front_depth', Int32, self.depth_callback)

    def button_callback(self, data):
        if data.data == 0:
            self.flag = True
        elif data.data == 1:
            self.flag = False

        elif data.data == 2:
            self.mode = 'manual'
        elif data.data == 3:
            self.mode = 'auto'

        else:
            self.button = data.data

    def depth_callback(self, data):
        self.depth = data.data

    def init_port(self):
        self.port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        if not self.port.is_open:
            print('fail to open port')
            sys.exit(1)
    
    def move(self):
        self.port.write(str(1).encode())

    def main_loop(self):
        while True:
            if 10 < self.depth < 250:
                self.port.write(str(3).encode())
                time.sleep(1)
                print('stop')
            else:
                if self.button is not None and self.mode=='manual':
                    print(self.button)
                    self.port.write(str(self.button).encode())
                    self.button = None
                elif self.mode == 'auto':
                    while self.mode == 'auto':
                        if 10 < self.depth < 250:
                            self.port.write(str(4).encode())
                        print('move')
                        time.sleep(1)


if __name__=='__main__':
    rospy.init_node('control', anonymous=False)

    c = Control()

    t1 = threading.Thread(target=c.main_loop, daemon=True)
    t1.start()
    
    while not rospy.is_shutdown():
        rospy.spin()
