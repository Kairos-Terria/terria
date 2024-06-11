#!/usr/bin/env python3
import rospy
from flask import Flask, request, render_template, Response
import cv2
import numpy as np
import serial
import sys

from std_msgs.msg import Int8
from sensor_msgs.msg import Image

from img_main import ImgProcesser

import buttons as b

class Button:
    def __init__(self):

        self.info = ImgProcesser()
        self.buttons = b.BUTTONS

        self.cnt = 0

        self.start_pub = rospy.Publisher('/start_end', Int8, queue_size=10)
        self.pub = rospy.Publisher('/button', Int8, queue_size=10)

    def main_loop(self):
        while True:
            color_img = self.info.get_object_img()
    
            if color_img is not None:
                ret, buffer = cv2.imencode('.jpg', color_img)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    def index(self):
        return render_template('index.html')

    def video_feed(self):
        return Response(self.main_loop(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def get_data(self):
        button_clicked = request.form['button']
        self.pub.publish(self.buttons[button_clicked])
        return '', 204
