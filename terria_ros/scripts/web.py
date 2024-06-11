#!/usr/bin/env python3
import rospy
from flask import Flask, request, render_template, Response
import cv2
import numpy as np
import serial
import sys

import config as c
from web_utils.web import Button

class Control:
    def __init__(self):
        self.app = Flask(__name__, 
                         template_folder=c.TEMPLATES_PATH)
        self.setup_routes()

        self.b = Button()

    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.app.route('/video_feed')
        def video_feed():
            return self.b.video_feed()

        @self.app.route('/get_data', methods=['POST'])
        def get_data():
            return self.b.get_data()

if __name__=='__main__':
    rospy.init_node('web', anonymous=False)

    my = Control()
    my.app.run(host='172.30.1.36', port=5000, debug=True)
