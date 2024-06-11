#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
#import torch

from std_msgs.msg import Int32

#from model_utils import model as models
#import config as c


class ImgProcesser:
    def __init__(self):

        self.color = (255, 0, 0)
        self.font = cv2.FONT_ITALIC

        self.color_img = None
        self.depth_img = None
        self.object_img = None

        #self.model = models.YOLOv7()

        self.color_img_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_img_callback)
        self.depth_img_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_img_callback)
        self.object_img_sub = rospy.Subscriber('/yolov7/yolov7/visualization', Image, self.object_img_callback)

        self.front_depth_pub = rospy.Publisher('/front_depth', Int32, queue_size=10)

        print("init img process")

    def color_img_callback(self, data):
        img = np.fromstring(data.data, dtype=np.uint8)
        img = img.reshape(data.height, data.width, -1)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        self.color_img = img

    def depth_img_callback(self, data):
        img = np.fromstring(data.data, dtype=np.uint16)
        img = img.reshape(data.height, data.width)

        self.depth_img = img

    def object_img_callback(self, data):
        img = np.fromstring(data.data, dtype=np.uint8)
        img = img.reshape(data.height, data.width, -1)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        self.object_img = img

    def get_color_img(self):
        return self.color_img

    def get_depth_img(self):
        return self.depth_img

    def get_object_img(self):
        return self.object_img

    """
    def get_object_img(self):
        preds = self.model.get_pred(self.color_img)

        if preds is not None:
            boxes, labels = preds
            for box, cls in zip(boxes, labels):
                cv2.rectangle(self.color_img, (box[0], box[1]), (box[2], box[3]), self.color, 2)
                cv2.putText(self.color_img, f'cls:{cls}', (box[0], box[1]), self.font, 1, self.color, 2)
        return self.color_img
    """

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.depth_img is not None:
                h, w = self.depth_img.shape
                cy, cx = int(h/2), int(w/2)
                self.front_depth_pub.publish(self.depth_img[cy][cx])

if __name__=='__main__':
    rospy.init_node('img_processer', anonymous=False)

    i = ImgProcesser()
    i.main_loop()
