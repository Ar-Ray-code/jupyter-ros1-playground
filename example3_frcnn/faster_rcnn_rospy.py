
import rospy
from sensor_msgs.msg import Image
import sys
from cv_bridge import CvBridge

import numpy as np
import pandas as pd

import glob
import cv2
 
import torch
import torchvision
from torchvision import transforms
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torch.utils.data import TensorDataset
import os

import time


class_name = "car"
weight = "model.pt"
test_path = "test"
output_path = "result"
dataset_class=[class_name]
colors = ((0,0,0),(255,0,0))


device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu') 

data_class=dataset_class
data_class.insert(0, class_name)
classes = tuple(data_class)

model=torch.load(weight)

model.to(device)

model.eval()


## Publish setting
pub = rospy.Publisher("output_image", Image, queue_size=1)

def process_image(msg):
    global pub
    try:
        bridge = CvBridge()
        img_bgr = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        img_bgr = cv2.resize(img_bgr,(512*img_bgr.shape[:2][1]//img_bgr.shape[:2][0],512))
        img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        
        image_tensor = torchvision.transforms.functional.to_tensor(img)
    
        with torch.no_grad():
            prediction = model([image_tensor.to(device)])
    
        for i,box in enumerate(prediction[0]['boxes']):
            score = prediction[0]['scores'][i].cpu().numpy()
            if score > 0.5:
                score = round(float(score),2)
                cat = prediction[0]['labels'][i].cpu().numpy()
                cat = 1
                txt = '{} {}'.format(classes[int(cat)], str(score))
                font = cv2.FONT_HERSHEY_SIMPLEX
                cat_size = cv2.getTextSize(txt, font, 0.5, 2)[0]
                c = colors[int(cat)]
                box=box.cpu().numpy().astype('int')
                cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), c , 2)
                cv2.rectangle(img,(box[0], box[1] - cat_size[1] - 2),(box[0] + cat_size[0], box[1] - 2), c, -1)
                cv2.putText(img, txt, (box[0], box[1] - 2), font, 0.5, (0, 0, 0), thickness=1, lineType=cv2.LINE_AA)
                output_img = bridge.cv2_to_imgmsg(img,"rgb8")
                pub.publish(output_img)
    except Exception as err:
        print (err)

def start_node():
    rospy.init_node('faster_rcnn')
    rospy.Subscriber("camera/color/image_raw", Image, process_image)
    rospy.spin()

try:
    start_node()
except rospy.ROSInterruptException as err:
    print(err)

