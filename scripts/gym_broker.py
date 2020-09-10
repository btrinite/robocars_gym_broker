#!/usr/bin/env python
import rospy
import os
import random
import json
import time
from io import BytesIO
import base64
import logging
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

from gym_donkeycar.core.sim_client import SDClient

from robocars_msgs.msg import robocars_radio_channels
from robocars_msgs.msg import robocars_actuator_output

steering_order = 0.0
throttling_order = 0.0
braking_order = 0.0
image_pub = {}
host="localhost"
bridge = CvBridge()

def throttling_callback(data):
   #rospy.loginfo(rospy.get_caller_id() + " throttling %s", str(data.norm))
   global throttling_order
   throttling_order = data.norm

def steering_callback(data):
   #rospy.loginfo(rospy.get_caller_id() + " steering %s", str(data.norm))
   global steering_order
   steering_order = -data.norm

def braking_callback(data):
   #rospy.loginfo(rospy.get_caller_id() + " steering %s", str(data.norm))
   global braking_order
   braking_order = -data.norm

def initRosNode():
   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   global image_pub
   rospy.init_node('gym_broker', anonymous=True)
   rospy.Subscriber("/throttling_ctrl/output", robocars_actuator_output, throttling_callback)
   rospy.Subscriber("/steering_ctrl/output", robocars_actuator_output, steering_callback)
   rospy.Subscriber("/braking_ctrl/output", robocars_actuator_output, braking_callback)
   image_pub = rospy.Publisher("/gym/image", Image, queue_size=10)
   
def getRosConfig():
    global host
    if rospy.has_param("simulatorHost"):
        rospy.set_param("simulatorHost", "localhost")
    host = rospy.get_param("simulatorHost")


class SimpleClient(SDClient):

    def __init__(self, address, poll_socket_sleep_time=0.01):
        super().__init__(*address, poll_socket_sleep_time=poll_socket_sleep_time)
        self.last_image = None
        self.car_loaded = False

    def on_msg_recv(self, json_packet):
        global image_pub
        if (json_packet["msg_type"] != "telemetry"):
            rospy.loginfo("GYM got message %s", str(json_packet["msg_type"]))

        if json_packet['msg_type'] == "car_loaded":
            rospy.loginfo("car_loaded event received")
            self.car_loaded = True
            self.send_car_config()
            self.send_cam_config()

        if json_packet['msg_type'] == "protocol_version":
            rospy.loginfo("GYM Protocol Version %s", str(json_packet["version"]))

        if json_packet['msg_type'] == "scene_selection_ready":
            rospy.loginfo("Scene Selection Ready")
            msg = '{ "msg_type" : "get_scene_names" }'
            self.send_now(msg)
            time.sleep(1)

        if json_packet['msg_type'] == "scene_names":
            rospy.loginfo("Available Scene(s) %s", str(json_packet["scene_names"]))
            # Load Scene message. Only one client needs to send the load scene.
            msg = '{ "msg_type" : "load_scene", "scene_name" : "warehouse" }'
            self.send_now(msg)
            time.sleep(1)
            
        if json_packet['msg_type'] == "telemetry":
            imgString = json_packet["image"]
            imgRaw = base64.b64decode(imgString)
            img = np.frombuffer(imgRaw, dtype='uint8')
            cv_image = cv2.imdecode(img,cv2.IMREAD_COLOR)
            image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            if image_pub:
                image_pub.publish(image_message)

        #print("got:", json_packet)

    def send_init(self):
        msg = '{ "msg_type" : "get_protocol_version" }'
        self.send_now(msg)
        time.sleep(1)

    def send_cam_config(self):
        # Camera config
        # set any field to Zero to get the default camera setting.
        # this will position the camera right above the car, with max fisheye and wide fov
        # this also changes the img output to 255x255x1 ( actually 255x255x3 just all three channels have same value)
        # the offset_x moves camera left/right
        # the offset_y moves camera up/down
        # the offset_z moves camera forward/back
        # with fish_eye_x/y == 0.0 then you get no distortion
        # img_enc can be one of JPG|PNG|TGA        
        msg = '{ "msg_type" : "cam_config", "fov" : "150", "fish_eye_x" : "0.0", "fish_eye_y" : "0.0", "img_w" : "160", "img_h" : "120", "img_d" : "3", "img_enc" : "JPG", "offset_x" : "0.0", "offset_y" : "1.0", "offset_z" : "3.0", "rot_x" : "75.0" }'
        self.send_now(msg)
        time.sleep(0.2)

    def send_car_config(self):
        # Car config
        # body_style = "donkey" | "bare" | "car01" choice of string
        # body_rgb  = (128, 128, 128) tuple of ints
        car_name = "GrumpyCar"

        msg = '{ "msg_type" : "car_config", "body_style" : "car01", "body_r" : "0", "body_g" : "255", "body_b" : "0", "car_name" : "%s", "font_size" : "100" }' % (car_name)
        self.send_now(msg)

        #this sleep gives the car time to spawn. Once it's spawned, it's ready for the camera config.
        time.sleep(0.2)

    def send_racer_config(self):
        '''
        send three config messages to setup car, racer, and camera
        '''
        racer_name = "Deep4L"
        car_name = "GrumpyCar"
        bio = "Reborn in ROS suit."
        country = "Montesson"
        guid = "RACYPMURG"

        # Racer info
        msg = {'msg_type': 'racer_info',
            'racer_name': racer_name,
            'car_name' : car_name,
            'bio' : bio,
            'country' : country,
            'guid' : guid }
        self.send_now(json.dumps(msg))
        time.sleep(0.2)
        

    def send_controls(self, steering, throttle):
        msg = { "msg_type" : "control",
                "steering" : steering.__str__(),
                "throttle" : throttle.__str__(),
                "brake" : "0.0" }
        self.send(json.dumps(msg))

        #this sleep lets the SDClient thread poll our message and send it out.
        time.sleep(self.poll_socket_sleep_sec)

    def update(self):
        # just random steering now
        global steering_order
        global throttling_order
        global braking_order
        st = steering_order 
        th = throttling_order
        if (braking_order > 0.2):
            th = - braking_order
        self.send_controls(st, th)

def gym_broker():
    global host
    logging.basicConfig(level=logging.DEBUG)

    # test params
    host = "FRLA000254-2.local" # "trainmydonkey.com" for virtual racing server
    port = 9091
    num_clients = 1
    clients = []

    initRosNode()
    getRosConfig()

    clients.append(SimpleClient(address=(host, port)))
    time.sleep(1)

    # Send random driving controls
    start = time.time()
    do_drive = True
    r = rospy.Rate(120) # 60hz
    while do_drive and not rospy.is_shutdown() :
        for c in clients:
            c.update()
            if c.aborted:
                print("Client socket problem, stopping driving.")
                do_drive = False
        r.sleep()

    time.sleep(3.0)

    #Exit Scene - optionally..
    msg = '{ "msg_type" : "exit_scene" }'
    clients[0].send_now(msg)

    # Close down clients
    print("waiting for msg loop to stop")
    for c in clients:
        c.stop()

    print("clients to stopped")



if __name__ == '__main__':
   gym_broker()
