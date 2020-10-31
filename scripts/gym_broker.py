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
from robocars_msgs.msg import robocars_telemetry
from robocars_msgs.msg import robocars_switch

steering_order = 0.0
throttling_order = 0.0
braking_order = 0.0
last_reset_order = 0.0
reset_order = 0.0
image_pub = {}
telem_pub = {}
hostSimulator="localhost"
hostPort=9091
sceneName=""
camOffsetX=""
camOffsetY=""
camOffsetZ=""
camRotX=""
camFov=""

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

def switch_callback(data):
   global reset_order
   reset_order = data.switchs[1]

def initRosNode():
   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   global image_pub
   global telem_pub
   rospy.init_node('gym_broker', anonymous=False)
   rospy.Subscriber("/throttling_ctrl/output", robocars_actuator_output, throttling_callback, queue_size=1)
   rospy.Subscriber("/steering_ctrl/output", robocars_actuator_output, steering_callback, queue_size=1)
   rospy.Subscriber("/braking_ctrl/output", robocars_actuator_output, braking_callback, queue_size=1)
   rospy.Subscriber("/switch_ctrl/state", robocars_switch, switch_callback, queue_size=1)
   image_pub = rospy.Publisher("/gym/image", Image, queue_size=1)
   telem_pub = rospy.Publisher('/gym/telemetry', robocars_telemetry, queue_size=1)
   
def getConfig():
    global hostSimulator
    global hostPort
    global sceneName
    global camOffsetX
    global camOffsetY
    global camOffsetZ
    global camRotX
    global camFov

    if not rospy.has_param("simulatorHost"):
        rospy.set_param("simulatorHost", "localhost")
    hostSimulator = rospy.get_param("simulatorHost")
    if not rospy.has_param("simulatorPort"):
        rospy.set_param("simulatorPort", 9091)
    hostPort = rospy.get_param("simulatorPort")
    
    if not rospy.has_param("sceneName"):
        rospy.set_param("sceneName", "roboracingleague_1")
    sceneName = rospy.get_param("sceneName")

    if not rospy.has_param("camOffsetX"):
        rospy.set_param("camOffsetX", "0")
    camOffsetX = rospy.get_param("camOffsetX")

    if not rospy.has_param("camOffsetY"):
        rospy.set_param("camOffsetY", "1")
    camOffsetY = rospy.get_param("camOffsetY")

    if not rospy.has_param("camOffsetZ"):
        rospy.set_param("camOffsetZ", "3")
    camOffsetZ = rospy.get_param("camOffsetZ")

    if not rospy.has_param("camRotX"):
        rospy.set_param("camRotX", "75")
    camRotX = rospy.get_param("camRotX")

    if not rospy.has_param("camFov"):
        rospy.set_param("camFov", "150")
    camFov = rospy.get_param("camFov")


class SimpleClient(SDClient):

    def __init__(self, address, poll_socket_sleep_time=0.01):
        super().__init__(*address, poll_socket_sleep_time=poll_socket_sleep_time)
        self.last_image = None
        self.car_loaded = False

    def on_msg_recv(self, json_packet):
        global image_pub
        global telem_pub
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
            global sceneName
            rospy.loginfo("Available Scene(s) %s", str(json_packet["scene_names"]))
            # Load Scene message. Only one client needs to send the load scene.
            msg = '{ "msg_type" : "load_scene", "scene_name" : "%s" }' % (sceneName)
            self.send_now(msg)
            time.sleep(1)
            
        if json_packet['msg_type'] == "telemetry":
            if json_packet["hit"] != "none":
                rospy.loginfo("Hit "+str(json_packet["hit"]))
                #self.send_reset_car()
            imgString = json_packet["image"]
            imgRaw = base64.b64decode(imgString)
            img = np.frombuffer(imgRaw, dtype='uint8')
            cv_image = cv2.imdecode(img,cv2.IMREAD_COLOR)
            blured_img = cv2.medianBlur(cv_image, 5)
            image_message = bridge.cv2_to_imgmsg(blured_img, encoding="bgr8")
            if image_pub:
                image_pub.publish(image_message)
            telem_msg = robocars_telemetry()
            telem_msg.speed = json_packet["speed"]/20.0
            telem_msg.cte = json_packet["cte"]
            if telem_pub:
                telem_pub.publish(telem_msg)


        #print("got:", json_packet)

    def send_init(self):
        msg = '{ "msg_type" : "get_protocol_version" }'
        self.send_now(msg)
        time.sleep(1)

    def send_reset_car(self):
        msg = '{ "msg_type" : "reset_car" }'
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

        global camOffsetX
        global camOffsetY
        global camOffsetZ
        global camRotX

        msg = '{ "msg_type" : "cam_config", "fov" : "%s", "fish_eye_x" : "0.0", "fish_eye_y" : "0.0", "img_w" : "160", "img_h" : "120", "img_d" : "3", "img_enc" : "JPG", "offset_x" : "%s", "offset_y" : "%s", "offset_z" : "%s", "rot_x" : "%s" }' % (camFov, camOffsetX, camOffsetY, camOffsetZ, camRotX)
        rospy.loginfo("Cam config")
        rospy.loginfo(msg)
        self.send_now(msg)
        time.sleep(0.2)
        rospy.loginfo("socket polling timer %s", str(self.poll_socket_sleep_sec))

    def send_car_config(self):
        # Car config
        # body_style = "donkey" | "bare" | "car01" choice of string
        # body_rgb  = (128, 128, 128) tuple of ints
        car_name = "GrumpyCar"

        msg = '{ "msg_type" : "car_config", "body_style" : "car01", "body_r" : "0", "body_g" : "255", "body_b" : "0", "car_name" : "%s", "font_size" : "20" }' % (car_name)
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
        

    def send_controls(self, steering, throttle, brake):
        msg = { "msg_type" : "control",
                "steering" : steering.__str__(),
                "throttle" : throttle.__str__(),
                "brake" : brake.__str__() }
        
        self.send(json.dumps(msg))

        #this sleep lets the SDClient thread poll our message and send it out.
        time.sleep(self.poll_socket_sleep_sec)

    def update(self):
        # just random steering now
        global steering_order
        global throttling_order
        global braking_order
        global reset_order
        global last_reset_order
        st = steering_order 
        th = throttling_order
        brk = braking_order
        #if (braking_order > 0.2):
        #    th = - braking_order
        if (reset_order != last_reset_order):
            if (reset_order == 2):
                self.send_reset_car()
            reset_order = last_reset_order

        self.send_controls(st, th, brk)

def gym_broker():
    global hostSimulator
    global hostPort
    logging.basicConfig(level=logging.DEBUG)

    # test params
    port = 9090
    num_clients = 1
    clients = []

    initRosNode()
    getConfig()
    rospy.loginfo("Will connect to simulator host %s", str(hostSimulator))


    clients.append(SimpleClient(address=(hostSimulator, hostPort)))
    time.sleep(1)

    # Send random driving controls
    start = time.time()
    do_drive = True
    while do_drive and not rospy.is_shutdown() :
        for c in clients:
            c.update()
            if c.aborted:
                print("Client socket problem, stopping driving.")
                do_drive = False

    time.sleep(3.0)

    #Exit Scene - optionally..
    #msg = '{ "msg_type" : "exit_scene" }'
    #clients[0].send_now(msg)

    # Close down clients
    print("waiting for msg loop to stop")
    for c in clients:
        c.stop()

    print("clients to stopped")



if __name__ == '__main__':
   gym_broker()
