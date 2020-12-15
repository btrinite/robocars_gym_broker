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
import itertools

from gym_donkeycar.core.sim_client import SDClient

from robocars_msgs.msg import robocars_radio_channels
from robocars_msgs.msg import robocars_actuator_output
from robocars_msgs.msg import robocars_telemetry
from robocars_msgs.msg import robocars_switch
from robocars_msgs.msg import robocars_brain_state
from std_msgs.msg import Int16

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
_count=0
num_clients="1"
clients = {}

bridge = CvBridge()

def throttling_callback(data):
   #rospy.loginfo(rospy.get_caller_id() + " throttling %s", str(data.norm))
   if clients.get(data.header.frame_id) != None:
        clients[data.header.frame_id].set_throttle(data.norm)

def steering_callback(data):
   #rospy.loginfo(rospy.get_caller_id() + " steering %s", str(data.norm))
   if clients.get(data.header.frame_id) != None:
       clients[data.header.frame_id].set_steering(-data.norm)

def braking_callback(data):
   #rospy.loginfo(rospy.get_caller_id() + " steering %s", str(data.norm))
   if clients.get(data.header.frame_id) != None:
       clients[data.header.frame_id].set_braking(-data.norm)

def switch_callback(data):
    for c in clients:
        clients[c].set_reset_order(data.switchs[1])

def state_callback(data):
    for c in clients:
        clients[c].set_state(data.state)

def rc_connect_sim_callback(data):
    if (data.data == 1):
        connect()
    else:
        disconnect()
        SimpleClient.initIdEnumerator()

def rc_reset_car_callback(data):
    for c in clients:
        clients[c].set_reset_order(data.data)

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
   rospy.Subscriber("/robocars_brain_state", robocars_brain_state, state_callback, queue_size=1)
   rospy.Subscriber("/remote_control/connect_sim", Int16, rc_connect_sim_callback, queue_size=1)
   rospy.Subscriber("/remote_control/reset_car", Int16, rc_reset_car_callback, queue_size=1)
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
    global num_clients

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

    if not rospy.has_param("num_clients"):
        rospy.set_param("num_clients", "1")
    num_clients = rospy.get_param("num_clients")

class SimpleClient(SDClient):

    id_iter = itertools.count(0)

    @staticmethod
    def initIdEnumerator():

        SimpleClient.id_iter = itertools.count(0)

    def __init__(self, address, poll_socket_sleep_time=0.01):
        super().__init__(*address, poll_socket_sleep_time=poll_socket_sleep_time)
        self.last_image = None
        self.car_loaded = False
        self.id = next(SimpleClient.id_iter)
        self.state = robocars_brain_state.BRAIN_STATE_IDLE
        self.last_state = -1
        self.throttle = 0
        self.steering = 0
        self.braking = 0
        self.reset_order = 0
        self.last_reset_order = 0

    def getId(self):
        return self.id

    def set_throttle(self, throttle):
        self.throttle=throttle

    def set_steering(self, steering):
        self.steering=steering

    def set_braking(self, braking):
        self.braking=braking

    def set_reset_order(self, reset_order):
        if ( reset_order==2):
            if (self.last_reset_order==0):
                self.reset_order=reset_order
            else:
                self.last_reset_order=0

    def set_state(self, state):
        self.state=state

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
            image_message.header.frame_id=str(self.id)
            if image_pub:
                image_pub.publish(image_message)
            telem_msg = robocars_telemetry()
            telem_msg.speed = json_packet["speed"]/20.0
            telem_msg.cte = json_packet["cte"]
            telem_msg.header.frame_id=str(self.id)
            if telem_pub:
                telem_pub.publish(telem_msg)


        #print("got:", json_packet)

    def send_init(self):
        msg = '{ "msg_type" : "get_protocol_version" }'
        self.send_now(msg)
        time.sleep(1)

    def send_reset_car(self):
        rospy.loginfo("Send reset order")
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
        #rospy.loginfo("socket polling timer %s", str(self.poll_socket_sleep_sec))

    def send_car_config(self, r=192, g=192, b=192):
        # Car config
        # body_style = "donkey" | "bare" | "car01" choice of string
        # body_rgb  = (128, 128, 128) tuple of ints
        car_name = "GrumpyCar"+str(self.id)

        msg = '{ "msg_type" : "car_config", "body_style" : "car01", "body_r" : "%s", "body_g" : "%s", "body_b" : "%s", "car_name" : "%s", "font_size" : "20" }' % (r, g, b, car_name)
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
        global _count
        _count=_count+1
        msg = { "msg_type" : "control",
                "steering" : steering.__str__(),
                "throttle" : throttle.__str__(),
                "brake" : brake.__str__(), 
                "_count" : _count}
        self.send(json.dumps(msg))

        #this sleep lets the SDClient thread poll our message and send it out.
        time.sleep(self.poll_socket_sleep_sec)

    def update(self):
        # just random steering now

        st = self.steering
        th = self.throttle
        brk = self.braking
        #if (braking_order > 0.2):
        #    th = - braking_order
        if (self.reset_order != self.last_reset_order):
            if (self.reset_order == 2):
                self.send_reset_car()
                self.last_reset_order = self.reset_order

        if (self.state != self.last_state):
            self.last_state = self.state
            rospy.loginfo("brain state change event %s", str(self.state))
            if (self.state == robocars_brain_state.BRAIN_STATE_IDLE):
                self.send_car_config(0,255,0)
            if (self.state == robocars_brain_state.BRAIN_STATE_MANUAL_DRIVING):
                self.send_car_config(255,0,0)
            if (self.state == robocars_brain_state.BRAIN_STATE_AUTONOMOUS_DRIVING):
                self.send_car_config(0,0,255)
        self.send_controls(st, th, brk)

def connect():
    global hostSimulator
    global hostPort
    global num_clients
    global clients

    rospy.loginfo("Will connect to simulator host %s", str(hostSimulator))
    rospy.loginfo("Will create %s cars instances", str(num_clients))

    for _ in range(0, int(num_clients)):
        c = SimpleClient(address=(hostSimulator, hostPort))
        clients[str(c.getId())]=c

    time.sleep(1)

def disconnect():
    global clients
    for c in list(clients):
        clients[c].stop()
        del clients[c]


def gym_broker():

    logging.basicConfig(level=logging.DEBUG)

    # test params
    port = 9090

    initRosNode()
    getConfig()
    rate = rospy.Rate(40) # ROS Rate at 40Hz
        
    # Send random driving controls
    start = time.time()
    do_drive = True
    while do_drive and not rospy.is_shutdown() :
        for c in list(clients):
            try:
                clients[c].update()
                if clients[c].aborted:
                    print("Client socket problem, stopping driving.")
                    do_drive = False
            except:
                pass
        rate.sleep()

    time.sleep(3.0)
    disconnect()
    #Exit Scene - optionally..
    #msg = '{ "msg_type" : "exit_scene" }'
    #clients[0].send_now(msg)

    # Close down clients
    print("waiting for msg loop to stop")

    print("clients to stopped")



if __name__ == '__main__':
   gym_broker()
