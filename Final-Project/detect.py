#!/usr/bin/env python3

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils

# Speed Control
from steer_pid_controller import steer_pid_controller

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy
import roslib

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from sensor_msgs.msg import Image
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd



class Detector:
    def __init__(self):
        # human detect
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        # traffic sign detect
        self.stop_detector = cv2.CascadeClassifier('/home/gem/demo_ws/src/vehicle_drivers/gem_gnss_control/scripts/models/stop_data.xml')
        self.car_detector = cv2.CascadeClassifier('/home/gem/demo_ws/src/vehicle_drivers/gem_gnss_control/scripts/models/car_data.xml')
        self.people_detector = cv2.CascadeClassifier('/home/gem/demo_ws/src/vehicle_drivers/gem_gnss_control/scripts/models/people_data.xml')


    def pedestrian_exists(self, image):
        # resize image
        image = imutils.resize(image, width=min(400, image.shape[1]))
        # "detectMultiScale" Ref: https://pyimagesearch.com/2015/11/16/hog-detectmultiscale-parameters-explained/
        rects, weights = self.hog.detectMultiScale(
            image, winStride=(4, 4),
            padding=(8, 8),
            scale=1.25,
            useMeanshiftGrouping=False
        )
        threshold = 0.2
        for weight in weights:
            if weight > threshold:
                return True
        return False

    def stop_exists(self, image):
        # resize image
        image = imutils.resize(image, width=min(400, image.shape[1]))
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = self.stop_detector.detectMultiScale(image_gray, minSize =(20, 20))
        return len(found) != 0

    def car_exists(self, image):
        # resize image
        image = imutils.resize(image, width=min(400, image.shape[1]))
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = self.car_detector.detectMultiScale(image_gray, minSize =(20, 20))
        return len(found) != 0

    def people_exists(self, image):
        # resize image
        image = imutils.resize(image, width=min(400, image.shape[1]))
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = self.people_detector.detectMultiScale(image_gray, minSize =(20, 20))
        return len(found) != 0


class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0

    def reset(self):
        self.iterm  = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de

class OnlineFilter(object):

    def __init__(self, cutoff, fs, order):
        
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted


# Ref: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
class Manager:
    def __init__(self):
    	
        # Ref: https://github.com/astuff/pacmod
        # self.image_sub = rospy.Subscriber("/mako_1/mako_1/image_raw", Image, self.callback)
        self.detector = Detector()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed2/zed_node/left_raw/image_raw_gray", Image, self.image_callback)


        self.speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.speed_callback)
        self.speed      = 0.0

        self.desired_speed = 0.75  # m/s, reference speed
        self.max_accel     = 0.4 # % of acceleration
        self.pid_speed     = PID(1.2, 0.2, 0.6, wg=20)
        self.speed_filter  = OnlineFilter(1.2, 30, 4)


        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size = 1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = True
        self.brake_cmd.f64_cmd = 0.0

        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size = 1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = True
        self.accel_cmd.f64_cmd = 0.0

        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second

        self.gear_pub = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.enable = True
        self.gear_cmd.ui16_cmd = 3
        
        print("init done")
        rospy.spin()

    def speed_callback(self, msg):
        self.speed = round(msg.data, 3) # forward velocity in m/s


    def reset_cmd_value(self):
        self.brake_cmd.f64_cmd = 0.0
        self.accel_cmd.f64_cmd = 0.0
        self.turn_cmd.ui16_cmd = 1
        self.gear_cmd.ui16_cmd = 3


    def image_callback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            # detectionget_control
            pedestrian_exists = 0
            stop_exists = 0

            # default setting, straight, no accel
            output_accel = 0.0

            # pedestrian/stop sign comes first
            # pedestrian_exists = self.detector.pedestrian_exists(cv_image)
            pedestrian_exists = self.detector.people_exists(cv_image)
            stop_exists = self.detector.stop_exists(cv_image)
            car_exists = self.detector.car_exists(cv_image)


            self.brake_cmd.enable = True
            self.brake_cmd.clear = False
            self.brake_cmd.ignore = False

            self.accel_cmd.enable = True
            self.accel_cmd.clear = False
            self.accel_cmd.ignore = False

            
            if (pedestrian_exists or stop_exists or car_exists):

                self.brake_cmd.f64_cmd = 0.7
                self.accel_cmd.f64_cmd = 0

                if (pedestrian_exists):
                    print("pedestrian_exists...")
                elif stop_exists:
                    print("stop sign exist...")
                else:
                    print("car exist...")

            else:
                current_time = rospy.get_time()
                filt_vel     = self.speed_filter.get_data(self.speed)
                output_accel = self.pid_speed.get_control(current_time, self.desired_speed - filt_vel)
                    
                # print("======= raw accel: ", output_accel)
                # fix accel
                if output_accel > self.max_accel:
                    output_accel = self.max_accel

                if output_accel < 0.4:
                    output_accel = 0.4
                
                # if over desired speed, then do not accel
                if (self.speed > self.desired_speed):
                    output_accel = 0.0

                self.accel_cmd.f64_cmd = output_accel
                self.brake_cmd.f64_cmd = 0.0
                self.gear_cmd.ui16_cmd = 3
                self.gear_pub.publish(self.gear_cmd)



            self.brake_pub.publish(self.brake_cmd)
            self.accel_pub.publish(self.accel_cmd)
            print("Final Acceleration: ", self.accel_cmd.f64_cmd)
            print("Final Brake: ", self.brake_cmd.f64_cmd)
            print("Speed: ", self.speed)
            self.reset_cmd_value()
            


        except CvBridgeError as e:
            print(e)


if __name__=="__main__":
    print("..")
    rospy.init_node("manager_node")
    m = Manager()
    

