#!/usr/bin/env python3
import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils

class Detector:
    def __init__(self):
        print("detect init")
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def pedestrian_exists(self, image):
        # resize image
        # cv2.imwrite("/home/gem/demo_ws/src/1.png", image)
        # print("111")

        image = imutils.resize(image, width=min(400, image.shape[1]))
        # print(image)
        # 其中窗口步长与Scale对结果影响最大，特别是Scale，
        # 小的尺度变化有利于检出低分辨率对象，同时也会导致FP发生，
        # 高的可以避免FP但是会产生FN（对象漏检）。
        # "detectMultiScale" Ref: https://pyimagesearch.com/2015/11/16/hog-detectmultiscale-parameters-explained/
        rects, weights = self.hog.detectMultiScale(
            image, winStride=(4, 4),
            padding=(8, 8),
            scale=1.25,
            useMeanshiftGrouping=False
        )
        # print(rects)
        threshold = 0.2
        for weight in weights:
            if weight > threshold:
                return True
        return False


# Ref: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
class Manager:
    def __init__(self):
        print("manager init")
        self.bridge = CvBridge()
        self.detector = Detector()
        # Ref: https://github.com/astuff/pacmod
        self.image_sub = rospy.Subscriber("/mako_1/mako_1/image_raw", Image, self.callback)
        self.brake_pub = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size = 1)
        self.enable_pub = rospy.Publisher('pacmod/as_rx/enable', Bool, queue_size = 1)
        self.forward_pub = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size = 1)
        self.turn_pub = rospy.Publisher("/pacmod/as_rx/turn_cmd",  PacmodCmd, queue_size = 1)
        # self.turn_cmd = PacmodCmd()
        rospy.spin()

    def callback(self, image):
        try:
            # print("running")
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            exists = self.detector.pedestrian_exists(cv_image)
            if exists:
                print("Pedestrian detected, braking")
                # self.brake_pub.publish(f64_cmd=0.3, enable=True)
                self.brake_pub.publish(ui16_cmd=1, enable=True)
                
            else:
                # self.brake_pub.publish(f64_cmd=1.0, enable=True)
                # print(1)
                self.forward_pub.publish(f64_cmd=0.3, enable=True)
                self.turn_pub.publish(ui16_cmd = 2)
        except CvBridgeError as e:
            print(e)


if __name__=="__main__":
    print("123")
    rospy.init_node("manager_node")
    m = Manager()

