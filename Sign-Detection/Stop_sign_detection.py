import cv2 
import argparse
import imutils
from matplotlib import pyplot as plt

# Opening image 
arg_parse = argparse.ArgumentParser()
arg_parse.add_argument("-i", "--image", default=None, help="path to Image File ")
args = vars(arg_parse.parse_args())

image_path = args["image"]

img = cv2.imread(image_path) 
img = imutils.resize(img, width=min(400, img.shape[1]))
# OpenCV opens images as BRG  
# but we want it as RGB We'll  
# also need a grayscale version 
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
  
  
# Use minSize because for not  
# bothering with extra-small  
# dots that would look like STOP signs 
stop_data = cv2.CascadeClassifier('./model/stop_data.xml')
stop_sign = cv2.CascadeClassifier("./model/stopsign_classifier.xml")
turn_right = cv2.CascadeClassifier("./model/turnRight_ahead.xml")
turn_left = cv2.CascadeClassifier("./model/turnLeft_ahead.xml")
car = cv2.CascadeClassifier("./model/car.xml")
ppl = cv2.CascadeClassifier("./model/fullbody.xml")


  
found = stop_data.detectMultiScale(img_gray,  
                                   minSize =(20, 20))
Stop = stop_sign.detectMultiScale(img_gray,1.02,10)
Turn_Right = turn_right.detectMultiScale(img_gray,1.02,10)
Turn_Left = turn_left.detectMultiScale(img_gray,1.02,10)
car_res = car.detectMultiScale(img_gray,1.02,10)
ppl_res = ppl.detectMultiScale(img_gray,1.02,10)
# Don't do anything if there's  
# no sign 
amount_found = len(found) 
print(found)
print(Stop)
print(Turn_Right)
print(Turn_Left)
print(car_res)
print(ppl_res)
  
# if len(Turn_Right) >= 1: 
#     found = Turn_Right[0]
      
#     # There may be more than one 
#     # sign in the image 
#     for (x, y, width, height) in found: 
          
#         # We draw a green rectangle around 
#         # every recognized sign 
#         cv2.rectangle(img_rgb, (x, y),  
#                       (x + height, y + width),  
#                       (0, 255, 0), 5) 

# else:
#     print("Fail to find stop sign or stop sign more than 1")  
#     exit(0)    
# # Creates the environment of  
# # the picture and shows it 
# plt.subplot(1, 1, 1) 
# plt.imshow(img_rgb) 
# plt.show() 