# Stop Sign/Pedestrian/Vehicle Detection

## Introudction
This project involves using computer vision and machine learning techniques to accurately identify and classify different types of objects in the environment, such as **stop signs**, **pedestrians**, and **vehicles**. 

Upon detecting target objects, the venicle will apply the brakes, and once all target objects are no longer present, it will resume driving.

## Steps
Open the 1st terminal, launch the joystick controller.
```
source devel/setup.bash
roslaunch basic_launch gem_dbw_joystick.launch
```

Open the 2nd terminal, launch the basic sensors.
```
source devel/setup.bash
roslaunch basic_launch gem_sensor_init.launch
```

Open the 3rd terminal
```
source devel/setup.bash
rosrun [PackageName] detect.py
```

## Links
- Haar Cascade Classifier: https://github.com/opencv/opencv/tree/master/data/haarcascades

- Computer Vision — Detecting objects using Haar Cascade Classifier: https://towardsdatascience.com/computer-vision-detecting-objects-using-haar-cascade-classifier-4585472829a9

- Traffic Sign Detection: https://github.com/Bassel-A-Elazab/Traffic-Sign-Detection-Python-OpenCV-

- Traffic signs detection and classification in real time: https://github.com/hoanglehaithanh/Traffic-Sign-Detection

- Guide to make Custom Haar Cascade XML file for Object detection with OpenCV: https://medium.com/@vipulgote4/guide-to-make-custom-haar-cascade-xml-file-for-object-detection-with-opencv-6932e22c3f0e

- Make Your Own Object Detector using Haar Cascade (Python & OpenCv): https://github.com/Bassel-A-Elazab/Make-Your-Own-Object-Detector


## Reference
- Hbaieb, Amal, Jihene Rezgui, and Lamia Chaari. "Pedestrian detection for autonomous driving within cooperative communication system." 2019 IEEE Wireless Communications and Networking Conference (WCNC). IEEE, 2019.

- Gupta, Abhishek, et al. "Deep learning for object detection and scene perception in self-driving cars: Survey, challenges, and open issues." Array 10 (2021): 100057.
