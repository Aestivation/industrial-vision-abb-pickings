# industrial-vision-abb-pickings
This code aims at picking and placing some objects on a fixed conveyor. For this, YOLO11 + XGBoost are used. YOLO detects the object  and draws a rectangle bounding box around; the center of the rectangle is regarded as a grasping point. Dealing with more complicated objects, orientation can be needed which is estimated by XGBoost here. 


A vision-guided robotic picking pipeline for sorting irregularly shaped objects on a conveyor, combining object detection, camera-to-robot coordinate calibration, and a learned grasp-orientation model to control an ABB industrial robot.

## How it works

1. Detection: a YOLO11 model, fine-tuned on a custom labeled dataset, detects objects in each camera frame and returns their class and bounding box.
2. Coordinate transform: a four-point homography calibration converts the object's pixel position into real-world millimeter coordinates in the robot's frame, correcting for the camera being mounted at an angle rather than straight down.
3. Grasp orientation: an XGBoost model, trained on bounding box geometry, predicts the angle the gripper should rotate to before picking up the object.
4. Robot control: the computed position, offset, and orientation are sent over a TCP socket to an ABB robot controller running a RAPID program, which parses the message and executes the pick-and-place motion.

**Repository structure:**

```
vision_pipeline/
    live_sorter.py       Python: camera capture, detection, coordinate transform,
                          angle prediction, and socket communication with the robot
robot_control/
    MainModule.mod        RAPID: socket server, message parsing, and robot motion
```

This repository contains the pipeline and integration code only. The training dataset, object images, and trained model weights (`best.pt`, `xgboost_decision_model.json`) are left out since they are confidential, and network details like the robot IP and file paths are replaced with placeholders (`YOUR_ROBOT_IP`, `models/best.pt`, etc.). To run this yourself, you'll need your own trained YOLO and XGBoost models, a completed homography calibration, and a Kinect v2 or similar RGB camera.

## Stack

- Python, OpenCV, [Ultralytics YOLO](https://github.com/ultralytics/ultralytics), XGBoost, NumPy
- PyKinect2 for camera capture
- RAPID (ABB robot programming language)
