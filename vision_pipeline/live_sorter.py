import time
try:
    time.clock = time.perf_counter
except:
    pass

import socket
import cv2
import numpy as np
import json
import xgboost as xgb  # Import XGBoost
from ultralytics import YOLO
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

# ==========================================
# 0. OFFSET SETTINGS (adjust here)
# ==========================================
# Only compon4 and compon6 get an offset. Every other object gets 0,0,0.
OFFSETS = {
    "compon4": {"x": 0.0, "y": 50.0, "z": 0.0},
    "compon6": {"x": 10.0, "y": -18.0, "z": -30.0},
}
DEFAULT_OFFSET = {"x": 0.0, "y": 0.0, "z": 0.0}

# ==========================================
# 1. LOAD YOLO, CALIBRATION AND XGBOOST
# ==========================================
# NOTE: model/data paths below are placeholders. Point these at your own
# trained weights and calibration file — none are included in this repo.
YOLO_WEIGHTS_PATH = "models/best.pt"
CALIBRATION_PATH = "config/calibration.json"
XGBOOST_MODEL_PATH = "models/xgboost_decision_model.json"

model = YOLO(YOLO_WEIGHTS_PATH)

# Load homography calibration
try:
    with open(CALIBRATION_PATH) as f:
        _kal = json.load(f)
    HOMOGRAPHY = np.array(_kal["homography"], dtype=np.float64)
    print("Homography calibration loaded!")
except FileNotFoundError:
    raise SystemExit(f"Could not find {CALIBRATION_PATH}.")

def pixel_to_robot_mm(pixel_x, pixel_y):
    vec = HOMOGRAPHY @ np.array([pixel_x, pixel_y, 1.0])
    vec /= vec[2]
    return float(vec[0]), float(vec[1])

# Load XGBoost
xgb_model = xgb.XGBClassifier()
xgb_model.load_model(XGBOOST_MODEL_PATH)
print("XGBoost model loaded!")

# ==========================================
# 2. CONNECT TO THE ABB ROBOT
# ==========================================
# NOTE: replace with your robot controller's actual IP and port.
ROBOT_IP = "YOUR_ROBOT_IP"
ROBOT_PORT = 1025
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print("Trying to connect to the ABB robot...")
try:
    s.connect((ROBOT_IP, ROBOT_PORT))
    print("Connected!")
except Exception as e:
    print(f"WARNING: no robot ({e}). Running in offline test mode.")

# ==========================================
# 3. START KINECT AND LIVE LOOP
# ==========================================
print("Starting Kinect v2... (please wait)")
kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color)

while True:
    if kinect.has_new_color_frame():
        frame = kinect.get_last_color_frame()
        frame = np.reshape(frame, (1080, 1920, 4))
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        results = model.predict(source=frame_bgr, show=False, conf=0.4)
        r = results[0]

        if len(r.boxes) > 0:
            for box in r.boxes:
                material = model.names[int(box.cls)]

                # YOLO coordinates
                pixel_x = int(box.xywh[0][0].item())
                pixel_y = int(box.xywh[0][1].item())
                box_w = float(box.xywh[0][2].item())
                box_h = float(box.xywh[0][3].item())

                # translate to robot-frame position
                robot_x_mm, robot_y_mm = pixel_to_robot_mm(pixel_x, pixel_y)

                # ---- OFFSET: only compon4 / compon6, otherwise 0 ----
                offset = OFFSETS.get(material, DEFAULT_OFFSET)
                robot_x_mm = robot_x_mm + offset["x"]
                robot_y_mm = robot_y_mm + offset["y"]
                robot_z_mm = offset["z"]

                robot_x_mm = round(robot_x_mm, 1)
                robot_y_mm = round(robot_y_mm, 1)
                robot_z_mm = round(robot_z_mm, 1)

                # -----------------------------------------------------
                # XGBOOST: predict grasp angle from box geometry
                # -----------------------------------------------------
                features = np.array([[pixel_x, pixel_y, box_w, box_h]])
                angle = float(xgb_model.predict(features)[0])
                angle = round(angle, 1)

                print(f"Camera sees {material} at X: {robot_x_mm}, Y: {robot_y_mm}, Z: {robot_z_mm}. XGBoost angle: {angle}")

                # send 5 values to the ABB robot
                message = f"{material},{robot_x_mm},{robot_y_mm},{robot_z_mm},{angle}"
                try:
                    s.send(message.encode())
                    time.sleep(5)
                except:
                    pass

                break

        annotated_frame = r.plot()
        annotated_frame_small = cv2.resize(annotated_frame, (960, 540))
        cv2.imshow("Vision-Guided Picking", annotated_frame_small)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

kinect.close()
cv2.destroyAllWindows()
s.close()
