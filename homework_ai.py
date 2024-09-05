import numpy as np
import cv2
import matplotlib as plt

img = cv2.imread('/Users/ray/Desktop/yolo_person_data/JPEG/1.jpg')
print('origin image shape is ', img.shape)


def svd_compression(img, k):
    res_image = np.zeros_like(img)
    for i in range(img.shape[2]):

        U, Sigma, VT = np.linalg.svd(img[:,:,i])
        res_image[:, :, i] = U[:,:k].dot(np.diag(Sigma[:k])).dot(VT[:k,:])

    return res_image

def MSE(imageA, imageB):

    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])

    return err
'''
x = []
y = []
for i in range(50, 500):
    print(i)
    res = svd_compression(img, k=i)
    mse = MSE(img, res)
    x.append(i)
    y.append(mse)
plt.xlabel("K value")
plt.ylabel("MSE")
plt.plot(x,y)
plt.show()
'''
res = svd_compression(img, k=423)
mse = MSE(img, res)
psnr = cv2.PSNR(img, res)
print(mse)
print(psnr)
m = img.shape[0]
n = img.shape[1]
k = 423
ratio = (m*n)/(k*(m+n+k))
print(ratio)

cv2.imshow('origin', img)
cv2.imshow('img', res)
cv2.waitKey(0)

'''
res1 = svd_compression(img, k=425)
mse1 = MSE(img, res1)
print(mse1)
res2 = svd_compression(img, k=423)
mse2 = MSE(img, res2)
print(mse2)
res3 = svd_compression(img, k=421)
mse3 = MSE(img, res3)
print(mse3)
res4 = svd_compression(img, k=419)
mse4 = MSE(img, res4)
print(mse4)
res5 = svd_compression(img, k=417)
mse5 = MSE(img, res5)
print(mse5)
res6 = svd_compression(img, k=415)
mse6 = MSE(img, res6)
print(mse6)
res7 = svd_compression(img, k=413)
mse7 = MSE(img, res7)
print(mse7)
res8 = svd_compression(img, k=411)
mse8 = MSE(img, res8)
print(mse8)

row11 = np.hstack((res1, res2))
row22 = np.hstack((res3, res4))
res = np.vstack((row11, row22))

row33 = np.hstack((res5, res6))
row44 = np.hstack((res7, res8))
res0 = np.vstack((row33, row44))

cv2.imshow('origin', img)
cv2.imshow('img', res)
cv2.imshow('img0', res0)
cv2.waitKey(0)
'''












import os
import math
import time
import argparse
import numpy as np
import tensorrt as trt
import inference as inf

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO
from deep_sort.deep.feature_extractor_trt import TrackerExtractor
from deep_sort.sort.nn_matching import NearestNeighborDistanceMetric
from deep_sort.sort.preprocessing import non_max_suppression
from deep_sort.sort.detection import Detection
from deep_sort.sort.tracker import Tracker

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from threading import Thread
import Jetson.GPIO as GPIO



####################################
#deepsort
####################################
WINDOW_NAME = 'TrtYOLODemo'
"initialize deepsort"
model_path = "./deep_sort/deep/checkpoint/deepsort.engine"
extractor = TrackerExtractor(model_path)
max_cosine_distance = 0.2 #0.2
nn_budget = 100
min_confidence=0.3 #0.3
nms_max_overlap=1.0
metric = NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
tracker = Tracker(metric, max_iou_distance=0.7, max_age=70, n_init=3)


def parse_args():
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-t', '--conf_thresh', type=float, default=0.3,
        help='set the detection confidence threshold')#0.3
    parser.add_argument(
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish|yolov4-p5]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args



#################################################
#Ultrasonic sensor
#################################################

def distance(GPIO_TRIGGER, GPIO_ECHO):
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime_n = time.time()
        if (StartTime_n - StartTime > 0.0003):
            break

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime_n = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime_n - StartTime_n
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

class ultrasonic_dist(Thread):

    def __init__(self, GPIO_TRIGGER, GPIO_ECHO):
        Thread.__init__(self)
        self.GPIO_TRIGGER = GPIO_TRIGGER
        self.GPIO_ECHO = GPIO_ECHO
        #GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)
        #set GPIO direction (IN / OUT)
        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(GPIO_ECHO, GPIO.IN)

    def distance(self):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            print(0)
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            print(1)
            StopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        return distance

    def get_dist(self):
        try:
            self.dist = distance(self.GPIO_TRIGGER, self.GPIO_ECHO)
            return self.dist
        except:
            return 9999

#################################################
#dronekit control
#################################################

def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def set_attitude(roll_angle=0.0, pitch_angle=0.0,
                 yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                 thrust=0.5):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)

    time.sleep(0.5)
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)
    time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(-roll_angle, -pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)

    time.sleep(0.1)
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def calculate(img, box):
    y,x = img.shape[:2]
    x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
    x_mean = (x1+x2)/2
    height = y2 - y1
    length = x2 - x1
    m = max(height/y, length/x)
    diff = abs(x_mean - x/2)
    if (x_mean - x/2) > 10:
        #set_attitude(yaw_angle=10, thrust=0.5)
        yaw = (10 + diff/30 )*m
        print("right", yaw)
        return yaw
    elif (x_mean - x/2) < -10:
        #set_attitude(yaw_angle=-10, thrust=0.5)
        yaw = -(10 + diff/30 )*m
        print("left", yaw)
        return yaw

#################################################
#Deepsort
#################################################

def _get_features(bbox_xywh, ori_img, clss):
    im_crops = []
    index = []
    zero_picture = 0
    for box in bbox_xywh:
        x, y, w, h = box[0], box[1], box[2], box[3]
        im = ori_img[int(y):int(y+h), int(x):int(x+w)]
        im_crops.append(im)
    features = extractor.track_extractor(im_crops)
    return features

def l2_normalize(x, axis=-1, epsilon=1e-10):
    output = x / np.sqrt(np.maximum(np.sum(np.square(x), axis=axis, keepdims=True), epsilon))
    return output

def loop_and_detect(cam, trt_yolo, conf_th, vis, cls_dict):
    """Continuously capture images from camera and do object detection.

    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    ####################################
    #yolo_deepsort
    ####################################
    full_scrn = False
    first_time = True
    fps = 0.0
    tic = time.time()
    x = 0
    user = []
    ####################################
    #video
    ####################################
    img = cam.read()
    width = int(img.shape[1])    # 取得影像寬度
    height = int(img.shape[0])  # 取得影像高度
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')          # 設定影片的格式為 MJPG
    out = cv2.VideoWriter('result/%d.mp4'%time.time(), fourcc, 20.0, (width,  height))  # 產生空的影片
    ####################################
    #dist
    ####################################
    Ultrasonic_forward = ultrasonic_dist(9, 25)
    Ultrasonic_forward.start()
    real_height = 167
    target_dist = 100
    non_zero = False
    dist = []
    d = 0

    while True:
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        img = cam.read()
        index = 0
        delete_index = []
        if img is None:
            break
        yaw = None
        vis_height = None
        pitch = 0
        ####################################
        #dist
        ####################################
        if(x%3 == 0):
            dist_forward = Ultrasonic_forward.get_dist() #200cm
            if(2 <= dist_forward <= 400):
                dist.append(dist_forward)
                d += 1
                non_zero = True
        if (x%1 == 0):
            ####################################
            #yolo
            ####################################
            boxes, confs, clss = trt_yolo.detect(img, conf_th)
            for box in boxes:
                xmin, ymin, xmax, ymax = box[0], box[1], box[2], box[3]
                width = xmax - xmin
                height = ymax - ymin
                box[0], box[1], box[2], box[3] = xmin, ymin, width, height
            for cls in clss:
                if cls != 0:
                    print("cls:",cls)
                    delete_index.append(index)
                index += 1
            boxes = np.delete(boxes, delete_index, axis=0)
            confs = np.delete(confs, delete_index, axis=0)
            clss = np.delete(clss, delete_index, axis=0)
            ####################################
            #deepsort
            ####################################
            features = _get_features(boxes, img, clss)
            detections = [Detection(boxes[i], clss[i], conf, features[i]) for i, conf in enumerate(confs) if conf > min_confidence]
            # run non-maxima supression
            boxs = np.array([d.tlwh for d in detections])
            scores = np.array([d.confidence for d in detections])
            classes = np.array([d.label for d in detections])
            indices = non_max_suppression(boxs, nms_max_overlap, scores)
            detections = [detections[i] for i in indices]

            # Call the tracker
            tracker.predict()
            tracker.update(detections)

            # update tracks
            track_boxs = []
            track_labels = []
            track_ids = []
            for track in tracker.tracks:
                if not track.is_confirmed() or track.time_since_update > 1:
                    continue
                track_label = track.track_label
                cls_name = cls_dict.get(track_label, 'CLS{}'.format(track_label))
                bbox = track.to_tlbr()
                if user == []:
                    track_boxs.append(bbox)
                    track_labels.append(track_label)
                    track_id = track.track_id
                    print("user:", track_id)
                    user.append(track_id)
                    track_ids.append(track_id)
                else:
                    track_boxs.append(bbox)
                    track_labels.append(track_label)
                    track_id = track.track_id
                    track_ids.append(track_id)
                    if (track_id == user[0]):
                       x_min, y_min, x_max, y_max = bbox[0], bbox[1], bbox[2], bbox[3]
                       vis_height = bbox[3] - bbox[1]
                       cv2.rectangle(img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 0, 255), 5)
            if(x%30 == 0 and vis_height != None and non_zero):
                #print("dist:", dist, "d:", d)
                dist_sum = sum(dist)/d
                d = 0
                dist = []
                non_zero = False
                print("dist:", dist_sum)
                f = (vis_height*dist_sum)/real_height
                #pitch_angle = 15*math.sin((dist_sum-100)/100)
                """if (dist_sum - target_dist) > 30:
                    print("forward")
                    #pitch = -10
                    set_attitude(pitch_angle=-10, thrust=0.5)
                elif (dist_sum - target_dist) < -30:
                    print("backward")
                    #pitch = 10
                    set_attitude(pitch_angle=10, thrust=0.5)"""
                #set_attitude(pitch_angle=pitch_angle, thrust=0.5)
            if (f != 0 and vis_height != None):
                vis_dist = (real_height*f)/vis_height
                print("vis_dist:", vis_dist)
                pitch = 15*math.sin((vis_dist-100)/100)
            if (vis_height != None):
                yaw = calculate(img, track_boxs[0])
                #set_attitude(pitch_angle=pitch_angle, yaw_angle=yaw, thrust=0.5)
                send_attitude_target(roll_angle=0.0, pitch_angle=pitch,
                         yaw_angle=yaw, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5)
        if(x == 30):
            x = 0

        x += 1
        img = vis.draw_bboxes(img, track_boxs, confs, track_labels, track_ids)
        img = show_fps(img, fps)
        cv2.imshow(WINDOW_NAME, img)
        out.write(img)
        toc = time.time()
        curr_fps = 1.0 / (toc - tic)
        # calculate an exponentially decaying average of fps number
        fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
        tic = toc
        key = cv2.waitKey(1)
        if key == 27:  # ESC key: quit program
            break
        elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
            full_scrn = not full_scrn
            set_display(WINDOW_NAME, full_scrn)
    out.release()



def main():
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)

    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        cam.img_width, cam.img_height)
    print('Connecting...')

    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    """
    time.sleep(30)
    vehicle.armed = True
    thrust = 0.6
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= 1*0.95: # Trigger just below target alt.
            thrust = 0.5
            print("Reached target altitude")
            break

        elif current_altitude >= 1*0.6:
            thrust = 0.6
        set_attitude(thrust = thrust)
    """

    loop_and_detect(cam, trt_yolo, args.conf_thresh, vis=vis, cls_dict=cls_dict)

    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
