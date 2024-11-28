
import socket
import os
import mediapipe as mp
import cv2
import json
import threading
import time
import global_vars
import struct
from clientUDP import ClientUDP

# kill port before initializing
def kill_port(port):
    print("try to kill %s pid..." % port)
    find_port = 'netstat -aon | findstr %s' % port
    result = os.popen(find_port)
    text = result.read()
    pid = text[-5:-1]
    if pid == "":
        print("not found %s pid..." % port)
        return
    else:
        find_kill = 'taskkill -f -pid %s' % pid
        result = os.popen(find_kill)
        print(result.read())

# the capture thread captures images from the WebCam on a separate thread (for performance)
class CaptureThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.cap = None
        self.ret = False
        self.frame = None
        self.isRunning = False
        self.counter = 0
        self.timer = time.time()
        self.lock = threading.Lock()

    def run(self):
        self.cap = cv2.VideoCapture(global_vars.CAM_INDEX)  # open camera need some time
        if global_vars.USE_CUSTOM_CAM_SETTINGS:
            self.cap.set(cv2.CAP_PROP_FPS, global_vars.FPS)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, global_vars.WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, global_vars.HEIGHT)

        time.sleep(1)

        print("Opened Capture @ %s fps" % str(self.cap.get(cv2.CAP_PROP_FPS)))
        while not global_vars.KILL_THREADS:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret = ret
                self.frame = frame.copy() if ret else None
            self.isRunning = True
            if global_vars.DEBUG:
                self.counter += 1
                if time.time() - self.timer >= 3:
                    fps = self.counter / (time.time() - self.timer)
                    print("Capture FPS: ", fps)
                    self.counter = 0
                    self.timer = time.time()

        self.cap.release()

# the body thread actually does the
# processing of the captured images, and communication with unity
class BodyThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.data = ""
        self.pipe = None
        self.timeSinceCheckedConnection = 0
        self.timeSincePostStatistics = 0

    def run(self):
        mp_drawing = mp.solutions.drawing_utils
        mp_holistic = mp.solutions.holistic

        self.setup_comms()

        capture = CaptureThread()
        capture.start()
        # feel free to ajust model parameters
        with mp_holistic.Holistic(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            model_complexity=1,
            static_image_mode=False,
            enable_segmentation=True
        ) as holistic:
            while not global_vars.KILL_THREADS and not capture.isRunning:
                print("Waiting for camera and capture thread.")
                time.sleep(0.5)
            print("Beginning capture")

            while not global_vars.KILL_THREADS and capture.cap.isOpened():
                ti = time.time()

                # Fetch stuff from the capture thread
                with capture.lock:
                    ret = capture.ret
                    image = capture.frame.copy() if ret else None

                if image is None:
                    continue

                # Image transformations and stuff
                image = cv2.flip(image, 1)
                image.flags.writeable = global_vars.DEBUG

                # Detections
                results = holistic.process(image)
                tf = time.time()

                # Rendering results
                if global_vars.DEBUG:
                    if time.time() - self.timeSincePostStatistics >= 1:
                        theoretical_fps = 1 / (tf - ti)
                        print("Theoretical Maximum FPS: %f" % theoretical_fps)
                        self.timeSincePostStatistics = time.time()

                    if results.pose_landmarks:
                        mp_drawing.draw_landmarks(
                            image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS,
                            mp_drawing.DrawingSpec(color=(255, 100, 0), thickness=2, circle_radius=4),
                            mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2, circle_radius=2),
                        )
                    if results.left_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                            mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=4),
                            mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                        )
                    if results.right_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                            mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=4),
                            mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                        )
                    cv2.imshow('Body Tracking', image)
                    if cv2.waitKey(3) & 0xFF == 27:  # ESC key to exit
                        global_vars.KILL_THREADS = True
                        break

                # Set up data for relay
                data = {
                    "left_hand": [{"x": lm.x, "y": lm.y, "z": lm.z} for lm in results.left_hand_landmarks.landmark] if results.left_hand_landmarks else [],
                    "right_hand": [{"x": lm.x, "y": lm.y, "z": lm.z} for lm in results.right_hand_landmarks.landmark] if results.right_hand_landmarks else [],
                    "pose": [{"x": lm.x, "y": lm.y, "z": lm.z} for lm in results.pose_landmarks.landmark] if results.pose_landmarks else []
                }

                # json conversion
                final_message = json.dumps(data)

                #print(f"Sending data: {final_message}")

                # UDP send to Unity
                self.send_data(final_message)

        self.cleanup(capture)

    def setup_comms(self):
        if not global_vars.USE_LEGACY_PIPES:
            self.client = ClientUDP(global_vars.HOST, global_vars.PORT)
            self.client.start()
        else:
            print("Using Pipes for interprocess communication (not supported on OSX or Linux).")

    def send_data(self, message):
        if not global_vars.USE_LEGACY_PIPES:
            if self.client.isConnected():
                self.client.sendMessage(message)
            else:
                print("UDP client not connected. Message not sent.")
        else:
            # Maintain pipe connection.
            if self.pipe is None and time.time() - self.timeSinceCheckedConnection >= 1:
                try:
                    self.pipe = open(r'\\.\pipe\UnityMediaPipeBody1', 'r+b', 0)
                    print("Named Pipes connection established.")
                except FileNotFoundError:
                    print("Waiting for Unity project to run...")
                    self.pipe = None
                self.timeSinceCheckedConnection = time.time()

            if self.pipe is not None:
                try:
                    s = message.encode('utf-8')
                    self.pipe.write(struct.pack('I', len(s)) + s)
                    self.pipe.seek(0)
                except Exception as ex:
                    print("Failed to write to pipe. Is the Unity project open?")
                    self.pipe = None

    def cleanup(self, capture):
        if not global_vars.USE_LEGACY_PIPES and hasattr(self, 'client'):
            self.client.socket.close()
        elif self.pipe is not None:
            self.pipe.close()
        cv2.destroyAllWindows()
        print("BodyThread terminated gracefully.")
