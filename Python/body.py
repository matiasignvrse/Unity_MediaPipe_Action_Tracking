
import socket
import os
import mediapipe as mp
import cv2
import json
import threading
import time
import global_vars
import struct
import datetime
from clientUDP import ClientUDP
import numpy as np
from scipy.spatial.transform import Rotation as R

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
        self.bvh_frames = []  # Store BVH frames

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

                # --- BVH frame collection ---
                if results.pose_landmarks:
                    # Collect only pose landmarks for BVH (expand as needed)
                    frame = []
                    for lm in results.pose_landmarks.landmark:
                        # BVH expects positions, here we just store x, y, z
                        frame.extend([lm.x, lm.y, lm.z])
                    self.bvh_frames.append(frame)

                # json conversion
                final_message = json.dumps(data)

                #print(f"Sending data: {final_message}")

                # UDP send to Unity
                self.send_data(final_message)

        # Write BVH file after capture ends
        self.write_bvh_file()
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

    def write_bvh_file(self):
        if not self.bvh_frames:
            print("No BVH frames to write.")
            return

        # BVH hierarchy as baseline (your template)
        bvh_header = """HIERARCHY
    ROOT Hips
    {
        OFFSET 0.00000 0.00000 0.00000
        CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation 
        JOINT LHipJoint
        {
            OFFSET 0 0 0
            CHANNELS 3 Zrotation Yrotation Xrotation
            JOINT LeftUpLeg
            {
                OFFSET 3.13874 -1.57224 1.49786
                CHANNELS 3 Zrotation Yrotation Xrotation
                JOINT LeftLeg
                {
                    OFFSET 2.10955 -5.79594 0.00000
                    CHANNELS 3 Zrotation Yrotation Xrotation
                    JOINT LeftFoot
                    {
                        OFFSET 2.41843 -6.64458 0.00000
                        CHANNELS 3 Zrotation Yrotation Xrotation
                        JOINT LeftToeBase
                        {
                            OFFSET 0.04713 -0.12948 1.66229
                            CHANNELS 3 Zrotation Yrotation Xrotation
                            End Site
                            {
                                OFFSET 0.00000 -0.00000 0.85167
                            }
                        }
                    }
                }
            }
        }
        JOINT RHipJoint
        {
            OFFSET 0 0 0
            CHANNELS 3 Zrotation Yrotation Xrotation
            JOINT RightUpLeg
            {
                OFFSET -3.13988 -1.57224 1.49786
                CHANNELS 3 Zrotation Yrotation Xrotation
                JOINT RightLeg
                {
                    OFFSET -2.15006 -5.90725 0.00000
                    CHANNELS 3 Zrotation Yrotation Xrotation
                    JOINT RightFoot
                    {
                        OFFSET -2.34496 -6.44273 0.00000
                        CHANNELS 3 Zrotation Yrotation Xrotation
                        JOINT RightToeBase
                        {
                            OFFSET -0.08699 -0.23900 1.55223
                            CHANNELS 3 Zrotation Yrotation Xrotation
                            End Site
                            {
                                OFFSET -0.00000 -0.00000 0.80281
                            }
                        }
                    }
                }
            }
        }
        JOINT LowerBack
        {
            OFFSET 0 0 0
            CHANNELS 3 Zrotation Yrotation Xrotation
            JOINT Spine
            {
                OFFSET 0.01292 1.96517 -0.16495
                CHANNELS 3 Zrotation Yrotation Xrotation
                JOINT Spine1
                {
                    OFFSET 0.02122 1.88666 0.19706
                    CHANNELS 3 Zrotation Yrotation Xrotation
                    JOINT Neck
                    {
                        OFFSET 0 0 0
                        CHANNELS 3 Zrotation Yrotation Xrotation
                        JOINT Neck1
                        {
                            OFFSET -0.01683 1.81591 0.07903
                            CHANNELS 3 Zrotation Yrotation Xrotation
                            JOINT Head
                            {
                                OFFSET 0.00815 1.74013 -0.47700
                                CHANNELS 3 Zrotation Yrotation Xrotation
                                End Site
                                {
                                    OFFSET -0.00204 1.83225 -0.16933
                                }
                            }
                        }
                    }
                    JOINT LeftShoulder
                    {
                        OFFSET 0 0 0
                        CHANNELS 3 Zrotation Yrotation Xrotation
                        JOINT LeftArm
                        {
                            OFFSET 3.38482 1.41533 0.19802
                            CHANNELS 3 Zrotation Yrotation Xrotation
                            JOINT LeftForeArm
                            {
                                OFFSET 4.32712 -0.00000 0.00000
                                CHANNELS 3 Zrotation Yrotation Xrotation
                                JOINT LeftHand
                                {
                                    OFFSET 3.28587 -0.00000 -0.00000
                                    CHANNELS 3 Zrotation Yrotation Xrotation
                                    JOINT LeftFingerBase
                                    {
                                        OFFSET 0 0 0
                                        CHANNELS 3 Zrotation Yrotation Xrotation
                                        JOINT LeftHandIndex1
                                        {
                                            OFFSET 0.43316 -0.00000 -0.00000
                                            CHANNELS 3 Zrotation Yrotation Xrotation
                                            End Site
                                            {
                                                OFFSET 0.34922 -0.00000 -0.00000
                                            }
                                        }
                                    }
                                    JOINT LThumb
                                    {
                                        OFFSET 0 0 0
                                        CHANNELS 3 Zrotation Yrotation Xrotation
                                        End Site
                                        {
                                            OFFSET 0.35456 -0.00000 0.35456
                                        }
                                    }
                                }
                            }
                        }
                    }
                    JOINT RightShoulder
                    {
                        OFFSET 0 0 0
                        CHANNELS 3 Zrotation Yrotation Xrotation
                        JOINT RightArm
                        {
                            OFFSET -3.09725 1.62858 -0.14144
                            CHANNELS 3 Zrotation Yrotation Xrotation
                            JOINT RightForeArm
                            {
                                OFFSET -4.29104 -0.00000 0.00000
                                CHANNELS 3 Zrotation Yrotation Xrotation
                                JOINT RightHand
                                {
                                    OFFSET -3.22720 -0.00000 -0.00000
                                    CHANNELS 3 Zrotation Yrotation Xrotation
                                    JOINT RightFingerBase
                                    {
                                        OFFSET 0 0 0
                                        CHANNELS 3 Zrotation Yrotation Xrotation
                                        JOINT RightHandIndex1
                                        {
                                            OFFSET -0.54362 -0.00000 -0.00000
                                            CHANNELS 3 Zrotation Yrotation Xrotation
                                            End Site
                                            {
                                                OFFSET -0.43828 -0.00000 -0.00000
                                            }
                                        }
                                    }
                                    JOINT RThumb
                                    {
                                        OFFSET 0 0 0
                                        CHANNELS 3 Zrotation Yrotation Xrotation
                                        End Site
                                        {
                                            OFFSET -0.44497 -0.00000 0.44497
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    """
    
            # Mapping: BVH joint order to MediaPipe pose landmark indices
            # (This is an approximate mapping for the main body parts)
        bvh_to_mp = [
            23,  # Hips (root, use left_hip)
            24,  # LHipJoint (right_hip)
            23,  # LeftUpLeg (left_hip)
            25,  # LeftLeg (left_knee)
            27,  # LeftFoot (left_ankle)
            31,  # LeftToeBase (left_foot_index)
            24,  # RHipJoint (right_hip)
            26,  # RightUpLeg (right_knee)
            28,  # RightLeg (right_ankle)
            32,  # RightFoot (right_foot_index)
            0,   # LowerBack (pelvis, use nose as fallback)
            11,  # Spine (left_shoulder)
            12,  # Spine1 (right_shoulder)
            0,   # Neck (nose)
            0,   # Neck1 (nose)
            0,   # Head (nose)
            11,  # LeftShoulder
            13,  # LeftArm (left_elbow)
            15,  # LeftForeArm (left_wrist)
            17,  # LeftHand (left_pinky)
            21,  # LeftFingerBase (left_index)
            19,  # LeftHandIndex1 (left_thumb)
            12,  # RightShoulder
            14,  # RightArm (right_elbow)
            16,  # RightForeArm (right_wrist)
            18,  # RightHand (right_pinky)
            22,  # RightFingerBase (right_index)
            20,  # RightHandIndex1 (right_thumb)
        ]
    
            # Parent indices for each joint in the above order (for local rotation calculation)
        bvh_parents = [
                -1,  # Hips
                0,   # LHipJoint
                1,   # LeftUpLeg
                2,   # LeftLeg
                3,   # LeftFoot
                4,   # LeftToeBase
                0,   # RHipJoint
                6,   # RightUpLeg
                7,   # RightLeg
                8,   # RightFoot
                0,   # LowerBack
                10,  # Spine
                11,  # Spine1
                12,  # Neck
                13,  # Neck1
                14,  # Head
                12,  # LeftShoulder
                16,  # LeftArm
                17,  # LeftForeArm
                18,  # LeftHand
                19,  # LeftFingerBase
                20,  # LeftHandIndex1
                12,  # RightShoulder
                22,  # RightArm
                23,  # RightForeArm
                24,  # RightHand
                25,  # RightFingerBase
                26,  # RightHandIndex1
        ]
    
            # --- MOTION section ---
        bvh_header += "MOTION\n"
        bvh_header += f"Frames: {len(self.bvh_frames)}\n"
        bvh_header += f"Frame Time: {1.0 / global_vars.FPS:.6f}\n"

        bvh_motion = ""
        for frame in self.bvh_frames:
            if len(frame) < 33 * 3:
                continue  # skip incomplete frames

            # Root position (Hips, MediaPipe left_hip, scaled)
            root_idx = bvh_to_mp[0]
            root_pos = [frame[root_idx * 3] * 1000, frame[root_idx * 3 + 1] * 1000, frame[root_idx * 3 + 2] * 1000]
            root_rot = [0, 0, 0]  # You can compute global orientation if needed

            joint_rots = []
            for i in range(1, len(bvh_to_mp)):
                idx = bvh_to_mp[i]
                parent = bvh_parents[i]
                if parent == -1:
                    joint_rots.append("0 0 0")
                    continue
                pidx = bvh_to_mp[parent]
                # Get vectors
                child = np.array([frame[idx*3], frame[idx*3+1], frame[idx*3+2]])
                par = np.array([frame[pidx*3], frame[pidx*3+1], frame[pidx*3+2]])
                vec = child - par
                # Reference direction: Y axis (up)
                ref = np.array([0, 1, 0])
                v = vec / (np.linalg.norm(vec) + 1e-8)
                axis = np.cross(ref, v)
                angle = np.arccos(np.clip(np.dot(ref, v), -1.0, 1.0))
                if np.linalg.norm(axis) < 1e-6:
                    euler = [0, 0, 0]
                else:
                    r = R.from_rotvec(axis / np.linalg.norm(axis) * angle)
                    euler = r.as_euler('zxy', degrees=True)
                joint_rots.append(f"{euler[0]:.3f} {euler[1]:.3f} {euler[2]:.3f}")

            bvh_motion += " ".join(map(str, root_pos + root_rot)) + " " + " ".join(joint_rots) + "\n"

        # Save to file
        filename = f"output_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}_customhier.bvh"
        with open(filename, "w") as f:
            f.write(bvh_header)
            f.write(bvh_motion)
        print(f"BVH file saved as {filename}")
