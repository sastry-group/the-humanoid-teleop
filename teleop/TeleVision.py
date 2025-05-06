#!/usr/bin/env python3

import asyncio
import numpy as np
import time
from vuer import Vuer
from vuer.events import ClientEvent
from vuer.schemas import Hands, DefaultScene, CameraView
from multiprocessing import Array, Value, Process, shared_memory

class TeleVision:
    def __init__(self):
        # def __init__(self, img_shm_name, img_shape):
        # Initialize the shared memory for the image
        # self.img_height = img_shape[0]
        # self.img_width  = img_shape[1] // 2
        # existing_shm = shared_memory.SharedMemory(name=img_shm_name)
        # self.img_array = np.ndarray(img_shape, dtype=np.uint8, buffer=existing_shm.buf)

        # Instantiate the Vuer app WITH ngrok (no certs needed)
        self.app = Vuer(host="0.0.0.0", queries=dict(grid=False), queue_len=3)

        # Register our HAND_MOVE and CAM_MOVE handlers
        self.app.add_handler("CAMERA_MOVE")(self.on_cam_move)
        self.app.add_handler("HAND_MOVE")(self.on_hand_move)

        # Spawn our very simple scene (hand skeleton overlay)
        self.app.spawn(start=False)(self.main_scene)

        # # Start the Vuer app in a separate process to avoid blocking in the main thread
        # self.process = Process(target=self.run)
        # self.process.daemon = True
        # self.process.start()

        # Initalize shared memory arrays for hand and head transforms
        self.left_landmarks_shared = Array('d', 400, lock=True)
        self.right_landmarks_shared = Array('d', 400, lock=True)
        self.head_matrix_shared = Array('d', 16, lock=True)
        self.aspect_shared = Value('d', 1.0, lock=True)
    
    async def on_hand_move(self, event: ClientEvent, session, fps=60):
        """
        Handler for HAND_MOVE events. Updates the shared memory arrays with:
        - leftLandmarks  (400 floats → 25x16 landmarks, includes wrist index at 0)
        - rightLandmarks  (400 floats → 25x16 landmarks, includes wrist index at 0)
        """
        try:
            raw_left = np.array(event.value["left"], dtype=float)
            raw_right = np.array(event.value["right"], dtype=float)
            self.left_landmarks_shared[:] = raw_left.tolist()
            self.right_landmarks_shared[:] = raw_right.tolist()
        except Exception as e:
            print("Error in HAND_MOVE handler:", e)

    async def on_cam_move(self, event: ClientEvent, session):
        """
        Handler for CAM_MOVE events. Updates the shared memory arrays with:
        - camera (16 floats → 4x4 homogenous transformation matrix)
        - aspect (1 float → aspect ratio of the camera)
        """
        try:
            # Update head matrix
            raw_matrix = np.array(event.value["camera"]["matrix"], dtype=float)
            self.head_matrix_shared[:] = raw_matrix.tolist()
            # Update aspect ratio
            self.aspect_shared.value = event.value["camera"]["aspect"]
        except Exception as e:
            print("Error in CAM_MOVE handler:", e)

    # async def main_image(self, session, fps=60):
        # session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)
        # while True:
        #     display_image = cv2.cvtColor(self.img_array, cv2.COLOR_BGR2RGB)
        #     # aspect_ratio = self.img_width / self.img_height
        #     session.upsert(
        #         [
        #             ImageBackground(
        #                 display_image[:, :self.img_width],
        #                 aspect=1.778,
        #                 height=1,
        #                 distanceToCamera=1,
        #                 # The underlying rendering engine supported a layer binary bitmask for both objects and the camera. 
        #                 # Below we set the two image planes, left and right, to layers=1 and layers=2. 
        #                 # Note that these two masks are associated with left eye’s camera and the right eye’s camera.
        #                 layers=1,
        #                 format="jpeg",
        #                 quality=50,
        #                 key="background-left",
        #                 interpolate=True,
        #             ),
        #             ImageBackground(
        #                 display_image[:, self.img_width:],
        #                 aspect=1.778,
        #                 height=1,
        #                 distanceToCamera=1,
        #                 layers=2,
        #                 format="jpeg",
        #                 quality=50,
        #                 key="background-right",
        #                 interpolate=True,
        #             ),
        #         ],
        #         to="bgChildren",
        #     )
        #     # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
        #     await asyncio.sleep(0.016 * 2)

    async def main_scene(self, session, fps=60):
        """
        A minimal Vuer scene that just streams and draws your hands
        so you can verify visually on the headset.
        """
        session.set @ DefaultScene(frameloop="always")
        session.upsert @ Hands(fps=fps, stream=True)
        session.upsert @ CameraView(fps=fps, stream=True)
        while True:
            await asyncio.sleep(1)


    def run(self):
        """
        Starts the Vuer app and runs the event loop.
        """
        self.app.run()

    def printPoses(self):
        """
        Prints the current hand and head poses.
        """
        print("Left Wrist:\n", self.left_wrist)
        print("Right Wrist:\n", self.right_wrist)
        print("Left Landmarks:\n", self.left_landmarks)
        print("Right Landmarks:\n", self.right_landmarks)
        print("Head Matrix:\n", self.head_matrix)
        print("Aspect Ratio:", self.aspect)
        print("―" * 60)

    @property
    def left_wrist(self): # Left hand 4x4 homogenous transformation matrix of first joint
        return np.array(self.left_landmarks_shared[:16]).reshape(4, 4, order="F")
    
    @property
    def right_wrist(self): # Right hand 4x4 homogenous transformation matrix of first joint
        return np.array(self.right_landmarks_shared[:16]).reshape(4, 4, order="F")
    
    @property
    def head_matrix(self): # Head 4x4 homogenous transformation matrix
        return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")
    
    @property
    def left_landmarks(self):
        matsL = np.array(self.left_landmarks_shared[:]).reshape(25, 16) # Left hand 24x16 landmarks
        landmarksL = np.stack([M[0:3, 3] for M in matsL], axis=0) # Extract the translation part of the matrix
        return landmarksL
    
    @property
    def right_landmarks(self):
        matsR = np.array(self.right_landmarks_shared[:]).reshape(25, 16) # Right hand 24x16 landmarks
        landmarksR = np.stack([M[0:3, 3] for M in matsR], axis=0) # Extract the translation part of the matrix
        return landmarksR
    
    @property
    def aspect(self): # Aspect ratio of the camera
        return float(self.aspect_shared.value)

if __name__ == "__main__":
    import os 
    import sys
    import threading
    from image_server.image_client import ImageClient

    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)
    sys.path.append(parent_dir)

    # # image
    # img_shape = (480, 640 * 2, 3)
    # img_shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
    # img_array = np.ndarray(img_shape, dtype=np.uint8, buffer=img_shm.buf)
    # img_client = ImageClient(tv_img_shape = img_shape, tv_img_shm_name = img_shm.name)
    # image_receive_thread = threading.Thread(target=img_client.receive_process, daemon=True)
    # image_receive_thread.start()

    # television
    # tv = TeleVision(img_shape, img_shm.name)
    tv = TeleVision()
    tv.run()