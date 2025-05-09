#!/usr/bin/env python3

import asyncio
import numpy as np
import time
from vuer import Vuer
from vuer.events import ClientEvent
from vuer.schemas import Hands, DefaultScene, CameraView, WebRTCStereoVideoPlane, ImageBackground
from multiprocessing import Array, Value, Process, shared_memory
import cv2

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
        # self.app.spawn(start=False)(self.main_scene)
        self.app.spawn(start=True)(self.main_image)
        # self.app.spawn(start=True)(self.main_camera)


        self.left_hand_shared = Array('d', 16, lock=True)
        self.right_hand_shared = Array('d', 16, lock=True)
        self.left_landmarks_shared = Array('d', 75, lock=True)
        self.right_landmarks_shared = Array('d', 75, lock=True)
        
        self.head_matrix_shared = Array('d', 16, lock=True)
        self.aspect_shared = Value('d', 1.0, lock=True)

        # Start the Vuer app in a separate process to avoid blocking in the main thread
        self.process = Process(target=self.vuer_run)
        self.process.daemon = True
        self.process.start()
    
    async def on_hand_move(self, event: ClientEvent, session, fps=60):
        """
        Handler for HAND_MOVE events. Updates the shared memory arrays with:
        - leftLandmarks  (400 floats → 25x16 landmarks, includes wrist index at 0)
        - rightLandmarks  (400 floats → 25x16 landmarks, includes wrist index at 0)
        """

        try:
            # raw_left = np.array(event.value["left"]).flatten(order="F")
            # raw_right = np.array(event.value["right"]).flatten(order="F")
            # # print("Shared size: ", len(self.left_landmarks_shared))
            # # print("Input size: ", np.array(event.value["left"]).flatten(order="F").shape)
            # self.left_landmarks_shared[:] = raw_left.tolist()
            # self.right_landmarks_shared[:] = raw_right.tolist()
            self.left_hand_shared[:] = event.value["leftHand"]
            self.right_hand_shared[:] = event.value["rightHand"]
            self.left_landmarks_shared[:] = np.array(event.value["leftLandmarks"]).flatten()
            self.right_landmarks_shared[:] = np.array(event.value["rightLandmarks"]).flatten()
        except Exception as e:
            print("Error in HAND_MOVE handler:", e)
        # print("Left: ", np.array(event.value["left"]).flatten(order="F"))
        # print("Right: ", np.array(event.value["right"]).flatten(order="F"))
        

    async def on_cam_move(self, event: ClientEvent, session):
        """
        Handler for CAM_MOVE events. Updates the shared memory arrays with:
        - camera (16 floats → 4x4 homogenous transformation matrix)
        - aspect (1 float → aspect ratio of the camera)
        """
        try:
            self.head_matrix_shared[:] = event.value["camera"]["matrix"]
            self.aspect_shared.value = event.value['camera']['aspect']
        except Exception as e:
            print("Error in CAM_MOVE handler:", e)

    # async def main_camera(self, session):
    #     fps = 30
    #     CAMERA_IDX = 0
    #     # 1) Basic Vuer scene
    #     session.set @ DefaultScene(frameloop="always")
    #     session.upsert @ Hands(fps=fps, stream=True)

    #     # 2) OpenCV capture
    #     cap = cv2.VideoCapture(CAMERA_IDX)
    #     if not cap.isOpened():
    #         raise RuntimeError(f"Cannot open camera {CAMERA_IDX}")

    #     # 3) Stream loop
    #     while True:
    #         ret, frame_bgr = cap.read()
    #         if not ret:
    #             await asyncio.sleep(1.0 / fps)
    #             continue

    #         # Convert to RGB for Vuer
    #         frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    #         # Upload as background quad (mono for now)
    #         session.upsert @ ImageBackground(
    #             frame_rgb,
    #             format="jpeg",
    #             quality=80,
    #             key="camera-feed",
    #             interpolate=True,
    #             aspect=frame_rgb.shape[1] / frame_rgb.shape[0],
    #             distanceToCamera=2,
    #             position=[0, 0, -3],
    #             height=3
    #         )

    #         await asyncio.sleep(1.0 / fps)


    async def main_image(self, session, fps=60):
        session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)
        CAMERA_IDX = 0

        # 1) Basic Vuer scene
        session.set @ DefaultScene(frameloop="always")
        # session.upsert @ Hands(fps=fps, stream=True)

        # # 2) OpenCV capture
        # cap = cv2.VideoCapture(CAMERA_IDX)
        # if not cap.isOpened():
        #     raise RuntimeError(f"Cannot open camera {CAMERA_IDX}")

        # while True:
        #     # display_image = cv2.cvtColor(self.img_array, cv2.COLOR_BGR2RGB)
        #     # # aspect_ratio = self.img_width / self.img_height
        #     # session.upsert(
        #     #     [
        #     #         ImageBackground(
        #     #             display_image[:, :self.img_width],
        #     #             aspect=1.778,
        #     #             height=1,
        #     #             distanceToCamera=1,
        #     #             # The underlying rendering engine supported a layer binary bitmask for both objects and the camera. 
        #     #             # Below we set the two image planes, left and right, to layers=1 and layers=2. 
        #     #             # Note that these two masks are associated with left eye’s camera and the right eye’s camera.
        #     #             layers=1,
        #     #             format="jpeg",
        #     #             quality=50,
        #     #             key="background-left",
        #     #             interpolate=True,
        #     #         ),
        #     #         ImageBackground(
        #     #             display_image[:, self.img_width:],
        #     #             aspect=1.778,
        #     #             height=1,
        #     #             distanceToCamera=1,
        #     #             layers=2,
        #     #             format="jpeg",
        #     #             quality=50,
        #     #             key="background-right",
        #     #             interpolate=True,
        #     #         ),
        #     #     ],
        #     #     to="bgChildren",
        #     # )

        #     ret, frame_bgr = cap.read()
        #     if not ret:
        #         await asyncio.sleep(1.0 / fps)
        #         continue

        #     # Convert to RGB for Vuer
        #     frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        #     # Upload as background quad (mono for now)
        #     session.upsert @ ImageBackground(
        #         frame_rgb,
        #         format="jpeg",
        #         quality=80,
        #         key="camera-feed",
        #         interpolate=True,
        #         aspect=frame_rgb.shape[1] / frame_rgb.shape[0],
        #         distanceToCamera=2,
        #         position=[0, 0, -3],
        #         height=3
        #     )

            # await asyncio.sleep(1.0 / fps)
            # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
            # await asyncio.sleep(0.016 * 2)

    async def main_scene(self, session, fps=60):
        """
        A minimal Vuer scene that just streams and draws your hands
        so you can verify visually on the headset.
        """
        session.set @ DefaultScene(frameloop="always")
        session.upsert @ Hands(fps=fps, stream=True)
        session.upsert @ WebRTCStereoVideoPlane(
            src="https://",
            key="zed",
            aspect=1.33334,
            height = 8,
            position=[0, -2, -0.2],
        )
        while True:
            await asyncio.sleep(1)


    def vuer_run(self):
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
        return np.array(self.left_hand_shared[:]).reshape(4, 4, order="F")    
    
    @property
    def right_wrist(self): # Right hand 4x4 homogenous transformation matrix of first joint
        return np.array(self.right_hand_shared[:]).reshape(4, 4, order="F")
    
    @property
    def head_matrix(self): # Head 4x4 homogenous transformation matrix
        return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")
    
    @property
    def left_landmarks(self):
        return np.array(self.left_landmarks_shared[:]).reshape(25, 3)
    
    @property
    def right_landmarks(self):
        return np.array(self.right_landmarks_shared[:]).reshape(25, 3)

    
    @property
    def aspect(self): # Aspect ratio of the camera
        return float(self.aspect_shared.value)

if __name__ == "__main__":
    import os 
    import sys
    import threading
    # from image_server.image_client import ImageClient

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
    while True:
        tv.printPoses()
        time.sleep(0.5)
    










































# #!/usr/bin/env python3

# import asyncio
# import numpy as np
# import time
# from vuer import Vuer
# from vuer.events import ClientEvent
# from vuer.schemas import Hands, DefaultScene, CameraView, ImageBackground, WebRTCStereoVideoPlane
# from multiprocessing import Array, Value, Process, shared_memory, Queue, Event
# from image_server.zed_server import *
# # from aiohttp import web

# import cv2

# class TeleVision:
#     def __init__(self, img_shape, queue, toggle_streaming, stream_mode="webrtc", cert_file="./cert.pem", key_file="./key.pem", ngrok=True):
#         self.img_shape = (img_shape[0], 2*img_shape[1], 3)
#         self.img_height, self.img_width = img_shape[:2]

#         # Instantiate the Vuer app WITH ngrok (no certs needed)
#         self.app = Vuer(host="0.0.0.0", port=8012, queries=dict(grid=False), queue_len=3)

#         # Register our HAND_MOVE and CAM_MOVE handlers
#         # self.app.add_handler("CAMERA_MOVE")(self.on_cam_move) # not sure if necessary, untested with servos
#         self.app.add_handler("HAND_MOVE")(self.on_hand_move)

#         # ---------------- from opentele ----------- #
#         # if stream_mode == "image":
#         #     existing_shm = shared_memory.SharedMemory(name=shm_name)
#         #     self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=existing_shm.buf)
#         #     self.app.spawn(start=False)(self.main_image)
#         # if stream_mode == "webrtc":
#         self.app.spawn(start=False)(self.main_webrtc)
#         # else:
#         #     print("stream_mode must be either 'webrtc' or 'image'")
#         #     # raise ValueError("stream_mode must be either 'webrtc' or 'image'")
#         #     self.app.spawn(start=False)(self.main_scene)

#         # Spawn our very simple scene (hand skeleton overlay)
#         # self.app.spawn(start=False)(self.main_scene)

#         # Initalize shared memory arrays for hand and head transforms
#         self.left_landmarks_shared = Array('d', 400, lock=True)
#         self.right_landmarks_shared = Array('d', 400, lock=True)
#         self.head_matrix_shared = Array('d', 16, lock=True)
#         self.aspect_shared = Value('d', 1.0, lock=True)

#         # if stream_mode == "webrtc":
#         # webrtc server
#         if Args.verbose:
#             logging.basicConfig(level=logging.DEBUG)
#         else:
#             logging.basicConfig(level=logging.INFO)
#         Args.img_shape = img_shape
#         # Args.shm_name = shm_name
#         Args.fps = 60

#         ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
#         ssl_context.load_cert_chain(cert_file, key_file)

#         app = web.Application()
#         cors = aiohttp_cors.setup(app, defaults={
#             "*": aiohttp_cors.ResourceOptions(
#                 allow_credentials=True,
#                 expose_headers="*",
#                 allow_headers="*",
#                 allow_methods="*",
#             )
#         })
#         rtc = RTC(img_shape, queue, toggle_streaming, 60)
#         app.on_shutdown.append(on_shutdown)
#         cors.add(app.router.add_get("/", index))
#         cors.add(app.router.add_get("/client.js", javascript))
#         cors.add(app.router.add_post("/offer", rtc.offer))

#         # self.webrtc_process = Process(target=web.run_app, args=(app,), kwargs={"host": "0.0.0.0", "port": 8080, "ssl_context": ssl_context})
#         # self.webrtc_process.daemon = True
#         # self.webrtc_process.start()
#         # web.run_app(app, host="0.0.0.0", port=8012, ssl_context=ssl_context)

#         # Start the Vuer app in a separate process to avoid blocking in the main thread
#         self.process = Process(target=self.vuer_run)
#         self.process.daemon = True
#         self.process.start()
    
#     async def on_hand_move(self, event: ClientEvent, session, fps=60):
#         """
#         Handler for HAND_MOVE events. Updates the shared memory arrays with:
#         - leftLandmarks  (400 floats → 25x16 landmarks, includes wrist index at 0)
#         - rightLandmarks  (400 floats → 25x16 landmarks, includes wrist index at 0)
#         """
#         try:
#             raw_left = np.array(event.value["left"]).flatten(order="F")
#             raw_right = np.array(event.value["right"]).flatten(order="F")
#             # print("Shared size: ", len(self.left_landmarks_shared))
#             # print("Input size: ", np.array(event.value["left"]).flatten(order="F").shape)
#             self.left_landmarks_shared[:] = raw_left.tolist()
#             self.right_landmarks_shared[:] = raw_right.tolist()
#             print("Left: ", raw_left.shape)
#             print("Right: ", raw_right.shape)
#         except Exception as e:
#             print("Error in HAND_MOVE handler:", e)
#             # print("Left: ", raw_left.shape)
#             # print("Right: ", raw_right.shape)
#         # print("Left: ", np.array(event.value["left"]).flatten(order="F"))
#         # print("Right: ", np.array(event.value["right"]).flatten(order="F"))
        

#     async def on_cam_move(self, event: ClientEvent, session):
#         """
#         Handler for CAM_MOVE events. Updates the shared memory arrays with:
#         - camera (16 floats → 4x4 homogenous transformation matrix)
#         - aspect (1 float → aspect ratio of the camera)
#         """
#         try:
#             # Update head matrix
#             raw_matrix = np.array(event.value["camera"]["matrix"], dtype=float)
#             self.head_matrix_shared[:] = raw_matrix.tolist()
#             # Update aspect ratio
#             self.aspect_shared.value = event.value["camera"]["aspect"]
#         except Exception as e:
#             print("Error in CAM_MOVE handler:", e)

#     async def main_image(self, session, fps=60):
#         session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)
#         while True:
#             display_image = cv2.cvtColor(self.img_array, cv2.COLOR_BGR2RGB)
#             # aspect_ratio = self.img_width / self.img_height
#             session.upsert(
#                 [
#                     ImageBackground(
#                         display_image[:, :self.img_width],
#                         aspect=1.778,
#                         height=1,
#                         distanceToCamera=1,
#                         # The underlying rendering engine supported a layer binary bitmask for both objects and the camera. 
#                         # Below we set the two image planes, left and right, to layers=1 and layers=2. 
#                         # Note that these two masks are associated with left eye’s camera and the right eye’s camera.
#                         layers=1,
#                         format="jpeg",
#                         quality=50,
#                         key="background-left",
#                         interpolate=True,
#                     ),
#                     ImageBackground(
#                         display_image[:, self.img_width:],
#                         aspect=1.778,
#                         height=1,
#                         distanceToCamera=1,
#                         layers=2,
#                         format="jpeg",
#                         quality=50,
#                         key="background-right",
#                         interpolate=True,
#                     ),
#                 ],
#                 to="bgChildren",
#             )
#             # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
#             await asyncio.sleep(0.016 * 2)

#     async def main_scene(self, session, fps=60):
#         """
#         A minimal Vuer scene that just streams and draws your hands
#         so you can verify visually on the headset.
#         """
#         print("START - main_scene")
#         session.set @ DefaultScene(frameloop="always")
#         session.upsert @ Hands(fps=fps, stream=True)
#         # session.upsert @ CameraView(fps=fps, stream=True)
        
#         while True:
#             print("WHILE LOOP - main_scene")
#             await asyncio.sleep(1)

#     async def main_webrtc(self, session, fps=60):
#         session.set @ DefaultScene(frameloop="always")
#         session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)
#         # session.upsert @ CameraView(fps=fps, stream=True)
#         # session.upsert @ WebRTCStereoVideoPlane(
#         #         src="https://169.229.222.121:8080/offer",
#         #         # iceServer={},
#         #         key="zed",
#         #         aspect=1.33334,
#         #         height = 8,
#         #         position=[0, -2, -0.2],
#         #     )
#         while True:
#             await asyncio.sleep(1)


#     def vuer_run(self):
#         """
#         Starts the Vuer app and runs the event loop.
#         """
#         self.app.run()

#     def printPoses(self):
#         """
#         Prints the current hand and head poses.
#         """
#         print("Left Wrist:\n", self.left_wrist)
#         print("Right Wrist:\n", self.right_wrist)
#         print("Left Landmarks:\n", self.left_landmarks)
#         print("Right Landmarks:\n", self.right_landmarks)
#         print("Head Matrix:\n", self.head_matrix)
#         print("Aspect Ratio:", self.aspect)
#         print("―" * 60) 





# #!/usr/bin/env python3

# import asyncio
# import numpy as np
# import time
# from vuer import Vuer
# from vuer.events import ClientEvent
# from vuer.schemas import Hands, DefaultScene, CameraView, ImageBackground, WebRTCStereoVideoPlane
# from multiprocessing import Array, Value, Process, shared_memory, Queue, Event
# from image_server.zed_server import *
# # from aiohttp import web

# import cv2

# class TeleVision:
#     def __init__(self, img_shape, queue, toggle_streaming, stream_mode="webrtc", cert_file="./cert.pem", key_file="./key.pem", ngrok=True):
#         self.img_shape = (img_shape[0], 2*img_shape[1], 3)
#     @property
#     def left_wrist(self): # Left hand 4x4 homogenous transformation matrix of first joint
#         return np.array(self.left_landmarks_shared[:16]).reshape(4, 4, order="F")
    
#     @property
#     def right_wrist(self): # Right hand 4x4 homogenous transformation matrix of first joint
#         return np.array(self.right_landmarks_shared[:16]).reshape(4, 4, order="F")
    
#     @property
#     def head_matrix(self): # Head 4x4 homogenous transformation matrix
#         return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")
    
#     @property
#     def left_landmarks(self):
#         matsL = np.array(self.left_landmarks_shared[:]).reshape(25, 4, 4, order="F") # Left hand 24x16 landmarks
#         landmarksL = np.stack([M[0:3, 3] for M in matsL], axis=0) # Extract the translation part of the matrix
#         return landmarksL
    
#     @property
#     def right_landmarks(self):
#         matsR = np.array(self.right_landmarks_shared[:]).reshape(25, 4, 4, order="F") # Right hand 24x16 landmarks
#         landmarksR = np.stack([M[0:3, 3] for M in matsR], axis=0) # Extract the translation part of the matrix
#         return landmarksR
    
#     @property
#     def aspect(self): # Aspect ratio of the camera
#         return float(self.aspect_shared.value)

# if __name__ == "__main__":
#     import os 
#     import sys
#     import threading
#     from image_server.image_client import ImageClient

#     current_dir = os.path.dirname(os.path.abspath(__file__))
#     parent_dir = os.path.dirname(current_dir)
#     sys.path.append(parent_dir)

#     # # image
#     # img_shape = (480, 640 * 2, 3)
#     # img_shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
#     # img_array = np.ndarray(img_shape, dtype=np.uint8, buffer=img_shm.buf)
#     # img_client = ImageClient(tv_img_shape = img_shape, tv_img_shm_name = img_shm.name)
#     # image_receive_thread = threading.Thread(target=img_client.receive_process, daemon=True)
#     # image_receive_thread.start()

#     resolution = (720, 1280)
#     crop_size_w = 340  # (resolution[1] - resolution[0]) // 2
#     crop_size_h = 270
#     resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)  # 450 * 600
#     img_shape = (2 * resolution_cropped[0], resolution_cropped[1], 3)  # 900 * 600
#     img_height, img_width = resolution_cropped[:2]  # 450 * 600
#     shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
#     shm_name = shm.name
#     img_array = np.ndarray((img_shape[0], img_shape[1], 3), dtype=np.uint8, buffer=shm.buf)

#     # television
#     cert_file = "cert.pem"
#     key_file = "key.pem"
#     image_queue = Queue()
#     toggle_streaming = Event()
#     tv = TeleVision(resolution_cropped, image_queue, toggle_streaming, cert_file=cert_file, key_file=key_file, stream_mode="webrtc") 
#     while True:
#         # tv.printPoses()
#         time.sleep(0.5)
    