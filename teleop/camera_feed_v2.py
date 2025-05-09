#!/usr/bin/env python3
import cv2
import asyncio
from vuer import Vuer
from vuer.schemas import DefaultScene, ImageBackground, Hands
import pyzed.sl as sl
import numpy as np

# Change this to your camera index (0, 1, …) or path to ZED’s video device
CAMERA_IDX = 0  
FPS = 30

app = Vuer(host="0.0.0.0", port=8012, queries=dict(grid=False), queue_len=3)

@app.spawn(start=True)
async def main(session, fps=FPS):
    # 1) Basic Vuer scene
    session.set @ DefaultScene(frameloop="always")
    session.upsert @ Hands(fps=fps, stream=True)

    # 1) Open ZED camera
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.depth_mode        = sl.DEPTH_MODE.PERFORMANCE
    zed.open(init)

    left_mat  = sl.Mat()
    right_mat = sl.Mat()


    # 3) Stream loop
    while True:
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            await asyncio.sleep(1/fps)
            continue

        # 2) Retrieve left & right RGBA images
        zed.retrieve_image(left_mat,  sl.VIEW.LEFT)
        zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
        left  = left_mat.get_data()[:, :, :3]  # H×W×3 BGR
        right = right_mat.get_data()[:, :, :3]

        # 3) Convert BGR→RGB
        left_rgb  = cv2.cvtColor(left,  cv2.COLOR_BGR2RGB)
        right_rgb = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)

        # 4) Upload two quads, tagging each for the correct eye
        session.upsert @ ImageBackground(
            left_rgb,
            format="jpeg",
            quality=75,
            key="zed-left",
            eye="left",                # <–– render this quad in the LEFT eye
            interpolate=True,
            distanceToCamera=2,
            height=3
        )
        session.upsert @ ImageBackground(
            right_rgb,
            format="jpeg",
            quality=75,
            key="zed-right",
            eye="right",               # <–– render this quad in the RIGHT eye
            interpolate=True,
            distanceToCamera=2,
            height=3
        )

        await asyncio.sleep(1.0 / fps)

if __name__ == "__main__":
    app.run()
