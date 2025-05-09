#!/usr/bin/env python3
import cv2
import asyncio
from vuer import Vuer
from vuer.schemas import DefaultScene, Hands, ImageBackground
# import pyzed.sl as sl

# Change this to your camera index (0, 1, …) or path to ZED’s video device
CAMERA_IDX = 0
FPS = 30

app = Vuer(host="0.0.0.0", port=8012, queries=dict(grid=False), queue_len=3)

@app.spawn(start=True)
async def main(session, fps=FPS):
    # 1) Basic Vuer scene
    session.set @ DefaultScene(frameloop="always")
    session.upsert @ Hands(fps=fps, stream=True)

    # 2) OpenCV capture
    cap = cv2.VideoCapture(CAMERA_IDX)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {CAMERA_IDX}")

    # 3) Stream loop
    while True:
        ret, frame_bgr = cap.read()
        if not ret:
            await asyncio.sleep(1.0 / fps)
            continue

        # Convert to RGB for Vuer
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Upload as background quad (mono for now)
        session.upsert @ ImageBackground(
            frame_rgb,
            format="jpeg",
            quality=80,
            key="camera-feed",
            interpolate=True,
            aspect=frame_rgb.shape[1] / frame_rgb.shape[0],
            distanceToCamera=2,
            position=[0, 0, -3],
            height=3
        )

        await asyncio.sleep(1.0 / fps)

if __name__ == "__main__":
    app.run()
