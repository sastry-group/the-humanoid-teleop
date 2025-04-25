#!/usr/bin/env python3
"""
teleop_print_standalone.py

A minimal WebXR server for Quest-2 hand‐pose streaming via ngrok.
Prints out each HAND_MOVE event’s wrist-transform and 25×3 landmarks,
without relying on TeleVision.py or shared_memory.
"""

import asyncio
import numpy as np
from vuer import Vuer
from vuer.events import ClientEvent
from vuer.schemas import DefaultScene, Hands

async def on_hand_move(event: ClientEvent, session, fps=60):
    """
    Handler for HAND_MOVE events. Reshapes and prints:
      - leftHand  (16 floats → 4×4 matrix)
      - leftLandmarks  (75 floats → 25×3 array)
      - rightHand
      - rightLandmarks
    """
    try:
        # reshape column-major (‘F’) 4×4 matrices
        Lw = np.array(event.value["leftHand"]).reshape(4, 4, order="F")
        Rw = np.array(event.value["rightHand"]).reshape(4, 4, order="F")
        # reshape 25×3 landmark arrays
        Lm = np.array(event.value["leftLandmarks"]).reshape(25, 3)
        Rm = np.array(event.value["rightLandmarks"]).reshape(25, 3)
        # reshape head transform

        # print a trimmed view
        print("⟵ Left Wrist Transform:\n", Lw)
        print("⟵ Left Landmarks (first 5):\n", Lm[:5])
        print("→  Right Wrist Transform:\n", Rw)
        print("→  Right Landmarks (first 5):\n", Rm[:5])
        print("―" * 60)
    except Exception as e:
        print("⚠️  Error in HAND_MOVE handler:", e)

async def on_cam_move(event: ClientEvent, session):
    """
    Handler for CAM_MOVE events. Reshapes and prints:
      - camera (4×4 matrix)
    """
    if "camera" in event.value:
        flat = event.value["camera"]["matrix"]
    else:
        flat = event.value.get("matrix", None)

    if flat is None:
        print("⚠️  No camera matrix in CAM_MOVE event")
        return
    
    head_mat = np.array(flat).reshape(4, 4, order="F")
    print("⟵ Camera Transform:\n", head_mat)
    print("―" * 60)

async def main_scene(session, fps=60):
    """
    A minimal Vuer scene that just streams and draws your hands
    so you can verify visually on the headset.
    """
    session.set @ DefaultScene(frameloop="always")
    session.upsert @ Hands(fps=fps, stream=True)
    while True:
        await asyncio.sleep(1)

def main():
    # 1) Instantiate the Vuer app WITH ngrok (no certs needed)
    app = Vuer(host="0.0.0.0", queries=dict(grid=False), queue_len=3)

    # 2) Register our HAND_MOVE handler
    app.add_handler("HAND_MOVE")(on_hand_move)
    app.add_handler("CAM_MOVE")(on_cam_move)

    # 3) Spawn our very simple scene (hand skeleton overlay)
    app.spawn(start=False)(main_scene)

    # 4) Run the server (default port 8012) and ngrok-tunnel it
    print("🚀 Starting standalone TeleOp print server (ngrok)…")
    app.run()

if __name__ == "__main__":
    main()

