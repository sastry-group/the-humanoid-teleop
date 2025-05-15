# import asyncio
# import numpy as np
# import cv2
# import pyzed.sl as sl
# from vuer import Vuer
# from vuer.schemas import DefaultScene, Hands, PointCloud

# CAMERA_IDX = 0
# FPS = 15  # point clouds are heavier

# app = Vuer(host="0.0.0.0", port=8012, queries=dict(grid=False), queue_len=3)

# @app.spawn(start=True)
# async def main(session, fps=FPS):
#     session.set @ DefaultScene(frameloop="always")
#     session.upsert @ Hands(fps=fps, stream=True)

#     # 1) Open ZED
#     zed = sl.Camera()
#     params = sl.InitParameters()
#     params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
#     zed.open(params)

#     xyzrgba = sl.Mat()

#     while True:
#         if zed.grab() != sl.ERROR_CODE.SUCCESS:
#             await asyncio.sleep(1/fps)
#             continue

#         # 2) Grab colored point cloud
#         zed.retrieve_measure(xyzrgba, sl.MEASURE.XYZRGBA)
#         pc = xyzrgba.get_data()  # H×W×4 (X,Y,Z,RGB packed)

#         # 3) Flatten and split into lists
#         points = []
#         colors = []
#         h, w, _ = pc.shape
#         flat = pc.reshape(-1, 4)

#         # 2) filter out any row with NaN in ANY of the 4 channels
#         valid_mask = np.isfinite(flat).all(axis=1)
#         valid = flat[valid_mask]
        

#         points_np = valid[:, :3]            # X, Y, Z
#         rgba_np   = valid[:, 3].astype(np.uint32)  # packed RGBA

#         r = (rgba_np & 0xFF0000) >> 16
#         g = (rgba_np & 0x00FF00) >> 8
#         b = (rgba_np & 0x0000FF)
#         colors_np = np.stack([r, g, b], axis=1) / 255.0

#         points = points_np.tolist()
#         colors = colors_np.tolist()

#         # for x, y, z, rgba in flat:
#         #     if z == 0:  # skip invalid depth
#         #         continue
#         #     points.append([float(x), float(y), float(z)])
#         #     # decode RGBA -> separate R,G,B
#         #     c = int(rgba)
#         #     r = (c & 0xFF0000) >> 16
#         #     g = (c & 0x00FF00) >> 8
#         #     b = (c & 0x0000FF)
#         #     colors.append([r/255, g/255, b/255])

#         # 4) Upload as a point cloud
#         session.upsert @ PointCloud(
#             points     = points,
#             colors     = colors,
#             point_size = 0.01,
#             key        = "zed-pc"
#         )

#         await asyncio.sleep(1/fps)

# if __name__ == "__main__":
#     app.run()
