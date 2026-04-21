import math
import cv2
import numpy as np
import pybullet as p

class MultiViewRenderer:
    def __init__(self, width=420, height=300, max_depth_m=8.0):
        self.width = width
        self.height = height
        self.max_depth_m = max_depth_m

    def _camera_image(self, view_matrix, proj_matrix):
        img = p.getCameraImage(
            self.width,
            self.height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )
        rgba = np.reshape(img[2], (self.height, self.width, 4))
        depth = np.reshape(img[3], (self.height, self.width))
        rgb = rgba[:, :, :3].astype(np.uint8)
        return rgb, depth

    def render(self, car_id: int):
        pos, orn = p.getBasePositionAndOrientation(car_id)
        x, y, z = pos
        _, _, yaw = p.getEulerFromQuaternion(orn)

        # -------- top-down camera --------
        top_eye = [0, 0, 12]
        top_target = [0, 0, 0]
        top_up = [0, 1, 0]
        top_view = p.computeViewMatrix(top_eye, top_target, top_up)
        top_proj = p.computeProjectionMatrixFOV(
            fov=55,
            aspect=self.width / self.height,
            nearVal=0.1,
            farVal=30.0,
        )
        top_rgb, _ = self._camera_image(top_view, top_proj)

        # -------- windshield camera --------
        # Use only yaw from the car base to reduce jitter.
        forward = [math.cos(yaw), math.sin(yaw), 0.0]
        up = [0.0, 0.0, 1.0]

        cam_eye = [
            x + 0.42 * forward[0],
            y + 0.42 * forward[1],
            z + 0.28,
        ]
        cam_target = [
            cam_eye[0] + 2.8 * forward[0],
            cam_eye[1] + 2.8 * forward[1],
            cam_eye[2] + 0.02,
        ]

        car_view = p.computeViewMatrix(cam_eye, cam_target, up)
        car_proj = p.computeProjectionMatrixFOV(
            fov=78,
            aspect=self.width / self.height,
            nearVal=0.02,
            farVal=25.0,
        )
        rgb, depth_buf = self._camera_image(car_view, car_proj)

        # Convert depth buffer to meters
        near = 0.02
        far = 25.0
        depth_m = far * near / (far - (far - near) * depth_buf)
        depth_m = np.clip(depth_m, 0.0, self.max_depth_m)

        # Make near = bright, far = dark
        depth_u8 = (255.0 * (1.0 - depth_m / self.max_depth_m)).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)

        # Convert RGB for OpenCV
        top_bgr = cv2.cvtColor(top_rgb, cv2.COLOR_RGB2BGR)
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        cv2.putText(top_bgr, "Top View", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(rgb_bgr, "Car RGB", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(depth_color, "Car Depth", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        combined = np.hstack([top_bgr, rgb_bgr, depth_color])
        cv2.imshow("Parking Simulator Views", combined)
        cv2.waitKey(1)

    def close(self):
        cv2.destroyAllWindows()