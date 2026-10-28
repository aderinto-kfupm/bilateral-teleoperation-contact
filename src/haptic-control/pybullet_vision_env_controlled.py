#!/usr/bin/env python3
import os
import math
import time
import random
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List

import numpy as np
import pybullet as p
import pybullet_data


@dataclass
class CameraConfig:
    name: str
    width: int = 640
    height: int = 480
    fov_deg: float = 60.0  # vertical FOV
    near: float = 0.01
    far: float = 5.0
    renderer: int = p.ER_BULLET_HARDWARE_OPENGL  # fallback to p.ER_TINY_RENDERER if headless

    # Static camera params (used when attach_to is None)
    pos_world: Tuple[float, float, float] = (0.7, 0.0, 0.75)
    target_world: Tuple[float, float, float] = (0.5, 0.0, 0.62)  # look at tabletop by default
    up_world: Tuple[float, float, float] = (0.0, 0.0, 1.0)

    # If attached to a robot link
    attach_to: Optional[Dict] = None  # {'body_id': int,'link_index': int,'rel_pos': (x,y,z),'rel_euler': (r,p,y)}


class VisionEnv:
    """
    PyBullet environment with a Franka Panda arm and table.
    Supports mounting the robot either on the floor or on the tabletop.
    Exposes RGB/Depth/Segmentation + intrinsics/extrinsics for vision tasks.
    """
    def __init__(self,
                 gui: bool = True,
                 time_step: float = 1.0/240.0,
                 realtime: bool = False,
                 seed: Optional[int] = None,
                 verbose: bool = True):
        self.gui = gui
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.resetSimulation()
        p.setTimeStep(time_step)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.realtime = realtime
        if realtime:
            p.setRealTimeSimulation(1)
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        self.verbose = verbose

        self.robot_id = None
        self.table_id = None
        self.table_top_z = 0.62  # will be updated from AABB
        self.cube_ids: List[int] = []
        self.cameras: Dict[str, CameraConfig] = {}
        self.ee_target = None

    # ------------------------
    # Loading helpers
    # ------------------------
    def _load_plane_and_table(self):
        if self.verbose:
            print("[env] Loading plane and table...")
        p.loadURDF("plane.urdf", [0, 0, 0])
        table_urdf = os.path.join(pybullet_data.getDataPath(), "table/table.urdf")
        self.table_id = p.loadURDF(table_urdf, [0.5, 0.0, 0.0], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        # infer tabletop z from AABB
        aabb_min, aabb_max = p.getAABB(self.table_id, -1)
        self.table_top_z = float(aabb_max[2])

    def _load_panda(self, base_pos=(0.0, 0.0, 0.0), base_orn=(0, 0, 0)):
        if self.verbose:
            print(f"[env] Loading Franka Panda robot at base_pos={base_pos}...")
        panda_urdf = os.path.join(pybullet_data.getDataPath(), "franka_panda/panda.urdf")
        self.robot_id = p.loadURDF(
            panda_urdf,
            basePosition=base_pos,
            baseOrientation=p.getQuaternionFromEuler(base_orn),
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE
        )

        # Default joint positions (simple folded pose)
        default_q = [0, -0.5, 0, -2.7, 0, 2.0, 0.8]
        joint_indices = [0, 1, 2, 3, 4, 5, 6]
        for j, q in zip(joint_indices, default_q):
            p.resetJointState(self.robot_id, j, q)
        # Open fingers (joints 9 and 10 in this URDF)
        p.resetJointState(self.robot_id, 9, 0.04)
        p.resetJointState(self.robot_id, 10, 0.04)

    def _find_link_index(self, name_substring: str) -> int:
        n = p.getNumJoints(self.robot_id)
        for i in range(n):
            info = p.getJointInfo(self.robot_id, i)
            link_name = info[12].decode("utf-8")
            if name_substring in link_name:
                return i
        return n - 1

    def _spawn_random_cubes(self, n: int = 8):
        if self.verbose:
            print(f"[env] Spawning {n} colored cubes on the table...")
        self.cube_ids.clear()
        cube_urdf = os.path.join(pybullet_data.getDataPath(), "cube_small.urdf")
        z = self.table_top_z + 0.05
        for _ in range(n):
            x = 0.45 + 0.15 * (random.random() - 0.5)
            y = 0.00 + 0.30 * (random.random() - 0.5)
            bid = p.loadURDF(cube_urdf, [x, y, z])
            self.cube_ids.append(bid)
            rgba = [random.random(), random.random(), random.random(), 1.0]
            p.changeVisualShape(bid, -1, rgbaColor=rgba)
            p.changeDynamics(bid, -1, lateralFriction=0.7, rollingFriction=0.01, spinningFriction=0.01)

    # ------------------------
    # Camera management
    # ------------------------
    def add_overhead_camera(self,
                            name: str = "overhead",
                            width: int = 640,
                            height: int = 480,
                            fov_deg: float = 60.0,
                            pos_world: Tuple[float, float, float] = (0.7, 0.0, 0.75),
                            target_world: Optional[Tuple[float, float, float]] = None,
                            up_world: Tuple[float, float, float] = (0.0, 0.0, 1.0),
                            renderer: int = p.ER_BULLET_HARDWARE_OPENGL):
        if target_world is None:
            target_world = (0.5, 0.0, self.table_top_z)
        cfg = CameraConfig(
            name=name, width=width, height=height, fov_deg=fov_deg,
            pos_world=pos_world, target_world=target_world, up_world=up_world,
            renderer=renderer, attach_to=None,
        )
        self.cameras[name] = cfg
        return cfg

    def add_wrist_camera(self,
                         name: str = "wrist",
                         width: int = 640,
                         height: int = 480,
                         fov_deg: float = 70.0,
                         rel_pos=(0.0, 0.0, 0.08),
                         rel_euler=(0.0, -math.pi/2, math.pi),
                         link_hint: str = "panda_hand",
                         renderer: int = p.ER_BULLET_HARDWARE_OPENGL):
        if self.robot_id is None:
            raise RuntimeError("Robot must be loaded before adding a wrist camera.")
        link_index = self._find_link_index(link_hint)
        cfg = CameraConfig(
            name=name, width=width, height=height, fov_deg=fov_deg,
            renderer=renderer,
            attach_to={
                'body_id': self.robot_id,
                'link_index': link_index,
                'rel_pos': rel_pos,
                'rel_euler': rel_euler,
            }
        )
        self.cameras[name] = cfg
        return cfg

    @staticmethod
    def _intrinsics_from_fov(width: int, height: int, fov_deg: float):
        fov_rad = math.radians(fov_deg)
        fy = (height / 2.0) / math.tan(fov_rad / 2.0)
        fx = fy * (width / float(height))
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0
        return {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 'width': width, 'height': height, 'fov_deg': fov_deg}

    @staticmethod
    def _list_to_mat(m: List[float]) -> np.ndarray:
        return np.array(m, dtype=np.float64).reshape(4, 4, order='F')

    @staticmethod
    def _invert_4x4(m: List[float]) -> np.ndarray:
        return np.linalg.inv(VisionEnv._list_to_mat(m))

    @staticmethod
    def _depth_buffer_to_meters(depth_buf: np.ndarray, near: float, far: float) -> np.ndarray:
        return far * near / (far - (far - near) * depth_buf)

    @staticmethod
    def _compute_view_and_pose(cfg: CameraConfig) -> Tuple[List[float], List[float], np.ndarray, np.ndarray]:
        aspect = cfg.width / float(cfg.height)
        proj = p.computeProjectionMatrixFOV(cfg.fov_deg, aspect, cfg.near, cfg.far)

        if cfg.attach_to is None:
            view = p.computeViewMatrix(cfg.pos_world, cfg.target_world, cfg.up_world)
        else:
            att = cfg.attach_to
            ls = p.getLinkState(att['body_id'], att['link_index'], computeForwardKinematics=True)
            link_pos = ls[4]
            link_orn = ls[5]
            cam_pos, cam_orn = p.multiplyTransforms(link_pos, link_orn,
                                                    att['rel_pos'], p.getQuaternionFromEuler(att['rel_euler']))
            forward = p.rotateVector(cam_orn, (0, 0, 0.1))
            target = (cam_pos[0] + forward[0], cam_pos[1] + forward[1], cam_pos[2] + forward[2])
            up = p.rotateVector(cam_orn, (0, 1, 0))
            view = p.computeViewMatrix(cam_pos, target, up)

        cam_T_world = VisionEnv._list_to_mat(view)   # world -> cam
        world_T_cam = np.linalg.inv(cam_T_world)     # cam -> world
        return view, proj, cam_T_world, world_T_cam

    def get_camera_observation(self, cam_name: str) -> Dict:
        cfg = self.cameras[cam_name]
        view, proj, cam_T_world, world_T_cam = self._compute_view_and_pose(cfg)
        width, height = cfg.width, cfg.height
        img = p.getCameraImage(width, height, view, proj,
                               renderer=cfg.renderer,
                               flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
        rgb = np.reshape(img[2], (height, width, 4)).astype(np.uint8)
        depth_buf = np.reshape(img[3], (height, width))
        seg = np.reshape(img[4], (height, width))
        depth_m = self._depth_buffer_to_meters(depth_buf, cfg.near, cfg.far)

        intr = self._intrinsics_from_fov(width, height, cfg.fov_deg)
        obs = {
            'name': cam_name,
            'rgb': rgb,
            'depth_m': depth_m,
            'seg': seg,
            'intrinsics': intr,
            'cam_T_world': cam_T_world,
            'world_T_cam': world_T_cam,
            'near': cfg.near,
            'far': cfg.far,
            'fov_deg': cfg.fov_deg,
        }
        return obs

    @staticmethod
    def segmentation_to_body_link(seg_val: int) -> Tuple[int, int]:
        if seg_val < 0:
            return -1, -1
        body_uid = seg_val >> 24
        link_idx = seg_val & ((1 << 24) - 1)
        return body_uid, link_idx

    @staticmethod
    def backproject_to_camera_xyzw(depth_m: np.ndarray, intr: Dict) -> np.ndarray:
        h, w = depth_m.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth_m
        x = (u - intr['cx']) * z / intr['fx']
        y = (v - intr['cy']) * z / intr['fy']
        ones = np.ones_like(z)
        return np.stack([x, y, z, ones], axis=-1)

    @staticmethod
    def camera_to_world(xyz1_cam: np.ndarray, world_T_cam: np.ndarray) -> np.ndarray:
        h, w, _ = xyz1_cam.shape
        flat = xyz1_cam.reshape(-1, 4).T
        out = world_T_cam @ flat
        return out.T.reshape(h, w, 4)

    # ------------------------
    # Public API
    # ------------------------
    def reset(self,
              n_cubes: int = 8,
              use_overhead: bool = True,
              use_wrist: bool = True,
              overhead_size: Tuple[int, int] = (640, 480),
              wrist_size: Tuple[int, int] = (640, 480),
              tiny_renderer_fallback: bool = False,
              robot_mount: str = "table",            # 'floor' or 'table'
              robot_xy: Tuple[float, float] = (0.15, 0.0),
              base_z_offset: float = 0.02,
              base_yaw_deg: float = 0.0):
        p.resetSimulation()
        self.ee_target = None
        p.setGravity(0, 0, -9.81)
        self._load_plane_and_table()

        # Decide robot base pose
        if robot_mount.lower() == "table":
            base_z = self.table_top_z + base_z_offset
            base_pos = (robot_xy[0], robot_xy[1], base_z)
        else:
            base_pos = (0.0, 0.0, 0.0)
        self._load_panda(base_pos=base_pos, base_orn=(0.0, 0.0, math.radians(base_yaw_deg)))

        self._spawn_random_cubes(n_cubes)

        renderer = p.ER_TINY_RENDERER if tiny_renderer_fallback else p.ER_BULLET_HARDWARE_OPENGL
        self.cameras.clear()
        if use_overhead:
            self.add_overhead_camera(width=overhead_size[0], height=overhead_size[1],
                                     renderer=renderer,
                                     target_world=(0.5, 0.0, self.table_top_z))
        if use_wrist:
            self.add_wrist_camera(width=wrist_size[0], height=wrist_size[1], renderer=renderer)

    def apply_controller(self, axes: List[float], buttons: List[bool], deadzone: float = 0.08,
                         linear_speed: float = 0.012, joint_speed: float = 2.5):
        """Apply one F710 sample as a small Cartesian end-effector command."""
        if self.robot_id is None:
            return
        hand_link = self._find_link_index("panda_hand")
        state = p.getLinkState(self.robot_id, hand_link, computeForwardKinematics=True)
        if self.ee_target is None:
            self.ee_target = np.array(state[4], dtype=np.float64)

        def axis(index: int) -> float:
            value = axes[index] if index < len(axes) else 0.0
            return 0.0 if abs(value) < deadzone else float(value)

        self.ee_target += np.array([
            axis(0) * linear_speed,
            -axis(1) * linear_speed,
            -axis(3) * linear_speed,
        ])
        target_joints = p.calculateInverseKinematics(
            self.robot_id, hand_link, self.ee_target, targetOrientation=state[5]
        )
        p.setJointMotorControlArray(
            self.robot_id,
            list(range(7)),
            p.POSITION_CONTROL,
            targetPositions=target_joints[:7],
            forces=[120.0] * 7,
            positionGains=[0.35] * 7,
            velocityGains=[1.0] * 7,
        )

        finger_position = 0.0 if buttons and buttons[0] else 0.04
        if len(buttons) > 1 and buttons[1]:
            finger_position = 0.04
        p.setJointMotorControlArray(
            self.robot_id,
            [9, 10],
            p.POSITION_CONTROL,
            targetPositions=[finger_position, finger_position],
            forces=[20.0, 20.0],
        )

    def cube_contact_state(self) -> Tuple[set, set]:
        """Return cubes touched by the hand and cubes touched by both fingers."""
        touched = set()
        grasped = set()
        finger_links = {9, 10}
        hand_link = self._find_link_index("panda_hand")
        for cube_id in self.cube_ids:
            contacts = p.getContactPoints(bodyA=self.robot_id, bodyB=cube_id)
            if not contacts:
                continue
            contact_links = {point[3] for point in contacts}
            if hand_link in contact_links or contact_links.intersection(finger_links):
                touched.add(cube_id)
            if finger_links.issubset(contact_links):
                grasped.add(cube_id)
        return touched, grasped

    def step(self, n: int = 1):
        for _ in range(n):
            p.stepSimulation()
            if self.realtime and self.gui:
                time.sleep(p.getPhysicsEngineParameters()["fixedTimeStep"])

    def close(self):
        if self.client >= 0:
            p.disconnect(self.client)
            self.client = -1


if __name__ == "__main__":
    import argparse
    import cv2

    parser = argparse.ArgumentParser(description="PyBullet Vision Environment Demo")
    parser.add_argument('--gui', action='store_true', help='Enable PyBullet GUI')
    parser.add_argument('--no-gui', action='store_true', help='Run headless (DIRECT)')
    parser.add_argument('--overhead', action='store_true', help='Enable overhead camera')
    parser.add_argument('--wrist', action='store_true', help='Enable wrist camera')
    parser.add_argument('--save', type=str, default=None, help='Directory to save frames')
    parser.add_argument('--n_cubes', type=int, default=8)
    parser.add_argument('--tiny', action='store_true', help='Use CPU TinyRenderer')
    parser.add_argument('--robot-mount', type=str, default='floor', choices=['floor', 'table'], help='Mount robot on floor or table')
    parser.add_argument('--robot-x', type=float, default=0.5, help='Robot X when mounted on table')
    parser.add_argument('--robot-y', type=float, default=0.0, help='Robot Y when mounted on table')
    parser.add_argument('--base-z-offset', type=float, default=0.02, help='Z offset above tabletop when mounted')
    args = parser.parse_args()

    env = VisionEnv(gui=(args.gui and not args.no_gui), realtime=False)
    env.reset(n_cubes=args.n_cubes,
              use_overhead=args.overhead or (not args.wrist),
              use_wrist=args.wrist,
              tiny_renderer_fallback=args.tiny,
              robot_mount=args.robot_mount,
              robot_xy=(args.robot_x, args.robot_y),
              base_z_offset=args.base_z_offset)

    os.makedirs(args.save or "", exist_ok=True) if args.save else None

    for t in range(300):
        env.step()
        if args.save and 'overhead' in env.cameras:
            obs = env.get_camera_observation('overhead')
            bgr = cv2.cvtColor(obs['rgb'], cv2.COLOR_RGBA2BGR)
            cv2.imwrite(os.path.join(args.save, f"overhead_{t:04d}.png"), bgr)

    env.close()
