#!/usr/bin/env python3
"""VisionBridge runner (stable indentation-safe build)
Adds safe-mode, OpenCV viewing, heartbeats, and robust CLI.
"""
import os
import argparse
from typing import Dict, List, Optional, Callable

import numpy as np
import cv2

from pybullet_vision_env_controlled import VisionEnv  # same folder

Detection = Dict  # expected keys: 'bbox'[x1,y1,x2,y2], 'label', 'score', optional 'mask' (H,W) bool/uint8


class VisionBridge:
    def __init__(self, detector: Optional[Callable[[Dict], List[Detection]]] = None):
        self.detector = detector

    @staticmethod
    def annotate(rgb: np.ndarray, detections: List[Detection]) -> np.ndarray:
        img = rgb.copy()
        for det in detections:
            if 'bbox' in det:
                x1, y1, x2, y2 = map(int, det['bbox'])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{det.get('label','obj')} {det.get('score', 1.0):.2f}"
                cv2.putText(img, label, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return img

    @staticmethod
    def mask_centroid(mask: np.ndarray) -> Optional[tuple]:
        ys, xs = np.where(mask > 0)
        if len(xs) == 0:
            return None
        return (int(xs.mean()), int(ys.mean()))

    @staticmethod
    def pixel_to_world(u: int, v: int, depth_m: float, obs: Dict) -> Optional[np.ndarray]:
        if depth_m <= 0 or np.isinf(depth_m) or np.isnan(depth_m):
            return None
        fx, fy = obs['intrinsics']['fx'], obs['intrinsics']['fy']
        cx, cy = obs['intrinsics']['cx'], obs['intrinsics']['cy']
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        cam_p = np.array([x, y, depth_m, 1.0])
        world_p = obs['world_T_cam'] @ cam_p
        return world_p[:3]

    # Built-in prototype detectors
    @staticmethod
    def segmentation_detector(obs: Dict, include_links: bool = False) -> List[Detection]:
        seg = obs['seg']
        detections: List[Detection] = []
        unique_vals, _ = np.unique(seg, return_counts=True)
        for val in unique_vals:
            if val < 0:
                continue
            body_uid = val >> 24
            link_idx = val & ((1 << 24) - 1)
            if (not include_links) and (link_idx not in (-1, 0)):
                continue
            ys, xs = np.where(seg == val)
            if len(xs) == 0:
                continue
            x1, y1, x2, y2 = xs.min(), ys.min(), xs.max(), ys.max()
            det = {
                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                'label': f"id{body_uid}:link{link_idx}",
                'score': 1.0,
                'mask': (seg == val).astype(np.uint8),
            }
            detections.append(det)
        return detections

    @staticmethod
    def user_stub_detector(obs: Dict) -> List[Detection]:
        return []  # plug your model here

    def perceive(self, obs: Dict) -> List[Detection]:
        if self.detector is None:
            return []
        return self.detector(obs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true', help='Enable PyBullet GUI')
    parser.add_argument('--use-overhead', action='store_true', help='Enable overhead camera')
    parser.add_argument('--use-wrist', action='store_true', help='Enable wrist camera')
    parser.add_argument('--use-seg-detector', action='store_true', help='Use built-in seg-based detector')
    parser.add_argument('--detector', type=str, default='stub', choices=['stub', 'seg'])
    parser.add_argument('--save', type=str, default=None, help='Directory to save annotated frames')
    parser.add_argument('--n_cubes', type=int, default=8)
    parser.add_argument('--tiny', action='store_true', help='Use CPU TinyRenderer')
    parser.add_argument('--realtime', action='store_true', help='Real-time simulation pacing')
    parser.add_argument('--steps', type=int, default=-1, help='Steps to run; -1 = until Ctrl+C')
    parser.add_argument('--hold', type=float, default=0.0, help='Keep GUI open N seconds after loop')
    parser.add_argument('--cv2-view', action='store_true', help='Show camera streams via OpenCV windows')
    parser.add_argument('--safe-mode', action='store_true', help='Single overhead @320x240 + TinyRenderer')
    parser.add_argument('--print-every', type=int, default=120, help='Heartbeat every N steps')
    parser.add_argument('--overhead-only', action='store_true', help='Force overhead only')
    parser.add_argument('--no-centroids', action='store_true', help='Skip centroid calculation/printing')
    parser.add_argument('--robot-mount', type=str, default='table', choices=['floor','table'], help='Mount robot on floor or table')
    parser.add_argument('--robot-x', type=float, default=0.15, help='Robot X when mounted on table')
    parser.add_argument('--robot-y', type=float, default=0.0, help='Robot Y when mounted on table')
    parser.add_argument('--base-z-offset', type=float, default=0.02, help='Z offset above tabletop when mounted')
    parser.add_argument('--controller', action='store_true', help='Enable F710 robot control and haptic feedback')
    parser.add_argument('--controller-index', type=int, default=0, help='Controller index (default: 0)')
    parser.add_argument('--rumble-strength', type=float, default=0.5, help='Contact rumble strength')
    parser.add_argument('--rumble-duration', type=int, default=100, help='Contact rumble duration in milliseconds')
    parser.add_argument('--control-speed', type=float, default=0.012, help='End-effector speed per simulation step in meters')
    parser.add_argument('--physics-steps', type=int, default=2, help='Physics steps per controller update')
    parser.add_argument('--debug-controller', action='store_true', help='Print controller axes while running')
    args = parser.parse_args()

    if args.use_seg_detector:
        args.detector = 'seg'

    # Safe-mode overrides
    use_overhead = True if (args.overhead_only or args.safe_mode or args.use_overhead or (not args.use_wrist)) else False
    use_wrist = False if (args.overhead_only or args.safe_mode) else args.use_wrist
    overhead_size = (320, 240) if args.safe_mode else (640, 480)
    wrist_size = (320, 240) if args.safe_mode else (640, 480)
    tiny_flag = (args.tiny or args.safe_mode)

    env = VisionEnv(gui=args.gui, realtime=args.realtime)
    env.reset(n_cubes=args.n_cubes,
              use_overhead=use_overhead,
              use_wrist=use_wrist,
              overhead_size=overhead_size,
              wrist_size=wrist_size,
              tiny_renderer_fallback=tiny_flag,
              robot_mount=args.robot_mount,
          robot_xy=(args.robot_x, args.robot_y),
          base_z_offset=args.base_z_offset)

    if args.detector == 'seg':
        bridge = VisionBridge(detector=VisionBridge.segmentation_detector)
    else:
        bridge = VisionBridge(detector=VisionBridge.user_stub_detector)

    controller = None
    if args.controller:
        from controller_test_ps5 import F710Controller
        import pygame

        pygame.init()
        controller = F710Controller(index=args.controller_index)

    if args.save:
        os.makedirs(args.save, exist_ok=True)

    cams = []
    if 'overhead' in env.cameras:
        cams.append('overhead')
    if 'wrist' in env.cameras:
        cams.append('wrist')

    try:
        t = 0
        running = True
        previous_touched = set()
        previous_grasped = set()
        last_axes = []
        while running:
            if controller is not None:
                axes, buttons = controller.read()
                last_axes = axes
                env.apply_controller(axes, buttons, linear_speed=args.control_speed)
            env.step(max(1, args.physics_steps))

            if controller is not None:
                touched, grasped = env.cube_contact_state()
                if touched - previous_touched:
                    controller.vibrate(args.rumble_strength, args.rumble_duration)
                if grasped - previous_grasped:
                    controller.vibrate(args.rumble_strength, args.rumble_duration * 2)
                previous_touched = touched
                previous_grasped = grasped

            for cname in cams:
                obs = env.get_camera_observation(cname)
                dets = bridge.perceive(obs)

                # Optional centroid computation
                centroids_world = []
                if not args.no_centroids:
                    for d in dets:
                        if 'mask' in d and isinstance(d['mask'], np.ndarray):
                            c = VisionBridge.mask_centroid(d['mask'])
                            if c is not None:
                                u, v = c
                                depth_m = float(obs['depth_m'][v, u])
                                wp = VisionBridge.pixel_to_world(u, v, depth_m, obs)
                                if wp is not None:
                                    centroids_world.append((d.get('label', 'obj'), wp))

                if (not args.no_centroids) and len(centroids_world) > 0:
                    print(f"[{cname}] 3D centroids:")
                    for label, pos in centroids_world:
                        print(f"  - {label}: (x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f})")

                # Annotate & optionally save/show
                ann = VisionBridge.annotate(obs['rgb'], dets) if len(dets) > 0 else obs['rgb']
                bgr = cv2.cvtColor(ann, cv2.COLOR_RGBA2BGR)
                if args.save:
                    cv2.imwrite(os.path.join(args.save, f"{cname}_{t:06d}.png"), bgr)
                if args.cv2_view:
                    win = f"cam:{cname}"
                    try:
                        cv2.imshow(win, bgr)
                        cv2.waitKey(1)
                    except Exception:
                        pass

            t += 1
            if args.print_every > 0 and (t % args.print_every == 0):
                print(f"[heartbeat] step={t}")
                if args.debug_controller and controller is not None:
                    print("[controller] axes=" + ", ".join(f"{value:+.3f}" for value in last_axes))

            if args.steps >= 0 and t >= args.steps:
                running = False

    except KeyboardInterrupt:
        pass
    finally:
        if controller is not None:
            controller.close()
            import pygame
            pygame.quit()
        if args.cv2_view:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        if args.gui and args.hold > 0.0:
            import time as _time
            _time.sleep(args.hold)
        env.close()


if __name__ == "__main__":
    main()
