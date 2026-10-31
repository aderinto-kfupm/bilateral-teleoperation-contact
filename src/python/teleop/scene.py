"""PyBullet scene: Franka arm + spherical probe + rigid inspection surface (plane or curved).

Plays the role of the paper's UR10 + force sensor + blackboard (Fig. 1, blocks 5-7):
joint-position-controlled arm, probe-surface contact wrench reconstructed from contact points,
and a ground-truth surface normal for scoring the online normal estimate.
"""
import os

import numpy as np
import pybullet as p
import pybullet_data

PROBE_R = 0.01          # probe sphere radius [m]
TOOL_OFFSET = 0.13      # hand frame -> probe centre along hand z [m]
Q_HOME = [0.0, -0.5, 0.0, -2.4, 0.0, 1.9, 0.785]


class Scene:
    def __init__(self, gui=False, surface="plane", dt=1 / 240, Ke=1e4):
        self.dt, self.surface_kind, self.Ke = dt, surface, Ke
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(dt)
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(numSolverIterations=100)
        p.loadURDF("plane.urdf")
        self.table = p.loadURDF("table/table.urdf", [0.5, 0, 0], useFixedBase=True)
        self.table_z = p.getAABB(self.table)[1][2]
        self.robot = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "franka_panda/panda.urdf"),
                                [0, 0, self.table_z], useFixedBase=True,
                                flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.hand = next(i for i in range(p.getNumJoints(self.robot))
                         if b"panda_hand" == p.getJointInfo(self.robot, i)[12])
        for j, q in enumerate(Q_HOME):
            p.resetJointState(self.robot, j, q)
        for f in (9, 10):
            p.resetJointState(self.robot, f, 0.0)
        self._build_surface()
        self._build_probe()
        self.hand_orn = self._hand_pose()[1]
        self.hand_R = np.array(p.getMatrixFromQuaternion(self.hand_orn)).reshape(3, 3)
        self.lo = np.array([p.getJointInfo(self.robot, j)[8] for j in range(7)])
        self.hi = np.array([p.getJointInfo(self.robot, j)[9] for j in range(7)])
        for _ in range(120):
            p.stepSimulation()

    # ---------------------------------------------------------------- build
    def _build_surface(self):
        top = self.table_z + 0.12           # crown / top face height
        self.surface_top = top
        rgba = [0.75, 0.78, 0.85, 1]
        if self.surface_kind == "plane":
            half = [0.25, 0.20, 0.05]
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
            self.surface = p.createMultiBody(0, col, vis, [0.55, 0.0, top - half[2]])
        else:  # convex cylinder, axis along y
            R, L = 0.30, 0.40
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=R, height=L)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=R, length=L, rgbaColor=rgba)
            self.cyl_R, self.cyl_c = R, np.array([0.55, 0.0, top - R])
            self.surface = p.createMultiBody(0, col, vis, self.cyl_c.tolist(),
                                             p.getQuaternionFromEuler([np.pi / 2, 0, 0]))
        p.changeDynamics(self.surface, -1, lateralFriction=0.3, contactStiffness=self.Ke,
                         contactDamping=2 * np.sqrt(self.Ke * 0.05))

    def _build_probe(self):
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=PROBE_R)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=PROBE_R, rgbaColor=[0.9, 0.2, 0.2, 1])
        self.probe = p.createMultiBody(0.05, col, vis, self._tip_from_hand())
        com = np.array(p.getLinkState(self.robot, self.hand)[2])   # constraint frames are COM frames
        p.createConstraint(self.robot, self.hand, self.probe, -1, p.JOINT_FIXED,
                           [0, 0, 0], (np.array([0, 0, TOOL_OFFSET]) - com).tolist(), [0, 0, 0])
        for l in range(-1, p.getNumJoints(self.robot)):          # probe only touches the surface
            p.setCollisionFilterPair(self.robot, self.probe, l, -1, 0)
        # PyBullet multiplies the two bodies' friction coefficients: 1.0 here gives mu = 0.3 overall
        p.changeDynamics(self.probe, -1, lateralFriction=1.0, contactStiffness=self.Ke,
                         contactDamping=2 * np.sqrt(self.Ke * 0.05))

    # --------------------------------------------------------------- state
    def _hand_pose(self):
        s = p.getLinkState(self.robot, self.hand, computeForwardKinematics=True)
        return np.array(s[4]), s[5]

    def _tip_from_hand(self):
        pos, orn = self._hand_pose()
        return (pos + np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3) @ [0, 0, TOOL_OFFSET]).tolist()

    def tip(self):
        """Measured probe-centre position and velocity (world frame)."""
        pos, _ = p.getBasePositionAndOrientation(self.probe)
        vel, _ = p.getBaseVelocity(self.probe)
        return np.array(pos), np.array(vel)

    def surface_height_normal(self, x):
        """Analytic surface height and outward normal under x -- ground truth for the path/normal."""
        if self.surface_kind == "plane":
            return self.surface_top, np.array([0.0, 0.0, 1.0])
        dx = x - self.cyl_c[0]
        z = self.cyl_c[2] + np.sqrt(self.cyl_R ** 2 - dx ** 2)
        return z, np.array([dx, 0.0, z - self.cyl_c[2]]) / self.cyl_R

    def contact_wrench(self):
        """Force the robot exerts on the environment [N] (= -force on probe), true normal, #points."""
        pts = p.getContactPoints(self.probe, self.surface)
        f_on_probe, n = np.zeros(3), np.zeros(3)
        for c in pts:
            nB = np.array(c[7])                     # contact normal on surface, points surface -> probe
            # PyBullet reports lateral friction as acting on body B (the surface): flip it for the probe
            f_on_probe += c[9] * nB - c[10] * np.array(c[11]) - c[12] * np.array(c[13])
            n += nB
        if pts:
            n /= np.linalg.norm(n)
        return -f_on_probe, (n if pts else None), len(pts)

    # ------------------------------------------------------------- command
    def command_tip(self, p_c, max_torque=87.0):
        """Position command for the probe centre -> IK -> joint position control."""
        hand_target = np.asarray(p_c) - self.hand_R @ [0, 0, TOOL_OFFSET]
        q = p.calculateInverseKinematics(self.robot, self.hand, hand_target.tolist(), self.hand_orn,
                                         lowerLimits=self.lo.tolist(), upperLimits=self.hi.tolist(),
                                         jointRanges=(self.hi - self.lo).tolist(),
                                         restPoses=Q_HOME, maxNumIterations=30, residualThreshold=1e-5)[:7]
        p.setJointMotorControlArray(self.robot, list(range(7)), p.POSITION_CONTROL, targetPositions=q,
                                    forces=[max_torque] * 7, positionGains=[0.2] * 7, velocityGains=[1.0] * 7)
        q = np.array(q)
        return {"near_joint_limit": bool(np.any(np.minimum(q - self.lo, self.hi - q) < 0.02)),
                "ik_err": float(np.linalg.norm(self._hand_pose()[0] - hand_target))}

    def step(self):
        p.stepSimulation()

    def close(self):
        p.disconnect(self.client)
