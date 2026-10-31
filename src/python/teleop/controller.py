"""Control law of Pagliara et al. (report Sec. III), pure numpy, no PyBullet.

Per-step pipeline (Fig. 1 block numbers in brackets):
  u -> [1] rate-to-position -> [2] moving average -> p_h
  p_h -> [3] reference saturation -> p_d -> [4] admittance -> p_c  (-> IK, outside)
  p_h - p_c -> [8] virtual spring-damper -> f_h -> [9] cue mapping -> (a_vib, a_trig)

Frames: everything is expressed in the robot-base (world-aligned) frame and the tool orientation is
held fixed, so R_{Re}^{Rb} = I in Eqs. (4)-(7). Normal n points out of the surface; pressing = -n.
"""
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class Params:
    dt: float = 1 / 240
    Md: float = 2.0            # admittance mass [kg]
    KD: float = 110.0          # damping [N s/m]
    KP: float = 1500.0         # stiffness [N/m]
    Ke: float = 5000.0         # environment stiffness estimate (Bullet: contactStiffness / 2)
    Kh: float = 1500.0         # virtual-feedback stiffness (== KP -> f_h ~ f_e in steady state)
    Dh: float = 25.0
    ma_window: int = 8         # moving-average samples (Eq. 3)
    f_th: float = 10.0         # saturation threshold [N]
    f_warn: float = 6.0        # trigger warning level [N]
    f_stop: float = 12.0       # trigger full-resistance level [N]
    f_vib_max: float = 10.0    # force mapped to full vibration [N]
    f_des: float = 0.0         # desired normal force (extension) [N]
    k_force: float = 0.004     # force-tracking integral gain [m/s per N] (extension)
    rest_correction: bool = True   # False = paper-literal p_e (last p_c); True adds fn/Ke so f -> f_th, not above
    contact_thr: float = 0.5   # |f_n| above which we call it contact [N]
    lp_force: float = 0.35     # 1st-order low-pass coefficient on the contact force (per step)
    lp_normal: float = 0.02    # low-pass coefficient on the online normal estimate (per step)


@dataclass
class Mode:
    """Scenario switches (report Sec. VII-B)."""
    admittance: bool = True
    saturation: bool = False
    virtual_fb: bool = True        # False -> scenario A: vibration driven by measured |f_e,n|
    force_track: bool = False      # extension: f_d = -f_des * n, reference guided along the surface
    normal: str = "fixed"          # 'fixed' | 'est' | 'oracle'


def unit(v, fallback):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.asarray(fallback, float)


class Controller:
    def __init__(self, P: Params, mode: Mode, p0):
        self.P, self.m = P, mode
        p0 = np.asarray(p0, float)
        self.p_raw = p0.copy()
        self.hist = deque([p0.copy()], maxlen=P.ma_window)
        self.p_h = p0.copy()
        self.pt, self.pt_v = np.zeros(3), np.zeros(3)      # admittance state (p_d - p_c) and its rate
        self.p_c = p0.copy()
        self.fe_f = np.zeros(3)                            # filtered contact force (robot on env)
        self.n_est = np.array([0.0, 0.0, 1.0])
        self.v_f = np.zeros(3)
        self.latched, self.s_bar = False, 0.0
        self.in_contact = self.touched = False

    # ------------------------------------------------------------------ pieces
    def _normal(self, n_true):
        if self.m.normal == "oracle" and n_true is not None:
            return n_true
        if self.m.normal == "est":
            return self.n_est
        return np.array([0.0, 0.0, 1.0])

    def _estimate_normal(self, vel):
        """Online normal from measured force with the friction component removed (Nasiri & Wang idea)."""
        P = self.P
        f_on_probe = -self.fe_f
        if np.linalg.norm(f_on_probe) < P.contact_thr:
            return
        self.v_f += 0.2 * (vel - self.v_f)
        sp = np.linalg.norm(self.v_f)
        if sp > 2e-3:
            vh = self.v_f / sp
            f_on_probe = f_on_probe - (f_on_probe @ vh) * vh
        n_raw = unit(f_on_probe, self.n_est)
        if n_raw @ self.n_est > 0:                         # ignore sign flips from noise
            self.n_est = unit((1 - P.lp_normal) * self.n_est + P.lp_normal * n_raw, self.n_est)

    # ------------------------------------------------------------------- step
    def step(self, u, fe_raw, tip_pos, tip_vel, n_true=None):
        """u: operator velocity command (3,) ; fe_raw: force robot exerts on env ; tip_*: measured probe state."""
        P, m = self.P, self.m
        self.fe_f += P.lp_force * (np.asarray(fe_raw) - self.fe_f)
        fe = self.fe_f
        if m.normal == "est":
            self._estimate_normal(tip_vel)
        n = self._normal(n_true)
        fn = max(0.0, -float(n @ fe))                      # pressing force along the normal
        self.in_contact = fn > P.contact_thr

        # [1] rate-to-position (tangential/normal split; guided normal in force-tracking mode)
        u = np.asarray(u, float)
        un, ut = float(n @ u), u - float(n @ u) * n
        self.p_raw = self.p_raw + ut * P.dt
        self.touched |= self.in_contact
        if m.force_track and self.touched:
            # extension: operator only steers tangentially; an outer integral loop on the force error moves
            # the normal reference (admittance alone reaches only f_des/KP past it and loses a falling surface)
            s_raw = float(n @ self.p_raw) - P.k_force * (P.f_des - fn) * P.dt
        else:
            s_raw = float(n @ self.p_raw) + un * P.dt
        self.p_raw = self.p_raw + n * (s_raw - float(n @ self.p_raw))
        # [2] moving average
        self.hist.append(self.p_raw.copy())
        self.p_h = np.mean(self.hist, axis=0)

        # [3] reference saturation along the normal (Eqs. 10-11)
        fd = -P.f_des * n if m.force_track else np.zeros(3)
        s_h = float(n @ self.p_h)
        s_d = s_h
        if m.saturation:
            Keq = P.KP * P.Ke / (P.KP + P.Ke)
            if not self.latched and fn >= P.f_th:
                self.latched = True
                s_e = float(n @ self.p_c)                  # rest estimate = last compliant command (paper)
                if P.rest_correction:                      # ...plus the penetration already in the spring
                    s_e += fn / P.Ke
                fd_n = float(-n @ fd)
                self.s_bar = s_e - (P.f_th - fd_n * P.Ke / (P.Ke + P.KP)) / Keq
            if self.latched:
                s_d = max(s_h, self.s_bar)
                if fn < 0.5 * P.f_th and s_h >= self.s_bar:
                    self.latched = False
        p_d = self.p_h + n * (s_d - s_h)

        # [4] admittance (Eq. 5, f_d = 0 unless force tracking) + Eq. 7
        if m.admittance:
            acc = (fe - fd - P.KD * self.pt_v - P.KP * self.pt) / P.Md
            self.pt_v = self.pt_v + acc * P.dt
            self.pt = self.pt + self.pt_v * P.dt
            self.p_c = p_d - self.pt
        else:
            self.pt, self.pt_v = np.zeros(3), np.zeros(3)
            self.p_c = p_d.copy()

        # [8] virtualized feedback from the UNsaturated error (Eqs. 12-14) and [9] cue mapping (Eq. 16)
        pt_h = self.p_h - self.p_c
        # + fd*Kh/KP removes the f_des/KP offset the admittance holds in free space (no phantom cue before contact)
        f_h = P.Kh * pt_h + P.Dh * self.pt_v + fd * P.Kh / P.KP if m.admittance else np.zeros(3)
        f_hn = abs(float(n @ f_h))
        a_vib_src = f_hn if m.virtual_fb else fn
        a_vib = float(np.clip(a_vib_src / P.f_vib_max, 0, 1))
        a_trig = float(np.clip(max(0.0, fn - P.f_warn) / (P.f_stop - P.f_warn), 0, 1))
        return dict(p_raw=self.p_raw, p_h=self.p_h, p_d=p_d, p_c=self.p_c, pt=self.pt, fe_f=fe.copy(),
                    f_h=f_h, f_hn=f_hn, fn=fn, n=n, n_est=self.n_est.copy(), a_vib=a_vib, a_trig=a_trig,
                    saturated=self.latched, in_contact=self.in_contact, s_bar=self.s_bar if self.latched else np.nan)
