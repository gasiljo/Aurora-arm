import numpy as np
from dataclasses import dataclass

@dataclass(frozen=True)
class Params:
    R: float = 0.8
    L: float = 1.2e-3
    Kt: float = 0.085
    Ke: float = 0.085
    J: float = 4.2e-4
    b: float = 1.5e-4
    N: float = 100.0
    Vmax: float = 48.0
    Imax: float = 8.0
    Ts: float = 2e-3
    tau_load: float = 0.25
    omega_critical: float = 450.0


def state_space(p: Params = Params()):
    # x = [theta_motor(rad), omega_motor(rad/s), current(A)], u = armature voltage
    A = np.array([[0.0, 1.0, 0.0],
                  [0.0, -p.b/p.J, p.Kt/p.J],
                  [0.0, -p.Ke/p.L, -p.R/p.L]])
    B = np.array([[0.0], [0.0], [1.0/p.L]])
    C = np.array([[1.0, 0.0, 0.0]])
    D = np.zeros((1, 1))
    return A, B, C, D


def operating_point(p: Params = Params()):
    i = (p.tau_load + p.b*p.omega_critical) / p.Kt
    v = p.R*i + p.Ke*p.omega_critical
    return {"current_A": i, "voltage_V": v,
            "current_margin_A": p.Imax-i, "voltage_margin_V": p.Vmax-v}


def controllability(A, B):
    return np.column_stack((B, A@B, A@A@B))


def observability(A, C):
    return np.vstack((C, C@A, C@A@A))


def euler_step(x, u, p: Params = Params(), load_torque=None):
    A, B, _, _ = state_space(p)
    tau = p.tau_load if load_torque is None else load_torque
    disturbance = np.array([0.0, -tau/p.J, 0.0])
    return x + p.Ts * (A @ x + B[:, 0]*u + disturbance)
