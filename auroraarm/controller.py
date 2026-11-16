from dataclasses import dataclass

@dataclass
class PID:
    kp: float = 10.0
    ki: float = 0.5
    kd: float = 0.3
    Ts: float = 2e-3
    umin: float = -48.0
    umax: float = 48.0
    kaw: float = 20.0
    integral: float = 0.0
    previous_error: float = 0.0

    def step(self, reference, measurement):
        error = reference - measurement
        derivative = (error - self.previous_error) / self.Ts
        unsaturated = self.kp*error + self.integral + self.kd*derivative
        u = max(self.umin, min(self.umax, unsaturated))
        self.integral += self.Ts * (self.ki*error + self.kaw*(u-unsaturated))
        self.previous_error = error
        return u

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
