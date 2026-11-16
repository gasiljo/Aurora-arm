from dataclasses import dataclass

@dataclass
class SafetySupervisor:
    state: str = "MOVING"
    transitions: set = None
    def __post_init__(self):
        self.transitions = set()
    def proximity_detected(self):
        if self.state == "MOVING":
            self.state = "SAFE_STOP_PENDING"; self.transitions.add("MOVING->SAFE_STOP_PENDING")
        return self.state
    def stop_command_applied(self):
        if self.state == "SAFE_STOP_PENDING":
            self.state = "SAFE_STOP"; self.transitions.add("SAFE_STOP_PENDING->SAFE_STOP")
        return self.state
    def operator_ack(self):
        if self.state == "SAFE_STOP":
            self.state = "ACK_REQUIRED"; self.transitions.add("SAFE_STOP->ACK_REQUIRED")
        return self.state
    def resume(self):
        if self.state == "ACK_REQUIRED":
            self.state = "MOVING"; self.transitions.add("ACK_REQUIRED->MOVING")
        return self.state


def verify_deadline(sensor_delays_ms=(0,5,10,15,20), jitter_ms=(0,2), control_ms=2, deadline_ms=100):
    cases = []
    for sensor in sensor_delays_ms:
        for jitter in jitter_ms:
            response = sensor + jitter + control_ms
            cases.append({"sensor_ms": sensor, "jitter_ms": jitter,
                          "control_ms": control_ms, "response_ms": response,
                          "pass": response <= deadline_ms})
    return cases
