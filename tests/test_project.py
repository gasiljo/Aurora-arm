import numpy as np
from auroraarm.plant import Params,state_space,operating_point,controllability,observability
from auroraarm.safety import SafetySupervisor,verify_deadline

def test_controlability_observability():
    A,B,C,D=state_space(); assert np.linalg.matrix_rank(controllability(A,B))==3; assert np.linalg.matrix_rank(observability(A,C))==3

def test_operating_point_within_limits():
    op=operating_point(); assert op['current_A']<Params().Imax; assert op['voltage_V']<Params().Vmax

def test_deadline_exhaustive(): assert all(x['pass'] for x in verify_deadline())

def test_supervisor_all_transitions():
    s=SafetySupervisor(); s.proximity_detected(); s.stop_command_applied(); s.operator_ack(); s.resume()
    assert s.state=='MOVING' and len(s.transitions)==4
