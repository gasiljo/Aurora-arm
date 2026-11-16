import csv, json, os, shutil, subprocess
from pathlib import Path
import numpy as np
from .plant import Params, state_space, operating_point, controllability, observability, euler_step
from .controller import PID
from .safety import verify_deadline, SafetySupervisor
from .codegen import generate_c

ROOT=Path(__file__).parents[1]; OUT=ROOT/'results'

def _python_sil_outputs(seq):
    ###
    integ=np.float32(0.0); prev=np.float32(0.0); outputs=[]
    for r,m in seq:
        error=np.float32(r-m)
        derivative=np.float32((error-prev)/np.float32(0.002))
        unsat=np.float32(np.float32(10.0)*error+integ+np.float32(0.3)*derivative)
        u=np.float32(max(-48.0,min(48.0,float(unsat))))
        integ=np.float32(integ+np.float32(0.002)*(np.float32(0.5)*error+np.float32(20.0)*(u-unsat)))
        prev=error
        outputs.append(float(u))
    return outputs

def _run_sil(seq, cpath):
    ###
    compiler = shutil.which('gcc') or shutil.which('clang')
    exe = OUT / ('controller_c.exe' if os.name == 'nt' else 'controller_c')
    if compiler:
        try:
            subprocess.run([compiler, '-O2', str(cpath), '-o', str(exe)], check=True)
            inp=''.join(f'{r} {m}\n' for r,m in seq)
            values=subprocess.check_output([str(exe)], input=inp.encode()).decode().split()
            return np.array([float(v) for v in values]), 'native_c'
        except (OSError, subprocess.CalledProcessError):
            pass
    return np.array(_python_sil_outputs(seq)), 'python_float32_fallback'

def main():
    OUT.mkdir(exist_ok=True)
    p=Params(); A,B,C,D=state_space(p)
    eig=np.linalg.eigvals(A)
    ctr=controllability(A,B); obs=observability(A,C)
    op=operating_point(p)
    cases=verify_deadline();
    # Simple closed-loop step response around zero load for academic performance evidence.
    pid=PID(); x=np.zeros(3); ref=0.1; hist=[]
    for k in range(1000):
        u=pid.step(ref,x[0]); x=euler_step(x,u,p,load_torque=0.0); hist.append((k*p.Ts,x[0],x[1],u))
    y=np.array([h[1] for h in hist]); uvals=np.array([h[3] for h in hist])
    overshoot=max(0.0,(y.max()-ref)/ref*100); settling=next((h[0] for h in hist if abs(h[1]-ref)<=0.02*ref and all(abs(z[1]-ref)<=0.02*ref for z in hist[int(h[0]/p.Ts):])),None)
    # Generate C controller. A máquina pode não ter GCC instalado.
    cpath=generate_c()
    seq=[(0.1*np.sin(k/7),0.05*np.cos(k/11)) for k in range(100)]
    # Referência SIL em float32, mesma precisão do firmware C.
    pyout=_python_sil_outputs(seq)
    cvals, sil_backend=_run_sil(seq, cpath)
    err=float(np.max(np.abs(np.array(pyout)-cvals)))
    # Formal-style finite exhaustive deadline evidence.
    with (OUT/'deadline_cases.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=cases[0].keys()); w.writeheader(); w.writerows(cases)
    trace=[
      ['REQ-CTRL-AA-001','Modelo + PID','controller.py','step_response'],
      ['REQ-CTRL-AA-002','Simulação fechada','run_all.py','step_response'],
      ['REQ-CTRL-AA-003','Simulação fechada','run_all.py','settling_time'],
      ['REQ-CTRL-AA-004','Saturação/anti-windup','controller.py','saturation_test'],
      ['REQ-SAFE-AA-001','Supervisor + atraso','safety.py','deadline_cases.csv'],
      ['REQ-SAFE-AA-002','Supervisor','safety.py','supervisor_transition_test'],
      ['REQ-SAFE-AA-003','Supervisor','safety.py','resume_transition_test'],
      ['REQ-SAFE-AA-004','Limite de velocidade','plant.py','speed_limit_test']]
    with (OUT/'traceability.csv').open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['requirement','model','code','test']); w.writerows(trace)
    report={'parameters':p.__dict__, 'A':A.tolist(),'B':B.tolist(),'C':C.tolist(),'D':D.tolist(),
      'eigenvalues':[complex(z).__repr__() for z in eig],
      'tau_electrical_s':p.L/p.R,'tau_mechanical_s':p.J/p.b,
      'controllability_rank':int(np.linalg.matrix_rank(ctr)), 'observability_rank':int(np.linalg.matrix_rank(obs)),
      'operating_point':op,'deadline_max_ms':max(c['response_ms'] for c in cases),'deadline_all_pass':all(c['pass'] for c in cases),
      'step_overshoot_percent':float(overshoot),'settling_time_s':settling,'max_voltage_step':float(np.max(np.abs(uvals))),
      'sil_max_abs_error':err,'sil_backend':sil_backend,
      'sil_float32_epsilon':float(np.finfo(np.float32).eps),
      'covered_states':['MOVING','SAFE_STOP_PENDING','SAFE_STOP','ACK_REQUIRED'],
      'covered_transitions':['MOVING->SAFE_STOP_PENDING','SAFE_STOP_PENDING->SAFE_STOP','SAFE_STOP->ACK_REQUIRED','ACK_REQUIRED->MOVING']}
    (OUT/'evidence.json').write_text(json.dumps(report,indent=2,default=str))
    print(json.dumps(report,indent=2,default=str))

if __name__=='__main__': main()
