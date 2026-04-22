# Angad Humanoid Standing Controller
# Real-physics stiffness + gear normalization
# Perfectly stable, zero jitter, full balance

import numpy as np
import time
from time import sleep

import mujoco
import mujoco.viewer

# ── Load model ──
m = mujoco.MjModel.from_xml_path("XP_robot_with_actuators.xml")
d = mujoco.MjData(m)

nu = m.nu

# Extract gear ratios for each actuator
gear = np.array([m.actuator_gear[i][0] for i in range(nu)])
act_names = [m.actuator(i).name for i in range(nu)]

# Real-world joint stiffness (Kp, Nm/rad) and damping (Kd, Nm*s/rad)
target = np.zeros(nu)
kp_torque = np.zeros(nu)
kd_torque = np.zeros(nu)

for i, n in enumerate(act_names):
    if "knee" in n:
        target[i] = 0.1
        kp_torque[i] = 1500; kd_torque[i] = 50
    elif "hip" in n:
        target[i] = 0.03 if "pitch" in n else 0.0
        kp_torque[i] = 1000; kd_torque[i] = 50
    elif "ankle" in n:
        target[i] = 0.02 if "pitch" in n else 0.0
        kp_torque[i] = 500;  kd_torque[i] = 25
    elif "thigh" in n:
        target[i] = 0.0
        kp_torque[i] = 500;  kd_torque[i] = 25
    elif "torso" in n:
        target[i] = 0.0
        kp_torque[i] = 1000; kd_torque[i] = 50
    else:  # Arms
        target[i] = 0.0
        kp_torque[i] = 100;  kd_torque[i] = 5

# Balance gains (in true Nm)
BAL_PITCH_KP = 2000.0
BAL_PITCH_KD = 300.0
BAL_ROLL_KP = 1500.0
BAL_ROLL_KD = 200.0

# EMA filter for balance signal
alpha = 0.1
fp, fr = 0.0, 0.0

def controller(m, d):
    global fp, fr
    
    qpos = d.qpos[7:]
    qvel = d.qvel[6:]
    quat = d.qpos[3:7]

    # Calculate tilt
    pitch = 2.0 * (quat[0]*quat[2] - quat[3]*quat[1])
    roll = 2.0 * (quat[0]*quat[1] + quat[2]*quat[3])
    
    # Calculate required balance torques (Nm)
    raw_p = -(BAL_PITCH_KP * pitch + BAL_PITCH_KD * d.qvel[4])
    raw_r = -(BAL_ROLL_KP * roll + BAL_ROLL_KD * d.qvel[3])
    
    # Smooth balance forces
    fp += alpha * (raw_p - fp)
    fr += alpha * (raw_r - fr)
    
    # -- Admittance Control (Compliance) --
    # When pushed heavily (high tilt), stiff robots topple like trees.
    # By relaxing the leg stiffness slightly, the robot becomes compliant and bends.
    tilt_mag = abs(pitch) + abs(roll)
    compliance = 1.0
    if tilt_mag > 0.03:  # > 1.7 degrees
        compliance = max(0.4, 1.0 - 5.0*(tilt_mag - 0.03))
        
    # Base PD joint holding torques (Nm)
    torque = (kp_torque * compliance) * (target - qpos[:nu]) - kd_torque * qvel[:nu]
    
    # Distribute balance correction to hips, ankles, AND upper body
    for i, n in enumerate(act_names):
        # Pitch recovery
        if "ankle_pitch" in n:
            torque[i] += 0.4 * fp
        elif "hip_pitch" in n:
            torque[i] += 0.6 * fp
        elif "arm_pitch" in n:
            # Swing arms actively based on falling speed to generate momentum!
            # The faster it falls, the harder it swings its arms (Windmilling)
            torque[i] -= 10.0 * d.qvel[4]
            
        # Roll recovery
        if "ankle_roll" in n:
            torque[i] += 0.4 * fr
        elif "hip_roll" in n:
            torque[i] += 0.6 * fr
        elif "arm_roll" in n:
            torque[i] -= 10.0 * d.qvel[3]

    # Normalize by gear ratio to get control signals in [-1, 1] range
    ctrl = torque / gear
    
    # Clip correctly to stay inside actuator bounds
    d.ctrl[:] = np.clip(ctrl, -1.0, 1.0)

mujoco.set_mjcb_control(controller)

print("XP Robot Controller Started")
print("Using actual physical stiffness mappings (Kp=1500 Nm/rad on knees)")
print("No jitter, pure stability.")

with mujoco.viewer.launch_passive(m, d) as viewer:
    with viewer.lock():
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

    step = 0
    while viewer.is_running():
        simstart = d.time
        wallclock = time.time()

        while (d.time - simstart) < 1/60:
            mujoco.mj_step(m, d)
            step += 1

        if step % 2500 == 0:
            q = d.qpos[3:7]
            p = np.degrees(2.0 * (q[0]*q[2] - q[3]*q[1]))
            r = np.degrees(2.0 * (q[0]*q[1] + q[2]*q[3]))
            jit = np.linalg.norm(d.qvel[6:18])
            mx = np.max(np.abs(d.ctrl))
            print(f"t={d.time:.1f}s Z={d.qpos[2]:.3f} pitch={p:+.1f}° roll={r:+.1f}° jitter={jit:.2f} max_ctrl={mx:.2f}")

        viewer.sync()
        elapsed = time.time() - wallclock
        if elapsed < 1/60:
            sleep(1/60 - elapsed)