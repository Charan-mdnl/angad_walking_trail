#!/usr/bin/env python3
"""
Angad Humanoid Robot Walking Simulator
=======================================
A complete walking controller that demonstrates:
1. Stable standing with LIPM-based balance
2. Crouching to prepare for walking
3. Dynamic walking with forward motion
4. Balance recovery during perturbations

Run: python3 robot_walking_simulator.py
"""

import numpy as np
import time
from time import sleep
import mujoco
import mujoco.viewer

# ===== CONFIGURATION =====
RUNNING_MODE = "walk"  # Options: "stand", "crouch", "walk"
WALKING_SPEED = 0.3    # Forward velocity (m/s)
WALKING_STEPS = 10     # Number of walking steps to perform
SIMULATION_TIME = 30   # Total simulation time (seconds)

# ===== LOAD ROBOT MODEL =====
print("[INFO] Loading MuJoCo robot model...")
m = mujoco.MjModel.from_xml_path("XP_robot_with_actuators.xml")
d = mujoco.MjData(m)
nu = m.nu

# Extract actuator information
gear = np.array([m.actuator_gear[i][0] for i in range(nu)])
act_names = [m.actuator(i).name for i in range(nu)]

print(f"[INFO] Robot loaded with {nu} actuators")
print(f"[INFO] Actuators: {', '.join(act_names[:5])}... (showing first 5)")

# ===== JOINT CONTROL PARAMETERS =====
base_target = np.zeros(nu)
kp_torque = np.zeros(nu)
kd_torque = np.zeros(nu)

for i, name in enumerate(act_names):
    if "knee" in name:
        base_target[i] = 0.1
        kp_torque[i] = 1500
        kd_torque[i] = 50
    elif "hip" in name:
        base_target[i] = 0.03 if "pitch" in name else 0.0
        kp_torque[i] = 1000
        kd_torque[i] = 50
    elif "ankle" in name:
        base_target[i] = 0.02 if "pitch" in name else 0.0
        kp_torque[i] = 500
        kd_torque[i] = 25
    elif "thigh" in name:
        base_target[i] = 0.0
        kp_torque[i] = 500
        kd_torque[i] = 25
    elif "torso" in name:
        base_target[i] = 0.0
        kp_torque[i] = 1000
        kd_torque[i] = 50
    else:  # Arms
        base_target[i] = 0.0
        kp_torque[i] = 100
        kd_torque[i] = 5

current_target = base_target.copy()

# ===== BALANCE PARAMETERS =====
BAL_PITCH_KP = 2000.0
BAL_PITCH_KD = 300.0
BAL_ROLL_KP = 1500.0
BAL_ROLL_KD = 200.0
alpha = 0.1
fp, fr = 0.0, 0.0

# ===== CROUCH PARAMETERS =====
CROUCH_KNEE = -0.30
CROUCH_HIP = +0.15
CROUCH_ANKLE = +0.15

# ===== WALKING PARAMETERS =====
LEAN_ANGLE = 0.035
KNEE_LIFT = 0.10
HIP_SWING = 0.03
SHIFT_TIME = 2.0
SWING_TIME = 1.5
SETTLE_TIME = 1.0

# ===== STATE MANAGEMENT =====
class WalkingState:
    STANDING = 0
    CROUCHING = 1
    WALKING = 2

state = WalkingState.STANDING
state_start_time = 0.0
step_count = 0
walk_offset = np.zeros(nu)
crouch_offset = np.zeros(nu)

# ===== JOINT INDICES =====
def get_joint_indices(pattern):
    return [i for i, n in enumerate(act_names) if pattern in n]

L_knee = get_joint_indices("knee_l")
R_knee = get_joint_indices("knee_r")
L_hip_pitch = get_joint_indices("hip_pitch_l")
R_hip_pitch = get_joint_indices("hip_pitch_r")
L_ankle_pitch = get_joint_indices("ankle_pitch_l")
R_ankle_pitch = get_joint_indices("ankle_pitch_r")
L_hip_roll = get_joint_indices("hip_roll_l")
R_hip_roll = get_joint_indices("hip_roll_r")
L_ankle_roll = get_joint_indices("ankle_roll_l")
R_ankle_roll = get_joint_indices("ankle_roll_r")

# ===== HELPER FUNCTIONS =====
def smooth_step(t, duration):
    """Smooth step function using cosine interpolation"""
    return 0.5 * (1.0 - np.cos(np.pi * np.clip(t / duration, 0, 1)))

def extract_euler_angles(quat):
    """Extract pitch and roll from quaternion"""
    q = quat  # [qw, qx, qy, qz]
    pitch = 2.0 * (q[0] * q[2] - q[3] * q[1])
    roll = 2.0 * (q[0] * q[1] + q[2] * q[3])
    return pitch, roll

# ===== MAIN CONTROLLER =====
def controller(m, d):
    global fp, fr, state, state_start_time, step_count, walk_offset, crouch_offset
    
    qpos = d.qpos[7:]
    qvel = d.qvel[6:]
    quat = d.qpos[3:7]
    
    # Calculate current tilt
    pitch, roll = extract_euler_angles(quat)
    
    # Calculate balance torques
    raw_p = -(BAL_PITCH_KP * pitch + BAL_PITCH_KD * d.qvel[4])
    raw_r = -(BAL_ROLL_KP * roll + BAL_ROLL_KD * d.qvel[3])
    
    fp += alpha * (raw_p - fp)
    fr += alpha * (raw_r - fr)
    
    # Admittance control for compliance
    tilt_mag = abs(pitch) + abs(roll)
    compliance = 1.0
    if tilt_mag > 0.03:
        compliance = max(0.4, 1.0 - 5.0 * (tilt_mag - 0.03))
    
    # Time since state change
    elapsed_state = d.time - state_start_time
    
    # ===== STATE MACHINE =====
    if RUNNING_MODE == "stand":
        state = WalkingState.STANDING
        target = base_target.copy()
        
    elif RUNNING_MODE == "crouch":
        if state == WalkingState.STANDING and elapsed_state > 2.0:
            state = WalkingState.CROUCHING
            state_start_time = d.time
        elif state == WalkingState.CROUCHING:
            progress = smooth_step(elapsed_state, 3.0)
            target = base_target.copy()
            for idx in L_knee + R_knee:
                target[idx] = base_target[idx] + progress * (CROUCH_KNEE - base_target[idx])
            for idx in L_hip_pitch + R_hip_pitch:
                target[idx] = base_target[idx] + progress * (CROUCH_HIP - base_target[idx])
            for idx in L_ankle_pitch + R_ankle_pitch:
                target[idx] = base_target[idx] + progress * (CROUCH_ANKLE - base_target[idx])
        else:
            target = base_target.copy()
            
    elif RUNNING_MODE == "walk":
        # Phase 1: Stand (0-2s)
        if elapsed_state < 2.0:
            state = WalkingState.STANDING
            target = base_target.copy()
            
        # Phase 2: Crouch (2-5s)
        elif elapsed_state < 5.0:
            state = WalkingState.CROUCHING
            progress = smooth_step(elapsed_state - 2.0, 3.0)
            target = base_target.copy()
            for idx in L_knee + R_knee:
                target[idx] = base_target[idx] + progress * (CROUCH_KNEE - base_target[idx])
            for idx in L_hip_pitch + R_hip_pitch:
                target[idx] = base_target[idx] + progress * (CROUCH_HIP - base_target[idx])
            for idx in L_ankle_pitch + R_ankle_pitch:
                target[idx] = base_target[idx] + progress * (CROUCH_ANKLE - base_target[idx])
                
        # Phase 3: Walk (5s+)
        else:
            state = WalkingState.WALKING
            time_in_walk = elapsed_state - 5.0
            
            # Stop after a certain number of steps
            if WALKING_STEPS > 0:
                total_walk_time = WALKING_STEPS * (SHIFT_TIME + SWING_TIME + SETTLE_TIME)
                if time_in_walk > total_walk_time:
                    target = base_target.copy()
                else:
                    # Crouch base + walking pattern
                    target = base_target.copy()
                    for idx in L_knee + R_knee:
                        target[idx] = CROUCH_KNEE
                    for idx in L_hip_pitch + R_hip_pitch:
                        target[idx] = CROUCH_HIP
                    for idx in L_ankle_pitch + R_ankle_pitch:
                        target[idx] = CROUCH_ANKLE
                    
                    # Walking oscillation
                    cycle_time = SHIFT_TIME + SWING_TIME + SETTLE_TIME
                    phase_in_cycle = time_in_walk % cycle_time
                    
                    if phase_in_cycle < SHIFT_TIME:
                        # Weight shift to right
                        progress = smooth_step(phase_in_cycle, SHIFT_TIME)
                        
                        # Lean right
                        for idx in L_hip_roll:
                            target[idx] = LEAN_ANGLE * progress
                        
                        # Swing left leg forward
                        for idx in L_hip_pitch:
                            target[idx] += HIP_SWING * progress
                        for idx in L_knee:
                            target[idx] += KNEE_LIFT * progress
                            
                    elif phase_in_cycle < SHIFT_TIME + SWING_TIME:
                        # Swing phase
                        progress = smooth_step(phase_in_cycle - SHIFT_TIME, SWING_TIME)
                        
                        for idx in L_hip_roll:
                            target[idx] = LEAN_ANGLE * (1.0 - progress)
                        for idx in R_hip_roll:
                            target[idx] = -LEAN_ANGLE * progress
                        
                        # Right leg swings
                        for idx in R_hip_pitch:
                            target[idx] += HIP_SWING * progress
                        for idx in R_knee:
                            target[idx] += KNEE_LIFT * progress
                            
                        # Left leg settles
                        for idx in L_knee:
                            target[idx] += KNEE_LIFT * (1.0 - progress)
                    else:
                        # Settle phase
                        for idx in L_knee + R_knee:
                            target[idx] = CROUCH_KNEE
                        for idx in L_hip_roll + R_hip_roll:
                            target[idx] = 0.0
            else:
                target = base_target.copy()
    else:
        target = base_target.copy()
    
    # ===== COMPUTE TORQUES =====
    torque = (kp_torque * compliance) * (target - qpos[:nu]) - kd_torque * qvel[:nu]
    
    # Apply balance corrections
    for i, name in enumerate(act_names):
        if "ankle_pitch" in name:
            torque[i] += 0.4 * fp
        elif "hip_pitch" in name:
            torque[i] += 0.6 * fp
        elif "arm_pitch" in name:
            torque[i] -= 10.0 * d.qvel[4]
            
        if "ankle_roll" in name:
            torque[i] += 0.4 * fr
        elif "hip_roll" in name:
            torque[i] += 0.6 * fr
        elif "arm_roll" in name:
            torque[i] -= 10.0 * d.qvel[3]
    
    # Normalize by gear ratio
    ctrl = torque / gear
    d.ctrl[:] = np.clip(ctrl, -1.0, 1.0)

# Set the controller callback
mujoco.set_mjcb_control(controller)

# ===== SIMULATION LOOP =====
print(f"\n[INFO] Starting {RUNNING_MODE.upper()} simulation...")
print(f"[INFO] Running mode: {RUNNING_MODE}")
print(f"[INFO] Max simulation time: {SIMULATION_TIME}s\n")

with mujoco.viewer.launch_passive(m, d) as viewer:
    with viewer.lock():
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    
    step_count = 0
    log_interval = 2500
    start_time = time.time()
    
    while viewer.is_running() and d.time < SIMULATION_TIME:
        sim_start = d.time
        wall_start = time.time()
        
        while (d.time - sim_start) < 1/60:
            mujoco.mj_step(m, d)
            step_count += 1
        
        # Log information
        if step_count % log_interval == 0:
            q = d.qpos[3:7]
            pitch_deg = np.degrees(2.0 * (q[0] * q[2] - q[3] * q[1]))
            roll_deg = np.degrees(2.0 * (q[0] * q[1] + q[2] * q[3]))
            jitter = np.linalg.norm(d.qvel[6:18])
            max_ctrl = np.max(np.abs(d.ctrl))
            x_pos = d.qpos[0]
            
            print(f"[{d.time:6.1f}s] X={x_pos:+.3f}m Z={d.qpos[2]:.3f}m " +
                  f"Pitch={pitch_deg:+.1f}° Roll={roll_deg:+.1f}° " +
                  f"Jitter={jitter:.2f} CtrlMax={max_ctrl:.2f}")
        
        viewer.sync()
        elapsed = time.time() - wall_start
        if elapsed < 1/60:
            sleep(1/60 - elapsed)

print(f"\n[INFO] Simulation completed!")
print(f"[INFO] Total simulation time: {d.time:.1f}s")
print(f"[INFO] Total steps: {step_count}")
