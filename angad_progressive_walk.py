#!/usr/bin/env python3
"""
Angad Walking v6 — Crouched Stance (Bent Knees)
─────────────────────────────────────────────────
User feedback: Robot cannot shift weight because legs are straight.
Humanoids must bend their knees (crouch) BEFORE walking to lower COM
and avoid kinematic singularities (locked knees).

New Flow:
1. Straight Stand (0-2s)
2. Crouch (2-5s) -> Bend knees to -17°, pitch hips/ankles to keep torso upright.
3. Walk (5s+) -> Easy weight shift since legs are bent and springy!
"""

import numpy as np
import time
from time import sleep
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path("XP_robot_with_actuators.xml")
d = mujoco.MjData(m)
nu = m.nu

gear = np.array([m.actuator_gear[i][0] for i in range(nu)])
act_names = [m.actuator(i).name for i in range(nu)]

# ── Base (Locked) Standing Targets ──
base_target = np.zeros(nu)
kp_torque = np.zeros(nu)
kd_torque = np.zeros(nu)
for i, n in enumerate(act_names):
    if "knee" in n:
        base_target[i] = 0.1  # Pushes against 0.0 limit (locked straight)
        kp_torque[i] = 1500; kd_torque[i] = 50
    elif "hip" in n:
        base_target[i] = 0.03 if "pitch" in n else 0.0
        kp_torque[i] = 1000; kd_torque[i] = 50
    elif "ankle" in n:
        base_target[i] = 0.02 if "pitch" in n else 0.0
        kp_torque[i] = 500; kd_torque[i] = 25
    elif "thigh" in n:
        base_target[i] = 0.0; kp_torque[i] = 500; kd_torque[i] = 25
    elif "torso" in n:
        base_target[i] = 0.0; kp_torque[i] = 1000; kd_torque[i] = 50
    else:
        base_target[i] = 0.0; kp_torque[i] = 100; kd_torque[i] = 5

current_target = base_target.copy()

# Balance Gains (Keep full strength)
BAL_PITCH_KP = 2000.0
BAL_PITCH_KD = 300.0
BAL_ROLL_KP  = 1500.0
BAL_ROLL_KD  = 200.0
alpha = 0.1
fp, fr = 0.0, 0.0

# ── Crouch & Walking Config ──
TIME_STRAIGHT = 2.0  # Stand straight initially
TIME_CROUCH   = 5.0  # Finish crouching at 5s

# The Athletic Crouch (Bent knees)
CROUCH_KNEE  = -0.30   # Bend knees backward 17°
CROUCH_HIP   = +0.15   # Pitch hip forward 8.5° to keep torso upright
CROUCH_ANKLE = +0.15   # Pitch ankle forward 8.5° to keep foot flat

# Walking parameters
LEAN_ANGLE   = 0.035   # ~2.0° gentle lean (easier now since legs are bent)
KNEE_LIFT    = 0.10    # Extra knee bend for foot lift
HIP_SWING    = 0.03    # Forward swing
SHIFT_TIME   = 2.0
SWING_TIME   = 1.5
SETTLE_TIME  = 1.0

walk_phase = -2  # -2=straight, -1=crouching, 0+=walking
phase_start = 0.0
step_count = 0
desired_roll = 0.0
walk_offset = np.zeros(nu)
crouch_offset = np.zeros(nu)
walk_paused = False

# Joint indices
L_knee = [i for i, n in enumerate(act_names) if "knee" in n and "_l" in n]
R_knee = [i for i, n in enumerate(act_names) if "knee" in n and "_r" in n]
L_hip_pitch = [i for i, n in enumerate(act_names) if "hip_pitch" in n and "_l" in n]
R_hip_pitch = [i for i, n in enumerate(act_names) if "hip_pitch" in n and "_r" in n]
L_ankle_pitch = [i for i, n in enumerate(act_names) if "ankle_pitch" in n and "_l" in n]
R_ankle_pitch = [i for i, n in enumerate(act_names) if "ankle_pitch" in n and "_r" in n]

def smooth(t, dur):
    return 0.5 * (1.0 - np.cos(np.pi * np.clip(t/dur, 0, 1)))

def controller(m_, d_):
    global fp, fr, walk_phase, phase_start, step_count
    global desired_roll, walk_offset, crouch_offset, walk_paused

    qpos = d_.qpos[7:]
    qvel = d_.qvel[6:]
    quat = d_.qpos[3:7]

    pitch = 2.0 * (quat[0]*quat[2] - quat[3]*quat[1])
    roll  = 2.0 * (quat[0]*quat[1] + quat[2]*quat[3])

    # ──── Balance ────
    raw_p = -(BAL_PITCH_KP * (pitch) + BAL_PITCH_KD * d_.qvel[4])
    raw_r = -(BAL_ROLL_KP * (roll - desired_roll) + BAL_ROLL_KD * d_.qvel[3])
    fp += alpha * (raw_p - fp)
    fr += alpha * (raw_r - fr)

    tilt_mag = abs(pitch) + abs(roll - desired_roll)
    compliance = max(0.4, 1.0 - 5.0*(tilt_mag - 0.03)) if tilt_mag > 0.03 else 1.0

    # ──── Safety Pause ────
    if walk_phase >= 0 and (abs(roll - desired_roll) > 0.15 or abs(pitch) > 0.15):
        if not walk_paused:
            print(f"  ⚠ SAFETY PAUSE (roll={np.degrees(roll):+.1f}°, pitch={np.degrees(pitch):+.1f}°)")
            walk_paused = True
        desired_roll *= 0.99
        walk_offset[:] *= 0.99
    else:
        walk_paused = False

    # ──── Phase Logic ────
    t = d_.time
    
    # 1. Straight -> Crouching
    if t >= TIME_STRAIGHT and walk_phase == -2:
        walk_phase = -1
        print("\n── CROUCHING (Bending knees) ──")
        
    if walk_phase == -1:
        s = smooth(t - TIME_STRAIGHT, TIME_CROUCH - TIME_STRAIGHT)
        for j in L_knee + R_knee: crouch_offset[j] = CROUCH_KNEE * s
        for j in L_hip_pitch + R_hip_pitch: crouch_offset[j] = CROUCH_HIP * s
        for j in L_ankle_pitch + R_ankle_pitch: crouch_offset[j] = CROUCH_ANKLE * s
        
        if t >= TIME_CROUCH:
            walk_phase = 0
            phase_start = t
            print(f"── CROUCH FINISHED. Z={d_.qpos[2]:.3f} ──")
            print(f"── WALKING (lean={np.degrees(LEAN_ANGLE):.1f}°) ──")

    # 2. Walking (from crouch)
    if walk_phase >= 0 and not walk_paused:
        pt = t - phase_start
        walk_offset[:] = 0.0

        if walk_phase == 0:  # Shift LEFT
            s = smooth(pt, SHIFT_TIME)
            desired_roll = LEAN_ANGLE * s
            if pt >= SHIFT_TIME:
                walk_phase = 1; phase_start = t
                print(f"[{step_count}] Lift RIGHT")

        elif walk_phase == 1:  # Lift RIGHT
            desired_roll = LEAN_ANGLE
            s = smooth(pt, SWING_TIME)
            for j in R_knee: walk_offset[j] = -KNEE_LIFT * s
            for j in R_hip_pitch: walk_offset[j] = HIP_SWING * s
            if pt >= SWING_TIME:
                walk_phase = 2; phase_start = t
                print(f"[{step_count}] Lower RIGHT")

        elif walk_phase == 2:  # Lower RIGHT
            desired_roll = LEAN_ANGLE
            s = smooth(pt, SWING_TIME * 0.5)
            for j in R_knee: walk_offset[j] = -KNEE_LIFT * (1-s)
            for j in R_hip_pitch: walk_offset[j] = HIP_SWING * (1-s)
            if pt >= SWING_TIME * 0.5:
                step_count += 1
                walk_phase = 3; phase_start = t
                print(f"[{step_count}] Settle...")

        elif walk_phase == 3:  # SETTLE
            s = smooth(pt, SETTLE_TIME)
            desired_roll = LEAN_ANGLE * (1 - s)
            if pt >= SETTLE_TIME:
                walk_phase = 4; phase_start = t
                print(f"[{step_count}] Shift RIGHT")

        elif walk_phase == 4:  # Shift RIGHT
            s = smooth(pt, SHIFT_TIME)
            desired_roll = -LEAN_ANGLE * s
            if pt >= SHIFT_TIME:
                walk_phase = 5; phase_start = t
                print(f"[{step_count}] Lift LEFT")

        elif walk_phase == 5:  # Lift LEFT
            desired_roll = -LEAN_ANGLE
            s = smooth(pt, SWING_TIME)
            for j in L_knee: walk_offset[j] = -KNEE_LIFT * s
            for j in L_hip_pitch: walk_offset[j] = HIP_SWING * s
            if pt >= SWING_TIME:
                walk_phase = 6; phase_start = t
                print(f"[{step_count}] Lower LEFT")

        elif walk_phase == 6:  # Lower LEFT
            desired_roll = -LEAN_ANGLE
            s = smooth(pt, SWING_TIME * 0.5)
            for j in L_knee: walk_offset[j] = -KNEE_LIFT * (1-s)
            for j in L_hip_pitch: walk_offset[j] = HIP_SWING * (1-s)
            if pt >= SWING_TIME * 0.5:
                step_count += 1
                walk_phase = 7; phase_start = t
                print(f"[{step_count}] Settle...")

        elif walk_phase == 7:  # SETTLE
            s = smooth(pt, SETTLE_TIME)
            desired_roll = -LEAN_ANGLE * (1 - s)
            if pt >= SETTLE_TIME:
                walk_phase = 0; phase_start = t
                print(f"[{step_count}] Cycle {step_count//2+1} → Shift LEFT")

    # ──── Apply Control ────
    current_target = base_target + crouch_offset + walk_offset
    torque = (kp_torque * compliance) * (current_target - qpos[:nu]) - kd_torque * qvel[:nu]
    
    for i, n in enumerate(act_names):
        if "ankle_pitch" in n:  torque[i] += 0.4 * fp
        elif "hip_pitch" in n:  torque[i] += 0.6 * fp
        elif "arm_pitch" in n:  torque[i] -= 10.0 * d_.qvel[4]
        if "ankle_roll" in n:   torque[i] += 0.4 * fr
        elif "hip_roll" in n:   torque[i] += 0.6 * fr
        elif "arm_roll" in n:   torque[i] -= 10.0 * d_.qvel[3]
        
    d_.ctrl[:] = np.clip(torque / gear, -1.0, 1.0)

    if d_.time > 2.0 and d_.qpos[2] < 0.4:
        print(f"  !! FELL t={d_.time:.1f}s Z={d_.qpos[2]:.3f} R={np.degrees(roll):+.1f}° !!")

mujoco.set_mjcb_control(controller)

print("=" * 60)
print("  Angad Walking v6 — Crouched Ready Stance")
print(f"  Straight 0-{TIME_STRAIGHT}s → Crouch to {TIME_CROUCH}s → Walk")
print(f"  Knee Bend: {np.degrees(CROUCH_KNEE):.1f}° | Lean: {np.degrees(LEAN_ANGLE):.1f}°")
print("=" * 60)

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
        if step % 5000 == 0:
            q = d.qpos[3:7]
            p = np.degrees(2.0*(q[0]*q[2]-q[3]*q[1]))
            r = np.degrees(2.0*(q[0]*q[1]+q[2]*q[3]))
            com_x = d.subtree_com[0][0]
            if walk_phase == -2: ph = "STRGHT"
            elif walk_phase == -1: ph = "CROUCH"
            else: ph = f"W{walk_phase}"
            print(f"[{ph}] t={d.time:.1f}s Z={d.qpos[2]:.3f} R={r:+.1f}° COM_X={com_x:+.3f} steps={step_count}")
        viewer.sync()
        elapsed = time.time() - wallclock
        if elapsed < 1/60: sleep(1/60 - elapsed)
