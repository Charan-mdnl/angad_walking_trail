# LIPM walking trial for Angad (ported from Venky's code)
# Adapted for the new robot model with dynamic joint mapping.

import numpy as np
import matplotlib.pyplot as plt
import time
from time import sleep

import mujoco
import mujoco.viewer

from venky_supp_funcs import *

# ─────────────────────────── Global variables ───────────────────────────
ppause = False

# Load MuJoCo model (with capsule feet + keyframe)
model_path = "angad_walking.mjb"
m = mujoco.MjModel.from_binary_path(model_path)
d = mujoco.MjData(m)

# ─────────────────────────── Helper functions ───────────────────────────
def get_qpos_idx(jname):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
    return m.jnt_qposadr[jid]

def get_dof_idx(jname):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
    return m.jnt_dofadr[jid]

# ─────────────────────────── Site IDs ───────────────────────────────────
hip_site_ID = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "base_site")
lf_site_ID  = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "LF_site")
rf_site_ID  = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "RF_site")
site_list_lf = np.array([hip_site_ID, lf_site_ID])
site_list_rf = np.array([hip_site_ID, rf_site_ID])

# ─────────────────────────── Joint mapping ──────────────────────────────
L_joint_names = [
    "base_hip_pitch_l", "hip_L_thigh_roll_l", "thigh_L_thigh_yaw_l",
    "upper_knee_L_knee_pitch_l", "lower_knee_L_ankle_pitch_l", "ankle_L_ankle_roll_l"
]
R_joint_names = [
    "base_hip_pitch_r", "hip_R_thigh_roll_r", "thigh_R_thigh_yaw_r",
    "upper_knee_R_knee_pitch_r", "lower_knee_R_ankle_pitch_r", "ankle_R_ankle_roll_r"
]

LL_DOF  = np.array([get_dof_idx(n) for n in L_joint_names])
RL_DOF  = np.array([get_dof_idx(n) for n in R_joint_names])
LL_QPOS = np.array([get_qpos_idx(n) for n in L_joint_names])
RL_QPOS = np.array([get_qpos_idx(n) for n in R_joint_names])

nv = m.nv
nu = m.nu

# ─────────────────────────── Actuator → Joint mapping ───────────────────
actuator_qpos_idx = np.zeros(nu, dtype=int)
actuator_qvel_idx = np.zeros(nu, dtype=int)
for i in range(nu):
    jnt_idx = m.actuator_trnid[i][0]
    actuator_qpos_idx[i] = m.jnt_qposadr[jnt_idx]
    actuator_qvel_idx[i] = m.jnt_dofadr[jnt_idx]

# ─────────────────────────── PD Control gains ───────────────────────────
Kp_HipKnee = 500
Kp_Ankle   = 500
Kp_UB      = 15
Kp_list = np.ones(nu)
Kd_list = np.ones(nu) * 0.5

Kp_list[0:4]  = Kp_HipKnee
Kp_list[6:10] = Kp_HipKnee
Kp_list[[4, 5, 10, 11]] = Kp_Ankle
Kp_list[12:] = Kp_UB

Kd_HipKnee = 50
Kd_list[0:4]  = Kd_HipKnee
Kd_list[6:10] = Kd_HipKnee

Tsat = 150
tau_sat_list = np.ones(nu) * Tsat

# ─────────────────────────── IK solution arrays ─────────────────────────
q_des_list  = np.zeros(nu)
dq_des_list = np.zeros(nu)

# ─────────────────────────── IK inputs ──────────────────────────────────
hip_pos_des      = np.zeros(3)
lf_pos_des       = np.zeros(3)
rf_pos_des       = np.zeros(3)
hip_pos_des_init = np.zeros(3)
lf_pos_des_init  = np.zeros(3)
rf_pos_des_init  = np.zeros(3)

hip_kphi_des = np.array([0, 1, 0, 0.])
lf_kphi_des  = np.array([0, 1, 0, 0.])
rf_kphi_des  = np.array([0, 1, 0, 0.])

conv_params = np.array([100, 1e-4, 1e-4])

# ─────────────────────────── Controller/motion state ────────────────────
mstime     = 2 * np.pi
idx        = 0
tw         = 0.
com_height = 0.

# ─────────────────────────── Standing pose ──────────────────────────────
STANDING_L  = np.array([-1.5655,  0.3491, -0.3504, -0.0937, -0.3491,  0.0])
STANDING_R  = np.array([ 1.5855, -0.3142,  0.3703, -0.0897, -0.3491,  0.0])
BASE_HEIGHT = 0.7572
BASE_QUAT   = np.array([0.7071068, 0.0, 0.0, -0.7071068])  # -90° Z rotation

# CRITICAL: The freejoint qpos that the IK solver needs to match the real robot.
# Without this, the IK Jacobian is computed in the wrong frame.
IK_BASE_QPOS = np.array([0.0, 0.0, BASE_HEIGHT, BASE_QUAT[0], BASE_QUAT[1], BASE_QUAT[2], BASE_QUAT[3]])


# ═══════════════════════════ Function declarations ═══════════════════════

def keyboard_func(keycode):
    global ppause
    if keycode == ord(' '):
        ppause = not ppause

def satFunc(x, abs_bound):
    n = 5
    num = x
    den = np.power(1 + np.power(x / abs_bound, 2 * n), 1 / (2 * n))
    return num / den

def apply_standing_pose():
    """Reset to the 'trial' keyframe (standing pose with correct base orientation)."""
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "trial")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(m, d, key_id)
    else:
        # Fallback: manual pose
        d.qpos[:] = 0
        d.qpos[0:3] = [0, 0, BASE_HEIGHT]
        d.qpos[3:7] = BASE_QUAT
        for i in range(6):
            d.qpos[LL_QPOS[i]] = STANDING_L[i]
            d.qpos[RL_QPOS[i]] = STANDING_R[i]
    mujoco.mj_forward(m, d)

def pd_control(m_: mujoco.MjModel, d_: mujoco.MjData):
    global nu, Kp_list, Kd_list, q_des_list, dq_des_list, tau_sat_list
    jt = np.zeros(nu)
    for i in range(nu):
        pos_err = q_des_list[i] - d_.qpos[actuator_qpos_idx[i]]
        vel_err = dq_des_list[i] - d_.qvel[actuator_qvel_idx[i]]
        jt[i] = Kp_list[i] * pos_err + Kd_list[i] * vel_err
    d_.ctrl = satFunc(jt, tau_sat_list)

def controller(m_: mujoco.MjModel, d_: mujoco.MjData):
    global hip_site_ID, lf_site_ID, rf_site_ID
    global site_list_lf, site_list_rf, LL_DOF, RL_DOF
    global hip_pos_des, lf_pos_des, rf_pos_des
    global hip_kphi_des, lf_kphi_des, rf_kphi_des
    global hip_pos_des_init, lf_pos_des_init, rf_pos_des_init
    global q_des_list, dq_des_list, conv_params
    global mstime, idx, com_height, tw

    # ── Walking trajectory ──
    if d_.time > mstime:
        if idx == 0:
            idx += 1
            tw = np.abs(d_.site_xpos[lf_site_ID][1] - d_.site_xpos[rf_site_ID][1])
            print("track width: ", tw)
            com_height = d_.site_xpos[hip_site_ID][2]
            print("com height: ", com_height)

            lf_pos_des_init = d_.site_xpos[lf_site_ID].copy()
            rf_pos_des_init = d_.site_xpos[rf_site_ID].copy()
            hip_pos_des_init = d_.site_xpos[hip_site_ID].copy()

        t = d_.time - mstime
        nseg, zmp, com, left_foot, right_foot = com_zmp_foot_pattern(
            t, T=1.0, t_dsp_frac=0.1, step_len=0.025,
            track_wid=tw, com_ht=com_height, fzh=0.08
        )

        lf_pos_des[0] = lf_pos_des_init[0] + left_foot[0]
        lf_pos_des[1] = left_foot[1]
        lf_pos_des[2] = lf_pos_des_init[2] + left_foot[2]

        rf_pos_des[0] = rf_pos_des_init[0] + right_foot[0]
        rf_pos_des[1] = right_foot[1]
        rf_pos_des[2] = rf_pos_des_init[2] + right_foot[2]

        hip_pos_des[0] = hip_pos_des_init[0] + com[0]
        hip_pos_des[1] = hip_pos_des_init[1] + com[1]
        hip_pos_des[2] = com[2]

    # ── Solve IK for both legs ──
    # Use live freejoint state so IK Jacobian is in the correct frame
    live_base_qpos = d_.qpos[0:7].copy()

    q_sol_L = np.zeros(6)
    q_sol_R = np.zeros(6)
    for c in range(6):
        act_L = np.where(actuator_qpos_idx == LL_QPOS[c])[0][0]
        act_R = np.where(actuator_qpos_idx == RL_QPOS[c])[0][0]
        q_sol_L[c] = q_des_list[act_L]
        q_sol_R[c] = q_des_list[act_R]

    IK_pos_v2(m_, site_list_lf,
              np.array([hip_pos_des, lf_pos_des]),
              [hip_kphi_des, lf_kphi_des],
              LL_DOF, q_sol_L, conv_params,
              base_qpos=live_base_qpos)

    IK_pos_v2(m_, site_list_rf,
              np.array([hip_pos_des, rf_pos_des]),
              [hip_kphi_des, rf_kphi_des],
              RL_DOF, q_sol_R, conv_params,
              base_qpos=live_base_qpos)

    # ── Map IK solutions back ──
    for c in range(6):
        act_L = np.where(actuator_qpos_idx == LL_QPOS[c])[0][0]
        act_R = np.where(actuator_qpos_idx == RL_QPOS[c])[0][0]
        q_des_list[act_L] = q_sol_L[c]
        q_des_list[act_R] = q_sol_R[c]

    pd_control(m_, d_)


# ═══════════════════════════ Main function ═══════════════════════════════

def main():
    np.set_printoptions(precision=4, suppress=True)

    global idx
    idx = 0
    mujoco.set_mjcb_control(controller)

    print("Total mass:", mujoco.mj_getTotalmass(m))
    print("nu:", nu, "nv:", nv)
    print("LL_DOF:", LL_DOF)
    print("RL_DOF:", RL_DOF)
    print("LL_QPOS:", LL_QPOS)
    print("RL_QPOS:", RL_QPOS)

    # ── 1. Apply standing pose ──
    apply_standing_pose()

    global q_des_list, dq_des_list
    for i in range(nu):
        q_des_list[i] = d.qpos[actuator_qpos_idx[i]]
        dq_des_list[i] = d.qvel[actuator_qvel_idx[i]]

    # ── 2. Set initial IK targets ──
    global hip_pos_des, lf_pos_des, rf_pos_des
    global hip_pos_des_init, lf_pos_des_init, rf_pos_des_init

    hip_pos_des_init = d.site_xpos[hip_site_ID].copy()
    lf_pos_des_init  = d.site_xpos[lf_site_ID].copy()
    rf_pos_des_init  = d.site_xpos[rf_site_ID].copy()

    hip_pos_des_init[0] += 0.05

    hip_pos_des = hip_pos_des_init.copy()
    lf_pos_des  = lf_pos_des_init.copy()
    rf_pos_des  = rf_pos_des_init.copy()

    print("hip_pos_des:", hip_pos_des)
    print("lf_pos_des: ", lf_pos_des)
    print("rf_pos_des: ", rf_pos_des)

    # ── 3. Run initial IK from standing pose ──
    q_sol_L = np.zeros(6)
    q_sol_R = np.zeros(6)
    for c in range(6):
        q_sol_L[c] = d.qpos[LL_QPOS[c]]
        q_sol_R[c] = d.qpos[RL_QPOS[c]]

    ok_L = IK_pos_v2(m, site_list_lf,
              np.array([hip_pos_des, lf_pos_des]),
              [hip_kphi_des, lf_kphi_des],
              LL_DOF, q_sol_L, conv_params,
              base_qpos=IK_BASE_QPOS)
    ok_R = IK_pos_v2(m, site_list_rf,
              np.array([hip_pos_des, rf_pos_des]),
              [hip_kphi_des, rf_kphi_des],
              RL_DOF, q_sol_R, conv_params,
              base_qpos=IK_BASE_QPOS)

    print("IK Left converged:", ok_L, "q_sol_L:", q_sol_L)
    print("IK Right converged:", ok_R, "q_sol_R:", q_sol_R)

    for c in range(6):
        d.qpos[LL_QPOS[c]] = q_sol_L[c]
        act_L = np.where(actuator_qpos_idx == LL_QPOS[c])[0][0]
        q_des_list[act_L] = q_sol_L[c]

    for c in range(6):
        d.qpos[RL_QPOS[c]] = q_sol_R[c]
        act_R = np.where(actuator_qpos_idx == RL_QPOS[c])[0][0]
        q_des_list[act_R] = q_sol_R[c]

    mujoco.mj_fwdPosition(m, d)

    # ── 4. Launch simulation ──
    global ppause
    ppause = True
    with mujoco.viewer.launch_passive(
            m, d, key_callback=keyboard_func) as viewer:

        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = False
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

        while viewer.is_running():
            simstart = d.time
            wallclock = time.time()
            if not ppause:
                while (d.time - simstart) < 1 / 60:
                    mujoco.mj_step(m, d)
            else:
                mujoco.mj_forward(m, d)

            viewer.sync()

            computation_time = time.time() - wallclock
            if computation_time < 1 / 60:
                sleep(1 / 60 - computation_time)

        viewer.close()


if __name__ == "__main__":
    main()
