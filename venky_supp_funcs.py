""" MIT License

Copyright (c) [2025] [Vyankatesh Ashtekar]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 """
# My functions

import numpy as np
import mujoco as mjc
import mujoco.viewer
from scipy.spatial import ConvexHull

# Walking 
# COM-ZMP and foot walking trajectory generator

def com_zmp_foot_pattern(t: float, T: float, t_dsp_frac: float, step_len: float, track_wid: float, com_ht: float, fzh: float):
    """
    This function generates desired COM and ZMP trajectories for biped walking.
    Based on Y. Choi's 2007 T-RO paper.

    @param t: time in seconds
    @param T: time period of one step in seconds
    @param t_dsp_frac: fraction of T spent in double support phase
    @param step_len: length of the step in meters
    @param track_wid: width of the track in meters
    @param com_ht: height of the center of mass in meters
    @param fzh: max height of the swing foot in meters
    @return: nseg: segment number
             p: ZMP position
             c: COM position
             fl: left foot position
             fr: right foot position
    """
    # All the coordinates are in a local frame.
    # Walking direction is along x-axis
    # Left foot center is at (0, A, 0)
    # Right foot center is at (0, -A, 0)
    # Right foot is put forward in the first step

    # Constants
    A = track_wid/2           # half of the track width
    B = step_len/2            # half of the step length
    wn = np.sqrt(9.81 / com_ht)   # natural frequency of the LIPM
    td = t_dsp_frac * T / 2   # half time duration of double support phase (DSP)

    nseg = int(t/T)    # Starts with zero. (nseg+1)th segment of the pattern
    lr:int = nseg % 2  # if lr = 0, right foot is being lifted
    t = t % (T)

    tdwn = td * wn
    Kx = (B * tdwn)
    t_tanh = np.tanh(wn*(T/2 -td))
    Kx /= (tdwn + t_tanh)
    Cx1 = Kx - B
    Cx2 = Kx / tdwn
    
    Ky = (A * tdwn * t_tanh) / (1 + tdwn*t_tanh)
    Cy1 = Ky - A
    Cy2 = Ky / tdwn

    wr = np.pi / (T -2*td)

    # DSP
    if t <= td:
        # ZMP
        px = (Kx/td)*t 
        py = (Ky/td)*t
        # COM
        cx = px
        cy = py
        cz = com_ht
        # Stance foot on ground
        flx = 0
        fly = A
        flz = 0
        # Swing foot on ground, ready to lift off
        frx = 0
        fry = -A
        frz = 0
    # SSP, assuming LIPM with point foot
    elif t <= T - td:
        # ZMP
        px = B
        py = A
        # COM
        cx = B + Cx1 * np.cosh(wn*(t - td)) + Cx2 * np.sinh(wn*(t - td))
        cy = A + Cy1 * np.cosh(wn*(t - td)) + Cy2 * np.sinh(wn*(t - td))
        cz = com_ht
        # Stance foot on ground
        flx = 0
        fly = A
        flz = 0
        # Swing foot in air
        frx = 2*B * (np.sin(wr*(t-td) - np.pi/2) + 1)
        if nseg == 0:
            frx /= 2
        fry = -A
        frz = (fzh/2) * (np.sin(2*wr*(t-td)-np.pi/2) + 1)
    # DSP
    else:
        # ZMP
        px = (2*B - Kx) + (Kx/td)*(t - (T - td))
        py = (Ky/td)*(T - t)
        # COM
        cx = px
        cy = py
        cz = com_ht
        # Stance foot on ground
        flx = 0
        fly = A
        flz = 0
        # Swing foot on ground, touch down complete
        frx = 2*2*B
        if nseg == 0:
            frx /= 2
        fry = -A 
        frz = 0

    if lr:
        # Mirror the COM and ZMP trajectories about XZ plane
        py = -py
        cy = -cy
        # Exchange the swing foot and the stance foot
        flx, frx = frx, flx
        fly, fry = -fry, -fly
        flz, frz = frz, flz
        
    # Conversion to absolute coordinates based on nseg, B, and A.
    # X coordinates increase by 2*B for each segment
    # Y coordinates remain the same
    # Z coordinates remain the same
    px += nseg * 2 * B
    cx += nseg * 2 * B

    if lr:
        frx += nseg * 2 * B
        flx += max(0, (nseg - 1) * 2 * B) 
    else:
        flx += nseg * 2 * B
        frx += max(0, (nseg - 1) * 2 * B) 

    # print("flx, frx: ", flx, frx)
    # nseg, zmp, com, left foot, right foot
    return nseg, np.array([px, py, 0.]), np.array([cx, cy, cz]), np.array([flx, fly, flz]), np.array([frx, fry, frz])


def special_fdm_qdot(m:mjc.MjModel, prv, nxt, dt):
    """
    Special function for robot with floating base
    """
    qvel = np.zeros(m.nv)
    mjc.mj_differentiatePos(m, qvel, dt, prv, nxt)
    return qvel

def special_fdm_qddot(m:mjc.MjModel, prv, curr, nxt, dt):
    """
    Special function for robot with floating base
    Central difference method to calculate the joint accelerations.
    """
    nv = m.nv
    qddot = np.zeros(nv)
    qvel2 = np.zeros(nv)
    qvel1 = np.zeros(nv)

    mjc.mj_differentiatePos(m, qvel1, dt, prv, curr)
    mjc.mj_differentiatePos(m, qvel2, dt, curr, nxt)
    qddot = (qvel2 - qvel1) / dt

    return qddot

# Representations of rotation

def Rx(theta: float) -> np.ndarray:
    """
    Rotation matrix about x-axis by angle theta.
    """
    R = np.array([[1, 0, 0],
                  [0, np.cos(theta), -np.sin(theta)],
                  [0, np.sin(theta), np.cos(theta)]])
    return R

def Ry(theta: float) -> np.ndarray:
    """
    Rotation matrix about y-axis by angle theta.
    """
    R = np.array([[np.cos(theta), 0, np.sin(theta)],
                  [0, 1, 0],
                  [-np.sin(theta), 0, np.cos(theta)]])
    return R

def Rz(theta: float) -> np.ndarray:
    """
    Rotation matrix about z-axis by angle theta.
    """
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0, 0, 1]])
    return R

def Rp2quat(c1c2c3: np.ndarray) -> np.ndarray:
    """
    Convert Rodrigues' parameters c1, c2, and c3 to a quaternion.
    """
    c1, c2, c3 = c1c2c3
    q = np.zeros(4)
    # Angle always positive
    angle = 2 * np.arctan(np.linalg.norm(c1c2c3))
    
    if angle < 1e-6:
        q= np.array([1, 0, 0, 0])
    else:
        q[0] = np.cos(angle / 2)
        q[1:] = np.array([c1, c2, c3]) * np.cos(angle / 2)

    return q

def quat2Rp(quat: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to Rodrigues' parameters c1, c2, and c3.
    """
    q0, q1, q2, q3 = quat
    
    c1c2c3 = np.zeros(3)
    if (1 - np.abs(q0)) < 1e-6:
        c1c2c3 = np.array([0, 0, 0])
    else:
        c1c2c3 = np.array([q1, q2, q3]) / q0
    
    return c1c2c3

def Rp2mat(c1c2c3: np.ndarray) -> np.ndarray:
    """
    Convert Rodrigues' parameters to a rotation matrix.
    """
    quat = Rp2quat(c1c2c3)
    Rmat = np.zeros((9, 1))
    mjc.mju_quat2Mat(Rmat, quat)
    return Rmat # Output is 9x1

def mat2Rp(Rmat: np.ndarray) -> np.ndarray:
    """
    Convert a rotation matrix to Rodrigues' parameters.
    """
    quat = np.zeros(4)
    mjc.mju_mat2Quat(quat, Rmat) # pass 9x1 Rmat only
    return quat2Rp(quat)

def Rp2axisAngle(c1c2c3: np.ndarray) -> np.ndarray:
    """
    Convert Rodrigues' parameters c1, c2, and c3 to axis-angle representation.
    """
    c1, c2, c3 = c1c2c3
    # Angle always positive
    angle = 2 * np.arctan(np.linalg.norm(c1c2c3))
    axisa = np.zeros(4)
    axisa[3] = angle
    if angle < 1e-6:
        axisa[:3] = [0, 0, 1]
    else:
        axisa[:3] = np.array([c1, c2, c3]) / np.tan(angle / 2)
        # Normalise the axis again
        axisa[:3] = axisa[:3] / np.linalg.norm(axisa[:3])

    return axisa

def axisAngle2Rp(axisa: np.ndarray) -> np.ndarray:
    """
    Convert axis-angle representation to Rodrigues' parameters c1, c2, and c3.
    """
    # Create a normalized copy of the axis
    axis_normalized = axisa[:3] / np.linalg.norm(axisa[:3])
    
    if axisa[3] < 1e-6:
        return np.zeros(3)
    else:
        return np.tan(axisa[3] / 2) * axis_normalized

def quat2axisAngle(quat: np.ndarray) -> np.ndarray:
    q0, q1, q2, q3 = quat
    
    # The angle is always positive
    sinphiby2 = np.sqrt(q1**2 + q2**2 + q3**2)

    # Compute axis-angle representation
    axisa = np.zeros(4)
    axisa[3] = 2 * np.arctan2(sinphiby2, q0)

    # Handle special case when q0 is close to ±1
    if abs(q0**2 - 1) < 1e-6:
        axisa[:3] = [0, 0, 1]
    else:
        temp = np.sqrt(1 - q0**2)
        axisa[:3] = quat[1:] / temp

    return axisa

def mat2axisAngle(mat: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mjc.mju_mat2Quat(q, mat)  # Input 9x1 mat only
    return quat2axisAngle(q)

def axisAngle2mat(axisa: np.ndarray) -> np.ndarray:
    """
    Convert axis-angle representation to rotation matrix.

    @output: Rmat: 3x3 rotation matrix
    """
    q = np.zeros(4)
    q[0] = np.cos(axisa[3] / 2)
    q[1:] = np.sin(axisa[3] / 2) * axisa[:3]
    Rmat = np.zeros(9)
    mjc.mju_quat2Mat(Rmat, q)
    Rmat = Rmat.reshape(3, 3)
    return Rmat

def Rpdot2bOmg(c1c2c3: np.ndarray, c1c2c3_dot: np.ndarray) -> np.ndarray:
    """
    Convert the time derivative of Rodrigues' parameters to angular velocity (in body fixed frame).
    """
    c1, c2, c3 = c1c2c3
    cdelta = 1 + c1**2 + c2**2 + c3**2
    Jac = np.array([[1,c3,-c2],[-c3,1,c1],[c2,-c1,1]]) # derived in Mathematica using w = R-tr Rdot
    Jac = 2 * Jac / cdelta
    omg = Jac @ c1c2c3_dot
    return omg

def Rpddot2bOmgdot(c1c2c3: np.ndarray, c1c2c3_dot: np.ndarray, c1c2c3_ddot: np.ndarray) -> np.ndarray:
    """
    Convert the second time derivative of Rodrigues' parameters to angular acceleration (in body fixed frame).
    """
    c1, c2, c3 = c1c2c3
    cdelta = 1 + c1**2 + c2**2 + c3**2
    Jac = np.array([[1,c3,-c2],[-c3,1,c1],[c2,-c1,1]]) # derived in Mathematica using w = R-tr Rdot
    Jac = 2 * Jac / cdelta
    
    Hess = np.array([[[-2 * c1, -2 * c2, -2 * c3], 
                    [-2 * c1 * c3, -2 * c2 * c3, (1 + c1**2 + c2**2 - c3**2)], 
                    [2 * c1 * c2, -1 * (1 + c1**2 - c2**2 + c3**2), 2 * c2 * c3]],
        
                    [[2 * c1 * c3, 2 * c2 * c3, -1 * (1 + c1**2 + c2**2 - c3**2)], 
                    [-2 * c1, -2 * c2, -2 * c3], 
                    [(1 - c1**2 + c2**2 + c3**2), -2 * c1 * c2, -2 * c1 * c3]],

                    [[-2 * c1 * c2, (1 + c1**2 - c2**2 + c3**2), -2 * c2 * c3], 
                    [(-1 + c1**2 - c2**2 - c3**2), 2 * c1 * c2, 2 * c1 * c3], 
                    [-2 * c1, -2 * c2, -2 * c3]] ]) # derived in Mathematica from Jac dot
    Hess = 2 * Hess / cdelta**2
        
    t1 = Jac @ c1c2c3_ddot
    t2 = np.einsum('i,ijk,j->k', c1c2c3_dot, Hess, c1c2c3_dot)
    omgdot = t1 + t2
    return omgdot

def crbiEllipsoid(m: mjc.MjModel, d: mjc.MjData, body: int, rho):
    """
    This function calculates the CRB inertia of a subtree starting from body b. 
    The inertia ellipsoid centred at the subtree COM is then calculated based on the CRB inertia matrix.
    """
    
    eigVal = np.zeros(3)
    eigVec = np.zeros(9)
    pa_quat = np.zeros(4)
    inertcepts = np.zeros(3)
    inertia = np.zeros(9)

    # Ellipsoid to be centered at COM
    cen = d.subtree_com[body]

    # Calculate the inertia matrix
    inertia_matrix = np.zeros((3, 3))
    mjc.mj_crb(m, d)
    inertia_matrix[0, 0] = d.crb[body][0]
    inertia_matrix[1, 1] = d.crb[body][1]
    inertia_matrix[2, 2] = d.crb[body][2]
    inertia_matrix[0, 1] = inertia_matrix[1, 0] = d.crb[body][3]
    inertia_matrix[0, 2] = inertia_matrix[2, 0] = d.crb[body][4]
    inertia_matrix[1, 2] = inertia_matrix[2, 1] = d.crb[body][5]

    # Calculate the eigenvalues and eigenvectors of the inertia matrix
    inertia = inertia_matrix.flatten()
    mjc.mju_eig3(eigVal, eigVec, pa_quat, inertia)    

    # Calculate the semi-axes of the ellipsoid. Based on Lee & Goswami's work
    c = (15 / (8 * np.pi * rho)) ** (1.0/5.0)
    t1 = (-eigVal[0] + eigVal[1] + eigVal[2])
    t2 = (eigVal[0] - eigVal[1] + eigVal[2])
    t3 = (eigVal[0] + eigVal[1] - eigVal[2])
    inertcepts[0] = c * (t1 ** (2.0/5.0) / (t2*t3) ** (1.0/10.0))
    inertcepts[1] = c * (t2 ** (2.0/5.0) / (t1*t3) ** (1.0/10.0))
    inertcepts[2] = c * (t3 ** (2.0/5.0) / (t1*t2) ** (1.0/10.0))

    # Rotation matrix to the the principal axes frame
    Rmat = np.zeros(9)
    mjc.mju_quat2Mat(Rmat,pa_quat)

    return cen, inertcepts, Rmat

# Inverse kinematics: position and velocity

def IK_pos(m:mjc.MjModel, 
       EF_BASE_SITE: np.array, 
       EF_BASE_Pos: np.ndarray,
       EF_BASE_kPhi: np.ndarray,
       DOFs_LIST_IK: np.array,
       q_init: list,
       nitr_ef_ex: np.array 
       ):
    
    """
    This function calculates the joint angles to reach the desired position of the end effector from base.
    """

    # Throw error if the number of DOFs for IK is not equal to the number of desired joint angles
    if len(DOFs_LIST_IK) != len(q_init):
        raise ValueError("Number of DOFs for IK must be equal to the number of desired joint angles")        
    # print("DOFs for IK: ", DOFs_LIST_IK)

    # Extract the site numbers
    EF_SITE = EF_BASE_SITE[0]
    BASE_SITE = EF_BASE_SITE[1]
    # print("EF_SITE: ", EF_SITE)
    # print("BASE_SITE: ", BASE_SITE)

    # Desired relative position: end effector - base
    des_rel_pos = np.zeros(3)
    des_rel_pos = EF_BASE_Pos[0] - EF_BASE_Pos[1]
    # print("Desired relative position: ", des_rel_pos)

    # Convert end effector k-phi to quaternion
    des_end_quat = np.zeros(4)
    mjc.mju_axisAngle2Quat(des_end_quat, EF_BASE_kPhi[0][0:3]/np.linalg.norm(EF_BASE_kPhi[0][0:3]), EF_BASE_kPhi[0][3])
    # print("Desired end effector quaternion: ", des_end_quat)

    # Convert base k-phi to quaternion
    des_base_quat = np.zeros(4)
    mjc.mju_axisAngle2Quat(des_base_quat, EF_BASE_kPhi[1][0:3]/np.linalg.norm(EF_BASE_kPhi[1][0:3]), EF_BASE_kPhi[1][3])
    # print("Desired base quaternion: ", des_base_quat)

    # Desired relative orientation: end effector - base
    des_rel_quat = np.zeros(4)
    qneg = np.zeros(4)
    mjc.mju_negQuat(qneg, des_base_quat)               # -qb
    mjc.mju_mulQuat(des_rel_quat, qneg, des_end_quat)  # -qb * qe
    # print("Desired relative quaternion: ", des_rel_quat)

    # q_sol will be the solution to the IK problem
    q_sol = np.array(q_init)
    # print("Initial joint angles: ", q_sol)

    # Make a MjData instance for the IK problem
    d_IK = mjc.MjData(m)
    nv = m.nv

    # Memory allocation for the linear velocity and angular velocity Jacobian matrix
    jv_full_BASE = np.zeros((3, nv))
    jv_full_EF = np.zeros((3, nv))
    jw_full_BASE = np.zeros((3, nv))
    jw_full_EF = np.zeros((3, nv))

    # pick DOFs relevant for the IK problem from the full robot's Jacobian
    jac = np.zeros((6, len(DOFs_LIST_IK)))

    # error in position and orientation
    tp_residual = np.zeros(6)
    end_site_quat = np.zeros(4)
    base_site_quat = np.zeros(4)
    rel_quat = np.zeros(4)

    # Loop
    nitr: int = 0
    ntrmax, ef, ex = nitr_ef_ex
    q_diff = np.zeros(len(DOFs_LIST_IK))

    while (nitr < ntrmax):
        # Set the joint positions, update the forward kinematics and Jacobians
        d_IK.qpos[1+DOFs_LIST_IK] = q_sol # IMPORTANT: MISMATCH DUE TO QUATERNION
        mjc.mj_fwdPosition(m, d_IK)

        mjc.mj_jacSite(m, d_IK, jv_full_BASE, jw_full_BASE, BASE_SITE)
        mjc.mj_jacSite(m, d_IK, jv_full_EF, jw_full_EF, EF_SITE)
        jv_full_EF = jv_full_EF - jv_full_BASE # relative to base
        jw_full_EF = jw_full_EF - jw_full_BASE # relative to base
        jac[0:3] = jv_full_EF[:, DOFs_LIST_IK]
        jac[3:6] = jw_full_EF[:, DOFs_LIST_IK]
        # print(jac)
        
        # Calculations for the position error (residual in task space)
        tp_residual[0:3] = des_rel_pos - (d_IK.site_xpos[EF_SITE] - d_IK.site_xpos[BASE_SITE])
        # Calculations for the current relative orientation of end effector wrt base
        mjc.mju_mat2Quat(end_site_quat, d_IK.site_xmat[EF_SITE])
        mjc.mju_mat2Quat(base_site_quat, d_IK.site_xmat[BASE_SITE])
        mjc.mju_negQuat(qneg, base_site_quat)           # -qb
        mjc.mju_mulQuat(rel_quat, qneg, end_site_quat)  # -qb * qe
        mjc.mju_subQuat(tp_residual[3:], des_rel_quat, rel_quat) # in tgt space

        # Solve for delta q
        # q_diff = np.linalg.pinv(jac) @ tp_residual # Biasing etc is pending
        q_diff = np.linalg.solve(jac, tp_residual)
        q_sol = q_sol + q_diff

        # Check for convergence in residual
        if np.linalg.norm(tp_residual) < ef:
            break

        # Check for convergence in joint angles
        if np.linalg.norm(q_diff) < ex:
            break

        # Increment the iteration counter
        nitr += 1
    
    

    if nitr == ntrmax:
        print("IK hitting max iterations, residual", np.linalg.norm(tp_residual))
    else:
        # Update the input joint angles with the solution
        q_init[:] = q_sol
        # print("IK converged in ", nitr, "iterations")

    # print("Solution: ", q_sol)
    # print("Number of iterations: ", nitr)
    # print("Residual: ", tp_residual)
    # print("ef, ex: ", np.linalg.norm(tp_residual), np.linalg.norm(q_diff))
    # return q_sol

def IK_pos_v2(m:mjc.MjModel, 
       EF_BASE_SITE: list, 
       EF_BASE_Pos: list,
       EF_BASE_kPhi: list,
       DOFs_LIST_IK: list,
       q_init: list,
       nitr_ef_ex: list = None
       ):
    
    """
    This function calculates the joint angles to reach the desired position of the end effector from base.
    @param m: MuJoCo model
    @param EF_BASE_SITE: [EF_SITE_NO, BASE_SITE_NO]
    @param EF_BASE_Pos: In ground frame: [EF_POS, BASE_POS]
    @param EF_BASE_kPhi: Orientations: [EF_kPhi or None, BASE_kPhi]
    @param DOFs_LIST_IK: List of DOFs for IK
    @param q_init: Initial guess for joint angles corresponding to DOFs_LIST_IK, solution updated in place.
    @param nitr_ef_ex: [ntrmax, ef, ex] = [10, 1e-5, 1e-5] by default

    @return: True if IK converges, False otherwise
    """
    # Termination conditions
    if nitr_ef_ex is None:
        ntrmax, ef, ex = [10, 1e-5, 1e-5]
    else:
        ntrmax, ef, ex = nitr_ef_ex

    # Throw error if the number of DOFs for IK is not equal to the number of desired joint angles
    DOFs_LIST_IK = np.array(DOFs_LIST_IK)
    if len(DOFs_LIST_IK) != len(q_init):
        raise ValueError("Number of DOFs for IK must be equal to the number of desired joint angles")        
        return

    # Extract the base and end effector site numbers
    if len(EF_BASE_SITE) == 2:
        EF_SITE = EF_BASE_SITE[0]
        BASE_SITE = EF_BASE_SITE[1]
    else:
        ns = len(EF_BASE_SITE)
        raise ValueError("Two sites are required for IK, but ", ns, " sites are provided")
        return

    # Extract the desired orientation of the end effector and base
    flag_ori = False
    if EF_BASE_kPhi[0] is not None: 
        # If the desired orientation of the end-effector is provided
        EF_BASE_kPhi = np.array(EF_BASE_kPhi)
        # Convert end effector k-phi to quaternion
        des_end_quat = np.zeros(4)
        mjc.mju_axisAngle2Quat(des_end_quat, EF_BASE_kPhi[0][0:3]/np.linalg.norm(EF_BASE_kPhi[0][0:3]), EF_BASE_kPhi[0][3])

        # Convert base k-phi to quaternion
        des_base_quat = np.zeros(4)
        mjc.mju_axisAngle2Quat(des_base_quat, EF_BASE_kPhi[1][0:3]/np.linalg.norm(EF_BASE_kPhi[1][0:3]), EF_BASE_kPhi[1][3])

        # Desired relative orientation: end effector - base
        des_rel_quat = np.zeros(4)
        qneg = np.zeros(4)
        mjc.mju_negQuat(qneg, des_base_quat)               # -qb
        mjc.mju_mulQuat(des_rel_quat, qneg, des_end_quat)  # -qb * qe
        flag_ori = True
    else:
        # Convert base k-phi to quaternion
        des_base_quat = np.zeros(4)
        mjc.mju_axisAngle2Quat(des_base_quat, EF_BASE_kPhi[1][0:3]/np.linalg.norm(EF_BASE_kPhi[1][0:3]), EF_BASE_kPhi[1][3])
        flag_ori = False

    # Desired relative position in base site frame: Rb^T * (end effector - base)
    Rb = np.zeros(9)
    mjc.mju_quat2Mat(Rb, des_base_quat)
    EF_BASE_Pos = np.array(EF_BASE_Pos)
    des_rel_pos = EF_BASE_Pos[0] - EF_BASE_Pos[1]
    des_rel_pos = Rb.reshape(3, 3).T @ des_rel_pos

    # q_sol will be the solution to the IK problem
    q_sol = np.array(q_init)

    # Make a MjData instance for the IK problem
    d_IK = mjc.MjData(m)
    nv = m.nv

    # Memory allocation for the linear velocity and angular velocity Jacobian matrix
    jv_full_BASE = np.zeros((3, nv))
    jv_full_EF = np.zeros((3, nv))
    jw_full_BASE = np.zeros((3, nv))
    jw_full_EF = np.zeros((3, nv))

    # pick DOFs relevant for the IK problem from the full robot's Jacobian
    if flag_ori:
        jac = np.zeros((6, len(DOFs_LIST_IK)))
        end_site_quat = np.zeros(4)
        rel_quat = np.zeros(4)
        tp_residual = np.zeros(6)
    else:
        jac = np.zeros((3, len(DOFs_LIST_IK)))
        tp_residual = np.zeros(3)

    base_site_quat = np.zeros(4)
    q_diff = np.zeros(len(DOFs_LIST_IK))
    
    
    # Track the best solution seen during iteration
    best_q_sol = q_sol.copy()
    best_residual = 1e10
    
    # Solve for the joint angles iteratively
    nitr: int = 0
    while (nitr < ntrmax):
        # Set the joint positions, update the forward kinematics and Jacobians
        d_IK.qpos[1+DOFs_LIST_IK] = q_sol.copy() # IMPORTANT: MISMATCH DUE TO FREEJOINT QUATERNION
        mjc.mj_fwdPosition(m, d_IK)
        Rb = d_IK.site_xmat[BASE_SITE].reshape(9, 1)
        mjc.mj_jacSite(m, d_IK, jv_full_BASE, jw_full_BASE, BASE_SITE)
        mjc.mj_jacSite(m, d_IK, jv_full_EF, jw_full_EF, EF_SITE)

        jv_rel = jv_full_EF - jv_full_BASE 
        jv_rel = Rb.reshape(3, 3).T @ jv_rel # in base site frame
        jw_rel = jw_full_EF - jw_full_BASE
        jw_rel = Rb.reshape(3, 3).T @ jw_rel # in base site frame
        
        if flag_ori:
            jac[:3] = jv_rel[:, DOFs_LIST_IK]
            jac[3:6] = jw_rel[:, DOFs_LIST_IK]
        else:
            jac = jv_rel[:, DOFs_LIST_IK]
        
        # Calculations for the position error (residual in task space)
        if flag_ori:
            tp_residual[:3] = des_rel_pos - Rb.reshape(3, 3).T @ (d_IK.site_xpos[EF_SITE] - d_IK.site_xpos[BASE_SITE])
            # Calculations for the current relative orientation of end effector wrt base
            mjc.mju_mat2Quat(end_site_quat, d_IK.site_xmat[EF_SITE])
            mjc.mju_mat2Quat(base_site_quat, Rb)
            mjc.mju_negQuat(qneg, base_site_quat)           # -qb
            mjc.mju_mulQuat(rel_quat, qneg, end_site_quat)  # -qb * qe
            mjc.mju_subQuat(tp_residual[3:], des_rel_quat, rel_quat) # in tgt space
        else:
            tp_residual = des_rel_pos - Rb.reshape(3, 3).T @ (d_IK.site_xpos[EF_SITE] - d_IK.site_xpos[BASE_SITE])

        cur_residual = np.linalg.norm(tp_residual)
        if cur_residual < best_residual:
            best_residual = cur_residual
            best_q_sol = q_sol.copy()

        # Solve for delta q using damped pseudo-inverse to handle straight-leg singularity (det ~ 0)
        q_diff = np.linalg.pinv(jac, rcond=1e-3) @ tp_residual
        q_sol += 0.5 * q_diff

        # Check for convergence in residual
        if cur_residual < ef:
            break

        # Check for convergence in joint angles
        if np.linalg.norm(q_diff) < ex:
            break

        # Increment the iteration counter
        nitr += 1
    
    # Always write back the best solution found
    q_init[:] = best_q_sol
    
    if nitr == ntrmax:
        # Only print if residual is actually large
        # Silenced to avoid stdout flooding that causes slow-mo
        # print("IK hitting max iterations, residual", best_residual)
        pass
        return False
    else:
        # print("IK converged in ", nitr, "iterations")
        return True

    # print("Solution: ", q_sol)
    # print("Number of iterations: ", nitr)
    # print("Residual: ", tp_residual)
    # print("ef, ex: ", np.linalg.norm(tp_residual), np.linalg.norm(q_diff))

def IK_vel(m:mjc.MjModel, 
           EF_BASE_SITE: int, 
           EF_BASE_Vel: np.ndarray,
           EF_BASE_omg: np.ndarray,
           DOFs_LIST_IK: np.array,
           q_sol: np.array,
           q_dot_sol: np.array):

    """
    This function calculates the joint velocities given the base and end effector linear and angular velocities.
    """

    # Relative (v, w): end effector - base
    des_rel_twist = np.zeros(6)
    des_rel_twist[0:3] = EF_BASE_Vel[0] - EF_BASE_Vel[1]
    des_rel_twist[3:6] = EF_BASE_omg[0] - EF_BASE_omg[1]

    # make a MjData instance for the IK problem
    d_IK = mjc.MjData(m)
    nv = m.nv

    # set the partial robot configuration 
    d_IK.qpos[1+DOFs_LIST_IK] = q_sol.copy()
    mjc.mj_fwdPosition(m, d_IK)

    # calculate the full robot Jacobian
    jac_full_ef = np.zeros((6, nv))
    jac_full_base = np.zeros((6, nv))
    mjc.mj_jacSite(m, d_IK, jac_full_base[0:3], jac_full_base[3:6], EF_BASE_SITE[1])
    mjc.mj_jacSite(m, d_IK, jac_full_ef[0:3], jac_full_ef[3:6], EF_BASE_SITE[0])
    jac = np.zeros((6, len(DOFs_LIST_IK)))
    jac[0:3] = jac_full_ef[0:3, DOFs_LIST_IK] - jac_full_base[0:3, DOFs_LIST_IK]
    jac[3:6] = jac_full_ef[3:6, DOFs_LIST_IK] - jac_full_base[3:6, DOFs_LIST_IK]

    # Solve for joint velocities
    q_dot_sol[:] = np.linalg.solve(jac, des_rel_twist)

def IK_pos_robot(m:mjc.MjModel,
                 EF_BASE1_BASE2_SITE: np.array,
                 EF_BASE1_BASE2_Pos: np.ndarray,
                 EF_BASE1_BASE2_kPhi: np.ndarray,
                 DOFs_1_2_IK: np.ndarray,
                 q_1_2_int: list,
                 nitr_ef_ex: np.array):
    """
    Wrapper for the IK_pos function for biped robot.
    """
    IK_pos_v2(m, 
           EF_BASE1_BASE2_SITE[0], 
           EF_BASE1_BASE2_Pos[0], 
           EF_BASE1_BASE2_kPhi[0], 
           DOFs_1_2_IK[0], 
           q_1_2_int[0], 
           nitr_ef_ex)

    IK_pos_v2(m, 
           EF_BASE1_BASE2_SITE[1], 
           EF_BASE1_BASE2_Pos[1], 
           EF_BASE1_BASE2_kPhi[1], 
           DOFs_1_2_IK[1], 
           q_1_2_int[1], 
           nitr_ef_ex)

def body_ancestorQ(model, root_body, b1):
    """Check if root_body is an ancestor of b1 in the kinematic tree."""
    while b1 != 0:  # 0 means no parent (root of the tree)
        if b1 == root_body:
            return True
        b1 = model.body_parentid[b1]
    return False

def rm_self_contacts(m: mjc.MjModel,
                     d: mjc.MjData,
                     robot_root_body: int):
    """
    This function filters out self collisions

    @return: cID_b1_b2 = [contact_id, robot_body (either body1 or body2), body1, body2]
    """

    # Collect the useful contacts (upper limit 25)
    # cID_b1_b2 = [contact_id, robot_body, body1, body2]
    cID_b1_b2 = np.zeros((25,4), dtype=int)

    # Choose a contact only if either of the two bodies are
    # part of subtree under robot_root_body or belong to other_body_ids
    ncon = d.ncon

    # No contacts detected
    if not ncon:
        return None
    
    # If there are non-zero contacts,
    idx:int = 0
    robot_body:int = 0
    other_body:int = 0 
    for i in range(ncon):
        # Contact bodies
        b1 = m.geom_bodyid[d.contact[i].geom[0]]
        b2 = m.geom_bodyid[d.contact[i].geom[1]]
        # Check if one body belongs to the robot subtree and the other does not
        is_b1_robot = body_ancestorQ(m, robot_root_body, b1)
        is_b2_robot = body_ancestorQ(m, robot_root_body, b2)
        # One is in the subtree, the other is not
        if is_b1_robot != is_b2_robot:  
            robot_body = b1 if is_b1_robot else b2
            other_body = b2 if is_b1_robot else b1
            cID_b1_b2[idx] = [i, robot_body, b1, b2]
            idx += 1
    # idx = the number of useful contacts

    # If there are no useful contacts, return the robot COM
    if idx == 0:
        #print("No useful contacts")
        return None

    return cID_b1_b2[:idx]  # Return only the useful contacts

def ZMP(m: mjc.MjModel,
        d: mjc.MjData,
        pt_plane: np.array,
        n_plane: np.array,
        robot_root_body: int):
    """
    This function calculates the Zero-tilting moment point (ZMP) on a given plane.
    """
    # Collect the useful contacts (upper limit 25)
    # cID_b1_b2 = [contact_id, robot_body, body1, body2]
    cID_b1_b2 = np.zeros((25,4), dtype=int)

    # Choose a contact only if either of the two bodies are
    # part of subtree under robot_root_body or belong to other_body_ids
    ncon = d.ncon

    # Robot is in the air
    if not ncon:
        # Return the robot COM vertical on the plane?
        if n_plane[2] == 0:
            # projection not possible
            return d.subtree_com[robot_root_body]
        else:
            # projection possible
            zmp = np.zeros(3)
            zmp[0:2] = d.subtree_com[robot_root_body][0:2]
            zmp[2] = d.subtree_com[robot_root_body][2] - np.dot(n_plane, d.subtree_com[robot_root_body] - pt_plane) / n_plane[2]
            return zmp
    
    # If there are non-zero contacts,
    idx:int = 0
    robot_body:int = 0
    other_body:int = 0 
    for i in range(ncon):
        # Contact bodies
        b1 = m.geom_bodyid[d.contact[i].geom[0]]
        b2 = m.geom_bodyid[d.contact[i].geom[1]]
        # Check if one body belongs to the robot subtree and the other does not
        is_b1_robot = body_ancestorQ(m, robot_root_body, b1)
        is_b2_robot = body_ancestorQ(m, robot_root_body, b2)
        # One is in the subtree, the other is not
        if is_b1_robot != is_b2_robot:  
            robot_body = b1 if is_b1_robot else b2
            other_body = b2 if is_b1_robot else b1
            cID_b1_b2[idx] = [i, robot_body, b1, b2]
            idx += 1
    # idx = the number of useful contacts

    # If there are no useful contacts, return the robot COM
    if idx == 0:
        print("No useful contacts")
        return d.subtree_com[robot_root_body]

    # Calculate the moment of the useful contact forces about the point pt_plane
    moment = np.zeros(3)
    lcf_6 = np.zeros(6) # Force-torque
    lcf = lcf_6[0:3]
    gcf = np.zeros(3)
    net_gcf = np.zeros(3)
    for i in range(idx):
        # Compute the contact force acting on the robot in the world frame
        mjc.mj_contactForce(m, d, cID_b1_b2[i, 0], lcf_6)
        mjc.mju_mulMatTVec(gcf, d.contact[cID_b1_b2[i, 0]].frame.reshape(3, 3), lcf)                    
        # Flip the direction of the force if robot body is b1. (forces are b1 -> b2)
        if cID_b1_b2[i, 1] == cID_b1_b2[i, 2]:
            gcf = -gcf
        # Compute the moment of the contact force about the point pt_plane
        moment += np.cross(d.contact[cID_b1_b2[i, 0]].pos - pt_plane, gcf)
        # Compute the net contact force acting on the robot in the world frame
        net_gcf += gcf
         
    # Calculate the ZMP
    zmp = np.zeros(3)
    den = np.zeros(3)
    den = np.dot(n_plane, net_gcf)
    # If den is more than 1e-6, calculate the ZMP
    if den < 1e-6:
        print("ZMP denominator close to zero", den)
        zmp = d.subtree_com[robot_root_body] # Yet to resolve
    else:
        zmp = pt_plane + np.cross(n_plane, moment) / den
    
    return zmp

    def fallQ_footRotation(m:mjc.MjModel, d:mjc.MjData):
        """
        This function checks if the robot has fallen base on foot rotation.
        """
        axisA_Hip = mat2axisAngle(d.site_xmat[0])
        axisA_LF = mat2axisAngle(d.site_xmat[1])
        axisA_RF = mat2axisAngle(d.site_xmat[2])

# Drawing

def draw_sphere(user_scene: mjc.viewer.Handle.user_scn,
                cen: np.ndarray, 
                rad: float = 0.025, 
                colour: np.ndarray = np.array([1, 0, 0, 0.5])):
    """
    Draw a line segment between two points in 3D space.
    """
    # We will add only one geom in this function
    user_scene.ngeom += 1
    idx_geom = user_scene.ngeom - 1
    mjc.mjv_initGeom(user_scene.geoms[idx_geom],
                    type=mjc.mjtGeom.mjGEOM_SPHERE,
                    size=np.array([rad, rad, rad]),
                    pos=cen,
                    mat=np.eye(3).flatten(),
                    rgba=colour)

def draw_box(user_scene: mjc.viewer.Handle.user_scn,
             cen: np.ndarray, 
             size: np.ndarray = np.array([0.025, 0.025, 0.025]),
             mat: np.ndarray = np.eye(3).flatten(),
             colour: np.ndarray = np.array([1, 0, 0, 0.5])):
    """
    Draw a box in 3D space.
    """
    # We will add only one geom in this function
    user_scene.ngeom += 1
    idx_geom = user_scene.ngeom - 1
    mjc.mjv_initGeom(user_scene.geoms[idx_geom],
                    type=mjc.mjtGeom.mjGEOM_BOX,
                    size=size,
                    pos=cen,
                    mat=mat,
                    rgba=colour)

def draw_line_segment(user_scene: mjc.viewer.Handle.user_scn,
                      p1: np.ndarray, 
                      p2: np.ndarray, 
                      linewidth: float = 5,
                      colour: np.ndarray = np.array([1, 0, 0, 0.5])):
    """
    Draw a line segment between two points in 3D space.
    """

    # We will add only one geom in this function    
    user_scene.ngeom += 1
    idx_geom = user_scene.ngeom - 1

    linesize = np.zeros(3)
    diff = p2 - p1
    linesize[2] = np.linalg.norm(diff)
    linesize[0] = linewidth
    linesize[1] = linewidth

    myquat = np.array([1.,0.,0.,0.])
    mjc.mju_quatZ2Vec(myquat, diff)
    Rmat = np.zeros(9)
    mjc.mju_quat2Mat(Rmat, myquat)

    mjc.mjv_initGeom(user_scene.geoms[idx_geom],
                                type=mjc.mjtGeom.mjGEOM_LINE,
                                size=linesize,
                                pos = p1,
                                mat=Rmat,
                                rgba=colour)

def draw_polyline(user_scene: mjc.viewer.Handle.user_scn,
                  vertices: np.ndarray,
                  vertex_markers: bool = False,
                  linewidth: float = 5,
                  vertex_marker_radius: float = 0.025,
                  colour_line: np.ndarray = np.array([1, 0, 0, 0.5]),
                  colour_vertex: np.ndarray = np.array([0, 1, 0, 0.5])):
    """
    Draw a polyline in 3D space.
    """
    # Find the number of vertices
    n_vert = vertices.shape[0]

    # Draw the line segments
    for i in range(n_vert - 1):
        draw_line_segment(user_scene, vertices[i], vertices[i + 1], linewidth, colour_line)
    
    # Draw the vertex markers
    if vertex_markers:
        for i in range(n_vert):
            draw_sphere(user_scene, vertices[i], vertex_marker_radius, colour_vertex)

def draw_arrow(user_scene: mjc.viewer.Handle.user_scn,
                p1: np.ndarray, 
                dirn: np.ndarray, 
                linewidth: float = 0.05,
                colour: np.ndarray = np.array([1, 0, 0, 0.5])):
    """
    Draw an arrow in 3D space.
    """

    # We will add only one geom in this function    
    user_scene.ngeom += 1
    idx_geom = user_scene.ngeom - 1

    linesize = np.zeros(3)
    
    linesize[2] = np.linalg.norm(dirn)
    linesize[0] = linewidth
    linesize[1] = linewidth

    # Draw an arrow only if non-zero arrow length is specified
    if linesize[2] > 1e-6:
        myquat = np.array([1.,0.,0.,0.])
        mjc.mju_quatZ2Vec(myquat, dirn)
        Rmat = np.zeros(9)
        mjc.mju_quat2Mat(Rmat, myquat)

        mjc.mjv_initGeom(user_scene.geoms[idx_geom],
                        type=mjc.mjtGeom.mjGEOM_ARROW,
                        size=linesize,
                        pos = p1,
                        mat=Rmat,
                        rgba=colour)
