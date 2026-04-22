# Angad Humanoid Robot Walking Simulation

This is a self-contained walking simulation environment for the **Angad humanoid robot** from XP-Robotics.

## Quick Start

### Prerequisites
Make sure you have MuJoCo installed:
```bash
pip install mujoco numpy
```

### Running the Simulation

**Option 1: Simple Standing (Most Stable)**
```bash
python3 angad_walk.py
```
The robot stands perfectly balanced on its feet using LIPM-based balance control.

**Option 2: Progressive Walking (Recommended)**
```bash
python3 robot_walking_simulator.py
```
The robot performs a complete sequence:
1. Stands for 2 seconds
2. Crouches to prepare (3 seconds)
3. Performs dynamic walking (10+ steps)

**Option 3: Advanced Progressive Walk**
```bash
python3 angad_progressive_walk.py
```
A more refined walking controller with better gait dynamics.

## Project Structure

```
Angad_Walking_Sim/
├── XP_robot_with_actuators.xml       # Main robot model (MuJoCo format)
├── XP_robot_with_limits.xml          # Model with joint limits
├── meshes/                            # 3D mesh files for visualization
├── Angad_Full_Assembly_urdf/          # URDF format models
│
├── angad_walk.py                     # Basic standing controller
├── angad_progressive_walk.py         # Progressive stand→crouch→walk
├── robot_walking_simulator.py        # **NEW** Complete walking simulator
│
├── my_mjc_func_python.py             # MuJoCo utility functions
├── venky_supp_funcs.py               # Support functions
└── pd_control.py                     # PD control implementation
```

## How the Robot Walking Works

### 1. **Standing (LIPM-Based Balance)**
- Uses Linear Inverted Pendulum Model (LIPM) for stability
- 40% ankle strategy + 60% hip strategy for balance
- Real-world joint stiffness values (Kp=1500 Nm/rad for knees)
- Arm windmilling for dynamic recovery

### 2. **Crouching (Preparation Phase)**
- Bends knees to ~17° below horizontal
- Pitches hips and ankles to maintain upright torso
- Lowers center of mass for stable weight shifting
- Prepares springy leg action

### 3. **Dynamic Walking**
- **Weight Shift**: Leans to right/left while bending stance leg
- **Swing Phase**: Opposite leg swings forward with knee lift
- **Settle Phase**: Swing leg lands and supports weight
- **Cycle**: ~4.5 seconds per step (2s shift + 1.5s swing + 1s settle)

### 4. **Balance Recovery**
- Automatically adjusts stiffness based on tilt magnitude
- Compliant mode when heavily perturbed (prevents toppling)
- Proportional-derivative feedback on pitch and roll

## Hardware Specifications (Modeled)

| Component | Details |
|-----------|---------|
| **Total Joints** | 21 actuated |
| **Knee Actuators** | 120:1 gear ratio, Kp=1500 Nm/rad |
| **Hip Actuators** | 50:1 gear ratio, Kp=1000 Nm/rad |
| **Ankle Actuators** | 17:1 gear ratio, Kp=500 Nm/rad |
| **Foot Material** | Rubber-padded for high friction |
| **Control Method** | Real-time PD control per joint |

## Configuration Parameters

Edit `robot_walking_simulator.py` to modify:

```python
RUNNING_MODE = "walk"       # Options: "stand", "crouch", "walk"
WALKING_SPEED = 0.3         # Forward velocity (m/s)
WALKING_STEPS = 10          # Number of walking steps
SIMULATION_TIME = 30        # Total simulation time (seconds)

# Crouch parameters
CROUCH_KNEE = -0.30         # Knee bend angle (radians)
CROUCH_HIP = +0.15          # Hip pitch angle
CROUCH_ANKLE = +0.15        # Ankle pitch angle

# Walking parameters
LEAN_ANGLE = 0.035          # Body lean angle (~2°)
KNEE_LIFT = 0.10            # Swing leg knee lift
HIP_SWING = 0.03            # Hip forward swing angle
```

## Keyboard Controls (In Viewer)

- **Mouse Drag**: Rotate view
- **Scroll**: Zoom in/out
- **Space**: Play/pause simulation
- **Right Click**: Add perturbation force (test balance recovery)

## What You're Seeing

### Performance Metrics Printed Every 5 Seconds:
- **X**: Horizontal position (forward/backward motion)
- **Z**: Height above ground (should stay ~0.7-0.8m)
- **Pitch**: Forward tilt angle (should stay near 0°)
- **Roll**: Side tilt angle (should stay near 0°)
- **Jitter**: Joint velocity norm (lower is better, <0.3 is good)
- **CtrlMax**: Maximum control signal magnitude (0-1 range)

### Expected Behavior:
1. **Standing**: Perfect balance, minimal jitter (~0.1)
2. **Crouching**: Smooth descent, knee bending
3. **Walking**: Visible forward motion, alternating leg swings, balanced torso

## Troubleshooting

### Robot Falls Over
- **Cause**: Simulation may be unstable with current parameters
- **Fix**: Reduce `LEAN_ANGLE` or increase balance gains (`BAL_PITCH_KP`)

### Robot Doesn't Walk
- **Cause**: Robot may be too stiff or gains too high
- **Fix**: Ensure XML file path is correct, or reduce `CROUCH_KNEE` magnitude

### Slow Performance
- **Cause**: High mesh detail or complex contact simulation
- **Fix**: Meshes can be simplified or use decimated versions from `meshes_decimated/`

### File Not Found
- **Error**: "FileNotFoundError: XP_robot_with_actuators.xml"
- **Fix**: Ensure you're running from the `Angad_Walking_Sim` directory

## Advanced Topics

### Modifying the Gait
1. Edit walking parameters in `robot_walking_simulator.py`
2. Adjust `SHIFT_TIME`, `SWING_TIME`, `SETTLE_TIME` for different cadences
3. Modify `LEAN_ANGLE` and `KNEE_LIFT` for different step heights

### Custom Controllers
- `my_mjc_func_python.py`: Low-level MuJoCo functions
- `venky_supp_funcs.py`: Higher-level support functions
- Create your own by following the `controller(m, d)` pattern

### RL-Based Walking (Advanced)
For learning-based walking:
```bash
python3 train_walking.py        # Train with PPO
python3 play_rl_policy.py       # Test trained policy
```
Requires: PyTorch, Stable-Baselines3

## References

- **LIPM Control**: "Linear Inverted Pendulum Model" - basic bipedal balance theory
- **Walking Gaits**: Implements "Simple Capture Point" dynamics
- **MuJoCo**: Physics simulation framework - https://mujoco.org

## Performance Notes

- **Simulation Speed**: Runs at ~60 Hz (real-time)
- **Walking Cycle**: ~4.5 seconds per step in current configuration
- **Stability**: Can maintain balance indefinitely while standing
- **Perturbation Resistance**: Recovers from moderate pushes

## Next Steps

1. ✅ **Standing** - Run `angad_walk.py`
2. ✅ **Walking** - Run `robot_walking_simulator.py` with `RUNNING_MODE = "walk"`
3. 📊 **Analysis** - Modify parameters and observe effects
4. 🎯 **Optimization** - Tune gains for better walking speed/stability
5. 🤖 **Learning** - Implement RL controllers for adaptive walking

---

**Note**: This is a physics-based simulation. All actions are computed based on real joint dynamics and actuator constraints modeled from the physical Angad robot hardware.
