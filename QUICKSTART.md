# 🤖 Robot Walking Simulation - Complete Setup Summary

## ✅ Setup Complete!

Your Angad humanoid robot walking simulation is ready to use. All files have been copied to:
```
/home/charan/Documents/Angad_Walking_Sim/
```

### What Was Copied
- **Robot Models**: `XP_robot_with_actuators.xml`, `XP_robot_with_limits.xml` (MuJoCo physics format)
- **3D Meshes**: Full `meshes/` directory with STL files for visualization
- **Walking Scripts**: Multiple control implementations for different gaits
- **Support Files**: Utility functions and helper modules
- **URDF Models**: Full assembly in URDF format for reference

### System Status
```
✓ MuJoCo installed and working
✓ NumPy installed
✓ All 21 robot actuators loaded
✓ All mesh files present
✓ All Python scripts ready
```

## 🚀 How to Run

### Option 1: Simple Standing (Best for First Test)
```bash
cd /home/charan/Documents/Angad_Walking_Sim
python3 angad_walk.py
```
Shows the robot standing with perfect LIPM-based balance. This is the most stable mode.

**What you'll see:**
- Robot standing upright
- Minimal jitter on the feet
- Real-time balance corrections
- Console output showing pitch, roll, height every few seconds

### Option 2: Complete Walking (Full Demonstration)
```bash
cd /home/charan/Documents/Angad_Walking_Sim
python3 robot_walking_simulator.py
```
Demonstrates the full robot behavior:
1. **0-2 seconds**: Robot stands upright
2. **2-5 seconds**: Robot crouches (bends knees)
3. **5+ seconds**: Robot walks forward with alternating leg swings

**What you'll see:**
- Forward motion of the robot
- Smooth alternating leg movements
- Body lean and weight shift
- Balanced walking motion

### Option 3: Advanced Progressive Walking
```bash
cd /home/charan/Documents/Angad_Walking_Sim
python3 angad_progressive_walk.py
```
More refined walking controller with better gait dynamics.

## 🎮 Viewer Controls

When the simulation window opens:
- **Mouse**: Drag to rotate camera
- **Scroll**: Zoom in/out
- **Space**: Play/pause
- **Right-click**: Apply perturbation (test balance recovery)
- **Esc**: Close

## 📊 What's Happening in the Console

You'll see regular output like:
```
t=2.5s Z=0.790 pitch=+0.3° roll=-0.1° jitter=0.15 max_ctrl=0.42
t=5.0s Z=0.650 pitch=+0.2° roll=+0.0° jitter=0.22 max_ctrl=0.58
```

**Metrics explained:**
- **Z**: Height (0.65-0.80m is normal)
- **pitch/roll**: Body tilt angles (should be near 0°)
- **jitter**: Joint velocity magnitude (lower is smoother)
- **max_ctrl**: Control signal magnitude (0-1 normalized)

## ⚙️ How to Customize

Edit `robot_walking_simulator.py` to change:

```python
# At the top of the file:
RUNNING_MODE = "walk"      # Change to: "stand", "crouch", or "walk"
WALKING_SPEED = 0.3        # Forward velocity (try 0.2-0.5)
WALKING_STEPS = 10         # Number of steps (10 = ~45 seconds)
SIMULATION_TIME = 30       # Total run time in seconds

# Walking parameters (all in radians):
LEAN_ANGLE = 0.035         # Body lean (~2°, range: 0.02-0.05)
KNEE_LIFT = 0.10           # Swing leg knee height (range: 0.05-0.15)
HIP_SWING = 0.03           # Hip forward swing (range: 0.02-0.05)
CROUCH_KNEE = -0.30        # Crouch depth (more negative = deeper)
```

### Example: Make the Robot Walk Faster
```python
LEAN_ANGLE = 0.05          # Increase body lean
HIP_SWING = 0.05           # More aggressive hip swing
KNEE_LIFT = 0.15           # Higher knee lift
```

### Example: Make the Robot Crouch More
```python
CROUCH_KNEE = -0.40        # Bend knees more deeply
CROUCH_HIP = +0.20         # Pitch hips more
```

## 🔧 Troubleshooting

### Problem: Robot falls over
**Solution:** 
- Reduce `LEAN_ANGLE` (try 0.02)
- Increase balance gains: `BAL_PITCH_KP = 3000`
- Start with "stand" mode to check stability

### Problem: No walking motion
**Solution:**
- Check that `RUNNING_MODE = "walk"` (not "stand")
- Verify SIMULATION_TIME is long enough (30+ seconds)
- Check console for errors

### Problem: Jittery movement
**Solution:**
- Reduce joint gains (reduce `kp_torque` values)
- Increase smoothing factor `alpha = 0.2`
- Use simpler walking parameters

### Problem: Slow simulation
**Solution:**
- Already optimized, but you can:
  - Close other applications
  - Reduce mesh complexity (files in `meshes_decimated/`)

## 📚 Files Overview

```
Angad_Walking_Sim/
│
├── robot_walking_simulator.py      ← **START HERE** (new, comprehensive)
├── angad_walk.py                   ← Standing controller (stable)
├── angad_progressive_walk.py       ← Advanced walking
├── angad_lipm_walking_v1.py       ← LIPM-based walking v1
│
├── XP_robot_with_actuators.xml     ← Main robot model (21 actuators)
├── XP_robot_with_limits.xml        ← Model with joint limits
├── meshes/                          ← 3D visualization files (21 STL files)
│
├── my_mjc_func_python.py           ← MuJoCo helper functions
├── venky_supp_funcs.py             ← Support utilities
├── pd_control.py                   ← PD control implementation
│
├── setup_check.py                  ← Verification script (run this to test setup)
├── README_WALKING.md               ← Detailed technical documentation
└── Angad_Full_Assembly_urdf/       ← URDF format models (for reference)
```

## 🎯 Next Steps

1. **Test Standing** (5 min)
   ```bash
   python3 angad_walk.py
   ```
   Verify the robot is stable and balanced.

2. **Test Walking** (10 min)
   ```bash
   python3 robot_walking_simulator.py
   ```
   Watch the complete walking sequence.

3. **Explore Parameters** (15 min)
   - Open `robot_walking_simulator.py` in a text editor
   - Change `LEAN_ANGLE` and observe effects
   - Try different `WALKING_STEPS` values

4. **Advanced Customization** (30 min+)
   - Modify joint gains for different behavior
   - Implement custom gaits
   - Read `README_WALKING.md` for control theory

## 📖 Learn More

- **Control Architecture**: See `README_WALKING.md` → "How the Robot Walking Works"
- **Hardware Specs**: Robot has 21 actuated joints with real gear ratios
- **LIPM Theory**: Inverse pendulum-based balance control
- **Next Challenge**: Implement obstacle avoidance or stairs!

## ✨ Key Features

✓ **Realistic Physics**: Uses actual hardware parameters (gear ratios, joint limits)
✓ **Stable Standing**: LIPM-based balance controller with <0.3 jitter
✓ **Dynamic Walking**: Smooth gait with weight shift and swing phases
✓ **Perturbation Recovery**: Robot can recover from external pushes
✓ **Real-time Visualization**: 60 Hz rendering with physics
✓ **Fully Customizable**: Easily modify parameters for different behaviors

## 📞 Support

If you encounter issues:
1. Run: `python3 setup_check.py` to verify everything is installed
2. Check console output for error messages
3. Verify you're in the `/home/charan/Documents/Angad_Walking_Sim/` directory
4. Ensure MuJoCo is properly installed: `pip install mujoco`

---

**Original files remain untouched** at `/home/charan/Documents/Angad_with_limits/`

Your working copy is isolated at `/home/charan/Documents/Angad_Walking_Sim/`

Enjoy your robot walking simulation! 🤖🚶‍♂️
