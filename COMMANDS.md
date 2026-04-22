# ⚡ Robot Walking - Quick Command Reference

## 🚀 Fast Start (Copy & Paste)

### Test 1: Robot Standing (30 seconds)
```bash
cd /home/charan/Documents/Angad_Walking_Sim
python3 angad_walk.py
```

### Test 2: Robot Walking (Full Demo, 30 seconds)
```bash
cd /home/charan/Documents/Angad_Walking_Sim
python3 robot_walking_simulator.py
```

### Test 3: Verify Setup (5 seconds)
```bash
cd /home/charan/Documents/Angad_Walking_Sim
python3 setup_check.py
```

---

## 🎯 Configuration Quick Reference

**File to Edit:** `robot_walking_simulator.py` (lines 13-17)

### Change Robot Mode
```python
RUNNING_MODE = "walk"      # Options: "stand", "crouch", "walk"
```

### Change How Long the Robot Walks
```python
WALKING_STEPS = 10         # Number of walking steps (each ~4.5s)
SIMULATION_TIME = 30       # Total simulation time (seconds)
```

### Make Robot Walk Faster
Change these lines (around line 100):
```python
LEAN_ANGLE = 0.05          # Increase from 0.035 → more lean
HIP_SWING = 0.05           # Increase from 0.03 → more forward swing
KNEE_LIFT = 0.15           # Increase from 0.10 → higher steps
SHIFT_TIME = 1.5           # Decrease from 2.0 → faster weight shift
```

### Make Robot Walk Slower
```python
LEAN_ANGLE = 0.02          # Decrease from 0.035 → less lean
HIP_SWING = 0.015          # Decrease from 0.03 → less swing
KNEE_LIFT = 0.05           # Decrease from 0.10 → lower steps
SHIFT_TIME = 3.0           # Increase from 2.0 → slower weight shift
```

### Make Robot Crouch More Deeply
```python
CROUCH_KNEE = -0.40        # More negative = deeper crouch
CROUCH_HIP = +0.20         # Increase hip pitch
CROUCH_ANKLE = +0.20       # Increase ankle pitch
```

---

## 📊 Output Interpretation

When you run the simulation, you'll see output like this:
```
[2.5s] X=+0.031m Z=0.790m Pitch=+0.3° Roll=-0.1° Jitter=0.15 CtrlMax=0.42
[5.0s] X=+0.062m Z=0.650m Pitch=+0.2° Roll=+0.0° Jitter=0.22 CtrlMax=0.58
```

| Metric | Good Value | Problem |
|--------|-----------|---------|
| **X** | Increases >0.05m/s | Not walking if stays 0 |
| **Z** | 0.65-0.80m | <0.60m = falling |
| **Pitch** | ±2° or less | >±5° = unstable |
| **Roll** | ±2° or less | >±5° = tipping over |
| **Jitter** | <0.3 | >0.5 = too twitchy |
| **CtrlMax** | 0.3-0.7 | 1.0 = at saturation limit |

---

## 🎮 Keyboard Controls in Viewer

| Action | Keys |
|--------|------|
| **Rotate View** | Click + Drag with Mouse |
| **Zoom** | Mouse Scroll Wheel |
| **Play/Pause** | Space Bar |
| **Perturbation** | Right Click (push robot) |
| **Close** | ESC or Click X |

---

## 🔍 If Something Goes Wrong

### Robot doesn't move forward
```
1. Check: RUNNING_MODE = "walk" (not "stand")
2. Check: SIMULATION_TIME = 30 (enough time)
3. Try: Increase LEAN_ANGLE to 0.05
```

### Robot falls over
```
1. Try: LEAN_ANGLE = 0.02 (reduce lean)
2. Try: Increase BAL_PITCH_KP to 3000 (stiffer balance)
3. Try: CROUCH_KNEE = -0.20 (less deep crouch)
```

### Robot shakes/vibrates
```
1. Reduce: kp_torque values by 10-20%
2. Increase: alpha = 0.2 (more smoothing)
3. Check: No negative SIMULATION_TIME
```

### Error: "FileNotFoundError: XP_robot_with_actuators.xml"
```
1. Make sure you're in: /home/charan/Documents/Angad_Walking_Sim/
2. Run: ls *.xml (verify file exists)
3. Copy file if missing: cp ../Angad_with_limits/XP_robot_with_actuators.xml .
```

### Import Error: "No module named mujoco"
```
Install: pip3 install mujoco numpy
```

---

## 📁 File Locations

**Main Simulation:**
- `/home/charan/Documents/Angad_Walking_Sim/robot_walking_simulator.py`

**Robot Model:**
- `/home/charan/Documents/Angad_Walking_Sim/XP_robot_with_actuators.xml`

**3D Meshes:**
- `/home/charan/Documents/Angad_Walking_Sim/meshes/` (21 STL files)

**Original Files (DO NOT EDIT):**
- `/home/charan/Documents/Angad_with_limits/` (untouched)

---

## 🧪 Experiment Ideas

### Experiment 1: Find the Fastest Walking
```python
# Try different combinations:
LEAN_ANGLE = [0.03, 0.04, 0.05, 0.06]
HIP_SWING = [0.02, 0.03, 0.04, 0.05]
# Measure: How far does X position go in 30 seconds?
# Record best values
```

### Experiment 2: Maximum Balance
```python
# Can the robot stay upright during a push?
LEAN_ANGLE = 0.01  # Minimal movement
CROUCH_KNEE = -0.10  # Minimal crouch
# Right-click in viewer to push robot
# See if it recovers
```

### Experiment 3: Different Gaits
```python
# Modify step timing:
SHIFT_TIME = [1.0, 2.0, 3.0, 4.0]
SWING_TIME = [0.5, 1.0, 1.5, 2.0]
SETTLE_TIME = [0.5, 1.0, 1.5]
# What combinations feel most natural?
```

### Experiment 4: Crouch Depth
```python
# How deep can the robot crouch?
CROUCH_KNEE = [-0.10, -0.20, -0.30, -0.40, -0.50]
# What's the limit before falling?
```

---

## 📈 Performance Benchmarks

**Typical Values (Reference):**
- **Standing Jitter**: 0.10 - 0.15
- **Crouching Time**: 3-4 seconds
- **Walking Speed**: 0.05 - 0.20 m/s
- **Step Frequency**: ~0.2 steps/second (5s per step)
- **Max Tilt During Walk**: ±3-5°

**Your Hardware:**
- **21 Actuated Joints**
- **Knee Stiffness**: 1500 Nm/rad
- **Balance Gains**: Kp=2000 Nm/rad for pitch
- **Control Rate**: 60 Hz

---

## 🎓 Learning Resources (In This Folder)

1. **QUICKSTART.md** - Full detailed guide (you're here)
2. **README_WALKING.md** - Technical documentation
3. **setup_check.py** - Verification tool
4. **robot_walking_simulator.py** - Source code with comments

---

## 💡 Pro Tips

1. **Start Simple**: Run `angad_walk.py` first to verify basic stability
2. **Change One Thing**: Modify only one parameter and observe effects
3. **Record Values**: Write down configurations that work well
4. **Use Console Output**: Watch X position to verify forward motion
5. **Perturb Gently**: Small right-clicks to test balance recovery
6. **Save Configs**: Keep working parameter sets as comments in the file

---

## 🆘 Still Having Issues?

1. **Run the verification:**
   ```bash
   python3 setup_check.py
   ```

2. **Check Python version:**
   ```bash
   python3 --version  # Should be 3.8+
   ```

3. **Verify MuJoCo:**
   ```bash
   python3 -c "import mujoco; print(mujoco.__version__)"
   ```

4. **Check robot model:**
   ```bash
   python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('XP_robot_with_actuators.xml'); print(f'Loaded: {m.nbody} bodies')"
   ```

---

**Created for Angad Humanoid Robot - XP Robotics**  
**Simulation: MuJoCo Physics Engine**  
**Status: ✅ Verified and Ready to Use**
