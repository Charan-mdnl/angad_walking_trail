#!/usr/bin/env python3
"""
Setup Verification for Angad Robot Walking Simulation
=====================================================
This script verifies that all required dependencies and files are present.
"""

import os
import sys

def check_dependencies():
    """Check if required Python packages are installed"""
    print("=" * 60)
    print("DEPENDENCY CHECK")
    print("=" * 60)
    
    dependencies = {
        'mujoco': 'MuJoCo Physics Engine',
        'numpy': 'NumPy (Numerical Computing)',
    }
    
    all_ok = True
    for module_name, description in dependencies.items():
        try:
            __import__(module_name)
            print(f"✓ {module_name:<15} ({description})")
        except ImportError:
            print(f"✗ {module_name:<15} ({description}) - MISSING!")
            all_ok = False
    
    return all_ok

def check_files():
    """Check if required model and script files exist"""
    print("\n" + "=" * 60)
    print("FILE CHECK")
    print("=" * 60)
    
    required_files = [
        'XP_robot_with_actuators.xml',
        'robot_walking_simulator.py',
        'angad_walk.py',
        'my_mjc_func_python.py',
        'meshes',
    ]
    
    all_ok = True
    for filename in required_files:
        path = os.path.join('.', filename)
        if os.path.exists(path):
            file_type = "DIR" if os.path.isdir(path) else "FILE"
            print(f"✓ {filename:<30} ({file_type})")
        else:
            print(f"✗ {filename:<30} - MISSING!")
            all_ok = False
    
    return all_ok

def check_mujoco_model():
    """Try to load the MuJoCo model"""
    print("\n" + "=" * 60)
    print("MUJOCO MODEL LOAD TEST")
    print("=" * 60)
    
    try:
        import mujoco
        m = mujoco.MjModel.from_xml_path("XP_robot_with_actuators.xml")
        d = mujoco.MjData(m)
        
        print(f"✓ Model loaded successfully!")
        print(f"  - Total bodies: {m.nbody}")
        print(f"  - Total actuators: {m.nu}")
        print(f"  - Total joints: {m.njnt}")
        
        actuator_names = [m.actuator(i).name for i in range(m.nu)]
        print(f"  - Actuators: {', '.join(actuator_names[:5])}... (first 5)")
        
        return True
    except Exception as e:
        print(f"✗ Failed to load model: {str(e)}")
        return False

def print_quick_start():
    """Print quick start instructions"""
    print("\n" + "=" * 60)
    print("QUICK START GUIDE")
    print("=" * 60)
    
    print("""
1. BASIC STANDING (Recommended First Test):
   $ python3 angad_walk.py
   
   Shows the robot standing perfectly balanced using LIPM control.
   Should see minimal jitter and stable balance.

2. PROGRESSIVE WALKING (Full Demo):
   $ python3 robot_walking_simulator.py
   
   Demonstrates: Stand → Crouch → Walk
   Watch for forward motion and alternating leg swings.

3. ADVANCED WALKING:
   $ python3 angad_progressive_walk.py
   
   More refined walking with better gait dynamics.

4. CUSTOMIZE THE SIMULATION:
   Edit robot_walking_simulator.py and change:
   - RUNNING_MODE = "walk"      (stand/crouch/walk)
   - WALKING_SPEED = 0.3        (forward velocity)
   - WALKING_STEPS = 10         (number of steps)
   - CROUCH_KNEE = -0.30        (crouch depth)
   - LEAN_ANGLE = 0.035         (body lean)

5. VIEW DOCUMENTATION:
   See README_WALKING.md for detailed information about:
   - How walking works
   - Hardware specifications
   - Control architecture
   - Troubleshooting
""")

def main():
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 58 + "║")
    print("║" + "  ANGAD HUMANOID ROBOT - WALKING SIMULATION SETUP  ".center(58) + "║")
    print("║" + " " * 58 + "║")
    print("╚" + "=" * 58 + "╝")
    
    # Run checks
    deps_ok = check_dependencies()
    files_ok = check_files()
    model_ok = check_mujoco_model()
    
    # Print summary
    print("\n" + "=" * 60)
    print("SETUP SUMMARY")
    print("=" * 60)
    
    status = "✓ READY" if (deps_ok and files_ok and model_ok) else "✗ INCOMPLETE"
    print(f"Status: {status}")
    
    if not deps_ok:
        print("\n⚠ Missing dependencies. Install with:")
        print("  pip install mujoco numpy")
    
    if not files_ok:
        print("\n⚠ Missing files. Make sure you're in the Angad_Walking_Sim directory")
        print("  and all files were copied correctly.")
    
    if deps_ok and files_ok and model_ok:
        print("\n✓ All checks passed! You're ready to run simulations.")
        print_quick_start()
    else:
        print("\n✗ Fix the issues above and try again.")
        sys.exit(1)

if __name__ == "__main__":
    main()
