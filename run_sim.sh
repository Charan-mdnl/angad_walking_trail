#!/bin/bash
python3 robot_walking_simulator.py > sim_run.txt 2>&1
echo "Simulation completed at $(date)" >> sim_run.txt
