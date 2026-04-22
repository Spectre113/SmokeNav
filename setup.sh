#!/bin/bash
# setup.sh - Run once after cloning

# Create venv
python3 -m venv ~/ros2_venv --system-site-packages
source ~/ros2_venv/bin/activate
pip install numpy scikit-learn opencv-python

echo "Setup complete. Run: source ~/ros2_venv/bin/activate"