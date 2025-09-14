#!/usr/bin/env python3
"""
Test script to verify main.py functionality without GUI
"""

import math
import sys
import os

# Add current directory to path to import main
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main import Robot3DOF

def test_robot():
    """Test the robot functionality"""
    print("Testing Robot3DOF class...")
    
    # Create robot instance
    robot = Robot3DOF()
    
    # Test robot parameters
    assert robot.L1 == 0
    assert robot.L2 == 0
    assert robot.L3 == 10
    assert robot.L4 == 10
    print("✓ Robot parameters correct")
    
    # Test forward kinematics
    x, y, z = robot.forward_kinematics(0, 0, 0)
    expected_x = 20  # L3 + L4 = 10 + 10 = 20
    expected_y = 0
    expected_z = 0
    
    assert abs(x - expected_x) < 1e-10, f"Expected x={expected_x}, got x={x}"
    assert abs(y - expected_y) < 1e-10, f"Expected y={expected_y}, got y={y}"
    assert abs(z - expected_z) < 1e-10, f"Expected z={expected_z}, got z={z}"
    print("✓ Forward kinematics test passed for home position")
    
    # Test another configuration
    x, y, z = robot.forward_kinematics(math.pi/2, 0, 0)
    expected_x = 0  # Should be 0 when theta1 = 90 degrees
    expected_y = 20  # Should be L3 + L4 = 20
    expected_z = 0
    
    assert abs(x - expected_x) < 1e-10, f"Expected x={expected_x}, got x={x}"
    assert abs(y - expected_y) < 1e-10, f"Expected y={expected_y}, got y={y}"
    assert abs(z - expected_z) < 1e-10, f"Expected z={expected_z}, got z={z}"
    print("✓ Forward kinematics test passed for 90° rotation")
    
    # Test vertical configuration
    x, y, z = robot.forward_kinematics(0, math.pi/2, 0)
    expected_x = 0  # Should be 0 when theta2 = 90 degrees (cos(π/2) = 0)
    expected_y = 0
    expected_z = 20  # Should be L3 + L4 = 20 when pointing straight up
    
    print(f"Vertical config: x={x}, y={y}, z={z}")
    assert abs(x - expected_x) < 1e-6, f"Expected x={expected_x}, got x={x}"
    assert abs(y - expected_y) < 1e-6, f"Expected y={expected_y}, got y={y}"
    assert abs(z - expected_z) < 1e-6, f"Expected z={expected_z}, got z={z}"
    print("✓ Forward kinematics test passed for vertical configuration")
    
    print("All tests passed! ✓")

if __name__ == "__main__":
    test_robot()