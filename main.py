import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math


class Robot3DOF:
    def __init__(self):
        # Link lengths from Q3 (L1=L2=0, L3=L4=10)
        self.L1 = 0
        self.L2 = 0
        self.L3 = 10
        self.L4 = 10
        
    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Calculate forward kinematics for the 3DOF robot
        
        Args:
            theta1, theta2, theta3: Joint angles in radians
            
        Returns:
            End effector position (x, y, z)
        """
        # Convert angles to radians if needed
        t1, t2, t3 = theta1, theta2, theta3
        
        # Forward kinematics calculations
        x = self.L3 * math.cos(t1) * math.cos(t2) + self.L4 * math.cos(t1) * math.cos(t2 + t3)
        y = self.L3 * math.sin(t1) * math.cos(t2) + self.L4 * math.sin(t1) * math.cos(t2 + t3)
        z = self.L3 * math.sin(t2) + self.L4 * math.sin(t2 + t3)
        
        return x, y, z
    
    def plot_robot(self, theta1, theta2, theta3):
        """
        Plot the 3DOF robot in 3D space
        
        Args:
            theta1, theta2, theta3: Joint angles in radians
        """
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Calculate joint positions
        # Base (origin)
        x0, y0, z0 = 0, 0, 0
        
        # Joint 1 (after first rotation)
        x1, y1, z1 = 0, 0, 0  # Since L1 = 0
        
        # Joint 2 (after second rotation)
        x2, y2, z2 = 0, 0, 0  # Since L2 = 0
        
        # Joint 3 (after third link)
        x3 = self.L3 * math.cos(theta1) * math.cos(theta2)
        y3 = self.L3 * math.sin(theta1) * math.cos(theta2)
        z3 = self.L3 * math.sin(theta2)
        
        # End effector
        x4, y4, z4 = self.forward_kinematics(theta1, theta2, theta3)
        
        # Plot links
        ax.plot([x0, x1], [y0, y1], [z0, z1], 'b-', linewidth=3, label='Link 1')
        ax.plot([x1, x2], [y1, y2], [z1, z2], 'g-', linewidth=3, label='Link 2')
        ax.plot([x2, x3], [y2, y3], [z2, z3], 'r-', linewidth=3, label='Link 3')
        ax.plot([x3, x4], [y3, y4], [z3, z4], 'm-', linewidth=3, label='Link 4')
        
        # Plot joints
        ax.scatter([x0, x1, x2, x3, x4], [y0, y1, y2, y3, y4], [z0, z1, z2, z3, z4], 
                  c=['black', 'blue', 'green', 'red', 'magenta'], s=100, alpha=0.8)
        
        # Set equal aspect ratio and labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        ax.set_title(f'3DOF Robot Configuration\nθ1={math.degrees(theta1):.1f}°, θ2={math.degrees(theta2):.1f}°, θ3={math.degrees(theta3):.1f}°')
        
        # Set axis limits
        max_reach = self.L3 + self.L4
        ax.set_xlim([-max_reach, max_reach])
        ax.set_ylim([-max_reach, max_reach])
        ax.set_zlim([0, max_reach])
        
        plt.show()


def main():
    """
    Main function to demonstrate the 3DOF robot
    """
    # Create robot instance
    robot = Robot3DOF()
    
    # Example 1: Basic configuration
    print("3DOF Robot Demonstration")
    print("=" * 40)
    print(f"Robot parameters:")
    print(f"L1 = {robot.L1}")
    print(f"L2 = {robot.L2}")
    print(f"L3 = {robot.L3}")
    print(f"L4 = {robot.L4}")
    print()
    
    # Test different configurations
    configurations = [
        (0, 0, 0),                    # Home position
        (math.pi/4, math.pi/6, 0),    # 45°, 30°, 0°
        (math.pi/2, math.pi/4, math.pi/3),  # 90°, 45°, 60°
        (-math.pi/3, -math.pi/6, math.pi/4), # -60°, -30°, 45°
    ]
    
    for i, (theta1, theta2, theta3) in enumerate(configurations):
        print(f"Configuration {i+1}:")
        print(f"  Joint angles: θ1={math.degrees(theta1):.1f}°, θ2={math.degrees(theta2):.1f}°, θ3={math.degrees(theta3):.1f}°")
        
        # Calculate forward kinematics
        x, y, z = robot.forward_kinematics(theta1, theta2, theta3)
        print(f"  End effector position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        print()
        
        # Plot the robot (comment out if running in headless environment)
        try:
            robot.plot_robot(theta1, theta2, theta3)
        except Exception as e:
            print(f"  Could not display plot (headless environment): {e}")
            print(f"  Robot configuration {i+1} would be plotted here")


if __name__ == "__main__":
    main()