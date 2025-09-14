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

    def dh_transform(self, a, alpha, d, theta):
        """Generate DH transformation matrix"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1],
            ]
        )

    def forward_kinematics(self, theta1, theta2, theta3):
        """Compute forward kinematics for given joint angles"""
        # Convert to radians
        theta1_rad = np.radians(theta1)
        theta2_rad = np.radians(theta2)
        theta3_rad = np.radians(theta3)

        # DH parameters from Q3
        # Joint 1: a0=0, alpha0=0, d1=0, theta1
        T01 = self.dh_transform(0, 0, 0, theta1_rad)

        # Joint 2: a1=0, alpha1=90°, d2=0, theta2
        T12 = self.dh_transform(0, np.pi / 2, 0, theta2_rad)

        # Joint 3: a2=L3=10, alpha2=0, d3=0, theta3
        T23 = self.dh_transform(self.L3, 0, 0, theta3_rad)

        # Tool frame: a3=L4=10, alpha3=0, d4=0, theta4=0
        T3T = self.dh_transform(self.L4, 0, 0, 0)

        # Forward kinematics
        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)
        T0T = np.dot(T03, T3T)

        # Extract position and orientation
        position = T0T[:3, 3]
        orientation = T0T[:3, :3]

        # Joint positions for plotting
        joint_positions = []
        joint_positions.append([0, 0, 0])  # Base
        joint_positions.append(T01[:3, 3])  # Joint 1
        joint_positions.append(T02[:3, 3])  # Joint 2
        joint_positions.append(T03[:3, 3])  # Joint 3
        joint_positions.append(T0T[:3, 3])  # Tool

        return position, orientation, joint_positions, T0T

    def inverse_kinematics(self, x, y, z, phi=0):
        """Compute inverse kinematics for given end-effector position and orientation"""
        solutions = []

        # For a planar 3R manipulator (z should be 0)
        if abs(z) > 0.001:
            print(
                f"Warning: z={z} is not zero. This manipulator operates in xy-plane."
            )
            return solutions

        # Total reach
        reach = self.L3 + self.L4
        distance = np.sqrt(x**2 + y**2)

        if distance > reach or distance < abs(self.L3 - self.L4):
            print(
                f"Target position ({x}, {y}) is unreachable. Distance: {distance}, Max reach: {reach}"
            )
            return solutions

        # For each possible theta1 configuration
        for theta1_deg in [np.degrees(np.arctan2(y, x))]:  # Primary solution
            theta1_rad = np.radians(theta1_deg)

            # Local coordinates in the plane of motion
            x_local = x * np.cos(theta1_rad) + y * np.sin(theta1_rad)
            y_local = -x * np.sin(theta1_rad) + y * np.cos(theta1_rad)

            # Two-link planar inverse kinematics
            # Distance from joint 2 to end effector
            r = np.sqrt(x_local**2 + y_local**2)

            if r <= (self.L3 + self.L4) and r >= abs(self.L3 - self.L4):
                # Angle from horizontal to end effector
                beta = np.arctan2(y_local, x_local)

                # Cosine rule for theta3
                cos_theta3 = (self.L3**2 + self.L4**2 - r**2) / (
                    2 * self.L3 * self.L4
                )

                if abs(cos_theta3) <= 1:
                    # Two solutions for theta3 (elbow up and elbow down)
                    theta3_options = [
                        np.arccos(cos_theta3),
                        -np.arccos(cos_theta3),
                    ]

                    for theta3_rad in theta3_options:
                        # theta2 calculation
                        alpha = np.arctan2(
                            self.L4 * np.sin(theta3_rad),
                            self.L3 + self.L4 * np.cos(theta3_rad),
                        )
                        theta2_rad = beta - alpha

                        # Convert to degrees
                        theta1_sol = theta1_deg
                        theta2_sol = np.degrees(theta2_rad)
                        theta3_sol = np.degrees(theta3_rad)

                        # Verify solution
                        pos_check, _, _, _ = self.forward_kinematics(
                            theta1_sol, theta2_sol, theta3_sol
                        )
                        error = np.linalg.norm(pos_check - np.array([x, y, z]))

                        if error < 0.01:  # 1cm tolerance
                            solutions.append(
                                [theta1_sol, theta2_sol, theta3_sol]
                            )

        return solutions

    def plot_robot(self, joint_positions, title="Robot Configuration"):
        """Plot robot configuration"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        # Extract coordinates
        x_coords = [pos[0] for pos in joint_positions]
        y_coords = [pos[1] for pos in joint_positions]
        z_coords = [pos[2] for pos in joint_positions]

        # Plot links
        ax.plot(x_coords, y_coords, z_coords, "b-", linewidth=3, label="Links")

        # Plot joints
        ax.scatter(
            x_coords[:-1],
            y_coords[:-1],
            z_coords[:-1],
            c="red",
            s=100,
            label="Joints",
        )

        # Plot end effector
        ax.scatter(
            x_coords[-1],
            y_coords[-1],
            z_coords[-1],
            c="green",
            s=150,
            marker="^",
            label="End Effector",
        )

        # Add joint labels
        for i, (x, y, z) in enumerate(joint_positions):
            if i == 0:
                ax.text(x, y, z, "  Base", fontsize=10)
            elif i == len(joint_positions) - 1:
                ax.text(x, y, z, "  Tool", fontsize=10)
            else:
                ax.text(x, y, z, f"  J{i}", fontsize=10)

        ax.set_xlabel("X (units)")
        ax.set_ylabel("Y (units)")
        ax.set_zlabel("Z (units)")
        ax.set_title(title)
        ax.legend()
        ax.grid(True)

        # Set equal aspect ratio
        max_range = (
            max(
                max(x_coords) - min(x_coords),
                max(y_coords) - min(y_coords),
                max(z_coords) - min(z_coords),
            )
            / 2.0
        )
        mid_x = (max(x_coords) + min(x_coords)) * 0.5
        mid_y = (max(y_coords) + min(y_coords)) * 0.5
        mid_z = (max(z_coords) + min(z_coords)) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        return fig, ax


def main():
    robot = Robot3DOF()

    print("=" * 60)
    print("3-DOF Robot Manipulator Kinematics Solver")
    print("=" * 60)

    # A) Forward Kinematics - 3 test cases
    print("\nA) FORWARD KINEMATICS")
    print("-" * 40)

    forward_cases = [
        [0, 0, 0],  # All joints at zero
        [30, 45, -30],  # Mixed angles
        [90, 45, 45],  # Larger angles
    ]

    forward_results = []

    for i, angles in enumerate(forward_cases):
        theta1, theta2, theta3 = angles
        position, orientation, joint_pos, T_matrix = robot.forward_kinematics(
            theta1, theta2, theta3
        )
        forward_results.append((angles, position, joint_pos))

        print(f"\nCase {i+1}: θ₁={theta1}°, θ₂={theta2}°, θ₃={theta3}°")
        print(
            f"End effector position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]"
        )
        print(f"Distance from origin: {np.linalg.norm(position):.3f} units")

        # Plot configuration
        fig, ax = robot.plot_robot(joint_pos, f"Forward Kinematics Case {i+1}")
        plt.show()

    # B) Inverse Kinematics - 3 test cases
    print("\nB) INVERSE KINEMATICS")
    print("-" * 40)

    inverse_cases = [
        [20, 0, 0],  # Point on x-axis
        [10, 10, 0],  # Point in first quadrant
        [0, 15, 0],  # Point on y-axis
    ]

    for i, target_pos in enumerate(inverse_cases):
        x, y, z = target_pos
        print(f"\nCase {i+1}: Target position [{x}, {y}, {z}]")

        solutions = robot.inverse_kinematics(x, y, z)

        if solutions:
            print(f"Found {len(solutions)} solution(s):")
            for j, sol in enumerate(solutions):
                theta1, theta2, theta3 = sol
                print(
                    f"  Solution {j+1}: θ₁={theta1:.1f}°, θ₂={theta2:.1f}°, θ₃={theta3:.1f}°"
                )

                # Verify with forward kinematics
                pos_verify, _, joint_pos, _ = robot.forward_kinematics(
                    theta1, theta2, theta3
                )
                error = np.linalg.norm(pos_verify - np.array([x, y, z]))
                print(f"    Verification error: {error:.6f} units")

                # Plot configuration
                fig, ax = robot.plot_robot(
                    joint_pos, f"Inverse Kinematics Case {i+1}, Solution {j+1}"
                )
                plt.show()
        else:
            print("  No valid solutions found!")

    # Summary plot showing workspace
    print("\nGenerating workspace visualization...")

    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot workspace boundary
    theta_range = np.linspace(0, 2 * np.pi, 100)
    outer_radius = robot.L3 + robot.L4
    inner_radius = abs(robot.L3 - robot.L4)

    ax.plot(
        outer_radius * np.cos(theta_range),
        outer_radius * np.sin(theta_range),
        "r--",
        label=f"Max reach (R={outer_radius})",
    )
    ax.plot(
        inner_radius * np.cos(theta_range),
        inner_radius * np.sin(theta_range),
        "r:",
        label=f"Min reach (R={inner_radius})",
    )

    # Plot forward kinematics results
    for i, (angles, position, _) in enumerate(forward_results):
        ax.plot(
            position[0],
            position[1],
            "bo",
            markersize=8,
            label=f"FK Case {i+1}" if i == 0 else "",
        )
        ax.annotate(
            f"FK{i+1}",
            (position[0], position[1]),
            xytext=(5, 5),
            textcoords="offset points",
        )

    # Plot inverse kinematics targets
    for i, target in enumerate(inverse_cases):
        ax.plot(
            target[0],
            target[1],
            "gs",
            markersize=8,
            label=f"IK Target {i+1}" if i == 0 else "",
        )
        ax.annotate(
            f"IK{i+1}",
            (target[0], target[1]),
            xytext=(5, 5),
            textcoords="offset points",
        )

    ax.set_xlabel("X (units)")
    ax.set_ylabel("Y (units)")
    ax.set_title("Robot Workspace and Test Cases")
    ax.grid(True)
    ax.axis("equal")
    ax.legend()
    plt.show()

    print("\nAnalysis complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
