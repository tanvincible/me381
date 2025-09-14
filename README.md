# ME381 - 3DOF Robot Kinematics

This repository contains a Python implementation of a 3-degree-of-freedom (3DOF) robot with forward kinematics calculations and 3D visualization capabilities.

## 🤖 Robot Description

The robot is configured with the following parameters:
- **L1 = 0** (First link length)
- **L2 = 0** (Second link length)  
- **L3 = 10** (Third link length)
- **L4 = 10** (Fourth link length)

This creates a robot where the first two joints are rotational without adding link length, and the last two links extend the robot's reach.

## 📋 Prerequisites

### System Requirements
- Python 3.7 or higher
- pip (Python package installer)

### Dependencies
- `numpy` - For mathematical calculations
- `matplotlib` - For 3D plotting and visualization

## 🚀 Installation & Setup

### 1. Clone the Repository
```bash
git clone https://github.com/tanvincible/me381.git
cd me381
```

### 2. Install Dependencies

#### Option A: Using requirements.txt (Recommended)
```bash
pip install -r requirements.txt
```

#### Option B: Manual Installation
```bash
pip install numpy matplotlib
```

#### Option C: Using conda
```bash
conda install numpy matplotlib
```

## 🏃‍♂️ Running the Code

### Basic Usage

To run the main demonstration:
```bash
python main.py
```

This will:
1. Create a `Robot3DOF` instance
2. Display robot parameters
3. Calculate forward kinematics for 4 different configurations
4. Show end effector positions for each configuration
5. Generate 3D plots of the robot (if display is available)

### Expected Output
```
3DOF Robot Demonstration
========================================
Robot parameters:
L1 = 0
L2 = 0
L3 = 10
L4 = 10

Configuration 1:
  Joint angles: θ1=0.0°, θ2=0.0°, θ3=0.0°
  End effector position: x=20.00, y=0.00, z=0.00

Configuration 2:
  Joint angles: θ1=45.0°, θ2=30.0°, θ3=0.0°
  End effector position: x=12.25, y=12.25, z=10.00

...
```

### Running Tests

To verify the implementation is working correctly:
```bash
python test_main.py
```

Expected test output:
```
Testing Robot3DOF class...
✓ Robot parameters correct
✓ Forward kinematics test passed for home position
✓ Forward kinematics test passed for 90° rotation
✓ Forward kinematics test passed for vertical configuration
All tests passed! ✓
```

## 📖 Code Structure

### `main.py`
The main file containing:

#### `Robot3DOF` Class
- `__init__()`: Initialize robot with link parameters
- `forward_kinematics(theta1, theta2, theta3)`: Calculate end effector position
- `plot_robot(theta1, theta2, theta3)`: Generate 3D visualization

#### `main()` Function
- Demonstrates robot functionality with various configurations
- Outputs joint angles and corresponding end effector positions

### Key Functions

#### Forward Kinematics
```python
x, y, z = robot.forward_kinematics(theta1, theta2, theta3)
```
- **Input**: Joint angles in radians (θ1, θ2, θ3)
- **Output**: End effector position (x, y, z) in Cartesian coordinates

#### 3D Visualization
```python
robot.plot_robot(theta1, theta2, theta3)
```
- **Input**: Joint angles in radians
- **Output**: Interactive 3D plot showing robot configuration

## 🧮 Mathematical Background

### Forward Kinematics Equations
```
x = L3 * cos(θ1) * cos(θ2) + L4 * cos(θ1) * cos(θ2 + θ3)
y = L3 * sin(θ1) * cos(θ2) + L4 * sin(θ1) * cos(θ2 + θ3)  
z = L3 * sin(θ2) + L4 * sin(θ2 + θ3)
```

Where:
- θ1: Base rotation (around Z-axis)
- θ2: Shoulder elevation (around Y-axis)
- θ3: Elbow joint angle

## 🎯 Example Configurations

| Configuration | θ1 (deg) | θ2 (deg) | θ3 (deg) | End Position |
|---------------|----------|----------|----------|--------------|
| Home          | 0        | 0        | 0        | (20, 0, 0)   |
| 45° Base      | 45       | 30       | 0        | (12.25, 12.25, 10) |
| Vertical      | 90       | 45       | 60       | (0, 4.48, 16.73) |
| Negative      | -60      | -30      | 45       | (9.16, -15.87, -2.41) |

## 🛠 Customization

### Modifying Robot Parameters
To change the robot configuration, edit the `__init__` method in the `Robot3DOF` class:

```python
def __init__(self):
    self.L1 = 0    # Change first link length
    self.L2 = 0    # Change second link length  
    self.L3 = 10   # Change third link length
    self.L4 = 10   # Change fourth link length
```

### Adding New Configurations
Add new test cases to the `configurations` list in `main()`:

```python
configurations = [
    (0, 0, 0),                    # Home position
    (math.pi/4, math.pi/6, 0),    # Your new configuration
    # Add more configurations here
]
```

## 🐛 Troubleshooting

### Common Issues

#### Import Errors
```
ModuleNotFoundError: No module named 'numpy'
```
**Solution**: Install dependencies using `pip install -r requirements.txt`

#### Display Issues (Headless Environment)
If running on a server without display, comment out the plotting lines:
```python
# robot.plot_robot(theta1, theta2, theta3)
```

#### Permission Errors
```
Permission denied: pip install
```
**Solution**: Use `pip install --user -r requirements.txt` for user-level installation

### Platform-Specific Notes

#### Linux/WSL
```bash
# May need to install tkinter for matplotlib
sudo apt-get install python3-tk
```

#### macOS
```bash
# Using Homebrew
brew install python-tk
```

#### Windows
- Ensure Python is added to PATH
- Use Command Prompt or PowerShell
- Consider using Anaconda for easier package management

## 📚 Learning Resources

### Robotics Concepts
- Forward Kinematics: Converting joint angles to end effector position
- Coordinate Frames: Understanding X, Y, Z positioning
- Joint Types: Revolute vs Prismatic joints

### Python Libraries Used
- **NumPy**: [Documentation](https://numpy.org/doc/)
- **Matplotlib**: [Documentation](https://matplotlib.org/stable/)
- **mpl_toolkits.mplot3d**: [3D Plotting Guide](https://matplotlib.org/stable/gallery/mplot3d/index.html)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## 📄 License

This project is part of ME381 coursework. Please respect academic integrity policies.

## 📞 Support

For questions or issues:
1. Check the troubleshooting section above
2. Review the test file (`test_main.py`) for usage examples
3. Open an issue on GitHub with detailed error messages

---

**Happy Robotics! 🤖**