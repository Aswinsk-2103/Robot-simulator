# API Reference & Code Examples

Detailed API documentation and advanced usage examples for the 3-DOF Robotic Arm Simulator.

---

## Core Classes

### RoboticArm3DOF

Main kinematics engine for the robotic arm.

#### `__init__()`
Initialize the arm with default parameters.
```python
arm = RoboticArm3DOF()
```

**Attributes:**
- `link_lengths`: [0.3, 0.4, 0.3] m
- `joint_limits`: Angular limits per joint
- `current_angles`: Current joint configuration

---

#### `dh_matrix(theta, d, a, alpha)`

Compute Denavit-Hartenberg transformation matrix.

**Parameters:**
- `theta` (float): Joint angle in radians
- `d` (float): Link offset
- `a` (float): Link length
- `alpha` (float): Link twist

**Returns:**
- `T` (4×4 ndarray): Homogeneous transformation matrix

**Example:**
```python
import numpy as np
from robotic_arm_simulator import RoboticArm3DOF

arm = RoboticArm3DOF()

# Compute single DH matrix
T = arm.dh_matrix(
    theta=np.pi/4,   # 45 degrees
    d=0,
    a=0.3,           # 300 mm link
    alpha=0
)
print(T)
# Output: 4x4 transformation matrix
```

---

#### `forward_kinematics(angles)`

Compute forward kinematics for given joint angles.

**Parameters:**
- `angles` (np.ndarray): Joint angles [θ₁, θ₂, θ₃] in radians

**Returns:**
- `T_ee` (4×4 ndarray): End-effector pose matrix
- `transforms` (list): Intermediate transformation matrices

**Example:**
```python
# Example 1: Query end-effector pose
angles = np.array([0, np.pi/4, -np.pi/6])  # [0°, 45°, -30°]
T_ee, transforms = arm.forward_kinematics(angles)

# Extract position
ee_position = T_ee[:3, 3]
print(f"EE Position: {ee_position}")
# Output: [0.35  0.21  0.70] (approximate)

# Extract rotation matrix
rotation_matrix = T_ee[:3, :3]
print(f"Rotation Matrix:\n{rotation_matrix}")
```

---

#### `forward_kinematics_position(angles)`

Get only the end-effector Cartesian position.

**Parameters:**
- `angles` (np.ndarray): Joint angles [θ₁, θ₂, θ₃]

**Returns:**
- `position` (3 ndarray): [x, y, z] coordinates in meters

**Example:**
```python
# Quick position-only query
angles = np.array([0, 0, 0])
pos = arm.forward_kinematics_position(angles)
print(f"EE at origin config: {pos}")
# Output: [0.7  0.0  0.0] (reach = sum of link lengths)
```

---

#### `forward_kinematics_full(angles)`

Get complete end-effector pose (position + orientation).

**Parameters:**
- `angles` (np.ndarray): Joint angles [θ₁, θ₂, θ₃]

**Returns:**
- `pose` (6 ndarray): [x, y, z, rx, ry, rz] with angles in radians

**Example:**
```python
angles = np.array([np.pi/6, np.pi/4, 0])  # [30°, 45°, 0°]
pose = arm.forward_kinematics_full(angles)

print(f"Position: {pose[:3]}")
print(f"Orientation (rad): {pose[3:]}")
print(f"Orientation (deg): {np.degrees(pose[3:])}")
```

---

#### `jacobian_numerical(angles, delta=1e-6)`

Compute numerical Jacobian matrix for kinematic sensitivity analysis.

**Parameters:**
- `angles` (np.ndarray): Current joint angles
- `delta` (float): Perturbation magnitude for differentiation

**Returns:**
- `J` (6×3 ndarray): Jacobian matrix

**Example:**
```python
angles = np.array([0, np.pi/4, 0])
J = arm.jacobian_numerical(angles)

print(f"Jacobian shape: {J.shape}")  # (6, 3)

# Singular configuration check
singular_values = np.linalg.svd(J, compute_uv=False)
print(f"Singular values: {singular_values}")

if min(singular_values) < 0.01:
    print("⚠️ Near singularity!")
```

---

#### `inverse_kinematics(target_pose, initial_guess=None, max_iterations=500)`

Solve inverse kinematics using SLSQP numerical optimization.

**Parameters:**
- `target_pose` (np.ndarray): Target pose [x, y, z, rx, ry, rz] (angles in radians)
- `initial_guess` (np.ndarray, optional): Starting joint angles (default: current)
- `max_iterations` (int): Maximum solver iterations

**Returns:**
- `success` (bool): Convergence flag (error < 1 mm)
- `angles` (np.ndarray): Solved joint angles [θ₁, θ₂, θ₃]

**Example 1: Basic IK solving**
```python
# Target pose: reach forward at height
target = np.array([
    0.5,    # x = 500 mm forward
    0.0,    # y = 0 (centered)
    0.3,    # z = 300 mm high
    0, 0, 0  # No rotation
])

success, angles = arm.inverse_kinematics(target)

if success:
    print(f"✓ IK Success!")
    print(f"Joint angles (deg): {np.degrees(angles)}")
else:
    print("✗ IK Failed to converge")
```

**Example 2: IK with specific initial guess**
```python
# Start from comfortable pose
initial = np.array([0, np.pi/4, -np.pi/4])

target = np.array([0.4, 0.2, 0.4, 0, 0, 0])

success, angles = arm.inverse_kinematics(
    target,
    initial_guess=initial,
    max_iterations=1000
)

print(f"Converged in ~{success} iterations")
```

**Example 3: Iterative IK for trajectories**
```python
# Solve sequence of waypoints
waypoints = [
    [0.5, 0.0, 0.3, 0, 0, 0],
    [0.4, 0.2, 0.4, 0, 0, 0],
    [0.3, 0.3, 0.3, 0, 0, 0],
]

current_angles = np.array([0, 0, 0])

for wp in waypoints:
    success, angles = arm.inverse_kinematics(
        np.array(wp),
        initial_guess=current_angles
    )
    
    if success:
        current_angles = angles
        print(f"✓ Reached {wp[:3]}")
    else:
        print(f"✗ Failed to reach {wp[:3]}")
```

---

#### `check_reachability(position)`

Verify if a position is within arm workspace.

**Parameters:**
- `position` (np.ndarray): Cartesian position [x, y, z]

**Returns:**
- `reachable` (bool): True if within reach

**Example:**
```python
positions = [
    [0.5, 0.0, 0.3],    # Likely reachable
    [1.5, 0.0, 0.0],    # Likely unreachable (> max reach)
    [0.2, 0.2, 0.2],    # Close to base
]

for pos in positions:
    if arm.check_reachability(np.array(pos)):
        print(f"✓ {pos} is reachable")
    else:
        print(f"✗ {pos} is unreachable")
```

---

### RobotVisualizer

Open3D-based 3D visualization system.

#### `__init__(arm)`
Initialize visualizer with arm reference.
```python
visualizer = RobotVisualizer(arm)
```

---

#### `update_robot_pose(angles)`

Update all robot meshes based on joint configuration.

**Parameters:**
- `angles` (np.ndarray): Joint angles [θ₁, θ₂, θ₃]

**Returns:**
- `meshes` (dict): Dictionary of transformed Open3D meshes

**Example:**
```python
angles = np.array([np.pi/6, 0, 0])
meshes = visualizer.update_robot_pose(angles)

# Access individual links
base_mesh = meshes['base']
link1_mesh = meshes['link1']
gripper_mesh = meshes['gripper']

# Get bounding box of gripper
gripper_bbox = gripper_mesh.get_axis_aligned_bounding_box()
print(f"Gripper extent: {gripper_bbox.get_extent()}")
```

---

#### `get_scene_geometry(angles)`

Get complete scene geometry for rendering (includes grid, frame).

**Parameters:**
- `angles` (np.ndarray): Joint angles

**Returns:**
- `geometry` (list): Open3D geometry objects ready to render

**Example:**
```python
angles = np.array([0, np.pi/4, -np.pi/6])
geometry_list = visualizer.get_scene_geometry(angles)

# Render with Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()
for geom in geometry_list:
    vis.add_geometry(geom)
vis.run()
```

---

### RobotGUIApp

Interactive GUI application wrapper.

#### `__init__()`
Initialize the complete GUI application with 3D view and controls.
```python
app = RobotGUIApp()
```

---

#### `run()`
Start the GUI event loop.
```python
app.run()  # Blocks until window closed
```

**Example - Custom app initialization:**
```python
from open3d.visualization import gui

# Create custom app with modified parameters
gui.Application.instance.initialize()

custom_app = RobotGUIApp()

# Modify animation speed before running
custom_app.animation_steps_per_target = 100

custom_app.run()
```

---

## Standalone Script Examples

### Example 1: Batch FK Computation

Compute FK for joint angle sweep:

```python
import numpy as np
from robotic_arm_simulator import RoboticArm3DOF

arm = RoboticArm3DOF()

# Sweep first joint
angles_sweep = np.linspace(-np.pi, np.pi, 20)

print("θ₁ (deg) | EE X (m) | EE Y (m) | EE Z (m)")
print("-" * 45)

for theta1 in angles_sweep:
    angles = np.array([theta1, 0, 0])
    pos = arm.forward_kinematics_position(angles)
    print(f"{np.degrees(theta1):7.1f}  | {pos[0]:8.3f} | {pos[1]:8.3f} | {pos[2]:8.3f}")
```

---

### Example 2: Workspace Analysis

Generate reachable workspace envelope:

```python
import numpy as np
import matplotlib.pyplot as plt
from robotic_arm_simulator import RoboticArm3DOF
from mpl_toolkits.mplot3d import Axes3D

arm = RoboticArm3DOF()

# Sample all joint combinations
n_samples = 10
angles1 = np.linspace(-np.pi, np.pi, n_samples)
angles2 = np.linspace(-np.pi/2, np.pi/2, n_samples)
angles3 = np.linspace(-np.pi/2, np.pi/2, n_samples)

positions = []

for a1 in angles1:
    for a2 in angles2:
        for a3 in angles3:
            angles = np.array([a1, a2, a3])
            pos = arm.forward_kinematics_position(angles)
            positions.append(pos)

positions = np.array(positions)

# Plot workspace
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
          c=positions[:, 2], cmap='viridis', s=1)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3-DOF Arm Workspace Envelope')
plt.colorbar(ax.collections[0], ax=ax, label='Z (m)')
plt.show()
```

---

### Example 3: IK Convergence Analysis

Study solver performance:

```python
import numpy as np
from robotic_arm_simulator import RoboticArm3DOF
import time

arm = RoboticArm3DOF()

# Test IK on random targets
n_tests = 100
convergence_times = []
success_count = 0

print("Testing IK solver on random targets...")
print(f"{'Test':<5} | {'Success':<7} | {'Time (ms)':<10}")
print("-" * 30)

for i in range(n_tests):
    # Random target within workspace
    distance = np.random.uniform(0.1, 0.9)  # 0.1-0.9 m from base
    theta_rand = np.random.uniform(0, 2*np.pi)
    phi_rand = np.random.uniform(0, np.pi)
    
    x = distance * np.sin(phi_rand) * np.cos(theta_rand)
    y = distance * np.sin(phi_rand) * np.sin(theta_rand)
    z = distance * np.cos(phi_rand)
    
    target = np.array([x, y, z, 0, 0, 0])
    
    # Measure IK time
    start = time.time()
    success, _ = arm.inverse_kinematics(target)
    elapsed = (time.time() - start) * 1000  # ms
    
    if success:
        success_count += 1
    
    convergence_times.append(elapsed)
    
    if (i + 1) % 20 == 0:
        print(f"{i+1:<5} | {success_count:<7} | {elapsed:<10.2f}")

print("\nStatistics:")
print(f"Success Rate: {success_count/n_tests*100:.1f}%")
print(f"Avg Time: {np.mean(convergence_times):.2f} ms")
print(f"Min Time: {np.min(convergence_times):.2f} ms")
print(f"Max Time: {np.max(convergence_times):.2f} ms")
```

---

### Example 4: Trajectory Interpolation

Smooth path between waypoints:

```python
import numpy as np
from robotic_arm_simulator import RoboticArm3DOF

arm = RoboticArm3DOF()

# Define waypoints
start = np.array([0.5, 0.0, 0.3, 0, 0, 0])
mid = np.array([0.3, 0.3, 0.5, 0, 0, 0])
end = np.array([0.2, 0.0, 0.2, 0, 0, 0])

waypoints = [start, mid, end]

# Interpolate between waypoints
n_steps = 20
trajectory = []

for i in range(len(waypoints) - 1):
    wp_start = waypoints[i]
    wp_end = waypoints[i + 1]
    
    for t in np.linspace(0, 1, n_steps):
        # Linear interpolation
        interpolated = wp_start + t * (wp_end - wp_start)
        trajectory.append(interpolated)

print(f"Generated trajectory with {len(trajectory)} points")

# Solve IK for each point
joint_trajectory = []
current_guess = np.array([0, 0, 0])

for point_idx, pose in enumerate(trajectory):
    success, angles = arm.inverse_kinematics(
        pose,
        initial_guess=current_guess
    )
    
    if success:
        joint_trajectory.append(angles)
        current_guess = angles
    else:
        print(f"⚠️ IK failed at trajectory point {point_idx}")

print(f"Successfully solved {len(joint_trajectory)} poses")

# Display first 5 joint configurations
print("\nFirst 5 joint configurations:")
for i, angles in enumerate(joint_trajectory[:5]):
    print(f"Point {i}: {np.degrees(angles)} (degrees)")
```

---

### Example 5: Singularity Detection

Identify and report singular configurations:

```python
import numpy as np
from robotic_arm_simulator import RoboticArm3DOF

arm = RoboticArm3DOF()

# Test configurations
test_angles = [
    [0, 0, 0],              # Fully extended
    [0, np.pi/2, -np.pi/2], # Folded
    [np.pi/6, np.pi/4, 0],  # Random
]

print("Singularity Analysis")
print("=" * 60)

for angles in test_angles:
    angles = np.array(angles)
    
    # Compute Jacobian
    J = arm.jacobian_numerical(angles)
    
    # Singular value decomposition
    U, S, Vt = np.linalg.svd(J)
    
    min_sv = np.min(S)
    condition_number = np.max(S) / min(np.min(S), 1e-10)
    
    print(f"\nAngles: {np.degrees(angles)} (degrees)")
    print(f"Singular Values: {S}")
    print(f"Min Singular Value: {min_sv:.6f}")
    print(f"Condition Number: {condition_number:.2f}")
    
    if min_sv < 0.01:
        print("⚠️ SINGULAR or NEAR-SINGULAR configuration!")
    else:
        print("✓ Non-singular configuration")
```

---

### Example 6: Real-time Performance Profiling

Benchmark kinematics operations:

```python
import numpy as np
from robotic_arm_simulator import RoboticArm3DOF
import time

arm = RoboticArm3DOF()

# Benchmark FK
n_trials = 10000
angles = np.random.uniform([-np.pi, -np.pi/2, -np.pi/2],
                           [np.pi, np.pi/2, np.pi/2],
                           (n_trials, 3))

start = time.time()
for a in angles:
    T, _ = arm.forward_kinematics(a)
fk_time = time.time() - start

print(f"FK Performance:")
print(f"  Trials: {n_trials}")
print(f"  Total Time: {fk_time:.3f}s")
print(f"  Per Call: {fk_time/n_trials*1000:.3f}ms")
print(f"  Calls/sec: {n_trials/fk_time:.0f}")
```

---

## Advanced Customization

### Modify Arm Geometry

```python
class CustomRoboticArm(RoboticArm3DOF):
    def __init__(self):
        super().__init__()
        # Change to 6-DOF (extended example)
        self.link_lengths = [0.3, 0.3, 0.3, 0.2, 0.2, 0.1]
        self.joint_limits = [
            (-np.pi, np.pi),
            (-np.pi, np.pi),
            (-np.pi/2, np.pi/2),
            (-np.pi, np.pi),
            (-np.pi/2, np.pi/2),
            (-np.pi, np.pi),
        ]
```

### Custom FK Implementation

```python
class AnalyticalIKArm(RoboticArm3DOF):
    def inverse_kinematics_analytical(self, target):
        """
        Analytical IK solution (if available for your specific arm geometry).
        """
        # Implement your analytical solution here
        pass
```

---

## References

- **DH Convention**: Introduction to Robotics: Mechanics and Control (Craig, J.J.)
- **SLSQP**: SLSQP: Sequential Least Squares Programming (Kraft, D.)
- **Open3D**: http://www.open3d.org/docs/

---

Last updated: 2024
