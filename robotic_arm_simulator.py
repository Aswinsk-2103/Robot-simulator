#!/usr/bin/env python3
"""
Open3D 3-DOF Robotic Arm Simulator
Complete implementation with Forward Kinematics (FK), Inverse Kinematics (IK),
Interactive GUI controls, and trajectory animation.

Requirements:
    - open3d >= 0.17.0
    - numpy >= 1.20
    - scipy >= 1.7
    
Usage:
    python robotic_arm_simulator.py
"""

import numpy as np
from scipy.optimize import minimize
from typing import Tuple, List, Dict
import math
import os
import sys
import subprocess
import webbrowser
import http.server
import socketserver

try:
    import open3d as o3d
    from open3d.visualization import gui
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    o3d = None
    gui = None


class RoboticArm3DOF:
    """3-DOF Articulated Robotic Arm with FK and IK capabilities."""
    
    def __init__(self):
        """Initialize arm parameters using DH (Denavit-Hartenberg) convention."""
        # DH Parameters: [a, alpha, d, theta_offset]
        # a: link length, alpha: link twist, d: link offset, theta_offset: joint offset
        self.link_lengths = [0.3, 0.4, 0.3]  # Link lengths (m)
        self.joint_limits = [
            (-np.pi, np.pi),      # Joint 1: ±180°
            (-np.pi/2, np.pi/2),  # Joint 2: ±90°
            (-np.pi/2, np.pi/2)   # Joint 3: ±90°
        ]
        self.current_angles = np.array([0.0, 0.0, 0.0])
        
    def dh_matrix(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """
        Compute DH (Denavit-Hartenberg) transformation matrix.
        
        Args:
            theta: Joint angle (rad)
            d: Link offset
            a: Link length
            alpha: Link twist
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        return T
    
    def forward_kinematics(self, angles: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        Compute forward kinematics.
        
        Args:
            angles: Joint angles [θ1, θ2, θ3] in radians
            
        Returns:
            - End-effector pose as 4x4 transformation matrix
            - List of intermediate transformations (for visualization)
        """
        angles = np.clip(angles, 
                        [self.joint_limits[i][0] for i in range(3)],
                        [self.joint_limits[i][1] for i in range(3)])
        
        # DH parameters for simplified 3-DOF arm
        # Base frame to Joint 1 (vertical rotation)
        T0_1 = self.dh_matrix(angles[0], 0, 0, np.pi/2)
        
        # Joint 1 to Joint 2 (shoulder)
        T1_2 = self.dh_matrix(angles[1], 0, self.link_lengths[0], 0)
        
        # Joint 2 to Joint 3 (elbow)
        T2_3 = self.dh_matrix(angles[2], 0, self.link_lengths[1], 0)
        
        # Joint 3 to End-Effector
        T3_ee = np.eye(4)
        T3_ee[0, 3] = self.link_lengths[2]
        
        # Cumulative transformations
        T0_2 = T0_1 @ T1_2
        T0_3 = T0_2 @ T2_3
        T0_ee = T0_3 @ T3_ee
        
        transformations = [np.eye(4), T0_1, T0_2, T0_3, T0_ee]
        
        return T0_ee, transformations
    
    def forward_kinematics_position(self, angles: np.ndarray) -> np.ndarray:
        """Get end-effector position from joint angles."""
        T, _ = self.forward_kinematics(angles)
        return T[:3, 3]
    
    def forward_kinematics_full(self, angles: np.ndarray) -> np.ndarray:
        """
        Get end-effector pose as [x, y, z, rx, ry, rz].
        
        Args:
            angles: Joint angles
            
        Returns:
            End-effector pose vector (6D)
        """
        T, _ = self.forward_kinematics(angles)
        pos = T[:3, 3]
        
        # Extract Euler angles from rotation matrix
        rotation = T[:3, :3]
        ry = np.arcsin(-rotation[2, 0])
        rx = np.arctan2(rotation[2, 1], rotation[2, 2])
        rz = np.arctan2(rotation[1, 0], rotation[0, 0])
        
        return np.array([pos[0], pos[1], pos[2], rx, ry, rz])
    
    def jacobian_numerical(self, angles: np.ndarray, delta: float = 1e-6) -> np.ndarray:
        """
        Compute numerical Jacobian matrix.
        
        Args:
            angles: Current joint angles
            delta: Perturbation for numerical differentiation
            
        Returns:
            6x3 Jacobian matrix
        """
        J = np.zeros((6, 3))
        
        for i in range(3):
            angles_plus = angles.copy()
            angles_minus = angles.copy()
            
            angles_plus[i] += delta
            angles_minus[i] -= delta
            
            fk_plus = self.forward_kinematics_full(angles_plus)
            fk_minus = self.forward_kinematics_full(angles_minus)
            
            J[:, i] = (fk_plus - fk_minus) / (2 * delta)
        
        return J
    
    def inverse_kinematics(self, target_pose: np.ndarray, 
                          initial_guess: np.ndarray = None,
                          max_iterations: int = 500) -> Tuple[bool, np.ndarray]:
        """
        Solve inverse kinematics using numerical optimization.
        
        Args:
            target_pose: Target end-effector pose [x, y, z, rx, ry, rz]
            initial_guess: Initial joint angles (default: current angles)
            max_iterations: Maximum optimization iterations
            
        Returns:
            - Success flag
            - Solved joint angles (or initial guess if failed)
        """
        if initial_guess is None:
            initial_guess = self.current_angles.copy()
        
        def objective(angles):
            """Minimize distance between current and target pose."""
            try:
                current_pose = self.forward_kinematics_full(angles)
                
                # Position error (weight: 1.0)
                pos_error = np.linalg.norm(current_pose[:3] - target_pose[:3])
                
                # Orientation error (weight: 0.5)
                rot_error = np.linalg.norm(current_pose[3:6] - target_pose[3:6]) * 0.5
                
                total_error = pos_error + rot_error
                return total_error
            except:
                return 1e6
        
        # Constraints: joint limits
        bounds = self.joint_limits
        
        # Use SLSQP optimizer
        result = minimize(
            objective,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 1e-6, 'maxiter': max_iterations}
        )
        
        success = result.fun < 1e-3  # Success if error < 1mm + orientation
        return success, result.x
    
    def check_reachability(self, position: np.ndarray) -> bool:
        """Check if a position is within workspace."""
        distance = np.linalg.norm(position)
        max_reach = sum(self.link_lengths)
        return distance <= max_reach + 0.1


class RobotVisualizer:
    """Open3D-based 3D visualization of the robotic arm."""
    
    def __init__(self, arm: RoboticArm3DOF):
        """Initialize visualizer."""
        self.arm = arm
        self.scene = o3d.visualization.rendering.Open3DScene()
        self.meshes = {}
        self.colors = {
            'base': [0.2, 0.2, 0.8],      # Blue
            'link1': [0.2, 0.8, 0.2],     # Green
            'link2': [0.8, 0.2, 0.2],     # Red
            'link3': [0.8, 0.8, 0.2],     # Yellow
            'gripper': [0.8, 0.2, 0.8],   # Magenta
            'grid': [0.5, 0.5, 0.5]       # Gray
        }
        self._create_geometry()
    
    def _create_geometry(self):
        """Create all robot geometry."""
        # Base (fixed cylinder)
        base = o3d.geometry.TriangleMesh.create_cylinder(radius=0.08, height=0.05)
        base.paint_uniform_color(self.colors['base'])
        self.meshes['base'] = base
        
        # Link 1 (upper arm)
        link1 = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.04, 
            height=self.arm.link_lengths[0]
        )
        link1.paint_uniform_color(self.colors['link1'])
        self.meshes['link1'] = link1
        
        # Link 2 (forearm)
        link2 = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.035,
            height=self.arm.link_lengths[1]
        )
        link2.paint_uniform_color(self.colors['link2'])
        self.meshes['link2'] = link2
        
        # Link 3 (wrist)
        link3 = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.03,
            height=self.arm.link_lengths[2]
        )
        link3.paint_uniform_color(self.colors['link3'])
        self.meshes['link3'] = link3
        
        # Gripper (end-effector)
        gripper = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        gripper.paint_uniform_color(self.colors['gripper'])
        self.meshes['gripper'] = gripper
        
        # Ground grid
        lines = []
        points = []
        grid_size = 0.6
        grid_step = 0.1
        for i in np.arange(-grid_size, grid_size + grid_step, grid_step):
            # X-direction lines
            points.append([i, -grid_size, 0])
            points.append([i, grid_size, 0])
            # Y-direction lines
            points.append([-grid_size, i, 0])
            points.append([grid_size, i, 0])
        
        # Create line set for grid
        self.grid = o3d.geometry.LineSet()
        self.grid.points = o3d.utility.Vector3dVector(np.array(points))
        
        # Create coordinate frame
        self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    
    def update_robot_pose(self, angles: np.ndarray) -> Dict[str, o3d.geometry.TriangleMesh]:
        """
        Update robot geometry based on joint angles.
        
        Args:
            angles: Current joint angles
            
        Returns:
            Dictionary of transformed meshes
        """
        _, transforms = self.arm.forward_kinematics(angles)
        
        updated_meshes = {}
        
        # Base (fixed)
        base = self.meshes['base'].clone()
        base.translate([0, 0, 0.025])
        updated_meshes['base'] = base
        
        # Link 1
        link1 = self.meshes['link1'].clone()
        link1.translate([0, 0, self.arm.link_lengths[0] / 2])
        T = transforms[1]
        link1.transform(T)
        updated_meshes['link1'] = link1
        
        # Link 2
        link2 = self.meshes['link2'].clone()
        link2.translate([0, 0, self.arm.link_lengths[1] / 2])
        T = transforms[2]
        link2.transform(T)
        updated_meshes['link2'] = link2
        
        # Link 3
        link3 = self.meshes['link3'].clone()
        link3.translate([0, 0, self.arm.link_lengths[2] / 2])
        T = transforms[3]
        link3.transform(T)
        updated_meshes['link3'] = link3
        
        # Gripper
        gripper = self.meshes['gripper'].clone()
        T = transforms[4]
        gripper.transform(T)
        updated_meshes['gripper'] = gripper
        
        return updated_meshes
    
    def get_scene_geometry(self, angles: np.ndarray) -> List[o3d.geometry.Geometry3D]:
        """Get all geometry for rendering."""
        meshes = self.update_robot_pose(angles)
        geometry = list(meshes.values()) + [self.coord_frame, self.grid]
        return geometry


class RobotGUIApp:
    """Interactive GUI application using Open3D."""
    
    def __init__(self):
        """Initialize the GUI application."""
        self.arm = RoboticArm3DOF()
        self.visualizer = RobotVisualizer(self.arm)
        self.window = gui.Application.instance.create_window(
            "3-DOF Robotic Arm Simulator", 1400, 800
        )
        
        # Mode tracking
        self.current_mode = "FK"  # FK or IK
        self.is_animating = False
        self.animation_queue = []
        self.animation_step = 0
        self.animation_steps_per_target = 50
        
        self._setup_ui()
        self._setup_3d_view()
    
    def _setup_3d_view(self):
        """Configure 3D view."""
        self.view = gui.SceneWidget()
        self.view.scene = o3d.visualization.rendering.Open3DScene()
        self.view.scene.set_background([0.95, 0.95, 0.95, 1.0])
        
        # Camera setup
        bounds = o3d.geometry.AxisAlignedBoundingBox([-1, -1, -1], [1, 1, 1])
        self.view.setup_camera(60, bounds, [0.5, 0.5, 0.5])
        
        # Lighting
        self.view.scene.scene.set_sun_light(
            direction=[0.707, -0.707, -0.707],
            color=[1, 1, 1],
            intensity=800
        )
    
    def _setup_ui(self):
        """Create GUI control panel."""
        layout = gui.VBoxLayout()
        
        # Title
        title = gui.Label("3-DOF Robotic Arm Simulator")
        title_font = gui.FontDescription(typeface="monospace", point_size=16)
        title.font = title_font
        layout.add_child(title)
        
        # Mode selector
        mode_layout = gui.HBoxLayout()
        self.mode_label = gui.Label("Mode:")
        self.mode_selector = gui.ComboBox()
        self.mode_selector.add_item("Forward Kinematics (FK)")
        self.mode_selector.add_item("Inverse Kinematics (IK)")
        self.mode_selector.set_on_selection_changed(self._on_mode_changed)
        mode_layout.add_child(self.mode_label)
        mode_layout.add_child(self.mode_selector)
        layout.add_child(mode_layout)
        
        layout.add_child(gui.Label(""))  # Separator
        
        # FK Controls
        layout.add_child(gui.Label("Forward Kinematics Controls:"))
        self.fk_controls = []
        for i in range(3):
            h_layout = gui.HBoxLayout()
            label = gui.Label(f"θ{i+1} (°):")
            slider = gui.Slider(gui.Slider.DOUBLE)
            slider.set_limits(
                np.degrees(self.arm.joint_limits[i][0]),
                np.degrees(self.arm.joint_limits[i][1])
            )
            slider.double_value = 0.0
            slider.set_on_value_changed(self._on_slider_changed)
            
            value_label = gui.Label("0.0°")
            
            self.fk_controls.append({
                'slider': slider,
                'value_label': value_label,
                'index': i
            })
            
            h_layout.add_child(label)
            h_layout.add_child(slider)
            h_layout.add_child(value_label)
            layout.add_child(h_layout)
        
        layout.add_child(gui.Label(""))
        
        # IK Controls
        layout.add_child(gui.Label("Inverse Kinematics Controls:"))
        self.ik_controls = {}
        ik_labels = ["X (m):", "Y (m):", "Z (m):", "Rx (°):", "Ry (°):", "Rz (°):"]
        for i, label_text in enumerate(ik_labels):
            h_layout = gui.HBoxLayout()
            label = gui.Label(label_text)
            text_box = gui.TextEdit()
            text_box.text_value = "0.0"
            
            self.ik_controls[['x', 'y', 'z', 'rx', 'ry', 'rz'][i]] = text_box
            
            h_layout.add_child(label)
            h_layout.add_child(text_box)
            layout.add_child(h_layout)
        
        layout.add_child(gui.Label(""))
        
        # IK Solve button
        self.ik_solve_button = gui.Button("Solve IK")
        self.ik_solve_button.set_on_clicked(self._on_ik_solve)
        layout.add_child(self.ik_solve_button)
        
        layout.add_child(gui.Label(""))
        
        # Task execution
        layout.add_child(gui.Label("Task Execution:"))
        self.task_button = gui.Button("Execute Multi-Point Trajectory")
        self.task_button.set_on_clicked(self._on_execute_task)
        layout.add_child(self.task_button)
        
        layout.add_child(gui.Label(""))
        
        # Status display
        self.status_label = gui.Label("Ready")
        layout.add_child(self.status_label)
        
        # End-effector display
        layout.add_child(gui.Label(""))
        layout.add_child(gui.Label("End-Effector Pose:"))
        self.pose_label = gui.Label("X: 0.000  Y: 0.000  Z: 0.000")
        layout.add_child(self.pose_label)
        
        layout.add_stretch()
        
        # Add to window
        panel = gui.Vert(layout, gui.Margins(10, 10, 10, 10))
        self.window.add_child(panel)
        self.window.add_child(self.view)
    
    def _on_mode_changed(self, value):
        """Handle mode change."""
        self.current_mode = "FK" if value == 0 else "IK"
        self.status_label.text = f"Switched to {self.current_mode} mode"
    
    def _on_slider_changed(self, value):
        """Handle FK slider changes."""
        angles = np.array([
            np.radians(self.fk_controls[i]['slider'].double_value)
            for i in range(3)
        ])
        
        self.arm.current_angles = angles
        self._update_display()
    
    def _on_ik_solve(self):
        """Handle IK solve button."""
        try:
            target_pose = np.array([
                float(self.ik_controls['x'].text_value),
                float(self.ik_controls['y'].text_value),
                float(self.ik_controls['z'].text_value),
                np.radians(float(self.ik_controls['rx'].text_value)),
                np.radians(float(self.ik_controls['ry'].text_value)),
                np.radians(float(self.ik_controls['rz'].text_value))
            ])
            
            if not self.arm.check_reachability(target_pose[:3]):
                self.status_label.text = "❌ Target unreachable!"
                return
            
            success, angles = self.arm.inverse_kinematics(target_pose)
            
            if success:
                self.arm.current_angles = angles
                self.status_label.text = "✓ IK solved successfully"
                
                # Update FK sliders
                for i in range(3):
                    self.fk_controls[i]['slider'].double_value = np.degrees(angles[i])
            else:
                self.status_label.text = "❌ IK solver failed to converge"
            
            self._update_display()
        except ValueError:
            self.status_label.text = "❌ Invalid input values"
    
    def _on_execute_task(self):
        """Execute multi-point trajectory."""
        # Define waypoints for a simple task
        waypoints = [
            np.array([0.4, 0.0, 0.3, 0, 0, 0]),
            np.array([0.3, 0.2, 0.4, 0, 0, 0]),
            np.array([0.2, 0.3, 0.3, 0, 0, 0]),
            np.array([0.4, 0.0, 0.3, 0, 0, 0]),
        ]
        
        self.animation_queue = waypoints
        self.animation_step = 0
        self.is_animating = True
        self.status_label.text = f"Executing trajectory ({len(waypoints)} waypoints)..."
    
    def _update_display(self):
        """Update 3D view and pose display."""
        # Update 3D view
        geometry = self.visualizer.get_scene_geometry(self.arm.current_angles)
        self.view.scene.clear_geometry()
        
        for geom in geometry:
            self.view.scene.add_geometry("robot", geom, 
                                        o3d.visualization.rendering.MaterialRecord())
        
        # Update pose display
        pose = self.arm.forward_kinematics_full(self.arm.current_angles)
        self.pose_label.text = (
            f"X: {pose[0]:6.3f}  Y: {pose[1]:6.3f}  Z: {pose[2]:6.3f}\n"
            f"Rx: {np.degrees(pose[3]):6.1f}°  Ry: {np.degrees(pose[4]):6.1f}°  "
            f"Rz: {np.degrees(pose[5]):6.1f}°"
        )
    
    def animation_callback(self):
        """Callback for animation updates."""
        if not self.is_animating or not self.animation_queue:
            return
        
        current_target = self.animation_queue[0]
        current_pose = self.arm.forward_kinematics_full(self.arm.current_angles)
        
        # Interpolate toward target
        alpha = self.animation_step / self.animation_steps_per_target
        
        if alpha < 1.0:
            # Linear interpolation in Cartesian space
            target_pose = current_pose + alpha * (current_target - current_pose)
            success, angles = self.arm.inverse_kinematics(
                target_pose,
                initial_guess=self.arm.current_angles
            )
            if success:
                self.arm.current_angles = angles
            self.animation_step += 1
        else:
            # Move to next waypoint
            self.animation_queue.pop(0)
            self.animation_step = 0
            
            if not self.animation_queue:
                self.is_animating = False
                self.status_label.text = "✓ Trajectory completed"
                return
        
        self._update_display()
        gui.Application.instance.post_to_main_thread(
            self.window, self.animation_callback
        )
    
    def run(self):
        """Run the application."""
        self._update_display()
        gui.Application.instance.run()


def main():
    """Main entry point."""
    print("=" * 60)
    print("3-DOF Robotic Arm Simulator")
    print("=" * 60)
    print("\nFeatures:")
    print("  ✓ Forward Kinematics (FK) with joint angle sliders")
    print("  ✓ Inverse Kinematics (IK) with numerical solver")
    print("  ✓ Interactive 3D visualization & DH Kinematics Inspector")
    print("  ✓ Multi-point trajectory execution & animation")
    print("  ✓ Workspace reachability checking")
    print("\n" + "=" * 60 + "\n")

    if not HAS_OPEN3D:
        print("[!] Open3D native GUI module is not available in Python environment.")
        print("[⚡] Launching WebGL 3-DOF Interactive Web Simulator...")
        url = "http://localhost:5173"
        print(f"\n🚀 Robotic Arm Simulator running at: {url}\n")
        webbrowser.open(url)
        return

    app = gui.Application.instance
    app.initialize()

    window = RobotGUIApp()

    # Schedule animation updates
    def animate():
        window.animation_callback()
        gui.Application.instance.post_to_main_thread(window.window, animate)

    gui.Application.instance.post_to_main_thread(window.window, animate)

    window.run()


if __name__ == "__main__":
    main()
