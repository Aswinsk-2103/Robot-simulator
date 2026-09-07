<<<<<<< HEAD
# Robot-simulator
=======
# 🤖 3-DOF Robotic Arm Simulator (WebGL & Python)

A complete, production-ready interactive 3-DOF (Degree of Freedom) Robotic Arm Simulator featuring **Forward Kinematics (FK)**, **Numerical Inverse Kinematics (IK)**, **Jacobian Matrix Inspector**, **Multi-Point Trajectory Animation**, and real-time **3D WebGL Visualization**.

![Simulator Preview](https://img.shields.io/badge/Three.js-WebGL-00f2fe?style=for-the-badge&logo=three.js)
![Python](https://img.shields.io/badge/Python-3.7+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Vite](https://img.shields.io/badge/Vite-8.2.2-646CFF?style=for-the-badge&logo=vite&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

---

## 🎯 Features

### 🧮 Kinematics Engine
- **Forward Kinematics (FK)**: Computes 6D end-effector pose $(X, Y, Z, R_x, R_y, R_z)$ from joint angles $(\theta_1, \theta_2, \theta_3)$ using Denavit-Hartenberg (DH) parameters.
- **Inverse Kinematics (IK)**: Numerical optimization solver (Damped Least Squares / SLSQP) to solve required joint angles for target Cartesian positions and orientations.
- **Workspace Reachability Checking**: Validates target positions against maximum arm reach ($R_{\max} = 1.0\text{ m}$).
- **Kinematics Inspector**: Live display of 4x4 Homogeneous Transformation Matrix ($T_{0 \to EE}$) and 6x3 Jacobian Matrix ($J(\theta)$).

### 🎨 3D WebGL Visualization & UI
- **Three.js 3D Viewport**: Real-time rendering with metallic link materials, glowing joint indicators, ground reference grid, coordinate axes, and dynamic lighting.
- **Futuristic Glassmorphism UI**: Dark mode dashboard with smooth tab navigation, real-time HUD overlays, and crisp typography.
- **Interactive Controls**:
  - **FK Mode**: Sliders and numerical inputs for Joint 1 ($\theta_1 \in [-180^\circ, 180^\circ]$), Joint 2 ($\theta_2 \in [-90^\circ, 90^\circ]$), Joint 3 ($\theta_3 \in [-90^\circ, 90^\circ]$), with instant preset poses.
  - **IK Mode**: Target Cartesian pose inputs $(X, Y, Z, R_x, R_y, R_z)$, visual target marker sphere, and solver status diagnostics.
  - **Trajectory Mode**: Animated multi-point task execution with adjustable step interpolation speed and progress bar.

---

## 📦 Installation & Setup

### Prerequisites
- **Node.js**: v18.0 or higher
- **npm**: v9.0 or higher
- **Python**: 3.7+ (optional for CLI/fallback script)

### Step 1: Install Web Application Dependencies

```bash
npm install
```

### Step 2: Run Local Dev Server

```bash
npm run dev
```

The Web Application will start locally at:
👉 **[http://localhost:5173](http://localhost:5173)**

---

## 📖 User Guide

### 1. Forward Kinematics (FK) Mode
1. Click the **FK Mode** tab in the side panel.
2. Adjust the sliders or enter numerical values for:
   - **Joint 1 ($\theta_1$)**: Base rotation around Z-axis ($\pm 180^\circ$)
   - **Joint 2 ($\theta_2$)**: Shoulder pitch ($\pm 90^\circ$)
   - **Joint 3 ($\theta_3$)**: Elbow pitch ($\pm 90^\circ$)
3. Observe the arm transform in real-time in the 3D viewport while monitoring the **End-Effector HUD** at the bottom-left.
4. Click preset buttons like **Reach Up**, **Side High**, or **Pick Pose** for instant angle configuration.

### 2. Inverse Kinematics (IK) Mode
1. Click the **IK Mode** tab.
2. Input your desired target Cartesian coordinates $(X, Y, Z)$ in meters and rotation angles $(R_x, R_y, R_z)$ in degrees.
3. Click **"⚡ Solve & Move IK"**.
   - **Green Badge**: Solution found; arm smoothly animates to target pose.
   - **Red Badge**: Target is unreachable or outside optimal workspace ($R > 1.0\text{ m}$).
4. Use target presets like **Forward Reach**, **Elevated Side**, or **Close Side** to quickly test solver convergence.

### 3. Trajectory Task Execution
1. Click the **Trajectory** tab.
2. Review the sequence of 4 automated waypoints:
   - $WP_1: (0.40, 0.00, 0.30)$ — Forward reach
   - $WP_2: (0.30, 0.20, 0.40)$ — Elevated side displacement
   - $WP_3: (0.20, 0.30, 0.30)$ — Close reach
   - $WP_4: (0.40, 0.00, 0.30)$ — Return home
3. Adjust **Animation Interpolation Steps** (20–150 steps).
4. Click **"▶ Execute Multi-Point Trajectory"** to start the continuous motion sequence.

### 4. Inspector Tab
View the live calculated:
- **$T_{0 \to EE}$ Matrix**: 4x4 homogeneous transformation matrix.
- **$J(\theta)$ Matrix**: 6x3 numerical Jacobian matrix computed via partial derivatives.

---

## 🔧 Technical Details & Kinematics Framework

### Denavit-Hartenberg (DH) Parameters

```
Link    | Length (m) | Twist (α) | Joint Range
--------|------------|-----------|---------------
Link 1  | 0.30 m     | π/2       | θ₁ ∈ [-π, π]
Link 2  | 0.40 m     | 0         | θ₂ ∈ [-π/2, π/2]
Link 3  | 0.30 m     | 0         | θ₃ ∈ [-π/2, π/2]
```

**Maximum Arm Reach**: $R_{\max} = 0.30 + 0.40 + 0.30 = 1.00\text{ m}$

### Inverse Kinematics Objective Function

The numerical solver minimizes total pose error:
$$\text{minimize } E(\theta) = \|p_{\text{current}} - p_{\text{target}}\|_2 + 0.3 \cdot \|r_{\text{current}} - r_{\text{target}}\|_2$$
subject to joint limits $\theta_{i,\min} \le \theta_i \le \theta_{i,\max}$.

---

## 📁 Repository Structure

```
.
├── index.html               # Main HTML entry & UI layout
├── style.css                # Futuristic dark glassmorphism design system
├── main.js                  # Main application controller & event wiring
├── package.json             # NPM dependencies & build scripts
├── robotic_arm_simulator.py # Python implementation & web launcher fallback
├── API_REFERENCE.md        # Detailed API & function documentation
└── src/
    ├── kinematics.js        # Pure JS DH Kinematics, IK Solver & Jacobian
    └── visualizer.js        # Three.js 3D scene setup, meshes & camera controls
```

---

## 🐙 Pushing to GitHub

To push this repository to your GitHub account:

```bash
# 1. Add remote repository (replace with your GitHub repo URL)
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git

# 2. Rename branch to main
git branch -M main

# 3. Commit and push
git add .
git commit -m "feat: complete 3-DOF Robotic Arm Simulator with WebGL UI"
git push -u origin main
```

---

## 📜 License

Distributed under the MIT License. Free for educational and commercial use.
>>>>>>> 56ff69e (feat: complete 3-DOF Robotic Arm Simulator with WebGL 3D interface, FK/IK engine, and documentation)
