import { RoboticArm3DOF } from './src/kinematics.js';
import { RobotVisualizer } from './src/visualizer.js';

document.addEventListener('DOMContentLoaded', () => {
  // DOM Containers
  const container = document.getElementById('canvas-container');

  // Kinematics & Visualizer Core
  const arm = new RoboticArm3DOF();
  const visualizer = new RobotVisualizer(container);

  // Application State
  let currentAngles = [0, 0, 0]; // radians [q1, q2, q3]
  let isAnimating = false;
  let animationFrameId = null;

  // FPS Counter variables
  let lastTime = performance.now();
  let frameCount = 0;
  const fpsBadge = document.getElementById('fps-badge');

  // DOM Controls - FK Sliders & Inputs
  const sliders = [
    document.getElementById('slider-j1'),
    document.getElementById('slider-j2'),
    document.getElementById('slider-j3')
  ];
  const numInputs = [
    document.getElementById('num-j1'),
    document.getElementById('num-j2'),
    document.getElementById('num-j3')
  ];
  const valDisplays = [
    document.getElementById('val-j1'),
    document.getElementById('val-j2'),
    document.getElementById('val-j3')
  ];

  // HUD Elements
  const hudPos = document.getElementById('hud-pos');
  const hudRot = document.getElementById('hud-rot');
  const hudReach = document.getElementById('hud-reach');

  // Matrix Inspector Elements
  const matrixT0ee = document.getElementById('matrix-t0ee');
  const matrixJacobian = document.getElementById('matrix-jacobian');

  // IK Solver Controls
  const ikInputs = {
    x: document.getElementById('ik-x'),
    y: document.getElementById('ik-y'),
    z: document.getElementById('ik-z'),
    rx: document.getElementById('ik-rx'),
    ry: document.getElementById('ik-ry'),
    rz: document.getElementById('ik-rz')
  };
  const btnSolveIK = document.getElementById('btn-solve-ik');
  const ikStatusDot = document.getElementById('ik-status-dot');
  const ikStatusText = document.getElementById('ik-status-text');
  const ikStatusDetails = document.getElementById('ik-status-details');

  // Trajectory Controls
  const btnExecuteTraj = document.getElementById('btn-execute-traj');
  const sliderSpeed = document.getElementById('slider-speed');
  const valSpeed = document.getElementById('val-speed');
  const trajProgressFill = document.getElementById('traj-progress-fill');
  const trajStatusLbl = document.getElementById('traj-status-lbl');
  const trajPercent = document.getElementById('traj-percent');

  // Tabs Navigation
  const tabButtons = document.querySelectorAll('.tab-btn');
  const tabPanes = document.querySelectorAll('.tab-pane');

  tabButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      const targetTab = btn.getAttribute('data-tab');
      tabButtons.forEach(b => b.classList.remove('active'));
      tabPanes.forEach(p => p.classList.remove('active'));
      btn.classList.add('active');
      document.getElementById(targetTab).classList.add('active');

      if (targetTab === 'tab-ik') {
        updateIKTargetMarkerFromInputs();
      }
    });
  });

  /**
   * Main function to sync FK angles with Visualizer, HUD, and Inspector
   */
  function updateArmState(anglesRad) {
    currentAngles = arm.clampAngles(anglesRad);
    arm.currentAngles = [...currentAngles];

    // Update 3D Visualizer Mesh
    visualizer.updateJointAngles(currentAngles);

    // Compute Kinematics Data
    const fullPose = arm.forwardKinematicsFull(currentAngles);
    const { T0_ee } = arm.forwardKinematics(currentAngles);
    const J = arm.jacobianNumerical(currentAngles);

    // Update Sliders & Inputs
    const degs = currentAngles.map(a => Math.round((a * 180) / Math.PI));
    degs.forEach((deg, i) => {
      sliders[i].value = deg;
      numInputs[i].value = deg;
      valDisplays[i].textContent = `${deg}°`;
    });

    // Update HUD
    hudPos.textContent = `X: ${fullPose[0].toFixed(3)}m | Y: ${fullPose[1].toFixed(3)}m | Z: ${fullPose[2].toFixed(3)}m`;
    const rxDeg = (fullPose[3] * 180 / Math.PI).toFixed(1);
    const ryDeg = (fullPose[4] * 180 / Math.PI).toFixed(1);
    const rzDeg = (fullPose[5] * 180 / Math.PI).toFixed(1);
    hudRot.textContent = `Rx: ${rxDeg}° | Ry: ${ryDeg}° | Rz: ${rzDeg}°`;

    const reachCheck = arm.isReachable(fullPose.slice(0, 3));
    hudReach.textContent = `R: ${reachCheck.distance.toFixed(3)}m / ${reachCheck.maxReach.toFixed(3)}m (${reachCheck.reachable ? 'Reachable' : 'Out of Reach'})`;
    hudReach.style.color = reachCheck.reachable ? 'var(--primary)' : 'var(--danger)';

    // Update Matrix Inspector
    formatMatrixDisplay(T0_ee, J);
  }

  /**
   * Format 4x4 Transformation Matrix & 6x3 Jacobian for Inspector
   */
  function formatMatrixDisplay(T, J) {
    let tStr = '[\n';
    for (let r = 0; r < 4; r++) {
      tStr += '  [' + T[r].map(v => v.toFixed(4).padStart(7, ' ')).join(', ') + ']\n';
    }
    tStr += ']';
    matrixT0ee.textContent = tStr;

    let jStr = '[\n';
    for (let r = 0; r < 6; r++) {
      jStr += '  [' + J[r].map(v => v.toFixed(4).padStart(8, ' ')).join(', ') + ']\n';
    }
    jStr += ']';
    matrixJacobian.textContent = jStr;
  }

  // FK Slider & Number Input Event Listeners
  sliders.forEach((slider, i) => {
    slider.addEventListener('input', (e) => {
      const deg = parseFloat(e.target.value);
      const angles = [...currentAngles];
      angles[i] = (deg * Math.PI) / 180;
      updateArmState(angles);
    });
  });

  numInputs.forEach((input, i) => {
    input.addEventListener('change', (e) => {
      const deg = parseFloat(e.target.value) || 0;
      const angles = [...currentAngles];
      angles[i] = (deg * Math.PI) / 180;
      updateArmState(angles);
    });
  });

  // FK Preset Buttons
  document.querySelectorAll('[data-fk]').forEach(btn => {
    btn.addEventListener('click', () => {
      const degs = btn.getAttribute('data-fk').split(',').map(Number);
      const rads = degs.map(d => (d * Math.PI) / 180);
      animateToAngles(rads, 500);
    });
  });

  /**
   * Smoothly animate joint angles to target angles
   */
  function animateToAngles(targetAngles, durationMs = 600) {
    const startAngles = [...currentAngles];
    const startTime = performance.now();
    isAnimating = true;

    function step(now) {
      const elapsed = now - startTime;
      const progress = Math.min(1, elapsed / durationMs);

      // Smooth easeInOutCubic curve
      const ease = progress < 0.5
        ? 4 * progress * progress * progress
        : 1 - Math.pow(-2 * progress + 2, 3) / 2;

      const interpAngles = startAngles.map((start, i) => start + (targetAngles[i] - start) * ease);
      updateArmState(interpAngles);

      if (progress < 1) {
        requestAnimationFrame(step);
      } else {
        isAnimating = false;
      }
    }
    requestAnimationFrame(step);
  }

  /**
   * IK Target Marker Helper
   */
  function updateIKTargetMarkerFromInputs() {
    const x = parseFloat(ikInputs.x.value) || 0;
    const y = parseFloat(ikInputs.y.value) || 0;
    const z = parseFloat(ikInputs.z.value) || 0;
    visualizer.setTargetPosition([x, y, z], true);
  }

  Object.values(ikInputs).forEach(inp => {
    inp.addEventListener('input', updateIKTargetMarkerFromInputs);
  });

  // Solve IK Button Listener
  btnSolveIK.addEventListener('click', () => {
    const x = parseFloat(ikInputs.x.value) || 0;
    const y = parseFloat(ikInputs.y.value) || 0;
    const z = parseFloat(ikInputs.z.value) || 0;
    const rx = ((parseFloat(ikInputs.rx.value) || 0) * Math.PI) / 180;
    const ry = ((parseFloat(ikInputs.ry.value) || 0) * Math.PI) / 180;
    const rz = ((parseFloat(ikInputs.rz.value) || 0) * Math.PI) / 180;

    const targetPose = [x, y, z, rx, ry, rz];
    visualizer.setTargetPosition([x, y, z], true);

    const result = arm.inverseKinematics(targetPose, currentAngles);

    if (result.success) {
      ikStatusDot.className = 'status-dot green';
      ikStatusText.textContent = 'IK Convergence Success';
      ikStatusDetails.textContent = `Solved in ${result.iterations} iterations (Error: ${result.error.toFixed(5)}m)`;
      animateToAngles(result.angles, 800);
    } else {
      ikStatusDot.className = 'status-dot red';
      ikStatusText.textContent = 'IK Solution Partial / Unreachable';
      ikStatusDetails.textContent = `Target outside optimal workspace (Min Error: ${result.error.toFixed(4)}m)`;
      animateToAngles(result.angles, 800);
    }
  });

  // IK Preset Buttons
  document.querySelectorAll('[data-ik]').forEach(btn => {
    btn.addEventListener('click', () => {
      const vals = btn.getAttribute('data-ik').split(',').map(Number);
      ikInputs.x.value = vals[0];
      ikInputs.y.value = vals[1];
      ikInputs.z.value = vals[2];
      ikInputs.rx.value = vals[3];
      ikInputs.ry.value = vals[4];
      ikInputs.rz.value = vals[5];
      btnSolveIK.click();
    });
  });

  // Animation Speed Slider
  sliderSpeed.addEventListener('input', (e) => {
    valSpeed.textContent = `${e.target.value} steps`;
  });

  // Execute Multi-Point Trajectory Listener
  btnExecuteTraj.addEventListener('click', async () => {
    if (isAnimating) return;
    isAnimating = true;

    const waypoints = [
      [0.4, 0.0, 0.3, 0, 0, 0],
      [0.3, 0.2, 0.4, 0, 0, 0],
      [0.2, 0.3, 0.3, 0, 0, 0],
      [0.4, 0.0, 0.3, 0, 0, 0]
    ];

    const stepsPerSegment = parseInt(sliderSpeed.value) || 60;
    const totalSteps = waypoints.length * stepsPerSegment;

    btnExecuteTraj.disabled = true;
    trajStatusLbl.textContent = 'Executing Trajectory...';

    for (let wpIdx = 0; wpIdx < waypoints.length; wpIdx++) {
      const targetWp = waypoints[wpIdx];
      visualizer.setTargetPosition(targetWp.slice(0, 3), true);

      // Solve IK for waypoint
      const ikResult = arm.inverseKinematics(targetWp, currentAngles);

      // Interpolate joint angles
      const startAngles = [...currentAngles];
      const endAngles = ikResult.angles;

      for (let s = 1; s <= stepsPerSegment; s++) {
        const t = s / stepsPerSegment;
        const currentProgress = ((wpIdx * stepsPerSegment + s) / totalSteps) * 100;

        const interp = startAngles.map((start, i) => start + (endAngles[i] - start) * t);
        updateArmState(interp);

        trajProgressFill.style.width = `${currentProgress}%`;
        trajPercent.textContent = `${Math.round(currentProgress)}%`;

        await new Promise(r => setTimeout(r, 16)); // ~60fps step delay
      }
    }

    trajStatusLbl.textContent = 'Completed!';
    btnExecuteTraj.disabled = false;
    isAnimating = false;
  });

  // Top Actions
  document.getElementById('btn-reset-view').addEventListener('click', () => {
    visualizer.camera.position.set(1.4, 1.2, 1.6);
    visualizer.controls.target.set(0, 0.4, 0);
  });

  document.getElementById('btn-home-pose').addEventListener('click', () => {
    animateToAngles([0, 0, 0], 600);
  });

  let markerVisible = false;
  document.getElementById('btn-toggle-target').addEventListener('click', () => {
    markerVisible = !markerVisible;
    visualizer.setTargetPosition([0.4, 0.0, 0.3], markerVisible);
  });

  // Initialize Home Pose
  updateArmState([0, 0, 0]);

  // Animation Loop
  function animate() {
    animationFrameId = requestAnimationFrame(animate);
    visualizer.render();

    // Calculate FPS
    frameCount++;
    const now = performance.now();
    if (now - lastTime >= 1000) {
      const fps = Math.round((frameCount * 1000) / (now - lastTime));
      fpsBadge.textContent = `${fps} FPS`;
      frameCount = 0;
      lastTime = now;
    }
  }

  animate();
});
