/**
 * 3-DOF Robotic Arm Kinematics Engine
 * Implements Denavit-Hartenberg (DH) Forward Kinematics,
 * Numerical Inverse Kinematics (IK), Jacobian matrix computation,
 * and Workspace Reachability checking.
 */

export class RoboticArm3DOF {
  constructor() {
    // Link lengths in meters
    this.linkLengths = [0.3, 0.4, 0.3]; // L1, L2, L3 (Total max reach = 1.0 m)

    // Joint limits in radians [min, max]
    this.jointLimits = [
      [-Math.PI, Math.PI],         // Joint 1: ±180° (Base rotation around Z)
      [-Math.PI / 2, Math.PI / 2], // Joint 2: ±90° (Shoulder)
      [-Math.PI / 2, Math.PI / 2]  // Joint 3: ±90° (Elbow)
    ];

    // Current joint angles in radians [θ1, θ2, θ3]
    this.currentAngles = [0.0, 0.0, 0.0];
  }

  /**
   * Compute DH Homogeneous Transformation Matrix
   * T = [ rotX/Y/Z , pos ; 0 0 0 1 ]
   */
  dhMatrix(theta, d, a, alpha) {
    const ct = Math.cos(theta), st = Math.sin(theta);
    const ca = Math.cos(alpha), sa = Math.sin(alpha);

    return [
      [ct, -st * ca, st * sa, a * ct],
      [st, ct * ca, -ct * sa, a * st],
      [0, sa, ca, d],
      [0, 0, 0, 1]
    ];
  }

  /**
   * Multiply 4x4 matrices A x B
   */
  multiply4x4(A, B) {
    const C = Array(4).fill(0).map(() => Array(4).fill(0));
    for (let r = 0; r < 4; r++) {
      for (let c = 0; c < 4; c++) {
        C[r][c] = A[r][0] * B[0][c] +
                  A[r][1] * B[1][c] +
                  A[r][2] * B[2][c] +
                  A[r][3] * B[3][c];
      }
    }
    return C;
  }

  /**
   * Clamp joint angles within physical limits
   */
  clampAngles(angles) {
    return [
      Math.max(this.jointLimits[0][0], Math.min(this.jointLimits[0][1], angles[0])),
      Math.max(this.jointLimits[1][0], Math.min(this.jointLimits[1][1], angles[1])),
      Math.max(this.jointLimits[2][0], Math.min(this.jointLimits[2][1], angles[2]))
    ];
  }

  /**
   * Compute Forward Kinematics (FK)
   * Returns:
   *  - T0_ee: 4x4 matrix for End Effector
   *  - transforms: Array of 4x4 cumulative matrices for each joint link
   */
  forwardKinematics(angles) {
    const q = this.clampAngles(angles);

    // DH parameters for 3-DOF arm
    // Joint 1: Yaw base (rotation around Z, twist pi/2)
    const T0_1 = this.dhMatrix(q[0], 0, 0, Math.PI / 2);

    // Joint 2: Shoulder (link 1 length 0.3m)
    const T1_2 = this.dhMatrix(q[1], 0, this.linkLengths[0], 0);

    // Joint 3: Elbow (link 2 length 0.4m)
    const T2_3 = this.dhMatrix(q[2], 0, this.linkLengths[1], 0);

    // Joint 3 to End Effector (link 3 length 0.3m along X)
    const T3_ee = [
      [1, 0, 0, this.linkLengths[2]],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];

    const T0_2 = this.multiply4x4(T0_1, T1_2);
    const T0_3 = this.multiply4x4(T0_2, T2_3);
    const T0_ee = this.multiply4x4(T0_3, T3_ee);

    const eye = [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];

    return {
      T0_ee,
      transforms: [eye, T0_1, T0_2, T0_3, T0_ee]
    };
  }

  /**
   * Get 6D pose vector [x, y, z, rx, ry, rz] from joint angles
   */
  forwardKinematicsFull(angles) {
    const { T0_ee } = this.forwardKinematics(angles);
    const pos = [T0_ee[0][3], T0_ee[1][3], T0_ee[2][3]];

    // Extract Euler angles (XYZ order)
    const R = T0_ee;
    let ry = Math.asin(Math.max(-1, Math.min(1, -R[2][0])));
    let rx, rz;

    if (Math.abs(Math.cos(ry)) > 1e-6) {
      rx = Math.atan2(R[2][1], R[2][2]);
      rz = Math.atan2(R[1][0], R[0][0]);
    } else {
      rx = 0;
      rz = Math.atan2(-R[0][1], R[1][1]);
    }

    return [...pos, rx, ry, rz];
  }

  /**
   * Compute Numerical Jacobian Matrix (6x3)
   */
  jacobianNumerical(angles, delta = 1e-5) {
    const J = Array(6).fill(0).map(() => Array(3).fill(0));

    for (let i = 0; i < 3; i++) {
      const qPlus = [...angles];
      const qMinus = [...angles];
      qPlus[i] += delta;
      qMinus[i] -= delta;

      const fkPlus = this.forwardKinematicsFull(qPlus);
      const fkMinus = this.forwardKinematicsFull(qMinus);

      for (let row = 0; row < 6; row++) {
        J[row][i] = (fkPlus[row] - fkMinus[row]) / (2 * delta);
      }
    }
    return J;
  }

  /**
   * Workspace reachability check
   */
  isReachable(targetPos) {
    const dist = Math.sqrt(
      targetPos[0] * targetPos[0] +
      targetPos[1] * targetPos[1] +
      targetPos[2] * targetPos[2]
    );
    const maxReach = this.linkLengths[0] + this.linkLengths[1] + this.linkLengths[2];
    const minReach = 0.05;
    return {
      reachable: dist <= maxReach && dist >= minReach,
      distance: dist,
      maxReach
    };
  }

  /**
   * Inverse Kinematics (IK) Numerical Solver
   * Solves joint angles q for targetPose [x, y, z, rx, ry, rz]
   */
  inverseKinematics(targetPose, initialGuess = null, maxIterations = 400) {
    const q = initialGuess ? [...initialGuess] : [...this.currentAngles];
    const targetPos = targetPose.slice(0, 3);
    const targetRot = targetPose.slice(3, 6);

    const check = this.isReachable(targetPos);
    if (!check.reachable && check.distance > check.maxReach) {
      const scale = (check.maxReach - 0.01) / check.distance;
      targetPos[0] *= scale;
      targetPos[1] *= scale;
      targetPos[2] *= scale;
    }

    let currentQ = [...q];
    let stepSize = 0.5;
    let bestQ = [...currentQ];
    let bestError = Infinity;

    for (let iter = 0; iter < maxIterations; iter++) {
      const currentPose = this.forwardKinematicsFull(currentQ);
      const posError = [
        targetPos[0] - currentPose[0],
        targetPos[1] - currentPose[1],
        targetPos[2] - currentPose[2]
      ];
      const rotError = [
        targetRot[0] - currentPose[3],
        targetRot[1] - currentPose[4],
        targetRot[2] - currentPose[5]
      ];

      const errorMagnitude = Math.sqrt(
        posError[0] * posError[0] +
        posError[1] * posError[1] +
        posError[2] * posError[2]
      ) + 0.3 * Math.sqrt(
        rotError[0] * rotError[0] +
        rotError[1] * rotError[1] +
        rotError[2] * rotError[2]
      );

      if (errorMagnitude < bestError) {
        bestError = errorMagnitude;
        bestQ = [...currentQ];
      }

      if (bestError < 1e-4) {
        return {
          success: true,
          angles: this.clampAngles(bestQ),
          error: bestError,
          iterations: iter
        };
      }

      const J = this.jacobianNumerical(currentQ);
      const JTe = [0, 0, 0];
      for (let j = 0; j < 3; j++) {
        JTe[j] = J[0][j] * posError[0] + J[1][j] * posError[1] + J[2][j] * posError[2] +
                 0.3 * (J[3][j] * rotError[0] + J[4][j] * rotError[1] + J[5][j] * rotError[2]);
      }

      for (let j = 0; j < 3; j++) {
        currentQ[j] += stepSize * JTe[j];
      }
      currentQ = this.clampAngles(currentQ);
    }

    const finalSuccess = bestError < 0.05;
    return {
      success: finalSuccess,
      angles: this.clampAngles(bestQ),
      error: bestError,
      iterations: maxIterations
    };
  }
}
