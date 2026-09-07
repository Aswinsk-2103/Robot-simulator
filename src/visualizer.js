import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export class RobotVisualizer {
  constructor(containerElement) {
    this.container = containerElement;

    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color('#090d16');
    this.scene.fog = new THREE.FogExp2('#090d16', 0.12);

    // Camera
    this.camera = new THREE.PerspectiveCamera(
      45,
      this.container.clientWidth / this.container.clientHeight,
      0.01,
      100
    );
    this.camera.position.set(1.4, 1.2, 1.6);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    // Controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;
    this.controls.target.set(0, 0.4, 0);

    // Lighting
    this.setupLighting();

    // Scene Elements
    this.setupGridAndAxes();

    // Robot Mesh Container
    this.robotGroup = new THREE.Group();
    this.scene.add(this.robotGroup);

    // Create Arm Components
    this.buildRobotArm();

    // Target Marker for IK
    this.createTargetMarker();

    // Handle Resize
    window.addEventListener('resize', () => this.onWindowResize());
  }

  setupLighting() {
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
    this.scene.add(ambientLight);

    const dirLight1 = new THREE.DirectionalLight(0x00f2fe, 1.2);
    dirLight1.position.set(3, 5, 4);
    dirLight1.castShadow = true;
    dirLight1.shadow.mapSize.width = 2048;
    dirLight1.shadow.mapSize.height = 2048;
    this.scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0xff007f, 0.5);
    dirLight2.position.set(-3, 3, -3);
    this.scene.add(dirLight2);

    const pointLight = new THREE.PointLight(0x38ef7d, 0.8, 4);
    pointLight.position.set(0, 1.5, 0);
    this.scene.add(pointLight);
  }

  setupGridAndAxes() {
    // Ground Grid
    const gridHelper = new THREE.GridHelper(4, 40, 0x00f2fe, 0x1e293b);
    gridHelper.position.y = 0;
    this.scene.add(gridHelper);

    // Circular Base Floor Disc
    const floorGeo = new THREE.CircleGeometry(2, 64);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.8,
      metalness: 0.2,
      side: THREE.DoubleSide
    });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.001;
    floor.receiveShadow = true;
    this.scene.add(floor);

    // Base Reference Axis Helper
    const axesHelper = new THREE.AxesHelper(0.3);
    this.scene.add(axesHelper);
  }

  buildRobotArm() {
    // Materials
    const baseMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.9, roughness: 0.2 });
    const jointMat = new THREE.MeshStandardMaterial({ color: 0x00f2fe, metalness: 0.8, roughness: 0.1, emissive: 0x005f73, emissiveIntensity: 0.3 });
    const linkMat1 = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.7, roughness: 0.3 });
    const linkMat2 = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.7, roughness: 0.3 });
    const gripperMat = new THREE.MeshStandardMaterial({ color: 0x38ef7d, metalness: 0.9, roughness: 0.1, emissive: 0x10b981, emissiveIntensity: 0.2 });

    // 1. Base Pedestal
    const baseGeo = new THREE.CylinderGeometry(0.12, 0.16, 0.08, 32);
    this.baseMesh = new THREE.Mesh(baseGeo, baseMat);
    this.baseMesh.position.y = 0.04;
    this.baseMesh.castShadow = true;
    this.baseMesh.receiveShadow = true;
    this.robotGroup.add(this.baseMesh);

    // 2. Base Joint (Joint 1 yaw)
    this.joint1Group = new THREE.Group();
    this.joint1Group.position.set(0, 0.08, 0);
    this.robotGroup.add(this.joint1Group);

    const j1SphereGeo = new THREE.SphereGeometry(0.06, 32, 16);
    const j1Sphere = new THREE.Mesh(j1SphereGeo, jointMat);
    j1Sphere.castShadow = true;
    this.joint1Group.add(j1Sphere);

    // Link 1 Mesh (Length 0.3m, extending up)
    const link1Geo = new THREE.CylinderGeometry(0.04, 0.04, 0.3, 32);
    const link1Mesh = new THREE.Mesh(link1Geo, linkMat1);
    link1Mesh.position.y = 0.15; // Center of 0.3m link
    link1Mesh.castShadow = true;
    this.joint1Group.add(link1Mesh);

    // 3. Shoulder Joint (Joint 2 pitch at top of link 1)
    this.joint2Group = new THREE.Group();
    this.joint2Group.position.set(0, 0.3, 0);
    this.joint1Group.add(this.joint2Group);

    const j2Sphere = new THREE.Mesh(j1SphereGeo, jointMat);
    j2Sphere.castShadow = true;
    this.joint2Group.add(j2Sphere);

    // Link 2 Mesh (Length 0.4m, extending along X in local frame)
    const link2Geo = new THREE.CylinderGeometry(0.035, 0.035, 0.4, 32);
    const link2Mesh = new THREE.Mesh(link2Geo, linkMat2);
    link2Mesh.rotation.z = -Math.PI / 2; // Orient along +X
    link2Mesh.position.x = 0.2; // Center of 0.4m link
    link2Mesh.castShadow = true;
    this.joint2Group.add(link2Mesh);

    // 4. Elbow Joint (Joint 3 pitch at end of link 2)
    this.joint3Group = new THREE.Group();
    this.joint3Group.position.set(0.4, 0, 0);
    this.joint2Group.add(this.joint3Group);

    const j3Sphere = new THREE.Mesh(j1SphereGeo, jointMat);
    j3Sphere.castShadow = true;
    this.joint3Group.add(j3Sphere);

    // Link 3 Mesh (Length 0.3m, extending along X)
    const link3Geo = new THREE.CylinderGeometry(0.025, 0.025, 0.3, 32);
    const link3Mesh = new THREE.Mesh(link3Geo, linkMat1);
    link3Mesh.rotation.z = -Math.PI / 2;
    link3Mesh.position.x = 0.15; // Center of 0.3m link
    link3Mesh.castShadow = true;
    this.joint3Group.add(link3Mesh);

    // 5. End Effector / Gripper Assembly
    this.endEffectorGroup = new THREE.Group();
    this.endEffectorGroup.position.set(0.3, 0, 0);
    this.joint3Group.add(this.endEffectorGroup);

    // Gripper Base Plate
    const gBaseGeo = new THREE.BoxGeometry(0.03, 0.08, 0.06);
    const gBase = new THREE.Mesh(gBaseGeo, gripperMat);
    gBase.castShadow = true;
    this.endEffectorGroup.add(gBase);

    // Gripper Finger 1 & 2
    const fingerGeo = new THREE.BoxGeometry(0.05, 0.015, 0.015);
    const finger1 = new THREE.Mesh(fingerGeo, gripperMat);
    finger1.position.set(0.025, 0.025, 0);
    finger1.castShadow = true;
    this.endEffectorGroup.add(finger1);

    const finger2 = new THREE.Mesh(fingerGeo, gripperMat);
    finger2.position.set(0.025, -0.025, 0);
    finger2.castShadow = true;
    this.endEffectorGroup.add(finger2);

    // End-Effector Coordinate Frame Tip Marker
    const eeAxes = new THREE.AxesHelper(0.12);
    this.endEffectorGroup.add(eeAxes);
  }

  createTargetMarker() {
    const geo = new THREE.SphereGeometry(0.035, 16, 16);
    const mat = new THREE.MeshStandardMaterial({
      color: 0xff0055,
      wireframe: true,
      emissive: 0xff0055,
      emissiveIntensity: 0.8
    });
    this.targetMarker = new THREE.Mesh(geo, mat);
    this.targetMarker.visible = false;

    // Glowing outer ring for target marker
    const ringGeo = new THREE.RingGeometry(0.04, 0.055, 32);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0xff0055, side: THREE.DoubleSide });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = Math.PI / 2;
    this.targetMarker.add(ring);

    this.scene.add(this.targetMarker);
  }

  setTargetPosition(pos, visible = true) {
    this.targetMarker.position.set(pos[0], pos[2], -pos[1]); // Match Open3D Z-up mapping to Three.js Y-up
    this.targetMarker.visible = visible;
  }

  /**
   * Update joint visual transforms from joint angles [q1, q2, q3]
   */
  updateJointAngles(angles) {
    // Joint 1: Rotation around vertical Y-axis
    this.joint1Group.rotation.y = angles[0];

    // Joint 2: Rotation around Z-axis (shoulder pitch)
    this.joint2Group.rotation.z = angles[1];

    // Joint 3: Rotation around Z-axis (elbow pitch)
    this.joint3Group.rotation.z = angles[2];
  }

  /**
   * Get world position of End Effector tip
   */
  getEndEffectorWorldPosition() {
    const pos = new THREE.Vector3();
    this.endEffectorGroup.getWorldPosition(pos);
    // Convert back from Three.js Y-up to robotics Z-up frame:
    // Robotics X = Three.js X
    // Robotics Y = -Three.js Z
    // Robotics Z = Three.js Y
    return [pos.x, -pos.z, pos.y];
  }

  onWindowResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
