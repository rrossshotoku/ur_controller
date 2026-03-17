// UR Controller Web UI - 3D Robot Viewer using URDF Loader

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import URDFLoader from './URDFLoader.js';

export class RobotViewer {
    constructor(container) {
        this.container = container;
        this.robot = null;
        this.trajectoryLine = null;
        this.waypointMarkers = [];
        this.baseAxes = null;
        this.tcpAxes = null;
        this.tcpAxesGroup = null;
        this.jointNames = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ];

        this.initScene();
        this.loadRobot();
        this.animate();
    }

    initScene() {
        // Scene setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);

        // Camera
        const aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(50, aspect, 0.01, 10);
        this.camera.position.set(1.0, 0.8, 1.0);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.shadowMap.enabled = true;
        this.container.appendChild(this.renderer.domElement);

        // Controls
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.target.set(0, 0.4, 0);
        this.controls.update();

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 10, 5);
        this.scene.add(directionalLight);

        const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.4);
        directionalLight2.position.set(-5, 5, -5);
        this.scene.add(directionalLight2);

        // Grid
        const gridHelper = new THREE.GridHelper(2, 20, 0x444444, 0x222222);
        this.scene.add(gridHelper);

        // Create labeled base frame axes
        this.baseAxes = this.createLabeledAxes(0.2, 'Base');
        this.scene.add(this.baseAxes);

        // Create TCP frame axes (will be updated based on robot pose)
        this.tcpAxesGroup = new THREE.Group();
        this.tcpAxes = this.createLabeledAxes(0.1, 'TCP');
        this.tcpAxesGroup.add(this.tcpAxes);
        this.scene.add(this.tcpAxesGroup);

        // Handle window resize
        window.addEventListener('resize', () => this.onResize());
    }

    createLabeledAxes(size, label) {
        const group = new THREE.Group();

        // Arrow parameters
        const headLength = size * 0.2;
        const headWidth = size * 0.1;

        // Robot coordinate system is Z-up, Three.js is Y-up
        // Conversion: Three.js (x, y, z) = Robot (x, z, -y)
        // So in Three.js coordinates:
        // - Robot X axis (forward) = Three.js X direction (1, 0, 0)
        // - Robot Y axis (left)    = Three.js -Z direction (0, 0, -1)
        // - Robot Z axis (up)      = Three.js Y direction (0, 1, 0)

        // Robot X axis (Red) - points in Three.js X direction
        const xDir = new THREE.Vector3(1, 0, 0);
        const xArrow = new THREE.ArrowHelper(xDir, new THREE.Vector3(0, 0, 0), size, 0xff0000, headLength, headWidth);
        group.add(xArrow);

        // Robot Y axis (Green) - points in Three.js -Z direction
        const yDir = new THREE.Vector3(0, 0, -1);
        const yArrow = new THREE.ArrowHelper(yDir, new THREE.Vector3(0, 0, 0), size, 0x00ff00, headLength, headWidth);
        group.add(yArrow);

        // Robot Z axis (Blue) - points in Three.js Y direction
        const zDir = new THREE.Vector3(0, 1, 0);
        const zArrow = new THREE.ArrowHelper(zDir, new THREE.Vector3(0, 0, 0), size, 0x0088ff, headLength, headWidth);
        group.add(zArrow);

        // Create text labels using sprites
        const labelOffset = size * 1.15;

        // X label at end of X axis (Three.js X direction)
        const xLabel = this.createTextSprite('X', 0xff0000);
        xLabel.position.set(labelOffset, 0, 0);
        xLabel.scale.set(0.1, 0.05, 1);
        group.add(xLabel);

        // Y label at end of Y axis (Three.js -Z direction)
        const yLabel = this.createTextSprite('Y', 0x00ff00);
        yLabel.position.set(0, 0, -labelOffset);
        yLabel.scale.set(0.1, 0.05, 1);
        group.add(yLabel);

        // Z label at end of Z axis (Three.js Y direction)
        const zLabel = this.createTextSprite('Z', 0x0088ff);
        zLabel.position.set(0, labelOffset, 0);
        zLabel.scale.set(0.1, 0.05, 1);
        group.add(zLabel);

        // Add frame label
        if (label) {
            const frameLabel = this.createTextSprite(label, 0xffffff);
            frameLabel.position.set(0, -size * 0.3, 0);
            frameLabel.scale.set(0.15, 0.06, 1);
            group.add(frameLabel);
        }

        return group;
    }

    createTextSprite(text, color) {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 128;
        canvas.height = 64;

        // Clear canvas
        context.fillStyle = 'transparent';
        context.fillRect(0, 0, canvas.width, canvas.height);

        // Draw text
        context.font = 'Bold 48px Arial';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillStyle = '#' + color.toString(16).padStart(6, '0');
        context.fillText(text, canvas.width / 2, canvas.height / 2);

        const texture = new THREE.CanvasTexture(canvas);
        texture.needsUpdate = true;

        const material = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false
        });

        return new THREE.Sprite(material);
    }

    loadRobot() {
        const loader = new URDFLoader();

        // Set the packages path for mesh resolution
        loader.packages = {
            '': '/urdf/'  // Root package maps to /urdf/
        };

        // Load the URDF
        loader.load(
            '/urdf/ur5e.urdf',
            (robot) => {
                console.log('Robot loaded:', robot);
                this.robot = robot;

                // Position the robot
                robot.rotation.x = -Math.PI / 2;  // URDF is Z-up, Three.js is Y-up

                this.scene.add(robot);

                // Log available joints
                console.log('Available joints:', Object.keys(robot.joints));
            },
            (progress) => {
                console.log('Loading progress:', progress);
            },
            (error) => {
                console.error('Error loading robot:', error);
            }
        );
    }

    updateJoints(jointPositions) {
        if (!this.robot || !jointPositions || jointPositions.length < 6) return;

        // Apply joint angles
        for (let i = 0; i < 6; i++) {
            const jointName = this.jointNames[i];
            if (this.robot.joints[jointName]) {
                this.robot.setJointValue(jointName, jointPositions[i]);
            }
        }
    }

    updateTcpPose(tcpPose) {
        // tcpPose: {x, y, z, rx, ry, rz} - position in meters, rotation as axis-angle
        if (!this.tcpAxesGroup || !tcpPose) return;

        // Convert from robot coordinates (Z-up) to Three.js (Y-up)
        // Robot: X=forward, Y=left, Z=up
        // Three.js: X=right, Y=up, Z=towards camera
        // Conversion: Three.js (x, y, z) = Robot (x, z, -y)
        this.tcpAxesGroup.position.set(tcpPose.x, tcpPose.z, -tcpPose.y);

        // Convert axis-angle rotation
        const rx = tcpPose.rx;
        const ry = tcpPose.ry;
        const rz = tcpPose.rz;
        const angle = Math.sqrt(rx * rx + ry * ry + rz * rz);

        if (angle > 1e-6) {
            // Normalize axis
            const ax = rx / angle;
            const ay = ry / angle;
            const az = rz / angle;

            // Convert axis from robot to Three.js coordinates
            const threeAxis = new THREE.Vector3(ax, az, -ay).normalize();

            // Create quaternion from axis-angle
            const quaternion = new THREE.Quaternion();
            quaternion.setFromAxisAngle(threeAxis, angle);
            this.tcpAxesGroup.quaternion.copy(quaternion);
        } else {
            this.tcpAxesGroup.quaternion.identity();
        }
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    onResize() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    // Fallback: simple colored spheres at joint positions
    createFallbackRobot() {
        console.log('Creating fallback robot visualization');
        const material = new THREE.MeshStandardMaterial({ color: 0x3377cc });

        // Just show a simple placeholder
        const geometry = new THREE.SphereGeometry(0.1, 16, 16);
        const sphere = new THREE.Mesh(geometry, material);
        sphere.position.y = 0.5;
        this.scene.add(sphere);

        // Add text indicating URDF failed to load
        console.warn('Robot visualization using fallback - URDF loader failed');
    }

    // Update trajectory path visualization
    updateTrajectoryPath(path) {
        // Remove existing trajectory line
        if (this.trajectoryLine) {
            this.scene.remove(this.trajectoryLine);
            this.trajectoryLine.geometry.dispose();
            this.trajectoryLine.material.dispose();
            this.trajectoryLine = null;
        }

        // Remove existing waypoint markers
        this.waypointMarkers.forEach(marker => {
            this.scene.remove(marker);
            marker.geometry.dispose();
            marker.material.dispose();
        });
        this.waypointMarkers = [];

        // If no path, we're done
        if (!path || path.length === 0) return;

        // Create path line
        const points = path.map(p => new THREE.Vector3(p.x, p.z, -p.y));  // Convert to Y-up

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({
            color: 0x00ff88,
            linewidth: 2
        });

        this.trajectoryLine = new THREE.Line(geometry, material);
        this.scene.add(this.trajectoryLine);

        // Add waypoint markers at key points
        const markerGeometry = new THREE.SphereGeometry(0.015, 16, 16);
        const markerMaterial = new THREE.MeshBasicMaterial({ color: 0x0099ff });

        // Add markers every N points
        const step = Math.max(1, Math.floor(path.length / 20));
        for (let i = 0; i < path.length; i += step) {
            const p = path[i];
            const marker = new THREE.Mesh(markerGeometry.clone(), markerMaterial.clone());
            marker.position.set(p.x, p.z, -p.y);  // Convert to Y-up
            this.scene.add(marker);
            this.waypointMarkers.push(marker);
        }

        // Always add end point marker
        if (path.length > 1) {
            const endPoint = path[path.length - 1];
            const endMarker = new THREE.Mesh(
                new THREE.SphereGeometry(0.02, 16, 16),
                new THREE.MeshBasicMaterial({ color: 0xff4444 })
            );
            endMarker.position.set(endPoint.x, endPoint.z, -endPoint.y);
            this.scene.add(endMarker);
            this.waypointMarkers.push(endMarker);
        }
    }
}
