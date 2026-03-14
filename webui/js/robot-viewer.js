// UR Controller Web UI - 3D Robot Viewer using URDF Loader

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import URDFLoader from './URDFLoader.js';

export class RobotViewer {
    constructor(container) {
        this.container = container;
        this.robot = null;
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

        // Axes helper at origin
        const axesHelper = new THREE.AxesHelper(0.15);
        this.scene.add(axesHelper);

        // Handle window resize
        window.addEventListener('resize', () => this.onResize());
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
}
