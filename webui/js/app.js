// UR Controller Web UI - Main Application

import { RobotViewer } from './robot-viewer.js';
import { WebSocketClient } from './websocket-client.js';
import { TrajectoryPanel } from './trajectory-panel.js';
import { JogController } from './jog-controller.js';

class App {
    constructor() {
        this.robotViewer = null;
        this.wsClient = null;
        this.trajectoryPanel = null;
        this.jogController = null;

        this.init();
    }

    init() {
        // Initialize 3D viewer
        const viewerContainer = document.getElementById('robot-viewer');
        this.robotViewer = new RobotViewer(viewerContainer);

        // Initialize trajectory panel
        this.trajectoryPanel = new TrajectoryPanel({
            onPathUpdate: (path) => this.updateTrajectoryPath(path)
        });
        // Expose for inline event handlers
        window.trajectoryPanel = this.trajectoryPanel;

        // Initialize WebSocket connection
        this.wsClient = new WebSocketClient({
            onStateUpdate: (state) => this.handleStateUpdate(state),
            onConnectionChange: (connected) => this.handleConnectionChange(connected)
        });

        // Initialize jog controller (needs wsClient)
        this.jogController = new JogController(this.wsClient);

        // Set up button handlers
        this.setupButtonHandlers();

        // Connect WebSocket
        this.wsClient.connect();
    }

    updateTrajectoryPath(path) {
        if (this.robotViewer && this.robotViewer.updateTrajectoryPath) {
            this.robotViewer.updateTrajectoryPath(path);
        }
    }

    setupButtonHandlers() {
        const btnStop = document.getElementById('btn-stop');
        const btnReconnect = document.getElementById('btn-reconnect');

        btnStop.addEventListener('click', () => {
            this.sendStop();
        });

        btnReconnect.addEventListener('click', () => {
            this.reconnectToRobot();
        });
    }

    async reconnectToRobot() {
        const statusEl = document.getElementById('connection-status');
        const statusText = statusEl.querySelector('.status-text');
        const btnReconnect = document.getElementById('btn-reconnect');

        // Show connecting state
        statusEl.classList.remove('connected', 'disconnected');
        statusEl.classList.add('connecting');
        statusText.textContent = 'Connecting...';
        btnReconnect.disabled = true;
        btnReconnect.textContent = 'Connecting...';

        try {
            // First try to connect
            const connectResponse = await fetch('/api/connect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({})
            });
            const connectResult = await connectResponse.json();
            console.log('Connect result:', connectResult);

            // Then try to enable control
            const enableResponse = await fetch('/api/enable-control', {
                method: 'POST'
            });
            const enableResult = await enableResponse.json();
            console.log('Enable control result:', enableResult);

            if (enableResult.success) {
                statusEl.classList.remove('connecting', 'disconnected');
                statusEl.classList.add('connected');
                statusText.textContent = 'Connected';
            } else {
                statusEl.classList.remove('connecting', 'connected');
                statusEl.classList.add('disconnected');
                statusText.textContent = 'No Control';
                console.warn('Enable control failed:', enableResult.message);
            }
        } catch (err) {
            console.error('Reconnect failed:', err);
            statusEl.classList.remove('connecting', 'connected');
            statusEl.classList.add('disconnected');
            statusText.textContent = 'Error';
        } finally {
            btnReconnect.disabled = false;
            btnReconnect.textContent = 'Reconnect';
        }
    }

    handleStateUpdate(state) {
        // Update 3D viewer
        if (this.robotViewer && state.joints) {
            this.robotViewer.updateJoints(state.joints);
        }

        // Update TCP axes in 3D viewer
        if (this.robotViewer && state.tcp_pose) {
            this.robotViewer.updateTcpPose(state.tcp_pose);
        }

        // Update joint value displays
        if (state.joints) {
            this.updateJointValues(state.joints);
        }

        // Update jog controller with TCP pose for frame transforms
        if (this.jogController && state.tcp_pose) {
            const tcpPose = [
                state.tcp_pose.x, state.tcp_pose.y, state.tcp_pose.z,
                state.tcp_pose.rx, state.tcp_pose.ry, state.tcp_pose.rz
            ];
            this.jogController.updateTcpPose(tcpPose);
        }

        // Update status display
        this.updateStatusDisplay(state);

        // Update control availability warning
        const warning = document.getElementById('control-warning');
        if (state.control_available) {
            warning.classList.add('hidden');
        } else {
            warning.classList.remove('hidden');
        }
    }

    updateJointValues(joints) {
        // Convert radians to degrees for display
        const RAD_TO_DEG = 180 / Math.PI;
        for (let i = 0; i < 6; i++) {
            const el = document.getElementById(`joint-value-${i}`);
            if (el) {
                el.textContent = (joints[i] * RAD_TO_DEG).toFixed(1) + '\u00B0';
            }
        }
    }

    updateStatusDisplay(state) {
        // Robot mode and safety
        document.getElementById('robot-mode').textContent = state.robot_mode || 'Unknown';
        document.getElementById('safety-status').textContent = state.safety_status || 'Unknown';
        document.getElementById('speed-scaling').textContent =
            ((state.speed_scaling || 0) * 100).toFixed(0) + '%';

        // TCP pose
        if (state.tcp_pose) {
            document.getElementById('tcp-x').textContent = state.tcp_pose.x.toFixed(4);
            document.getElementById('tcp-y').textContent = state.tcp_pose.y.toFixed(4);
            document.getElementById('tcp-z').textContent = state.tcp_pose.z.toFixed(4);
            document.getElementById('tcp-rx').textContent = state.tcp_pose.rx.toFixed(3);
            document.getElementById('tcp-ry').textContent = state.tcp_pose.ry.toFixed(3);
            document.getElementById('tcp-rz').textContent = state.tcp_pose.rz.toFixed(3);
        }
    }

    handleConnectionChange(connected) {
        const statusEl = document.getElementById('connection-status');
        const statusText = statusEl.querySelector('.status-text');

        if (connected) {
            statusEl.classList.remove('disconnected');
            statusEl.classList.add('connected');
            statusText.textContent = 'Connected';
        } else {
            statusEl.classList.remove('connected');
            statusEl.classList.add('disconnected');
            statusText.textContent = 'Disconnected';
        }
    }

    sendStop() {
        this.wsClient.send({
            type: 'stop'
        });
    }
}

// Start the application
window.addEventListener('DOMContentLoaded', () => {
    new App();
});
