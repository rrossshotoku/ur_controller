// UR Controller Web UI - Main Application

import { RobotViewer } from './robot-viewer.js';
import { ControlPanel } from './control-panel.js';
import { WebSocketClient } from './websocket-client.js';
import { TrajectoryPanel } from './trajectory-panel.js';

class App {
    constructor() {
        this.robotViewer = null;
        this.controlPanel = null;
        this.wsClient = null;
        this.trajectoryPanel = null;
        this.controlEnabled = false;

        this.init();
    }

    init() {
        // Initialize 3D viewer
        const viewerContainer = document.getElementById('robot-viewer');
        this.robotViewer = new RobotViewer(viewerContainer);

        // Initialize control panel
        this.controlPanel = new ControlPanel({
            onServoJ: (joints) => this.sendServoJ(joints),
            onStop: () => this.sendStop(),
            onSnapToCurrent: () => this.snapToCurrent()
        });

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
        const btnEnable = document.getElementById('btn-enable');
        const btnStop = document.getElementById('btn-stop');
        const btnSnap = document.getElementById('btn-snap');
        const btnReconnect = document.getElementById('btn-reconnect');

        btnEnable.addEventListener('click', () => {
            this.controlEnabled = !this.controlEnabled;
            btnEnable.textContent = this.controlEnabled ? 'Disable Control' : 'Enable Control';
            btnEnable.classList.toggle('btn-danger', this.controlEnabled);
            btnEnable.classList.toggle('btn-primary', !this.controlEnabled);
            this.controlPanel.setEnabled(this.controlEnabled);
        });

        btnStop.addEventListener('click', () => {
            this.sendStop();
        });

        btnSnap.addEventListener('click', () => {
            this.snapToCurrent();
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

        // Update control panel (actual values)
        if (this.controlPanel && state.joints) {
            this.controlPanel.updateActualPositions(state.joints);
        }

        // Update status display
        this.updateStatusDisplay(state);

        // Update control availability
        const btnEnable = document.getElementById('btn-enable');
        const warning = document.getElementById('control-warning');

        if (state.control_available) {
            btnEnable.disabled = false;
            warning.classList.add('hidden');
        } else {
            btnEnable.disabled = true;
            this.controlEnabled = false;
            btnEnable.textContent = 'Enable Control';
            btnEnable.classList.remove('btn-danger');
            btnEnable.classList.add('btn-primary');
            this.controlPanel.setEnabled(false);
            warning.classList.remove('hidden');
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

    sendServoJ(joints) {
        if (!this.controlEnabled) return;

        this.wsClient.send({
            type: 'servoj',
            joints: joints
        });
    }

    sendStop() {
        this.wsClient.send({
            type: 'stop'
        });
    }

    snapToCurrent() {
        this.controlPanel.snapToCurrent();
    }
}

// Start the application
window.addEventListener('DOMContentLoaded', () => {
    new App();
});
