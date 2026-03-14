// Trajectory Panel - Waypoint management and trajectory visualization

export class TrajectoryPanel {
    constructor(options = {}) {
        this.onPathUpdate = options.onPathUpdate || (() => {});
        this.waypoints = [];
        this.plannedTrajectory = null;
        this.executionInterval = null;
        this.timingChart = null;

        this.init();
    }

    init() {
        this.setupEventListeners();
        this.initTimingChart();
    }

    setupEventListeners() {
        // Add waypoint button
        const btnAdd = document.getElementById('btn-add-waypoint');
        if (btnAdd) {
            btnAdd.addEventListener('click', () => this.addCurrentPose());
        }

        // Clear waypoints button
        const btnClear = document.getElementById('btn-clear-waypoints');
        if (btnClear) {
            btnClear.addEventListener('click', () => this.clearWaypoints());
        }

        // Plan trajectory button
        const btnPlan = document.getElementById('btn-plan-trajectory');
        if (btnPlan) {
            btnPlan.addEventListener('click', () => this.planTrajectory());
        }

        // Execute trajectory button
        const btnExecute = document.getElementById('btn-execute-trajectory');
        if (btnExecute) {
            btnExecute.addEventListener('click', () => this.executeTrajectory());
        }

        // Stop trajectory button
        const btnStop = document.getElementById('btn-stop-trajectory');
        if (btnStop) {
            btnStop.addEventListener('click', () => this.stopTrajectory());
        }
    }

    initTimingChart() {
        const ctx = document.getElementById('timing-chart');
        if (!ctx || typeof Chart === 'undefined') {
            console.warn('Chart.js not loaded or canvas not found');
            return;
        }

        // Set fixed canvas size to prevent resize loops
        const container = ctx.parentElement;
        if (container) {
            ctx.width = container.clientWidth || 400;
            ctx.height = 180;
        }

        try {
            this.timingChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Speed (m/s)',
                        data: [],
                        borderColor: '#0099ff',
                        backgroundColor: 'rgba(0, 153, 255, 0.1)',
                        tension: 0.3,
                        fill: true,
                        pointRadius: 0
                    },
                    {
                        label: 'Acceleration (m/s²)',
                        data: [],
                        borderColor: '#00cc66',
                        backgroundColor: 'rgba(0, 204, 102, 0.1)',
                        tension: 0.3,
                        fill: true,
                        pointRadius: 0
                    }
                ]
            },
            options: {
                responsive: false,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: {
                        title: {
                            display: true,
                            text: 'Time (s)',
                            color: '#a0a0a0'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#a0a0a0'
                        }
                    },
                    y: {
                        title: {
                            display: true,
                            text: 'Value',
                            color: '#a0a0a0'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#a0a0a0'
                        }
                    }
                },
                plugins: {
                    legend: {
                        labels: {
                            color: '#e8e8e8'
                        }
                    }
                }
            }
        });
        } catch (err) {
            console.error('Failed to initialize timing chart:', err);
        }
    }

    async addCurrentPose() {
        console.log('Adding current pose...');
        try {
            const response = await fetch('/api/trajectory/current-pose');
            console.log('Response status:', response.status);

            if (!response.ok) {
                console.error('Failed to get current pose:', response.status, response.statusText);
                return;
            }

            const data = await response.json();
            console.log('Current pose data:', data);

            if (!data.position) {
                console.error('No position data in response');
                return;
            }

            const waypoint = {
                position: data.position,
                orientation: data.orientation || { w: 1, x: 0, y: 0, z: 0 },
                blend_radius: 0.0,
                segment_time: 0.0,  // Auto
                pause_time: 0.0
            };

            this.waypoints.push(waypoint);
            console.log('Waypoints:', this.waypoints);
            this.renderWaypoints();
            this.updateButtons();
        } catch (err) {
            console.error('Failed to get current pose:', err);
        }
    }

    clearWaypoints() {
        this.waypoints = [];
        this.plannedTrajectory = null;
        this.renderWaypoints();
        this.updateButtons();
        this.clearVisualization();
    }

    removeWaypoint(index) {
        this.waypoints.splice(index, 1);
        this.plannedTrajectory = null;
        this.renderWaypoints();
        this.updateButtons();
    }

    updateWaypointParam(index, param, value) {
        if (this.waypoints[index]) {
            this.waypoints[index][param] = parseFloat(value) || 0;
        }
    }

    renderWaypoints() {
        const container = document.getElementById('waypoints-container');
        if (!container) return;

        container.innerHTML = '';

        this.waypoints.forEach((wp, index) => {
            const item = document.createElement('div');
            item.className = 'waypoint-item';

            const pos = wp.position;
            item.innerHTML = `
                <div class="waypoint-info">
                    <span class="waypoint-index">Waypoint ${index + 1}</span>
                    <span class="waypoint-position">
                        X: ${pos.x.toFixed(3)} Y: ${pos.y.toFixed(3)} Z: ${pos.z.toFixed(3)}
                    </span>
                </div>
                <div class="waypoint-controls">
                    <div class="waypoint-param">
                        <label>Blend (m)</label>
                        <input type="number" class="waypoint-input"
                               value="${wp.blend_radius}" step="0.01" min="0"
                               title="Blend radius in meters (0 = stop at point)"
                               onchange="window.trajectoryPanel.updateWaypointParam(${index}, 'blend_radius', this.value)">
                    </div>
                    <div class="waypoint-param">
                        <label>Time (s)</label>
                        <input type="number" class="waypoint-input"
                               value="${wp.segment_time}" step="0.1" min="0"
                               title="Time to reach this waypoint (0 = auto)"
                               onchange="window.trajectoryPanel.updateWaypointParam(${index}, 'segment_time', this.value)">
                    </div>
                    <button class="btn btn-small btn-danger"
                            onclick="window.trajectoryPanel.removeWaypoint(${index})">X</button>
                </div>
            `;

            container.appendChild(item);
        });
    }

    updateButtons() {
        const btnPlan = document.getElementById('btn-plan-trajectory');
        const btnExecute = document.getElementById('btn-execute-trajectory');

        if (btnPlan) btnPlan.disabled = this.waypoints.length < 2;
        if (btnExecute) btnExecute.disabled = !this.plannedTrajectory || !this.plannedTrajectory.valid;
    }

    async planTrajectory() {
        if (this.waypoints.length < 2) {
            alert('At least 2 waypoints required');
            return;
        }

        try {
            const response = await fetch('/api/trajectory/plan', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    waypoints: this.waypoints,
                    config: {
                        max_linear_velocity: 0.5,
                        max_joint_velocity: 1.5
                    }
                })
            });

            const data = await response.json();
            this.plannedTrajectory = data;

            // Show validation messages
            this.showValidationMessages(data.messages || []);

            // Update timing chart
            if (data.valid && data.visualization) {
                this.updateTimingChart(data.visualization.timing || []);
                this.onPathUpdate(data.visualization.path || []);
            }

            // Show trajectory status
            const trajStatus = document.getElementById('trajectory-status');
            const trajState = document.getElementById('traj-state');
            const trajDuration = document.getElementById('traj-duration');

            if (trajStatus) trajStatus.classList.remove('hidden');
            if (trajState) trajState.textContent = data.valid ? 'Ready' : 'Invalid';
            if (trajDuration) trajDuration.textContent = (data.duration || 0).toFixed(2) + 's';

            this.updateButtons();
        } catch (err) {
            console.error('Failed to plan trajectory:', err);
            alert('Failed to plan trajectory');
        }
    }

    showValidationMessages(messages) {
        const container = document.getElementById('messages-container');
        const section = document.getElementById('validation-messages');

        if (!container || !section) return;

        if (!messages || messages.length === 0) {
            section.classList.add('hidden');
            return;
        }

        section.classList.remove('hidden');
        container.innerHTML = '';

        messages.forEach(msg => {
            const div = document.createElement('div');
            div.className = `validation-message ${msg.severity}`;
            div.textContent = msg.message;
            if (msg.waypoint !== undefined) {
                div.textContent = `[WP${msg.waypoint + 1}] ${msg.message}`;
            }
            container.appendChild(div);
        });
    }

    updateTimingChart(timing) {
        if (!this.timingChart || !timing.length) return;

        this.timingChart.data.labels = timing.map(t => t.time.toFixed(2));
        this.timingChart.data.datasets[0].data = timing.map(t => t.speed);
        this.timingChart.data.datasets[1].data = timing.map(t => t.acceleration);
        this.timingChart.update();
    }

    async executeTrajectory() {
        try {
            const response = await fetch('/api/trajectory/execute', {
                method: 'POST'
            });

            const data = await response.json();

            if (data.success) {
                const trajState = document.getElementById('traj-state');
                if (trajState) trajState.textContent = 'Running';
                this.startStatusPolling();
            } else {
                console.error('Failed to execute:', data.message || 'Unknown error');
            }
        } catch (err) {
            console.error('Failed to execute trajectory:', err);
        }
    }

    async stopTrajectory() {
        try {
            await fetch('/api/trajectory/stop', { method: 'POST' });
            this.stopStatusPolling();

            const trajState = document.getElementById('traj-state');
            const trajProgress = document.getElementById('traj-progress');
            const trajProgressBar = document.getElementById('traj-progress-bar');

            if (trajState) trajState.textContent = 'Stopped';
            if (trajProgress) trajProgress.textContent = '0%';
            if (trajProgressBar) trajProgressBar.style.width = '0%';
        } catch (err) {
            console.error('Failed to stop trajectory:', err);
        }
    }

    startStatusPolling() {
        if (this.executionInterval) return;

        this.executionInterval = setInterval(async () => {
            try {
                const response = await fetch('/api/trajectory/status');
                if (!response.ok) {
                    console.error('Status request failed:', response.status);
                    return;
                }
                const status = await response.json();

                const stateEl = document.getElementById('traj-state');
                const progressEl = document.getElementById('traj-progress');
                const progressBar = document.getElementById('traj-progress-bar');

                if (stateEl) stateEl.textContent = status.state || 'unknown';
                if (progressEl) progressEl.textContent = (status.progress_percent || 0).toFixed(1) + '%';
                if (progressBar) progressBar.style.width = (status.progress_percent || 0) + '%';

                if (status.state === 'idle' || status.state === 'error') {
                    this.stopStatusPolling();
                }
            } catch (err) {
                console.error('Failed to get trajectory status:', err);
                this.stopStatusPolling();
            }
        }, 200);  // Poll every 200ms (reduced from 100ms)
    }

    stopStatusPolling() {
        if (this.executionInterval) {
            clearInterval(this.executionInterval);
            this.executionInterval = null;
        }
    }

    clearVisualization() {
        if (this.timingChart) {
            try {
                this.timingChart.data.labels = [];
                this.timingChart.data.datasets[0].data = [];
                this.timingChart.data.datasets[1].data = [];
                this.timingChart.update();
            } catch (err) {
                console.error('Failed to clear chart:', err);
            }
        }

        const trajStatus = document.getElementById('trajectory-status');
        const valMessages = document.getElementById('validation-messages');
        if (trajStatus) trajStatus.classList.add('hidden');
        if (valMessages) valMessages.classList.add('hidden');

        this.onPathUpdate([]);
    }
}
