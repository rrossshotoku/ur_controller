// Trajectory Panel - Setup Poses and Sequences management

export class TrajectoryPanel {
    constructor(options = {}) {
        this.onPathUpdate = options.onPathUpdate || (() => {});
        this.onWaypointsUpdate = options.onWaypointsUpdate || (() => {});
        this.elements = [];  // Array of {type: 'setup_pose'|'sequence', data: ...}
        this.plannedTrajectory = null;
        this.executionInterval = null;
        this.timingChart = null;
        this.kinematics = null;  // Will be set when we get FK data

        this.init();
    }

    init() {
        this.setupEventListeners();
        this.initTimingChart();
    }

    setupEventListeners() {
        // Add Setup Pose button
        const btnAddSetup = document.getElementById('btn-add-setup-pose');
        if (btnAddSetup) {
            btnAddSetup.addEventListener('click', () => this.addSetupPose());
        }

        // Add Sequence Waypoint button
        const btnAddSeqWp = document.getElementById('btn-add-sequence-wp');
        if (btnAddSeqWp) {
            btnAddSeqWp.addEventListener('click', () => this.addSequenceWaypoint());
        }

        // Clear all button
        const btnClear = document.getElementById('btn-clear-waypoints');
        if (btnClear) {
            btnClear.addEventListener('click', () => this.clearAll());
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

        // IK method toggle buttons
        this.setupIKMethodToggle();
    }

    setupIKMethodToggle() {
        const btnAnalytical = document.getElementById('btn-ik-analytical');
        const btnNumerical = document.getElementById('btn-ik-numerical');

        if (btnAnalytical) {
            btnAnalytical.addEventListener('click', () => this.setIKMethod('analytical'));
        }
        if (btnNumerical) {
            btnNumerical.addEventListener('click', () => this.setIKMethod('numerical'));
        }

        // Load current IK method on startup
        this.loadCurrentIKMethod();
    }

    async loadCurrentIKMethod() {
        try {
            const response = await fetch('/api/ik-method');
            const data = await response.json();
            this.updateIKMethodButtons(data.method);
        } catch (error) {
            console.warn('Failed to load IK method:', error);
        }
    }

    async setIKMethod(method) {
        try {
            const response = await fetch('/api/ik-method', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ method })
            });

            const data = await response.json();
            if (data.success) {
                this.updateIKMethodButtons(data.method);
                console.log('IK method set to:', data.method);
            } else {
                console.error('Failed to set IK method:', data.message);
            }
        } catch (error) {
            console.error('Error setting IK method:', error);
        }
    }

    updateIKMethodButtons(method) {
        const btnAnalytical = document.getElementById('btn-ik-analytical');
        const btnNumerical = document.getElementById('btn-ik-numerical');

        if (btnAnalytical) {
            btnAnalytical.classList.toggle('active', method === 'analytical');
        }
        if (btnNumerical) {
            btnNumerical.classList.toggle('active', method === 'numerical');
        }
    }

    initTimingChart() {
        const ctx = document.getElementById('timing-chart');
        if (!ctx || typeof Chart === 'undefined') {
            console.warn('Chart.js not loaded or canvas not found');
            return;
        }

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
                            title: { display: true, text: 'Time (s)', color: '#a0a0a0' },
                            grid: { color: 'rgba(255, 255, 255, 0.1)' },
                            ticks: { color: '#a0a0a0' }
                        },
                        y: {
                            title: { display: true, text: 'Value', color: '#a0a0a0' },
                            grid: { color: 'rgba(255, 255, 255, 0.1)' },
                            ticks: { color: '#a0a0a0' }
                        }
                    },
                    plugins: {
                        legend: { labels: { color: '#e8e8e8' } }
                    }
                }
            });
        } catch (err) {
            console.error('Failed to initialize timing chart:', err);
        }
    }

    // Add a Setup Pose (captures current joint positions)
    async addSetupPose() {
        try {
            const response = await fetch('/api/trajectory/current-pose');
            if (!response.ok) {
                console.error('Failed to get current pose');
                return;
            }

            const data = await response.json();
            if (!data.joints) {
                console.error('No joint data in response');
                return;
            }

            const setupPose = {
                type: 'setup_pose',
                name: `Setup ${this.elements.length + 1}`,
                joints: data.joints,
                position: data.position,  // Also capture TCP position for visualization
                move_time: 0  // Auto
            };

            this.elements.push(setupPose);
            this.renderElements();
            this.updateButtons();
            this.notifyWaypointsChanged();
        } catch (err) {
            console.error('Failed to add setup pose:', err);
        }
    }

    // Add a waypoint to the current sequence (or create new sequence)
    async addSequenceWaypoint() {
        try {
            const response = await fetch('/api/trajectory/current-pose');
            if (!response.ok) {
                console.error('Failed to get current pose');
                return;
            }

            const data = await response.json();
            if (!data.position) {
                console.error('No position data in response');
                return;
            }

            const waypoint = {
                position: data.position,
                orientation: data.orientation || { w: 1, x: 0, y: 0, z: 0 },
                blend_factor: 0.0,
                segment_time: 0.0,
                pause_time: 0.0,
                joints: data.joints  // Save joint positions when waypoint is taught
            };

            // Check if last element is a sequence - add to it
            const lastElem = this.elements[this.elements.length - 1];
            if (lastElem && lastElem.type === 'sequence') {
                lastElem.waypoints.push(waypoint);
            } else {
                // Create new sequence - capture entry joints!
                if (!data.joints) {
                    console.error('No joint data for entry_joints');
                    return;
                }
                this.elements.push({
                    type: 'sequence',
                    name: `Sequence ${this.elements.length + 1}`,
                    entry_joints: data.joints,  // Capture current joints as entry config
                    entry_position: data.position,  // Capture entry position for visualization
                    waypoints: [waypoint]
                });
            }

            this.renderElements();
            this.updateButtons();
            this.notifyWaypointsChanged();
        } catch (err) {
            console.error('Failed to add sequence waypoint:', err);
        }
    }

    clearAll() {
        this.elements = [];
        this.plannedTrajectory = null;
        this.renderElements();
        this.updateButtons();
        this.clearVisualization();
        this.notifyWaypointsChanged();
    }

    removeElement(index) {
        this.elements.splice(index, 1);
        this.plannedTrajectory = null;
        this.renderElements();
        this.updateButtons();
        this.notifyWaypointsChanged();
    }

    removeWaypointFromSequence(elemIndex, wpIndex) {
        const elem = this.elements[elemIndex];
        if (elem && elem.type === 'sequence') {
            elem.waypoints.splice(wpIndex, 1);
            // Remove sequence if empty
            if (elem.waypoints.length === 0) {
                this.elements.splice(elemIndex, 1);
            }
            this.plannedTrajectory = null;
            this.renderElements();
            this.updateButtons();
            this.notifyWaypointsChanged();
        }
    }

    updateWaypointParam(elemIndex, wpIndex, param, value) {
        const elem = this.elements[elemIndex];
        if (elem && elem.type === 'sequence' && elem.waypoints[wpIndex]) {
            elem.waypoints[wpIndex][param] = parseFloat(value) || 0;
        }
    }

    renderElements() {
        const container = document.getElementById('elements-container');
        if (!container) return;

        container.innerHTML = '';

        this.elements.forEach((elem, elemIndex) => {
            const div = document.createElement('div');
            div.className = `trajectory-element ${elem.type}`;

            if (elem.type === 'setup_pose') {
                div.innerHTML = this.renderSetupPose(elem, elemIndex);
            } else if (elem.type === 'sequence') {
                div.innerHTML = this.renderSequence(elem, elemIndex);
            }

            container.appendChild(div);
        });
    }

    renderSetupPose(elem, elemIndex) {
        const joints = elem.joints;
        const jointsDeg = joints.map(j => (j * 180 / Math.PI).toFixed(1));

        return `
            <div class="element-header setup-pose-header">
                <span class="element-type">Setup Pose</span>
                <span class="element-name">${elem.name}</span>
                <button class="btn btn-small btn-danger"
                        onclick="window.trajectoryPanel.removeElement(${elemIndex})">X</button>
            </div>
            <div class="element-content moveable-target"
                 data-joints='${JSON.stringify(elem.joints)}'
                 onmousedown="event.preventDefault(); window.trajectoryPanel.startMoveToJoints(this)"
                 onmouseup="window.trajectoryPanel.stopMove()"
                 onmouseleave="window.trajectoryPanel.stopMove()"
                 ontouchstart="event.preventDefault(); window.trajectoryPanel.startMoveToJoints(this)"
                 ontouchend="window.trajectoryPanel.stopMove()"
                <span class="move-hint">Hold to move</span>
                <span class="joints-display">
                    J1: ${jointsDeg[0]}° J2: ${jointsDeg[1]}° J3: ${jointsDeg[2]}°
                    J4: ${jointsDeg[3]}° J5: ${jointsDeg[4]}° J6: ${jointsDeg[5]}°
                </span>
            </div>
        `;
    }

    renderSequence(elem, elemIndex) {
        // Entry point display (if entry_joints captured)
        let entryHtml = '';
        if (elem.entry_joints) {
            const jointsDeg = elem.entry_joints.map(j => (j * 180 / Math.PI).toFixed(1));
            entryHtml = `
                <div class="sequence-entry-point moveable-target"
                     data-joints='${JSON.stringify(elem.entry_joints)}'
                     onmousedown="event.preventDefault(); window.trajectoryPanel.startMoveToJoints(this)"
                     onmouseup="window.trajectoryPanel.stopMove()"
                     onmouseleave="window.trajectoryPanel.stopMove()"
                     ontouchstart="event.preventDefault(); window.trajectoryPanel.startMoveToJoints(this)"
                     ontouchend="window.trajectoryPanel.stopMove()"
                    <span class="entry-label">Entry Point (hold to move)</span>
                    <span class="joints-display">
                        J1: ${jointsDeg[0]}° J2: ${jointsDeg[1]}° J3: ${jointsDeg[2]}°
                        J4: ${jointsDeg[3]}° J5: ${jointsDeg[4]}° J6: ${jointsDeg[5]}°
                    </span>
                </div>
            `;
        }

        let waypointsHtml = '';
        elem.waypoints.forEach((wp, wpIndex) => {
            const pos = wp.position;
            const poseData = JSON.stringify({position: wp.position, orientation: wp.orientation});
            waypointsHtml += `
                <div class="sequence-waypoint">
                    <div class="waypoint-info moveable-pose"
                         data-pose='${poseData}'
                         onmousedown="event.preventDefault(); window.trajectoryPanel.startMoveToPose(this)"
                         onmouseup="window.trajectoryPanel.stopMove()"
                         onmouseleave="window.trajectoryPanel.stopMove()"
                         ontouchstart="event.preventDefault(); window.trajectoryPanel.startMoveToPose(this)"
                         ontouchend="window.trajectoryPanel.stopMove()">
                        <span class="waypoint-index">WP ${wpIndex + 1}</span>
                        <span class="waypoint-position">
                            X: ${pos.x.toFixed(3)} Y: ${pos.y.toFixed(3)} Z: ${pos.z.toFixed(3)}
                        </span>
                    </div>
                    <div class="waypoint-params">
                        <div class="waypoint-param">
                            <label>Blend</label>
                            <input type="number" class="waypoint-input" value="${wp.blend_factor}"
                                   step="0.01" min="0"
                                   onchange="window.trajectoryPanel.updateWaypointParam(${elemIndex}, ${wpIndex}, 'blend_factor', this.value)">
                        </div>
                        <div class="waypoint-param">
                            <label>Time</label>
                            <input type="number" class="waypoint-input" value="${wp.segment_time}"
                                   step="0.1" min="0"
                                   onchange="window.trajectoryPanel.updateWaypointParam(${elemIndex}, ${wpIndex}, 'segment_time', this.value)">
                        </div>
                        <button class="btn btn-small btn-danger"
                                onclick="window.trajectoryPanel.removeWaypointFromSequence(${elemIndex}, ${wpIndex})">X</button>
                    </div>
                </div>
            `;
        });

        return `
            <div class="element-header sequence-header">
                <span class="element-type">Sequence</span>
                <span class="element-name">${elem.name}</span>
                <span class="waypoint-count">(${elem.waypoints.length} waypoints)</span>
                <button class="btn btn-small btn-danger"
                        onclick="window.trajectoryPanel.removeElement(${elemIndex})">X</button>
            </div>
            <div class="element-content">
                ${entryHtml}
                ${waypointsHtml}
            </div>
        `;
    }

    updateButtons() {
        const btnPlan = document.getElementById('btn-plan-trajectory');
        const btnExecute = document.getElementById('btn-execute-trajectory');

        // Need at least one element with content
        const hasContent = this.elements.some(e =>
            e.type === 'setup_pose' || (e.type === 'sequence' && e.waypoints.length >= 1)
        );

        if (btnPlan) btnPlan.disabled = !hasContent;
        if (btnExecute) btnExecute.disabled = !this.plannedTrajectory || !this.plannedTrajectory.valid;
    }

    // Notify robot viewer of waypoint changes for 3D visualization
    notifyWaypointsChanged() {
        const waypoints = [];
        let waypointIndex = 1;

        this.elements.forEach((elem, elemIndex) => {
            if (elem.type === 'setup_pose' && elem.position) {
                // Setup pose - show as orange marker
                waypoints.push({
                    type: 'setup_pose',
                    position: elem.position,
                    label: `S${elemIndex + 1}`
                });
            } else if (elem.type === 'sequence') {
                // Entry point - show as blue marker
                if (elem.entry_position) {
                    waypoints.push({
                        type: 'sequence',
                        isEntry: true,
                        position: elem.entry_position,
                        label: `E${elemIndex + 1}`
                    });
                }
                // Sequence waypoints - show as pink markers
                elem.waypoints.forEach((wp, wpIndex) => {
                    waypoints.push({
                        type: 'sequence',
                        isEntry: false,
                        position: wp.position,
                        label: `WP${waypointIndex++}`
                    });
                });
            }
        });

        this.onWaypointsUpdate(waypoints);
    }

    async planTrajectory() {
        if (this.elements.length === 0) {
            alert('Add at least one element');
            return;
        }

        try {
            // Build elements array for API
            const apiElements = this.elements.map(elem => {
                if (elem.type === 'setup_pose') {
                    return {
                        type: 'setup_pose',
                        name: elem.name,
                        joints: elem.joints,
                        move_time: elem.move_time || 0
                    };
                } else {
                    const seqData = {
                        type: 'sequence',
                        name: elem.name,
                        waypoints: elem.waypoints
                    };
                    // Include entry_joints if captured
                    if (elem.entry_joints) {
                        seqData.entry_joints = elem.entry_joints;
                    }
                    return seqData;
                }
            });

            const response = await fetch('/api/trajectory/plan2', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    elements: apiElements,
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

            // Update visualization
            if (data.valid && data.visualization) {
                if (data.visualization.timing) {
                    this.updateTimingChart(data.visualization.timing);
                }
                if (data.visualization.path) {
                    this.onPathUpdate(data.visualization.path);
                }
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
            alert('Failed to plan trajectory: ' + err.message);
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
            container.appendChild(div);
        });
    }

    updateTimingChart(timing) {
        if (!this.timingChart || !timing || !timing.length) return;

        this.timingChart.data.labels = timing.map(t => t.time.toFixed(2));
        this.timingChart.data.datasets[0].data = timing.map(t => t.speed || 0);
        this.timingChart.data.datasets[1].data = timing.map(t => t.acceleration || 0);
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
                alert('Failed to execute: ' + (data.message || 'Unknown error'));
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
                if (!response.ok) return;

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
        }, 200);
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

    // Start moving robot to joint positions (called on mousedown/touchstart)
    async startMoveToJoints(element) {
        console.log('startMoveToJoints called', element);
        const jointsStr = element.getAttribute('data-joints');
        console.log('data-joints:', jointsStr);
        if (!jointsStr) {
            console.error('No data-joints attribute found');
            return;
        }

        try {
            const joints = JSON.parse(jointsStr);
            console.log('Parsed joints:', joints);
            element.classList.add('moving');

            const response = await fetch('/api/move-to-joints', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    joints: joints,
                    speed: 0.5,
                    acceleration: 1.0
                })
            });

            const data = await response.json();
            console.log('Move response:', data);
            if (!data.success) {
                console.error('Move failed:', data.message);
                element.classList.remove('moving');
            }
        } catch (err) {
            console.error('Failed to start move:', err);
            element.classList.remove('moving');
        }
    }

    // Start moving robot linearly to a pose (called on mousedown/touchstart for waypoints)
    async startMoveToPose(element) {
        console.log('startMoveToPose called', element);
        const poseStr = element.getAttribute('data-pose');
        console.log('data-pose:', poseStr);
        if (!poseStr) {
            console.error('No data-pose attribute found');
            return;
        }

        try {
            const poseData = JSON.parse(poseStr);
            console.log('Parsed pose:', poseData);
            element.classList.add('moving');

            const response = await fetch('/api/move-to-pose', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    position: poseData.position,
                    orientation: poseData.orientation,
                    speed: 0.1,       // Linear speed in m/s
                    acceleration: 0.5  // Linear acceleration in m/s²
                })
            });

            const data = await response.json();
            console.log('MoveL response:', data);
            if (!data.success) {
                console.error('MoveL failed:', data.message);
                element.classList.remove('moving');
            }
        } catch (err) {
            console.error('Failed to start linear move:', err);
            element.classList.remove('moving');
        }
    }

    // Stop robot movement (called on mouseup/mouseleave/touchend)
    async stopMove() {
        // Remove moving class from all elements (both joint and pose targets)
        document.querySelectorAll('.moveable-target.moving, .moveable-pose.moving').forEach(el => {
            el.classList.remove('moving');
        });

        try {
            await fetch('/api/stop', { method: 'POST' });
        } catch (err) {
            console.error('Failed to stop move:', err);
        }
    }
}
