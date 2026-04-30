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

        // Chart popup state
        this.chartPopupWindow = null;
        this.chartPopupReady = false;
        this.lastTimingData = null;
        this.lastSegmentTimes = null;

        this.init();
    }

    init() {
        this.setupEventListeners();
        this.initTimingChart();
        this.loadMotionLimits();
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

        // Apply motion limits button
        const btnApplyLimits = document.getElementById('btn-apply-limits');
        if (btnApplyLimits) {
            btnApplyLimits.addEventListener('click', () => this.applyMotionLimits());
        }

        // Chart pop-out button
        const btnPopoutChart = document.getElementById('btn-popout-chart');
        if (btnPopoutChart) {
            btnPopoutChart.addEventListener('click', () => this.openChartPopup());
        }

        // Listen for messages from chart popup
        window.addEventListener('message', (event) => {
            if (event.origin !== window.location.origin) return;
            if (event.data.type === 'chartPopupReady') {
                this.chartPopupReady = true;
                // Send current data to popup
                if (this.lastTimingData) {
                    this.sendToChartPopup(this.lastTimingData, this.lastSegmentTimes || []);
                }
            } else if (event.data.type === 'chartPopupClosed') {
                this.chartPopupWindow = null;
                this.chartPopupReady = false;
            }
        });
    }

    // Motion limits management
    async loadMotionLimits() {
        try {
            const response = await fetch('/api/trajectory/config');
            if (!response.ok) {
                console.warn('Failed to load motion limits');
                return;
            }

            const config = await response.json();
            this.updateMotionLimitsUI(config);
        } catch (error) {
            console.warn('Failed to load motion limits:', error);
        }
    }

    updateMotionLimitsUI(config) {
        const velInput = document.getElementById('limit-velocity');
        const accelInput = document.getElementById('limit-acceleration');
        const jerkInput = document.getElementById('limit-jerk');

        if (velInput && config.max_linear_velocity !== undefined) {
            velInput.value = config.max_linear_velocity.toFixed(2);
        }
        if (accelInput && config.max_linear_acceleration !== undefined) {
            accelInput.value = config.max_linear_acceleration.toFixed(1);
        }
        if (jerkInput && config.max_linear_jerk !== undefined) {
            jerkInput.value = config.max_linear_jerk.toFixed(1);
        }
    }

    async applyMotionLimits() {
        const velInput = document.getElementById('limit-velocity');
        const accelInput = document.getElementById('limit-acceleration');
        const jerkInput = document.getElementById('limit-jerk');

        const config = {};
        if (velInput) config.max_linear_velocity = parseFloat(velInput.value);
        if (accelInput) config.max_linear_acceleration = parseFloat(accelInput.value);
        if (jerkInput) config.max_linear_jerk = parseFloat(jerkInput.value);

        try {
            const response = await fetch('/api/trajectory/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            });

            const data = await response.json();
            if (data.success) {
                console.log('Motion limits updated:', config);
                // Update UI with the actual values from server
                this.updateMotionLimitsUI(data.config || config);

                // If we have a trajectory planned, suggest replanning
                if (this.plannedTrajectory) {
                    const shouldReplan = confirm('Motion limits updated. Do you want to replan the trajectory with the new limits?');
                    if (shouldReplan) {
                        this.planTrajectory();
                    }
                }
            } else {
                console.error('Failed to update motion limits:', data.message);
                alert('Failed to update motion limits: ' + (data.message || 'Unknown error'));
            }
        } catch (error) {
            console.error('Error applying motion limits:', error);
            alert('Error applying motion limits: ' + error.message);
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
                            label: 'Velocity (m/s)',
                            data: [],
                            borderColor: '#0099ff',
                            backgroundColor: 'rgba(0, 153, 255, 0.1)',
                            tension: 0.3,
                            fill: false,
                            pointRadius: 0,
                            yAxisID: 'y'
                        },
                        {
                            label: 'Acceleration (m/s²)',
                            data: [],
                            borderColor: '#00cc66',
                            backgroundColor: 'rgba(0, 204, 102, 0.1)',
                            tension: 0.3,
                            fill: false,
                            pointRadius: 0,
                            yAxisID: 'y'
                        },
                        {
                            label: 'Jerk (m/s³)',
                            data: [],
                            borderColor: '#ff6644',
                            backgroundColor: 'rgba(255, 102, 68, 0.1)',
                            tension: 0.3,
                            fill: false,
                            pointRadius: 0,
                            yAxisID: 'y2'
                        }
                    ]
                },
                options: {
                    responsive: false,
                    maintainAspectRatio: false,
                    animation: false,
                    interaction: {
                        mode: 'index',
                        intersect: false
                    },
                    scales: {
                        x: {
                            title: { display: true, text: 'Time (s)', color: '#a0a0a0' },
                            grid: { color: 'rgba(255, 255, 255, 0.1)' },
                            ticks: { color: '#a0a0a0' }
                        },
                        y: {
                            type: 'linear',
                            position: 'left',
                            title: { display: true, text: 'Vel / Accel', color: '#a0a0a0' },
                            grid: { color: 'rgba(255, 255, 255, 0.1)' },
                            ticks: { color: '#a0a0a0' }
                        },
                        y2: {
                            type: 'linear',
                            position: 'right',
                            title: { display: true, text: 'Jerk (m/s³)', color: '#ff6644' },
                            grid: { drawOnChartArea: false },
                            ticks: { color: '#ff6644' }
                        }
                    },
                    plugins: {
                        legend: { labels: { color: '#e8e8e8' } },
                        annotation: {
                            annotations: {}  // Will be populated with waypoint lines
                        }
                    }
                }
            });

            // Store segment times for waypoint markers
            this.segmentTimes = [];
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
        console.log('planTrajectory called, elements:', this.elements.length);

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

            // Get current motion limits from UI
            const velInput = document.getElementById('limit-velocity');
            const accelInput = document.getElementById('limit-acceleration');
            const jerkInput = document.getElementById('limit-jerk');
            const methodSelect = document.getElementById('planning-method');
            const planningMethod = methodSelect ? methodSelect.value : 'geometric';

            const config = {
                max_linear_velocity: velInput ? parseFloat(velInput.value) : 0.5,
                max_linear_acceleration: accelInput ? parseFloat(accelInput.value) : 1.0,
                max_linear_jerk: jerkInput ? parseFloat(jerkInput.value) : 5.0,
                max_joint_velocity: 1.5,
                planning_method: planningMethod
            };

            const response = await fetch('/api/trajectory/plan2', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    elements: apiElements,
                    planning_method: planningMethod,
                    config: config
                })
            });

            const data = await response.json();
            console.log('planTrajectory response:', data);
            this.plannedTrajectory = data;

            // Show validation messages
            this.showValidationMessages(data.messages || []);

            // Update visualization
            console.log('Plan response:', { valid: data.valid, hasViz: !!data.visualization });
            if (data.valid && data.visualization) {
                console.log('Visualization data:', {
                    hasTiming: !!data.visualization.timing,
                    timingType: typeof data.visualization.timing,
                    timingKeys: data.visualization.timing ? Object.keys(data.visualization.timing).length : 0,
                    hasSegTimes: !!data.visualization.segment_times
                });
                if (data.visualization.timing) {
                    const segTimes = data.visualization.segment_times || [];
                    this.updateTimingChart(data.visualization.timing, segTimes);
                }
                if (data.visualization.path) {
                    // Convert object with numeric keys to array if needed
                    const pathArray = Array.isArray(data.visualization.path)
                        ? data.visualization.path
                        : Object.values(data.visualization.path);
                    console.log('Updating path with', pathArray.length, 'points');
                    this.onPathUpdate(pathArray);
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

    updateTimingChart(timing, segmentTimes = []) {
        if (!this.timingChart || !timing) return;

        // Convert object with numeric keys to array if needed
        let timingArray = Array.isArray(timing) ? timing : Object.values(timing);
        if (!timingArray.length) return;

        this.timingChart.data.labels = timingArray.map(t => t.time.toFixed(2));
        this.timingChart.data.datasets[0].data = timingArray.map(t => t.speed || 0);
        this.timingChart.data.datasets[1].data = timingArray.map(t => t.acceleration || 0);
        this.timingChart.data.datasets[2].data = timingArray.map(t => t.jerk || 0);

        // Add waypoint marker lines using annotations
        const annotations = {};
        // Convert object with numeric keys to array if needed
        const segTimesArray = Array.isArray(segmentTimes) ? segmentTimes : Object.values(segmentTimes || {});
        const labels = this.timingChart.data.labels;

        if (segTimesArray.length > 0 && labels.length > 0) {
            segTimesArray.forEach((time, idx) => {
                // Find the closest label to this segment time
                let closestLabel = labels[0];
                let minDiff = Math.abs(parseFloat(labels[0]) - time);

                for (const label of labels) {
                    const diff = Math.abs(parseFloat(label) - time);
                    if (diff < minDiff) {
                        minDiff = diff;
                        closestLabel = label;
                    }
                }

                annotations[`wp${idx}`] = {
                    type: 'line',
                    xMin: closestLabel,
                    xMax: closestLabel,
                    borderColor: 'rgba(255, 68, 136, 0.7)',
                    borderWidth: 2,
                    borderDash: [5, 5],
                    label: {
                        display: true,
                        content: `WP${idx + 1}`,
                        position: 'start',
                        backgroundColor: 'rgba(255, 68, 136, 0.8)',
                        color: '#fff',
                        font: { size: 10 }
                    }
                };
            });
        }

        // Update annotations if the plugin is available
        if (this.timingChart.options.plugins.annotation) {
            this.timingChart.options.plugins.annotation.annotations = annotations;
        }

        this.timingChart.update();

        // Store for pop-out window and send to popup if open
        this.lastTimingData = timing;
        this.lastSegmentTimes = segmentTimes;
        this.sendToChartPopup(timing, segmentTimes);
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

    // Open velocity profile chart in popup window
    openChartPopup() {
        // If popup already open, focus it
        if (this.chartPopupWindow && !this.chartPopupWindow.closed) {
            this.chartPopupWindow.focus();
            return;
        }

        // Open new popup window
        const width = 900;
        const height = 500;
        const left = window.screenX + 100;
        const top = window.screenY + 100;

        this.chartPopupWindow = window.open(
            '/chart-popup.html',
            'VelocityProfile',
            `width=${width},height=${height},left=${left},top=${top},resizable=yes`
        );

        this.chartPopupReady = false;
    }

    // Send timing data to chart popup
    sendToChartPopup(timing, segmentTimes) {
        if (this.chartPopupWindow && !this.chartPopupWindow.closed && this.chartPopupReady) {
            this.chartPopupWindow.postMessage({
                type: 'chartData',
                timing: timing,
                segmentTimes: segmentTimes
            }, window.location.origin);
        }
    }
}
