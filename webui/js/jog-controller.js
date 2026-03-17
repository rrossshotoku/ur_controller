// Jog Controller - TCP and Joint velocity control

export class JogController {
    constructor(wsClient) {
        this.wsClient = wsClient;

        // TCP jog settings
        this.jogSpeed = 0.05;  // m/s
        this.frame = 'base';   // 'base' or 'tcp'
        this.currentTcpRotation = null;  // 3x3 rotation matrix (flat array)
        this.isJogging = false;
        this.jogInterval = null;  // Interval for continuous command sending
        this.currentVelocity = [0, 0, 0];  // Current TCP jog velocity

        // Joint jog settings
        this.jointJogSpeed = 0.2;  // rad/s
        this.isJointJogging = false;
        this.jointJogInterval = null;
        this.currentJointVelocity = [0, 0, 0, 0, 0, 0];

        this.init();
    }

    init() {
        this.setupFrameSelector();
        this.setupJogButtons();
        this.setupSpeedSlider();
        this.setupJointJogButtons();
        this.setupJointSpeedSlider();
    }

    setupFrameSelector() {
        const radios = document.querySelectorAll('input[name="jog-frame"]');
        radios.forEach(radio => {
            radio.addEventListener('change', (e) => {
                this.frame = e.target.value;
                console.log('Jog frame changed to:', this.frame);
            });
        });
    }

    setupJogButtons() {
        const buttons = document.querySelectorAll('.jog-btn');
        buttons.forEach(btn => {
            const axis = btn.dataset.axis;
            const dir = parseInt(btn.dataset.dir, 10);

            // Start jogging on mouse/touch down
            btn.addEventListener('mousedown', (e) => {
                e.preventDefault();
                this.startJog(axis, dir);
            });
            btn.addEventListener('touchstart', (e) => {
                e.preventDefault();
                this.startJog(axis, dir);
            });

            // Stop jogging on mouse/touch up
            btn.addEventListener('mouseup', () => this.stopJog());
            btn.addEventListener('mouseleave', () => {
                if (this.isJogging) this.stopJog();
            });
            btn.addEventListener('touchend', () => this.stopJog());
            btn.addEventListener('touchcancel', () => this.stopJog());
        });

        // Also stop jogging if mouse leaves the window
        window.addEventListener('mouseup', () => {
            if (this.isJogging) this.stopJog();
        });
    }

    setupSpeedSlider() {
        const slider = document.getElementById('jog-speed-slider');
        const valueDisplay = document.getElementById('jog-speed-value');

        if (slider) {
            slider.addEventListener('input', (e) => {
                this.jogSpeed = parseFloat(e.target.value);
                if (valueDisplay) {
                    valueDisplay.textContent = this.jogSpeed.toFixed(2);
                }
            });
        }
    }

    startJog(axis, direction) {
        if (!this.wsClient || !this.wsClient.isConnected()) {
            console.warn('WebSocket not connected, cannot jog');
            return;
        }

        // Stop any existing jog first
        this.stopJogInterval();

        this.isJogging = true;
        this.activeAxis = axis;
        this.activeDirection = direction;

        // Build velocity vector in specified frame
        let velocity = [0, 0, 0];
        const axisIndex = { x: 0, y: 1, z: 2 }[axis];
        velocity[axisIndex] = this.jogSpeed * direction;

        // Store the base velocity (before transform) for continuous updates
        this.currentVelocity = velocity.slice();

        console.log(`START Jog ${axis}${direction > 0 ? '+' : '-'} in ${this.frame} frame, speed=${this.jogSpeed}:`, velocity);

        // Send command immediately
        this.sendJogCommand();

        // Send commands repeatedly (every 100ms) to keep robot moving
        // ur_rtde speedL needs continuous commands or robot stops
        this.jogInterval = setInterval(() => {
            this.sendJogCommand();
        }, 100);

        console.log('Jog interval started');
    }

    stopJogInterval() {
        if (this.jogInterval) {
            clearInterval(this.jogInterval);
            this.jogInterval = null;
        }
    }

    sendJogCommand() {
        if (!this.wsClient || !this.wsClient.isConnected()) {
            console.warn('WebSocket disconnected during jog');
            this.stopJog();
            return;
        }

        let velocity = this.currentVelocity.slice();

        // If TCP frame, transform velocity to base frame using current TCP rotation
        if (this.frame === 'tcp' && this.currentTcpRotation) {
            velocity = this.transformToBase(velocity);
        }

        const cmd = {
            type: 'speedl',
            velocity: velocity,
            angular_velocity: [0, 0, 0],
            acceleration: 0.5
        };

        this.wsClient.send(cmd);
    }

    stopJog() {
        // Clear the interval first
        this.stopJogInterval();

        if (!this.isJogging) return;

        this.isJogging = false;
        this.currentVelocity = [0, 0, 0];
        console.log('STOP Jog');

        // Send stop command to exit velocity control mode
        if (this.wsClient && this.wsClient.isConnected()) {
            this.wsClient.send({ type: 'stop' });
        }
    }

    transformToBase(tcpVelocity) {
        // Transform velocity from TCP frame to base frame
        // v_base = R_base_tcp * v_tcp
        // R is stored as [r00, r10, r20, r01, r11, r21, r02, r12, r22] (column-major)
        // or [r00, r01, r02, r10, r11, r12, r20, r21, r22] (row-major)
        // We use row-major here: R[row][col] = R[row*3 + col]
        const R = this.currentTcpRotation;
        return [
            R[0] * tcpVelocity[0] + R[1] * tcpVelocity[1] + R[2] * tcpVelocity[2],
            R[3] * tcpVelocity[0] + R[4] * tcpVelocity[1] + R[5] * tcpVelocity[2],
            R[6] * tcpVelocity[0] + R[7] * tcpVelocity[1] + R[8] * tcpVelocity[2]
        ];
    }

    updateTcpPose(tcpPose) {
        // TCP pose from robot state: [x, y, z, rx, ry, rz]
        // rx, ry, rz is axis-angle rotation
        // Convert to rotation matrix for frame transform
        const rx = tcpPose[3];
        const ry = tcpPose[4];
        const rz = tcpPose[5];

        this.currentTcpRotation = this.axisAngleToMatrix(rx, ry, rz);
    }

    axisAngleToMatrix(rx, ry, rz) {
        // Convert axis-angle representation to 3x3 rotation matrix
        // angle = norm([rx, ry, rz])
        // axis = [rx, ry, rz] / angle
        const angle = Math.sqrt(rx * rx + ry * ry + rz * rz);

        if (angle < 1e-6) {
            // Identity rotation
            return [1, 0, 0, 0, 1, 0, 0, 0, 1];
        }

        const ax = rx / angle;
        const ay = ry / angle;
        const az = rz / angle;

        const c = Math.cos(angle);
        const s = Math.sin(angle);
        const t = 1 - c;

        // Rodrigues' rotation formula
        // R = I * cos(angle) + (1 - cos(angle)) * (axis * axis^T) + sin(angle) * [axis]_x
        return [
            t * ax * ax + c,      t * ax * ay - s * az, t * ax * az + s * ay,
            t * ax * ay + s * az, t * ay * ay + c,      t * ay * az - s * ax,
            t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c
        ];
    }

    // === Joint Jog Methods ===

    setupJointJogButtons() {
        const buttons = document.querySelectorAll('.joint-jog-btn');
        buttons.forEach(btn => {
            const joint = parseInt(btn.dataset.joint, 10);
            const dir = parseInt(btn.dataset.dir, 10);

            // Start jogging on mouse/touch down
            btn.addEventListener('mousedown', (e) => {
                e.preventDefault();
                this.startJointJog(joint, dir);
            });
            btn.addEventListener('touchstart', (e) => {
                e.preventDefault();
                this.startJointJog(joint, dir);
            });

            // Stop jogging on mouse/touch up
            btn.addEventListener('mouseup', () => this.stopJointJog());
            btn.addEventListener('mouseleave', () => {
                if (this.isJointJogging) this.stopJointJog();
            });
            btn.addEventListener('touchend', () => this.stopJointJog());
            btn.addEventListener('touchcancel', () => this.stopJointJog());
        });

        // Also stop jogging if mouse leaves the window
        window.addEventListener('mouseup', () => {
            if (this.isJointJogging) this.stopJointJog();
        });
    }

    setupJointSpeedSlider() {
        const slider = document.getElementById('joint-jog-speed-slider');
        const valueDisplay = document.getElementById('joint-jog-speed-value');

        if (slider) {
            slider.addEventListener('input', (e) => {
                this.jointJogSpeed = parseFloat(e.target.value);
                if (valueDisplay) {
                    valueDisplay.textContent = this.jointJogSpeed.toFixed(2);
                }
            });
        }
    }

    startJointJog(joint, direction) {
        if (!this.wsClient || !this.wsClient.isConnected()) {
            console.warn('WebSocket not connected, cannot jog');
            return;
        }

        // Stop any existing jog first
        this.stopJointJogInterval();

        this.isJointJogging = true;
        this.activeJoint = joint;
        this.activeJointDirection = direction;

        // Build velocity vector - all zeros except the active joint
        this.currentJointVelocity = [0, 0, 0, 0, 0, 0];
        this.currentJointVelocity[joint] = this.jointJogSpeed * direction;

        console.log(`START Joint ${joint + 1} jog ${direction > 0 ? '+' : '-'}, speed=${this.jointJogSpeed}`);

        // Send command immediately
        this.sendJointJogCommand();

        // Send commands repeatedly (every 100ms) to keep robot moving
        this.jointJogInterval = setInterval(() => {
            this.sendJointJogCommand();
        }, 100);
    }

    stopJointJogInterval() {
        if (this.jointJogInterval) {
            clearInterval(this.jointJogInterval);
            this.jointJogInterval = null;
        }
    }

    sendJointJogCommand() {
        if (!this.wsClient || !this.wsClient.isConnected()) {
            console.warn('WebSocket disconnected during jog');
            this.stopJointJog();
            return;
        }

        const cmd = {
            type: 'speedj',
            velocity: this.currentJointVelocity,
            acceleration: 1.0
        };

        this.wsClient.send(cmd);
    }

    stopJointJog() {
        // Clear the interval first
        this.stopJointJogInterval();

        if (!this.isJointJogging) return;

        this.isJointJogging = false;
        this.currentJointVelocity = [0, 0, 0, 0, 0, 0];
        console.log('STOP Joint Jog');

        // Send stop command
        if (this.wsClient && this.wsClient.isConnected()) {
            this.wsClient.send({ type: 'stop' });
        }
    }
}
