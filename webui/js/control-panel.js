// UR Controller Web UI - Joint Control Panel

const RAD_TO_DEG = 180 / Math.PI;
const DEG_TO_RAD = Math.PI / 180;

const JOINT_NAMES = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3'];
const JOINT_LIMITS = {
    min: -2 * Math.PI,
    max: 2 * Math.PI
};

export class ControlPanel {
    constructor(options = {}) {
        this.onServoJ = options.onServoJ || (() => {});
        this.onStop = options.onStop || (() => {});
        this.onSnapToCurrent = options.onSnapToCurrent || (() => {});

        this.enabled = false;
        this.targetPositions = [0, 0, 0, 0, 0, 0];
        this.actualPositions = [0, 0, 0, 0, 0, 0];
        this.sliders = [];
        this.actualLabels = [];
        this.targetLabels = [];

        this.createSliders();
    }

    createSliders() {
        const container = document.getElementById('joint-sliders');
        container.innerHTML = '';

        for (let i = 0; i < 6; i++) {
            const row = document.createElement('div');
            row.className = 'joint-row';

            // Joint name
            const nameLabel = document.createElement('span');
            nameLabel.className = 'joint-name';
            nameLabel.textContent = JOINT_NAMES[i];
            row.appendChild(nameLabel);

            // Slider
            const slider = document.createElement('input');
            slider.type = 'range';
            slider.min = JOINT_LIMITS.min;
            slider.max = JOINT_LIMITS.max;
            slider.step = 0.01;
            slider.value = 0;
            slider.disabled = true;
            slider.addEventListener('input', (e) => this.handleSliderChange(i, e.target.value));
            this.sliders.push(slider);
            row.appendChild(slider);

            // Actual value label
            const actualLabel = document.createElement('span');
            actualLabel.className = 'actual-value';
            actualLabel.textContent = '0.0';
            this.actualLabels.push(actualLabel);
            row.appendChild(actualLabel);

            // Target value label
            const targetLabel = document.createElement('span');
            targetLabel.className = 'target-value';
            targetLabel.textContent = '0.0';
            this.targetLabels.push(targetLabel);
            row.appendChild(targetLabel);

            container.appendChild(row);
        }
    }

    handleSliderChange(jointIndex, value) {
        const radValue = parseFloat(value);
        this.targetPositions[jointIndex] = radValue;
        this.targetLabels[jointIndex].textContent = (radValue * RAD_TO_DEG).toFixed(1);

        if (this.enabled) {
            this.onServoJ([...this.targetPositions]);
        }
    }

    updateActualPositions(positions) {
        for (let i = 0; i < 6 && i < positions.length; i++) {
            this.actualPositions[i] = positions[i];
            this.actualLabels[i].textContent = (positions[i] * RAD_TO_DEG).toFixed(1);
        }
    }

    setEnabled(enabled) {
        this.enabled = enabled;

        for (let i = 0; i < 6; i++) {
            this.sliders[i].disabled = !enabled;

            if (enabled) {
                // When enabling, set sliders to current actual positions
                this.targetPositions[i] = this.actualPositions[i];
                this.sliders[i].value = this.actualPositions[i];
                this.targetLabels[i].textContent = (this.actualPositions[i] * RAD_TO_DEG).toFixed(1);
            }
        }
    }

    snapToCurrent() {
        for (let i = 0; i < 6; i++) {
            this.targetPositions[i] = this.actualPositions[i];
            this.sliders[i].value = this.actualPositions[i];
            this.targetLabels[i].textContent = (this.actualPositions[i] * RAD_TO_DEG).toFixed(1);
        }
    }

    getTargetPositions() {
        return [...this.targetPositions];
    }
}
