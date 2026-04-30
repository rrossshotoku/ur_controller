// Terminal Controller - Fetches and displays server logs

export class TerminalController {
    constructor() {
        this.terminalOutput = document.getElementById('terminal-output');
        this.autoScrollCheckbox = document.getElementById('terminal-autoscroll');
        this.clearButton = document.getElementById('btn-clear-terminal');
        this.refreshButton = document.getElementById('btn-refresh-terminal');

        this.autoScroll = true;
        this.refreshInterval = null;
        this.lastLogCount = -1;  // Start at -1 to ensure first fetch renders

        console.log('TerminalController initialized, terminalOutput:', this.terminalOutput);

        if (!this.terminalOutput) {
            console.error('Terminal output element not found! Check that #terminal-output exists in HTML');
        }

        this.init();
    }

    init() {
        // Set up event listeners
        if (this.autoScrollCheckbox) {
            this.autoScrollCheckbox.addEventListener('change', (e) => {
                this.autoScroll = e.target.checked;
            });
        }

        if (this.clearButton) {
            this.clearButton.addEventListener('click', () => {
                this.clearLogs();
            });
        }

        if (this.refreshButton) {
            this.refreshButton.addEventListener('click', () => {
                this.fetchLogs();
            });
        }

        // Initial fetch
        this.fetchLogs();

        // Start periodic refresh (every 1 second)
        this.refreshInterval = setInterval(() => {
            this.fetchLogs();
        }, 1000);
    }

    async fetchLogs() {
        try {
            const response = await fetch('/api/logs');
            if (!response.ok) {
                console.error('Failed to fetch logs:', response.status);
                return;
            }

            const data = await response.json();
            console.log('Fetched logs:', data.count, 'entries');
            this.renderLogs(data.logs || []);
        } catch (error) {
            console.error('Error fetching logs:', error);
        }
    }

    renderLogs(logs) {
        if (!this.terminalOutput) return;

        // Only update if there are new logs
        if (logs.length === this.lastLogCount) return;
        this.lastLogCount = logs.length;

        // Clear and re-render
        this.terminalOutput.innerHTML = '';

        for (const log of logs) {
            const entry = document.createElement('div');
            entry.className = 'log-entry';

            const timestamp = document.createElement('span');
            timestamp.className = 'log-timestamp';
            timestamp.textContent = log.timestamp;

            const level = document.createElement('span');
            level.className = `log-level ${log.level.toLowerCase()}`;
            level.textContent = log.level;

            const message = document.createElement('span');
            message.className = 'log-message';
            message.textContent = log.message;

            entry.appendChild(timestamp);
            entry.appendChild(level);
            entry.appendChild(message);
            this.terminalOutput.appendChild(entry);
        }

        // Auto-scroll to bottom
        if (this.autoScroll) {
            this.terminalOutput.scrollTop = this.terminalOutput.scrollHeight;
        }
    }

    async clearLogs() {
        try {
            const response = await fetch('/api/logs/clear', { method: 'POST' });
            if (response.ok) {
                this.terminalOutput.innerHTML = '';
                this.lastLogCount = 0;
            }
        } catch (error) {
            console.error('Error clearing logs:', error);
        }
    }

    destroy() {
        if (this.refreshInterval) {
            clearInterval(this.refreshInterval);
            this.refreshInterval = null;
        }
    }
}
