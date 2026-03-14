// UR Controller Web UI - WebSocket Client

export class WebSocketClient {
    constructor(options = {}) {
        this.onStateUpdate = options.onStateUpdate || (() => {});
        this.onConnectionChange = options.onConnectionChange || (() => {});

        this.ws = null;
        this.reconnectInterval = 2000;
        this.reconnectTimer = null;
        this.connected = false;
    }

    connect() {
        // Determine WebSocket URL based on current page location
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host;
        const url = `${protocol}//${host}/ws/state`;

        console.log('Connecting to WebSocket:', url);

        try {
            this.ws = new WebSocket(url);

            this.ws.onopen = () => {
                console.log('WebSocket connected');
                this.connected = true;
                this.onConnectionChange(true);
                this.clearReconnectTimer();
            };

            this.ws.onclose = (event) => {
                console.log('WebSocket closed:', event.code, event.reason);
                this.connected = false;
                this.onConnectionChange(false);
                this.scheduleReconnect();
            };

            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };

            this.ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'state') {
                        this.onStateUpdate(data);
                    }
                } catch (e) {
                    console.error('Failed to parse WebSocket message:', e);
                }
            };
        } catch (e) {
            console.error('Failed to create WebSocket:', e);
            this.scheduleReconnect();
        }
    }

    disconnect() {
        this.clearReconnectTimer();
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
        this.connected = false;
        this.onConnectionChange(false);
    }

    send(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
        }
    }

    scheduleReconnect() {
        this.clearReconnectTimer();
        this.reconnectTimer = setTimeout(() => {
            console.log('Attempting to reconnect...');
            this.connect();
        }, this.reconnectInterval);
    }

    clearReconnectTimer() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
    }

    isConnected() {
        return this.connected;
    }
}
