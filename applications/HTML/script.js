document.addEventListener('DOMContentLoaded', () => {
    const websocket = new WebSocket('ws://localhost:8765');

    // --- DOM Elements ---
    const ball = document.getElementById('ball');
    const targetLine = document.getElementById('target-line');
    const tunnel = document.querySelector('.tunnel');
    const targetHeightInput = document.getElementById('target_height_input');
    const kpInput = document.getElementById('kp_input');
    const kiInput = document.getElementById('ki_input');
    const kdInput = document.getElementById('kd_input');
    const setTargetBtn = document.getElementById('set_target_btn');
    const setPidBtn = document.getElementById('set_pid_btn');
    const ffTableBody = document.getElementById('ff_table_body');

    // --- Constants ---
    const TUNNEL_HEIGHT_PX = 400;
    const MAX_SENSOR_HEIGHT_MM = 500;
    let isFirstMessage = true;

    // --- Initial Data (from main.c) ---
    const ff_table_initial = [
        { height: 50.0, base_fan_speed: 0.35 },
        { height: 100.0, base_fan_speed: 0.35 },
        { height: 150.0, base_fan_speed: 0.35 },
        { height: 200.0, base_fan_speed: 0.35 },
        { height: 250.0, base_fan_speed: 0.3 },
        { height: 300.0, base_fan_speed: 0.3 },
        { height: 350.0, base_fan_speed: 0.3 },
        { height: 400.0, base_fan_speed: 0.3 },
        { height: 450.0, base_fan_speed: 0.3 }
    ];

    // --- WebSocket Handlers ---
    websocket.onopen = () => {
        console.log('Connected to WebSocket server');
        renderFeedforwardTable();
    };

    websocket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (isFirstMessage) {
                initializeControlPanel(data);
                isFirstMessage = false;
            }
            updateDashboard(data);
            updateAnimation(data);
        } catch (error) {
            console.error('Error parsing JSON or updating UI:', error);
        }
    };

    websocket.onclose = () => {
        console.log('Disconnected from WebSocket server');
    };

    websocket.onerror = (error) => {
        console.error('WebSocket Error:', error);
    };

    // --- Event Listeners ---
    setTargetBtn.addEventListener('click', () => {
        const t = targetHeightInput.value;
        if (t) {
            const command = `pid_tune -t ${t}`;
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        }
    });

    setPidBtn.addEventListener('click', () => {
        const p = kpInput.value;
        const i = kiInput.value;
        const d = kdInput.value;
        let command = 'pid_tune';

        if (p) command += ` -p ${p}`;
        if (i) command += ` -i ${i}`;
        if (d) command += ` -d ${d}`;

        if (command !== 'pid_tune') {
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        }
    });

    // --- UI Update Functions ---
    function initializeControlPanel(data) {
        targetHeightInput.value = data.target_height.toFixed(1);
        kpInput.value = data.pid_kp.toFixed(6);
        kiInput.value = data.pid_ki.toFixed(6);
        kdInput.value = data.pid_kd.toFixed(6);
    }

    function updateDashboard(data) {
        for (const key in data) {
            const element = document.getElementById(key);
            if (element) {
                let value = data[key];
                if (typeof value === 'number' && !Number.isInteger(value)) {
                    value = value.toFixed(6);
                }
                element.textContent = value;
            }
        }
    }

    function updateAnimation(data) {
        const { current_height, target_height } = data;
        const scale = TUNNEL_HEIGHT_PX / MAX_SENSOR_HEIGHT_MM;
        const ballPosition = Math.min(current_height * scale - (ball.offsetHeight / 2), TUNNEL_HEIGHT_PX - ball.offsetHeight);
        const targetPosition = Math.min(target_height * scale, TUNNEL_HEIGHT_PX);
        ball.style.bottom = `${Math.max(0, ballPosition)}px`;
        targetLine.style.bottom = `${Math.max(0, targetPosition)}px`;
    }

    function renderFeedforwardTable() {
        ffTableBody.innerHTML = ''; // Clear existing rows
        ff_table_initial.forEach((entry, index) => {
            const row = document.createElement('tr');
            row.innerHTML = `
                <td>${index}</td>
                <td><input type="number" class="control-input" value="${entry.height.toFixed(1)}" id="ff-h-${index}"></td>
                <td><input type="number" class="control-input" value="${entry.base_fan_speed.toFixed(4)}" id="ff-s-${index}" step="0.01"></td>
                <td><button class="control-button ff-update-btn" data-index="${index}">Update</button></td>
            `;
            ffTableBody.appendChild(row);
        });

        // Add event listeners to the new buttons
        document.querySelectorAll('.ff-update-btn').forEach(button => {
            button.addEventListener('click', (e) => {
                const index = e.target.dataset.index;
                const height = document.getElementById(`ff-h-${index}`).value;
                const speed = document.getElementById(`ff-s-${index}`).value;
                if (height && speed) {
                    const command = `pid_tune -ff_set ${index} ${height} ${speed}`;
                    console.log(`Sending command: ${command}`);
                    websocket.send(command);
                }
            });
        });
    }
});