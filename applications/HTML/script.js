document.addEventListener('DOMContentLoaded', () => {
    const websocket = new WebSocket('ws://localhost:8765');

    const ball = document.getElementById('ball');
    const targetLine = document.getElementById('target-line');
    const tunnel = document.querySelector('.tunnel');

    // 定义风洞的高度（像素）和传感器可测量的最大高度（毫米）
    const TUNNEL_HEIGHT_PX = 500;
    const MAX_SENSOR_HEIGHT_MM = 500;

    websocket.onopen = () => {
        console.log('Connected to WebSocket server');
    };

    websocket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
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

    function updateDashboard(data) {
        for (const key in data) {
            const element = document.getElementById(key);
            if (element) {
                let value = data[key];
                // 对浮点数进行格式化，保留合适的位数
                if (typeof value === 'number' && !Number.isInteger(value)) {
                    value = value.toFixed(6);
                }
                element.textContent = value;
            }
        }
    }

    function updateAnimation(data) {
        const { current_height, target_height } = data;

        // 将高度从毫米转换为在风洞div中的像素位置
        // 计算比例因子
        const scale = TUNNEL_HEIGHT_PX / MAX_SENSOR_HEIGHT_MM;

        // 计算小球和目标线的位置（bottom值）
        // 我们需要减去小球自身的高度的一半，使其中心对准高度
        const ballPosition = Math.min(current_height * scale - (ball.offsetHeight / 2), TUNNEL_HEIGHT_PX - ball.offsetHeight);
        const targetPosition = Math.min(target_height * scale, TUNNEL_HEIGHT_PX);

        // 保证位置不小于0
        ball.style.bottom = `${Math.max(0, ballPosition)}px`;
        targetLine.style.bottom = `${Math.max(0, targetPosition)}px`;
    }
});