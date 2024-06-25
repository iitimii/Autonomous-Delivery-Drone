document.addEventListener('DOMContentLoaded', (event) => {
    const connectionStatus = document.getElementById('connectionStatus');
    const loopTimeElement = document.getElementById('loop_time');
    const errorElement = document.getElementById('error');
    const startElement = document.getElementById('start');
    const armedElement = document.getElementById('armed');
    const anglePitchElement = document.getElementById('angle_pitch');
    const angleRollElement = document.getElementById('angle_roll');
    const batteryVoltageElement = document.getElementById('battery_voltage');
    const sendForm = document.getElementById('sendForm');
    const inputField = document.getElementById('inputField');
    const responseField = document.getElementById('responseField');
    const customMessageInput = document.getElementById('customMessageInput');
    const sendCustomMessageButton = document.getElementById('sendCustomMessageButton');
    const customMessageResponseField = document.getElementById('customMessageResponseField');

    const ws = new WebSocket('ws://gs.local:80');
    let loop_time = 'N/A', error = 'N/A', battery_voltage = 'N/A';
    let ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10;
    let armed = 'N/A', start = 'N/A';
    let angle_pitch_acc, angle_roll_acc, angle_pitch = 'N/A', angle_roll = 'N/A';

    ws.onopen = () => {
        connectionStatus.textContent = 'Connected';
        connectionStatus.className = 'connected';
    };

    ws.onclose = () => {
        connectionStatus.textContent = 'Disconnected';
        connectionStatus.className = 'disconnected';
    };

    ws.onmessage = (event) => {
        const data = event.data.split(',');
        switch (String.fromCharCode(data[0])) {
            case 'T':
                loop_time = data[1];
                error = data[2];
                ch1 = data[3];
                ch2 = data[4];
                battery_voltage = data[5];
                angle_pitch = data[6];
                break;
            case 'I':
                ch3 = data[1];
                ch4 = data[2];
                ch5 = data[3];
                ch6 = data[4];
                angle_pitch_acc = data[5];
                angle_roll = data[6];
                break;
            case 'M':
                ch7 = data[1];
                ch8 = data[2];
                ch9 = data[3];
                ch10 = data[4];
                angle_roll_acc = data[5];
                break;
            case 'L':
                armed = data[1];
                start = data[2];
                break;
            default:
                break;
        }
        updateTelemetryData();
    };

    function updateTelemetryData() {
        loopTimeElement.textContent = loop_time;
        errorElement.textContent = error;
        startElement.textContent = start;
        armedElement.textContent = armed;
        anglePitchElement.textContent = angle_pitch;
        angleRollElement.textContent = angle_roll;
        batteryVoltageElement.textContent = battery_voltage;
    }

    function showResponseMessage(message, type) {
        const responseMessage = document.createElement('div');
        responseMessage.classList.add('response-message', type);
        responseMessage.textContent = message;
        responseField.appendChild(responseMessage);
        setTimeout(() => {
            responseField.removeChild(responseMessage);
        }, 3000);
    }

    sendForm.addEventListener('submit', (event) => {
        event.preventDefault();
        const inputValue = inputField.value;
        const floatValue = parseFloat(inputValue);

        if (isNaN(floatValue)) {
            showResponseMessage('Invalid input. Please enter a number.', 'error');
            return;
        }

        const selectedButton1 = document.querySelector('.pid-control-buttons .column:nth-child(1) .button-selected');
        const selectedButton2 = document.querySelector('.pid-control-buttons .column:nth-child(2) .button-selected');
        const selectedButton3 = document.querySelector('.pid-control-buttons .column:nth-child(3) .button-selected');

        if (!selectedButton1 || !selectedButton2 || !selectedButton3) {
            showResponseMessage('Please select a button from each column.', 'error');
            return;
        }

        const command = `${selectedButton1.textContent},${selectedButton2.textContent},${selectedButton3.textContent},${floatValue}`;
        // ws.send(command);
        ws.send(JSON.stringify({ type: 'PID', message: command }));
        showResponseMessage(`Sent command: ${command}`, 'success');
    });

    sendCustomMessageButton.addEventListener('click', () => {
        const message = customMessageInput.value;
        ws.send(JSON.stringify({ type: 'custom', message: message }));
        customMessageResponseField.textContent = `Sent custom message: ${message}`;
    });

    const buttons1 = document.querySelectorAll('.pid-control-buttons .column:nth-child(1) button');
    const buttons2 = document.querySelectorAll('.pid-control-buttons .column:nth-child(2) button');
    const buttons3 = document.querySelectorAll('.pid-control-buttons .column:nth-child(3) button');

    buttons1.forEach((button) => {
        button.addEventListener('click', function() {
            buttons1.forEach((btn) => btn.classList.remove('button-selected'));
            this.classList.add('button-selected');
        });
    });

    buttons2.forEach((button) => {
        button.addEventListener('click', function() {
            buttons2.forEach((btn) => btn.classList.remove('button-selected'));
            this.classList.add('button-selected');
        });
    });

    buttons3.forEach((button) => {
        button.addEventListener('click', function() {
            buttons3.forEach((btn) => btn.classList.remove('button-selected'));
            this.classList.add('button-selected');
        });
    });
});
