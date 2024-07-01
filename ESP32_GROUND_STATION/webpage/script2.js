document.addEventListener('DOMContentLoaded', () => {
    const connectionStatus = document.getElementById('connectionStatus');
    const loopTimeElement = document.getElementById('loop_time');
    const errorElement = document.getElementById('error');
    const startElement = document.getElementById('start');
    const armedElement = document.getElementById('armed');
    const flightModeElement = document.getElementById('flight_mode');
    const batteryVoltageElement = document.getElementById('battery_voltage');
    const pitchElement = document.getElementById('pitch');
    const rollElement = document.getElementById('roll');
    const yawElement = document.getElementById('yaw');
    const pitchRateElement = document.getElementById('pitch_rate');
    const rollRateElement = document.getElementById('roll_rate');
    const yawRateElement = document.getElementById('yaw_rate');
    const pitchAccelElement = document.getElementById('pitch_accel');
    const rollAccelElement = document.getElementById('roll_accel');
    const altitudeElement = document.getElementById('altitude');
    const gpsLongitudeElement = document.getElementById('longitude');
    const gpsLatitudeElement = document.getElementById('latitude');
    const compassHeadingElement = document.getElementById('compass_heading');
    const headingLockElement = document.getElementById('heading_lock');
    const numSatellitesElement = document.getElementById('num_sats');
    const fixTypeElement = document.getElementById('fix_type');
    const temperatureElement = document.getElementById('temperature');
    const airPressureElement = document.getElementById('air_pressure');
    const channel1Element = document.getElementById('channel1');
    const channel2Element = document.getElementById('channel2');
    const channel3Element = document.getElementById('channel3');
    const channel4Element = document.getElementById('channel4');
    const channel5Element = document.getElementById('channel5');
    const channel6Element = document.getElementById('channel6');
    const channel7Element = document.getElementById('channel7');
    const channel8Element = document.getElementById('channel8');
    const channel9Element = document.getElementById('channel9');
    const channel10Element = document.getElementById('channel10');
    const motorFrontLeftElement = document.getElementById('front_left');
    const motorFrontRightElement = document.getElementById('front_right');
    const motorBackRightElement = document.getElementById('back_right');
    const motorBackLeftElement = document.getElementById('back_left');
    const pidGainRatePitchKpElement = document.getElementById('rate_pitch_kp');
    const pidGainRatePitchKiElement = document.getElementById('rate_pitch_ki');
    const pidGainRatePitchKdElement = document.getElementById('rate_pitch_kd');
    const pidGainRateRollKpElement = document.getElementById('rate_roll_kp');
    const pidGainRateRollKiElement = document.getElementById('rate_roll_ki');
    const pidGainRateRollKdElement = document.getElementById('rate_roll_kd');
    const pidGainRateYawKpElement = document.getElementById('rate_yaw_kp');
    const pidGainRateYawKiElement = document.getElementById('rate_yaw_ki');
    const pidGainRateYawKdElement = document.getElementById('rate_yaw_kd');
    const pidGainAnglePitchKpElement = document.getElementById('angle_pitch_kp');
    const pidGainAnglePitchKiElement = document.getElementById('angle_pitch_ki');
    const pidGainAnglePitchKdElement = document.getElementById('angle_pitch_kd');
    const pidGainAngleRollKpElement = document.getElementById('angle_roll_kp');
    const pidGainAngleRollKiElement = document.getElementById('angle_roll_ki');
    const pidGainAngleRollKdElement = document.getElementById('angle_roll_kd');
    const pitchOutputElement = document.getElementById('pitch_output');
    const rollOutputElement = document.getElementById('roll_output');
    const throttleOutputElement = document.getElementById('throttle_output');
    const yawOutputElement = document.getElementById('yaw_output');
    const anglePitchOutputElement = document.getElementById('angle_pitch_output');
    const angleRollOutputElement = document.getElementById('angle_roll_output');

    const ws = new WebSocket('ws://gs.local:80');

    ws.onopen = () => {
        connectionStatus.textContent = 'Connected';
        connectionStatus.className = 'connected';
    };

    ws.onclose = () => {
        connectionStatus.textContent = 'Disconnected';
        connectionStatus.className = 'disconnected';
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);

        switch (String.fromCharCode(data.signature)) {
            case 'T':
                loopTimeElement.textContent = data.payload1;
                errorElement.textContent = data.payload2;
                startElement.textContent = data.payload3;
                batteryVoltageElement.textContent = data.payload4;
                pitchElement.textContent = data.payload5;
                rollElement.textContent = data.payload6;
                yawElement.textContent = data.payload7;
                pitchAccelElement.textContent = data.payload8;
                rollAccelElement.textContent = data.payload9;
                break;
            case 'I':
                armedElement.textContent = data.payload1;
                flightModeElement.textContent = data.payload2;
                headingLockElement.textContent = data.payload3;
                pitchRateElement.textContent = data.payload4;
                rollRateElement.textContent = data.payload5;
                yawRateElement.textContent = data.payload6;
                gpsLatitudeElement.textContent = data.payload7;
                gpsLongitudeElement.textContent = data.payload8;
                compassHeadingElement.textContent = data.payload9;
                break;
            case 'M':
                numSatellitesElement.textContent = data.payload1;
                fixTypeElement.textContent = data.payload2;
                airPressureElement.textContent = data.payload4;
                altitudeElement.textContent = data.payload5;
                temperatureElement.textContent = data.payload6;
                channel1Element.textContent = data.payload7;
                channel2Element.textContent = data.payload8;
                channel3Element.textContent = data.payload9;
                break;
            case 'L':
                channel4Element.textContent = data.payload1;
                channel5Element.textContent = data.payload2;
                channel6Element.textContent = data.payload3;
                channel7Element.textContent = data.payload4;
                channel8Element.textContent = data.payload5;
                channel9Element.textContent = data.payload6;
                channel10Element.textContent = data.payload7;
                pitchOutputElement.textContent = data.payload8;
                rollOutputElement.textContent = data.payload9;
                break;
            case 'H':
                motorBackRightElement.textContent = data.payload1;
                motorBackLeftElement.textContent = data.payload2;
                throttleOutputElement.textContent = data.payload3;
                yawOutputElement.textContent = data.payload4;
                anglePitchOutputElement.textContent = data.payload5;
                angleRollOutputElement.textContent = data.payload6;
                pidGainRatePitchKpElement.textContent = data.payload7;
                pidGainRatePitchKiElement.textContent = data.payload8;
                pidGainRatePitchKdElement.textContent = data.payload9;
                break;
            case 'N':
                motorFrontLeftElement.textContent = data.payload1;
                motorFrontRightElement.textContent = data.payload2;
                pidGainRateRollKpElement.textContent = data.payload4;
                pidGainRateRollKiElement.textContent = data.payload5;
                pidGainRateRollKdElement.textContent = data.payload6;
                pidGainRateYawKpElement.textContent = data.payload7;
                pidGainRateYawKiElement.textContent = data.payload8;
                pidGainRateYawKdElement.textContent = data.payload9;
                break;
            case 'O':
                pidGainAnglePitchKpElement.textContent = data.payload4;
                pidGainAnglePitchKiElement.textContent = data.payload5;
                pidGainAnglePitchKdElement.textContent = data.payload6;
                pidGainAngleRollKpElement.textContent = data.payload7;
                pidGainAngleRollKiElement.textContent = data.payload8;
                pidGainAngleRollKdElement.textContent = data.payload9;
                break;
            default:
                break;
        }
    };

    const sendForm = document.getElementById('sendForm');
    const inputField = document.getElementById('inputField');
    const responseField = document.getElementById('responseField');

    sendForm.addEventListener('submit', (event) => {
        event.preventDefault();
        const inputValue = inputField.value;
        const floatValue = parseFloat(inputValue);

        if (isNaN(floatValue)) {
            showResponseMessage('Invalid input. Please enter a number.', 'error');
            return;
        }

        const selectedButtons = document.querySelectorAll('.pid-control-buttons .button-selected');
        if (selectedButtons.length !== 3) {
            showResponseMessage('Please select a button from each column.', 'error');
            return;
        }

        const selectedButtonValues = Array.from(selectedButtons).map(button => button.textContent);
        const command = `${selectedButtonValues.join(',')},${floatValue}`;
        ws.send(JSON.stringify({ type: 'PID', message: command }));
        showResponseMessage(`Sent command: ${command}`, 'success');
    });

    const customMessageInput = document.getElementById('customMessageInput');
    const sendCustomMessageButton = document.getElementById('sendCustomMessageButton');
    const customMessageResponseField = document.getElementById('customMessageResponseField');

    sendCustomMessageButton.addEventListener('click', () => {
        const message = customMessageInput.value;
        ws.send(JSON.stringify({ type: 'custom', message: message }));
        customMessageResponseField.textContent = `Sent custom message: ${message}`;
    });

    function showResponseMessage(message, type) {
        const responseMessage = document.createElement('div');
        responseMessage.classList.add('response-message', type);
        responseMessage.textContent = message;
        responseField.appendChild(responseMessage);
        setTimeout(() => {
            responseField.removeChild(responseMessage);
        }, 3000);
    }

    const pidControlButtons = document.querySelectorAll('.pid-control-buttons .column button');

    pidControlButtons.forEach(button => {
        button.addEventListener('click', () => {
            const columnButtons = button.parentElement.querySelectorAll('button');
            columnButtons.forEach(btn => btn.classList.remove('button-selected'));
            button.classList.add('button-selected');
        });
    });
});
