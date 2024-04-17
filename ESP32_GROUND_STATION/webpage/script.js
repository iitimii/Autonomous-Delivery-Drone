var ws = new WebSocket('ws://172.20.10.2:80');
// var ws = new WebSocket('ws://192.168.4.1:80');
var loop_time, error, ch1, ch2, battery_voltage, angle_pitch, ch3, ch4, angle_roll;
var ch5, ch6, ch7, ch8, ch9, ch10;
var armed, start;
var angle_pitch_acc, angle_roll_acc;

ws.onmessage = function(event) {
    var data = event.data.split(',');
    switch (String.fromCharCode(data[0])){
        case "T":
            loop_time = data[1];
            error = data[2];
            ch1 = data[3];
            ch2 = data[4];
            battery_voltage = data[5];
            angle_pitch = data[6];
            break;

        case "I":
            ch3 = data[1];
            ch4 = data[2];
            ch5 = data[3];
            ch6 = data[4];
            angle_pitch_acc = data[5];
            angle_roll = data[6];
            break;

        case "M":
            ch7 = data[1];
            ch8 = data[2];
            ch9 = data[3];
            ch10 = data[4];
            angle_roll_acc = data[5];
            break;

        case "L":
            armed = data[1];
            start = data[2];
            break;

        default:
            break;
    }
    displayData();
    updateStatusIndicator();
}

function displayData() {
    var container = document.getElementById('dataContainer');
    container.innerHTML = `
        <div class="data-item">Loop Time: ${loop_time}</div>
        <div class="data-item">Error: ${error}</div>
        <div class="data-item">Start: ${start}</div>
        <div class="data-item">Armed: ${armed}</div>
        <div class="data-item">Angle Pitch: ${angle_pitch}</div>
        <div class="data-item">Angle Roll: ${angle_roll}</div>
        <div class="data-item">Battery Voltage: ${battery_voltage}</div>
    `;
};

displayData();

document.getElementById('sendForm').addEventListener('submit', function(event) {
    event.preventDefault();
    var inputValue = document.getElementById('inputField').value;
    var floatValue = parseFloat(inputValue);

    if (isNaN(floatValue)) {
        showResponseMessage('Invalid input. Please enter a number.', 'error');
        return;
    }

    var selectedButton1 = Array.from(document.querySelectorAll('.pid-control-buttons .column:nth-child(1) button')).find(button => button.classList.contains('button-selected'));
    var selectedButton2 = Array.from(document.querySelectorAll('.pid-control-buttons .column:nth-child(2) button')).find(button => button.classList.contains('button-selected'));
    var selectedButton3 = Array.from(document.querySelectorAll('.pid-control-buttons .column:nth-child(3) button')).find(button => button.classList.contains('button-selected'));

    if (!selectedButton1 || !selectedButton2 || !selectedButton3) {
        showResponseMessage('Please select a button from each column.', 'error');
        return;
    }

    var command = `${selectedButton1.textContent},${selectedButton2.textContent},${selectedButton3.textContent},${floatValue}`;
    ws.send(command);
    showResponseMessage(`Sent command: ${command}`, 'success');
});

ws.onopen = function() {
    document.getElementById('connectionStatus').style.color = 'green';
    document.getElementById('connectionStatus').textContent = 'Connected';
};

ws.onclose = function() {
    document.getElementById('connectionStatus').style.color = 'red';
    document.getElementById('connectionStatus').textContent = 'Disconnected';
};

var buttons1 = document.querySelectorAll('.pid-control-buttons .column:nth-child(1) button');
var buttons2 = document.querySelectorAll('.pid-control-buttons .column:nth-child(2) button');
var buttons3 = document.querySelectorAll('.pid-control-buttons .column:nth-child(3) button');

buttons1.forEach(function(button) {
    button.addEventListener('click', function() {
        buttons1.forEach(btn => btn.classList.remove('button-selected'));
        this.classList.add('button-selected');
    });
});

buttons2.forEach(function(button) {
    button.addEventListener('click', function() {
        buttons2.forEach(btn => btn.classList.remove('button-selected'));
        this.classList.add('button-selected');
    });
});

buttons3.forEach(function(button) {
    button.addEventListener('click', function() {
        buttons3.forEach(btn => btn.classList.remove('button-selected'));
        this.classList.add('button-selected');
    });
});

function showResponseMessage(message, type) {
    var responseField = document.getElementById('responseField');
    var responseMessage = document.createElement('div');
    responseMessage.classList.add('response-message', type);
    responseMessage.textContent = message;
    responseField.appendChild(responseMessage);

    setTimeout(function() {
        responseField.removeChild(responseMessage);
    }, 3000);
}
function updateStatusIndicator() {
    var statusIndicator = document.getElementById('statusIndicator');
    if (error !== '0') {
        statusIndicator.style.backgroundColor = 'red';
    } else {
        statusIndicator.style.backgroundColor = 'green';
    }
}