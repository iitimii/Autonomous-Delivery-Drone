var ws = new WebSocket('ws://192.168.8.137:80');
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
        <div class="data-item">Angle Pitch Acc: ${angle_pitch_acc}</div>
        <div class="data-item">Angle Roll Acc: ${angle_roll_acc}</div>
        <div class="data-item">ch1: ${ch1}</div>
        <div class="data-item">ch2: ${ch2}</div>
        <div class="data-item">ch3: ${ch3}</div>
        <div class="data-item">ch4: ${ch4}</div>
        <div class="data-item">ch5: ${ch5}</div>
        <div class="data-item">ch6: ${ch6}</div>
        <div class="data-item">ch7: ${ch7}</div>
        <div class="data-item">ch8: ${ch8}</div>
        <div class="data-item">ch9: ${ch9}</div>
        <div class="data-item">ch10: ${ch10}</div>
        <div class="data-item">Battery Voltage: ${battery_voltage}</div>
    `;
};

displayData();

document.getElementById('sendForm').addEventListener('submit', function(event) {
event.preventDefault();
var inputValue = document.getElementById('inputField').value;
ws.send(inputValue);
});

ws.onopen = function() {
    document.getElementById('connectionStatus').style.color = 'green';
    document.getElementById('connectionStatus').textContent = 'Connected';
};

ws.onclose = function() {
    document.getElementById('connectionStatus').style.color = 'red';
    document.getElementById('connectionStatus').textContent = 'Disconnected';
};

var selectedButton1 = null;
var selectedButton2 = null;

var buttons1 = document.querySelectorAll('.pid-control-buttons .column1');
var buttons2 = document.querySelectorAll('.pid-control-buttons .column2');

buttons1.forEach(function(button) {
button.addEventListener('click', function() {
    if (selectedButton1) {
        selectedButton1.classList.remove('button-selected');
    }

    this.classList.add('button-selected');
    selectedButton1 = this;
});
});

buttons2.forEach(function(button) {
button.addEventListener('click', function() {
    if (selectedButton2) {
        selectedButton2.classList.remove('button-selected');
    }

    this.classList.add('button-selected');
    selectedButton2 = this;
});
});

document.getElementById('sendForm').addEventListener('submit', function(event) {
    event.preventDefault();

    var inputValue = document.getElementById('inputField').value;
    var floatValue = parseFloat(inputValue);

    if (isNaN(floatValue)) {
        alert('Invalid input. Please enter a number.');
        return;
    }

    // if (!selectedButton1 || !selectedButton2) {
    //     alert('Please select a button from each column.');
    //     return;
    // }

    // Use floatValue, selectedButton1, and selectedButton2 here
    console.log(floatValue, selectedButton1, selectedButton2);
    ws.send(floatValue);
});