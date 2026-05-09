// UI Elements
const rosUrlInput = document.getElementById('ros-url');
const connectBtn = document.getElementById('connect-btn');
const statusText = document.getElementById('ros-status-text');
const statusIndicator = document.getElementById('connection-status');
const lineErrorVal = document.getElementById('line-error-value');
const lineErrorBar = document.getElementById('line-error-bar');
const linearSpeedVal = document.getElementById('linear-speed');
const angularSpeedVal = document.getElementById('angular-speed');
const manualToggle = document.getElementById('manual-mode-toggle');
const joystickContainer = document.getElementById('joystick-container');

// State Variables
let isManualMode = false;
let cmdVelInterval = null;
let currentTwist = {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
};

// Initialize ROS
const ros = new ROSLIB.Ros();

// ROS Connection Callbacks
ros.on('connection', () => {
    statusText.innerText = 'Connected';
    statusText.style.color = '#10b981';
    statusIndicator.classList.add('connected');
    connectBtn.innerText = 'Connected';
    connectBtn.classList.remove('btn-primary');
    connectBtn.classList.add('btn-success');
    connectBtn.style.backgroundColor = '#10b981';
    console.log('Connected to websocket server.');
    
    // Subscribe/Advertise after connection
    subscribeToTopics();
});

ros.on('error', (error) => {
    statusText.innerText = 'Error Connecting';
    statusText.style.color = '#ef4444';
    statusIndicator.classList.remove('connected');
    connectBtn.innerText = 'Connect to ROS';
    alert('Error connecting to ROSBridge. Make sure it is running.');
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', () => {
    statusText.innerText = 'Disconnected';
    statusText.style.color = '#94a3b8';
    statusIndicator.classList.remove('connected');
    connectBtn.innerText = 'Connect to ROS';
    connectBtn.style.backgroundColor = '';
    console.log('Connection to websocket server closed.');
});

// Connect Button Event
connectBtn.addEventListener('click', () => {
    const url = rosUrlInput.value;
    console.log('Connecting to: ', url);
    ros.connect(url);
});

// ROS Topics (will be initialized after connection)
let lineErrorSub, odomSub, cmdVelPub, missionControlPub, tuningPub;
let sensorSubs = {};

function subscribeToTopics() {
    lineErrorSub = new ROSLIB.Topic({
        ros: ros,
        name: '/line_error',
        messageType: 'std_msgs/Float32'
    });

    odomSub = new ROSLIB.Topic({
        ros: ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
    });

    cmdVelPub = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    missionControlPub = new ROSLIB.Topic({
        ros: ros,
        name: '/mission_control',
        messageType: 'std_msgs/String'
    });

    tuningPub = new ROSLIB.Topic({
        ros: ros,
        name: '/tuning_params',
        messageType: 'std_msgs/String'
    });

    // Sensor Topics
    const sensorNames = ['l2', 'l1', 'mid', 'r1', 'r2'];
    const threshold = 0.015;
    document.getElementById('threshold-display').innerText = threshold.toFixed(3);

    sensorNames.forEach(name => {
        const topicName = `/sensor_${name}`;
        sensorSubs[name] = new ROSLIB.Topic({
            ros: ros,
            name: topicName,
            messageType: 'sensor_msgs/LaserScan'
        });

        sensorSubs[name].subscribe((message) => {
            const dist = message.ranges[0];
            const indicator = document.getElementById(`sensor-${name}-ui`);
            const valueSpan = document.getElementById(`val-${name}`);
            
            valueSpan.innerText = dist.toFixed(3);
            
            if (dist < threshold) {
                indicator.classList.add('active');
            } else {
                indicator.classList.remove('active');
            }
        });
    });

    // Telemetry Subscriptions
    lineErrorSub.subscribe((message) => {
        const error = message.data;
        lineErrorVal.innerText = error.toFixed(2);
        
        // Map -2.0 to 2.0 to 0% to 100%
        const percentage = ((error + 2.0) / 4.0) * 100;
        lineErrorBar.style.width = `${Math.max(0, Math.min(100, percentage))}%`;
    });

    odomSub.subscribe((message) => {
        const linear = message.twist.twist.linear.x;
        const angular = message.twist.twist.angular.z;
        
        linearSpeedVal.innerText = linear.toFixed(2);
        angularSpeedVal.innerText = angular.toFixed(2);
    });
}

// Manual Control Logic
manualToggle.addEventListener('change', (e) => {
    isManualMode = e.target.checked;
    if (isManualMode) {
        joystickContainer.classList.add('active');
        document.querySelectorAll('.d-btn').forEach(btn => btn.disabled = false);
        startPublishingCmdVel();
        if (missionControlPub) {
            missionControlPub.publish(new ROSLIB.Message({ data: 'stop' }));
        }
    } else {
        joystickContainer.classList.remove('active');
        document.querySelectorAll('.d-btn').forEach(btn => btn.disabled = true);
        stopPublishingCmdVel();
        // Send a stop command
        publishTwist(0, 0);
    }
});

function publishTwist(linear, angular) {
    if (!cmdVelPub) return;
    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular }
    });
    cmdVelPub.publish(twist);
}

function startPublishingCmdVel() {
    if (cmdVelInterval) clearInterval(cmdVelInterval);
    cmdVelInterval = setInterval(() => {
        if (isManualMode) {
            publishTwist(currentTwist.linear.x, currentTwist.angular.z);
        }
    }, 100); // 10Hz
}

function stopPublishingCmdVel() {
    if (cmdVelInterval) {
        clearInterval(cmdVelInterval);
        cmdVelInterval = null;
    }
}

// Button Events
document.getElementById('btn-up').addEventListener('mousedown', () => { currentTwist.linear.x = 0.5; });
document.getElementById('btn-up').addEventListener('mouseup', () => { currentTwist.linear.x = 0; });
document.getElementById('btn-down').addEventListener('mousedown', () => { currentTwist.linear.x = -0.5; });
document.getElementById('btn-down').addEventListener('mouseup', () => { currentTwist.linear.x = 0; });
document.getElementById('btn-left').addEventListener('mousedown', () => { currentTwist.angular.z = 1.0; });
document.getElementById('btn-left').addEventListener('mouseup', () => { currentTwist.angular.z = 0; });
document.getElementById('btn-right').addEventListener('mousedown', () => { currentTwist.angular.z = -1.0; });
document.getElementById('btn-right').addEventListener('mouseup', () => { currentTwist.angular.z = 0; });

// Key Events
window.addEventListener('keydown', (e) => {
    if (!isManualMode) return;
    switch(e.key.toLowerCase()) {
        case 'w': currentTwist.linear.x = 0.5; break;
        case 's': currentTwist.linear.x = -0.5; break;
        case 'a': currentTwist.angular.z = 1.0; break;
        case 'd': currentTwist.angular.z = -1.0; break;
    }
});

window.addEventListener('keyup', (e) => {
    if (!isManualMode) return;
    switch(e.key.toLowerCase()) {
        case 'w': if (currentTwist.linear.x > 0) currentTwist.linear.x = 0; break;
        case 's': if (currentTwist.linear.x < 0) currentTwist.linear.x = 0; break;
        case 'a': if (currentTwist.angular.z > 0) currentTwist.angular.z = 0; break;
        case 'd': if (currentTwist.angular.z < 0) currentTwist.angular.z = 0; break;
    }
});

// Mission Control Buttons
document.getElementById('btn-start-auto').addEventListener('click', () => {
    if (missionControlPub) {
        missionControlPub.publish(new ROSLIB.Message({ data: 'start' }));
    }
});

document.getElementById('btn-estop').addEventListener('click', () => {
    isManualMode = false;
    manualToggle.checked = false;
    joystickContainer.classList.remove('active');
    document.querySelectorAll('.d-btn').forEach(btn => btn.disabled = true);
    stopPublishingCmdVel();
    publishTwist(0, 0);
    if (missionControlPub) {
        missionControlPub.publish(new ROSLIB.Message({ data: 'stop' }));
    }
});

document.getElementById('btn-calibrate').addEventListener('click', () => {
    alert('Sensor Calibration Triggered');
});

// Tuning Apply Button
document.getElementById('btn-apply-tuning').addEventListener('click', () => {
    if (!tuningPub) {
        alert('Not connected to ROS!');
        return;
    }
    
    const params = {
        base_speed: parseFloat(document.getElementById('tune-speed').value),
        kp: parseFloat(document.getElementById('tune-kp').value),
        ki: parseFloat(document.getElementById('tune-ki').value),
        kd: parseFloat(document.getElementById('tune-kd').value)
    };
    
    tuningPub.publish(new ROSLIB.Message({ data: JSON.stringify(params) }));
    
    // Quick visual feedback on button
    const btn = document.getElementById('btn-apply-tuning');
    const originalText = btn.innerText;
    btn.innerText = 'Applied!';
    btn.style.background = 'var(--accent-green)';
    setTimeout(() => {
        btn.innerText = originalText;
        btn.style.background = '';
    }, 1000);
});
