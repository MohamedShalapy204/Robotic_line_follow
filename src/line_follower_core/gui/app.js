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
const telemetryStatus = document.getElementById('telemetry-status');

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
    startSystemMonitor();
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

// ROS Topics
let lineErrorSub, odomSub, cmdVelPub, missionControlPub, tuningPub;
let encLSub, encRSub, pwmLSub, pwmRSub;

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

    missionControlPub.subscribe((message) => {
        const cmd = message.data.toLowerCase().trim();
        const startBtn = document.getElementById('btn-start-auto');
        if (cmd === 'start') {
            isManualMode = false;
            manualToggle.checked = false;
            joystickContainer.classList.remove('active');
            document.querySelectorAll('.d-btn').forEach(btn => btn.disabled = true);
            stopPublishingCmdVel();
            
            startBtn.innerText = "Autonomous Active!";
            startBtn.style.background = "var(--accent-green)";
            startBtn.style.boxShadow = "0 0 15px var(--accent-green)";
        } else if (cmd === 'stop') {
            startBtn.innerText = "Start Autonomous Lap";
            startBtn.style.background = "";
            startBtn.style.boxShadow = "";
        }
    });

    tuningPub = new ROSLIB.Topic({
        ros: ros,
        name: '/tuning_params',
        messageType: 'std_msgs/String'
    });



    // Telemetry Subscriptions
    lineErrorSub.subscribe((message) => {
        const error = message.data;
        lineErrorVal.innerText = error.toFixed(2);
        const percentage = ((error + 2.0) / 4.0) * 100;
        lineErrorBar.style.width = `${Math.max(0, Math.min(100, percentage))}%`;
    });

    odomSub.subscribe((message) => {
        linearSpeedVal.innerText = message.twist.twist.linear.x.toFixed(2);
        angularSpeedVal.innerText = message.twist.twist.angular.z.toFixed(2);
    });

    // Hardware Debug Subscriptions
    encLSub = new ROSLIB.Topic({ ros: ros, name: '/raw/encoder_l', messageType: 'std_msgs/Int32' });
    encRSub = new ROSLIB.Topic({ ros: ros, name: '/raw/encoder_r', messageType: 'std_msgs/Int32' });
    pwmLSub = new ROSLIB.Topic({ ros: ros, name: '/motor/left/pwm', messageType: 'std_msgs/Int32' });
    pwmRSub = new ROSLIB.Topic({ ros: ros, name: '/motor/right/pwm', messageType: 'std_msgs/Int32' });

    encLSub.subscribe(m => document.getElementById('raw-enc-l').innerText = m.data);
    encRSub.subscribe(m => document.getElementById('raw-enc-r').innerText = m.data);
    pwmLSub.subscribe(m => document.getElementById('raw-pwm-l').innerText = m.data);
    pwmRSub.subscribe(m => document.getElementById('raw-pwm-r').innerText = m.data);
}

function startSystemMonitor() {
    setInterval(() => {
        if (!ros.isConnected) return;

        ros.getNodes((nodes) => {
            const list = document.getElementById('node-list');
            list.innerHTML = nodes.map(n => `<div style="margin-bottom:2px;">• ${n}</div>`).join('');
        });

        ros.getTopics((topics) => {
            const list = document.getElementById('topic-list');
            // topics is an object with {topics: [], types: []} or array depending on version
            const topicNames = topics.topics || topics;
            list.innerHTML = topicNames.map(t => `<div style="margin-bottom:2px;">• ${t}</div>`).join('');
        });
    }, 2000); // Every 2 seconds
}

// Manual Control State Toggle
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
    }, 100);
}

function stopPublishingCmdVel() {
    if (cmdVelInterval) {
        clearInterval(cmdVelInterval);
        cmdVelInterval = null;
    }
}

// ==========================================
// --- IMPROVED MANUAL CONTROL LOGIC ---
// ==========================================

// 1. Track the exact state of each key
const keys = { w: false, a: false, s: false, d: false };

function updateTwistFromKeys() {
    let linear = 0;
    let angular = 0;

    // Combine inputs (holding W and S cancels out, holding W and A curves)
    if (keys.w) linear += 0.5;
    if (keys.s) linear -= 0.5;
    if (keys.a) angular += 1.0;
    if (keys.d) angular -= 1.0;

    currentTwist.linear.x = linear;
    currentTwist.angular.z = angular;
}

// 2. Keyboard Events
window.addEventListener('keydown', (e) => {
    // e.repeat ignores the continuous firing when a key is held down by the OS
    if (!isManualMode || e.repeat) return;
    const key = e.key.toLowerCase();

    if (keys.hasOwnProperty(key)) {
        keys[key] = true;
        updateTwistFromKeys();
    }
});

window.addEventListener('keyup', (e) => {
    if (!isManualMode) return;
    const key = e.key.toLowerCase();

    if (keys.hasOwnProperty(key)) {
        keys[key] = false;
        updateTwistFromKeys();
    }
});

// 3. Screen Button Events (Mouse + Touch + Failsafe)
function setupDriveButton(id, lin_val, ang_val) {
    const btn = document.getElementById(id);

    const startAction = (e) => {
        if (!isManualMode) return;
        e.preventDefault(); // Prevents touch from double-firing as a mouse click
        currentTwist.linear.x = lin_val;
        currentTwist.angular.z = ang_val;
    };

    const stopAction = (e) => {
        if (!isManualMode) return;
        e.preventDefault();
        currentTwist.linear.x = 0;
        currentTwist.angular.z = 0;
    };

    // Trigger movement on click or touch
    btn.addEventListener('mousedown', startAction);
    btn.addEventListener('touchstart', startAction, { passive: false });

    // Stop movement on release
    btn.addEventListener('mouseup', stopAction);
    btn.addEventListener('touchend', stopAction);

    // CRITICAL FAILSAFE: Stop if cursor/finger slides off the button
    btn.addEventListener('mouseleave', stopAction);
    btn.addEventListener('touchcancel', stopAction);
}

setupDriveButton('btn-up', 0.8, 0);
setupDriveButton('btn-down', -0.8, 0);
setupDriveButton('btn-left', 0, 5.0);
setupDriveButton('btn-right', 0, -5.0);


// ==========================================
// --- MISSION CONTROL & TUNING LOGIC ---
// ==========================================

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
        kd: parseFloat(document.getElementById('tune-kd').value),
        sensor_threshold: parseInt(document.getElementById('tune-sensor-threshold').value),
        kickstart_enabled: document.getElementById('tune-kickstart').checked
    };

    tuningPub.publish(new ROSLIB.Message({ data: JSON.stringify(params) }));
    document.getElementById('threshold-display').innerText = params.sensor_threshold;

    const btn = document.getElementById('btn-apply-tuning');
    const originalText = btn.innerText;
    btn.innerText = 'Synced!';
    btn.style.background = 'var(--accent-green)';
    setTimeout(() => {
        btn.innerText = originalText;
        btn.style.background = '';
    }, 1000);
});

// Slider Value Display Listener
document.getElementById('tune-sensor-threshold').addEventListener('input', (e) => {
    document.getElementById('sensor-threshold-val').innerText = e.target.value;
});