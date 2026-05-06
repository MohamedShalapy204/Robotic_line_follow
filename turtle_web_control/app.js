let ros;
let cmdVelTopic;
let connected = false;

const statusBadge = document.getElementById('status-badge');
const connectBtn = document.getElementById('connect-btn');
const rosUrlInput = document.getElementById('ros-url');
const linVal = document.getElementById('lin-val');
const angVal = document.getElementById('ang-val');

function updateStatus(isConnected) {
    connected = isConnected;
    if (isConnected) {
        statusBadge.textContent = 'Connected';
        statusBadge.className = 'badge connected';
        connectBtn.textContent = 'Disconnect';
    } else {
        statusBadge.textContent = 'Disconnected';
        statusBadge.className = 'badge disconnected';
        connectBtn.textContent = 'Connect to Robot';
    }
}

function connect() {
    const url = rosUrlInput.value;
    
    ros = new ROSLIB.Ros({
        url: url
    });

    ros.on('connection', () => {
        console.log('Connected to websocket server.');
        updateStatus(true);
        
        cmdVelTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/turtle1/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
    });

    ros.on('error', (error) => {
        console.log('Error connecting to websocket server: ', error);
        updateStatus(false);
    });

    ros.on('close', () => {
        console.log('Connection to websocket server closed.');
        updateStatus(false);
    });
}

function disconnect() {
    if (ros) {
        ros.close();
    }
}

connectBtn.addEventListener('click', () => {
    if (connected) {
        disconnect();
    } else {
        connect();
    }
});

function move(linear, angular) {
    if (!connected) {
        alert('Please connect to ROS first!');
        return;
    }

    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular }
    });

    cmdVelTopic.publish(twist);
    
    linVal.textContent = linear.toFixed(2);
    angVal.textContent = angular.toFixed(2);
}

// Control Event Listeners
document.getElementById('up').addEventListener('click', () => move(2.0, 0.0));
document.getElementById('down').addEventListener('click', () => move(-2.0, 0.0));
document.getElementById('left').addEventListener('click', () => move(0.0, 2.0));
document.getElementById('right').addEventListener('click', () => move(0.0, -2.0));
document.getElementById('stop').addEventListener('click', () => move(0.0, 0.0));

// Keyboard Support
window.addEventListener('keydown', (e) => {
    if (!connected) return;
    
    switch(e.key) {
        case 'ArrowUp': move(2.0, 0.0); break;
        case 'ArrowDown': move(-2.0, 0.0); break;
        case 'ArrowLeft': move(0.0, 2.0); break;
        case 'ArrowRight': move(0.0, -2.0); break;
        case ' ': move(0.0, 0.0); break; // Space for stop
    }
});
