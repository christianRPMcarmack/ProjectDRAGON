// ROS CONNECTION ///////////////////////////////////////////////////////////////////////////

// Error Handling
var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var teleop;
var ros;
var marker;

// Sleep Function
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// Subscriber
var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/gps',
    messageType : '/twist/linear'
});

// Kill Function
function kill() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVel.advertise();
    kill();
}

// Move Action
function moveAction(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    cmdVel.publish(twist);
}

// Velocity Publisher
function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVel.advertise();
}

// Keyboard
function initTeleopKeyboard() {
    // Use w, s, a, d keys to drive your robot

    // Check if keyboard controller was aready created
    if (teleop == null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel'
        });
    }

    // Add event listener for slider moves
    robotSpeedRange = document.getElementById("robot-speed");
    robotSpeedRange.oninput = function () {
        teleop.scale = robotSpeedRange.value / 100
    }
}

// Create Joystick
function createJoystick() {
    // Check if joystick was aready created
    if (manager == null) {
        joystickContainer = document.getElementById('joystick');
        // joystck configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 50 + '%', top: 105 + 'px' },
            mode: 'static',
            size: 200,
            color: '#0066ff',
            restJoystick: true
        };
        manager = nipplejs.create(options);
        // event listener for joystick move
        manager.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordiantes
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if youwant robot to drvie faster or slower
            var lin = Math.cos(direction / 57.29) * nipple.distance * 0.005;
            var ang = Math.sin(direction / 57.29) * nipple.distance * 0.05;
            // nipplejs is triggering events when joystic moves each pixel
            // we need delay between consecutive messege publications to 
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(lin, ang);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        manager.on('end', function () {
            moveAction(0, 0);
        });
    }
}

// Connect to ROS
window.onload = function () {
    // determine robot address automatically
    // robot_IP = location.hostname;
    // set robot address statically
    robot_IP = "localhost";

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });

    initVelocityPublisher();
    createJoystick();   
    initTeleopKeyboard();
};


// GOOGLE MAPS API //////////////////////////////////////////////////////////////////////////

// Google Maps 
function initMap() {
    // Setup Initial Location and Zoom of Map
    var myLatLng = {lat: 40.001, lng: -105.26};
    var map = new google.maps.Map(document.getElementById('map'), {
      zoom: 18,
      center: myLatLng,
      mapTypeId: 'satellite'
    });
    map.setTilt(0);

    // Grab GPS Coordinates
    function getGPS(){
        try{
        listener.subscribe(function(message) {
            console.log('Received message on ' + listener.name + ': ' + message.data);
            listener.unsubscribe();
        });
        return [listener.lat, listener.lon];
        }catch(e){
            console.log("No ROS WebSocket on Specified IP:port")
            return [40, -105.261];
        }
    }

    // Add Marker Function
    this.gpsplot = async function plotGPS(){
    
        var gpslatlng = getGPS();
        console.table(gpslatlng)
        var latlng = new google.maps.LatLng(gpslatlng[0],gpslatlng[1]);
        console.table(latlng)
        

        var marker = new google.maps.Marker({
            position: latlng,
            title:"!"
        });
           
        // To add the marker to the map, call setMap();
        marker.setMap(map);
        await sleep(3000);
        plotGPS();
    }
}