/*  RobotCtrl.js - Version 1.0 2013-09-29

    A simple HTML5/rosbridge script to control and monitor a ROS robot

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
 */

// Set the rosbridge and mjpeg_server port
var rosbridgePort = "9090";

// Get the current hostname
thisHostName = document.location.hostname;

// Set the rosbridge and mjpeg server hostname accordingly
var rosbridgeHost = thisHostName;

// Build the websocket URL to the rosbride server
var serverURL = "ws://" + rosbridgeHost + ":" + rosbridgePort;

// A varible to hold the chatter topic and publisher
var chatterTopic;

// A variable to hold the chatter message
var chatterMsg;

// The ROS namespace containing parameters for this script
var param_ns = '/robot_gui';

// Are we on a touch device?
var isTouchDevice = true;
// var isTouchDevice = 'ontouchstart' in document.documentElement;

// A flag to indicate when the Shift key is being depressed
// The Shift key is used as a "dead man's switch" when using the mouse
// to control the motion of the robot.
var shiftKey = false;

var sendcmd = false;

// The topic on which to publish Twist messages
var cmdVelTopic = "/cmd_vel";
var robotBatteryVoltage = "/battery_voltage";
var robotBatteryCurrent = "/battery_current";
var robotBatterySOC = "/battery_soc";
var robotSonarSensor = "/sonar";
var robotLEDCtrl = "/led_control_command";
var robotAngle = "/heading_angle";
var robotImu = "/imu";

// Default linear and angular speed
var defaultLinearSpeed = 0.2;
var defaultAngularSpeed = 1.0;

// Current maximum linear and angular speeds (can be overriden in rosbridge.launch)
var maxLinearSpeed = defaultLinearSpeed;
var maxAngularSpeed = defaultAngularSpeed;

// Minimum linear and angular speed
var minLinearSpeed = 0.05;
var minAngularSpeed = 0.1;

// How much to increment speeds with each key press
var vx_click_increment = 0.05;
var vz_click_increment = 0.1;

// Current desired linear and angular speed
var vx = 0.0;
var vz = 0.0;

// A handle for the publisher timer
var pubHandle = null;

// A handle for the stop timer
var stopHandle = null;

// The rate in Hz for the main ROS publisher loop
var rate = 5;

// Get the current window width and height
var windowWidth = this.window.innerWidth;
var windowHeight = this.window.innerHeight;

// Set the video width to 1/2 of the window width and scale the height
// appropriately.
var videoWidth = Math.round(windowWidth / 2.0);
var videoHeight = Math.round(videoWidth * 240 / 320);

//The main ROS object
var ros = new ROSLIB.Ros();

var cur_topic ="";

var run_first = true;
var runAP = false;

// Connect to ROS
function init_ros(topic) {
	ros.connect(serverURL);

	cur_topic = topic;

}

// If there is an error on the back end, an 'error' emit will be emitted.
ros.on('error', function(error) {
	console.log("Rosbridge Error: " + error);
});

// Wait until a connection is made before continuing
ros.on('connection', function() {
	console.log('Rosbridge connected.');

	if(runRobotCtrl == true)
	{
		// Create a Param object for the max linear speed
		var maxLinearSpeedParam = new ROSLIB.Param({
			ros : ros,
			name : param_ns + '/maxLinearSpeed'
		});

		// Get the value of the max linear speed paramater
		maxLinearSpeedParam.get(function(value) {
			if (value != null) {
				maxLinearSpeed = value;
							
				// Update the value on the GUI
				var element = document.getElementById('maxLinearSpeed');
				element.setAttribute("max", maxLinearSpeed);
				element.setAttribute("value", defaultLinearSpeed);

				writeStatusMessage('maxLinearSpeedDisplay', maxLinearSpeed.toFixed(2));
			}
		});

		// Create a Param object for the max angular speed
		var maxAngularSpeedParam = new ROSLIB.Param({
			ros : ros,
			name : param_ns + '/maxAngularSpeed'
		});

		// Get the value of the max angular speed paramater
		maxAngularSpeedParam.get(function(value) {
			if (value != null) {
				maxAngularSpeed = value;
				
				// Update the value on the GUI
				var element = document.getElementById('maxAngularSpeed');
				element.setAttribute("max", maxAngularSpeed);
				element.setAttribute("value", defaultAngularSpeed);

				writeStatusMessage('maxAngularSpeedDisplay', maxAngularSpeed.toFixed(2));
			}
		});
	}
	
	document.addEventListener('keydown', function(e) {
		if (e.shiftKey)
			shiftKey = true;
		else
			shiftKey = false;
		setSpeed(e.keyCode);
	}, true);

	document.addEventListener('keyup', function(e) {
		if (!e.shiftKey) {
			shiftKey = false;
			stopRobot();
		}
	}, true);	

	if(runRobotCtrl == true)
	{
		// Display a line of instructions on how to move the robot
		if (isTouchDevice) {
			// Set the Nav instructions appropriately for touch
			var navLabel = document.getElementById("navInstructions");
			navLabel.innerHTML = "Tap an arrow to move the robot";
			
			// Hide the publish/subscribe rows on touch devices
			// var pubSubBlock = document.getElementById("pubSubBlock");
			// pubSubBlock.style.visibility = 'hidden';
		}
		else { 
			// Set the Nav instructions appropriately for mousing
			var navLabel = document.getElementById("navInstructions");
			navLabel.innerHTML = "Hold down SHIFT Key when clicking arrows";
		}
	}

	if(run_first == true)
	{
		run_first = false;

		$("input[type=number]").knobby();

		console.log("define knob type input...");
	}

	startSubscribers();

	// Start the publisher loop
	console.log("Starting publishers");
	pubHandle = setInterval(refreshPublishers, 1000 / rate);
});

function refreshPublishers() {
	// Keep the /cmd_vel messages alive
	if(sendcmd == true)
	{
		pubCmdVel();
	}
		
}

var cmdVelPub = new ROSLIB.Topic({
	ros : ros,
	name : cmdVelTopic,
	messageType : 'geometry_msgs/Twist'
});

function pubCmdVel() {
	vx = Math.min(Math.abs(vx), maxLinearSpeed) * sign(vx);
	vz = Math.min(Math.abs(vz), maxAngularSpeed) * sign(vz);

	if (isNaN(vx) || isNaN(vz)) {
		vx = 0;
		vz = 0;
	}

	var cmdVelMsg = new ROSLIB.Message({
		linear : {
			x : vx,
			y : 0.0,
			z : 0.0
		},
		angular : {
			x : 0.0,
			y : 0.0,
			z : vz
		}
	});
	
	if(runRobotCtrl == true)
	{
		var statusMessage = "vx: " + vx.toFixed(2) + " vz: " + vz.toFixed(2);
		writeStatusMessage('cmdVelStatusMessage', statusMessage);
		// console.log("writeStatusMessage ==> inside pubCmdVel");
	}

	if(sendcmd == true)
	{
		cmdVelPub.publish(cmdVelMsg);
	}
}

// Speed control using the arrow keys or icons
function setSpeed(code) {
	// Stop if the deadman key (Shift) is not depressed
	if (!shiftKey && !isTouchDevice) {
		stopRobot();
		return;
	}

	// Use space bar as an emergency stop
	if (code == 32) {
		vx = 0;
		vz = 0;
	}
	// Left arrow
	else if (code == "left" || code == 37) {
		vz += vz_click_increment;
	}
	// Up arrow
	else if (code == 'forward' || code == 38) {
		vx += vx_click_increment;
	}
	// Right arrow
	else if (code == 'right' || code == 39) {
		vz -= vz_click_increment;
	}
	// Down arrow
	else if (code == 'backward' || code == 40) {
		vx -= vx_click_increment;
	}
}

function stopRobot() {
	vx = vz = 0;
	if(runRobotCtrl == true)
	{
		var statusMessage = "vx: " + vx.toFixed(2) + " vz: " + vz.toFixed(2);
		writeStatusMessage('cmdVelStatusMessage', statusMessage);
	}
	pubCmdVel();
}

function timedStopRobot() {
    stopHandle = setTimeout(function() { stopRobot() }, 1000);
}

function clearTimedStop() {
	clearTimeout(stopHandle);
}


function setROSParam() {
	var paramName = document.getElementById('setParamName');
	var paramValue = document.getElementById('setParamValue');
	var param = new ROSLIB.Param({
		ros : ros,
		name : paramName.value
	});
	
    if (isNumeric(paramValue.value)) {
		param.set(parseFloat(paramValue.value));
    }
    else {
    	param.set(paramValue.value);
    }
}

function getROSParam() {
	var paramName = document.getElementById('getParamName');
	var paramValue = document.getElementById('getParamValue');
	var param = new ROSLIB.Param({
		ros : ros,
		name : paramName.value
	});
	param.get(function(value) {
		paramValue.value = value;
	});
}

function setMaxLinearSpeed(speed) {
	maxLinearSpeed = speed;
	console.log("Max linear speed set to: " + maxLinearSpeed);
}

function getMaxLinearSpeed() {
	return maxLinearSpeed;
}

function setMaxAngularSpeed(speed) {
	maxAngularSpeed = speed;
	console.log("Max angular speed set to: " + maxAngularSpeed);
}

function connectDisconnect() {
	var connect = document.getElementById('connectROS').checked;

	if (connect)
		connectServer();
	else
		disconnectServer();
}

function disconnectServer() {
	console.log("Disconnecting from ROS.");
	mjpegViewer.changeStream();
	ros.close();
}

function connectServer() {
	rosbridgeHost = document.getElementById("rosbridgeHost").value;
	rosbridgePort = document.getElementById("rosbridgePort").value;
	serverURL = "ws://" + rosbridgeHost + ":" + rosbridgePort;
	mjpegViewer.changeStream(videoTopic);
	try {
		ros.connect(serverURL);
		console.log("Connected to ROS.");
	} catch (error) {
		console.write(error);
	}
}

function writeStatusMessage(field, message, color) {
	color = typeof color !== 'undefined' ? color : "#c8c8c8";
	// color = typeof color !== 'undefined' ? color : "#006600";
	element = document.getElementById(field);
	element.innerHTML = message;
	element.style.font = "14pt Calibri";
	// element.style.font = "18pt Calibri";
	element.style.color = color;
}

function sign(x) {
	if (x < 0) {
		return -1;
	}
	if (x > 0) {
		return 1;
	}
	return 0;
}

function isNumeric(n) {
  return !isNaN(parseFloat(n)) && isFinite(n);
}

function startSubscribers() {
    // Subscribe to the robot battery topic

    // Use for fake battery level published on /battery_level
    var subRobotBatteryVoltage = new ROSLIB.Topic({
		ros : ros,
		name : robotBatteryVoltage,
		messageType: 'std_msgs/Float32',
		throttle_rate: 5000 // milliseconds
    });
	
    var vol = 0;

    subRobotBatteryVoltage.subscribe(function(msg) {
    	vol = msg.data;
	
		var batvol_mess = vol + " [V]";
		writeStatusMessage('bat_vol', batvol_mess);

    });

	var subRobotBatteryCurrent = new ROSLIB.Topic({
		ros : ros,
		name : robotBatteryCurrent,
		messageType: 'std_msgs/Float32',
		throttle_rate: 5000 // milliseconds
	});
		
	var cur = 0;

	subRobotBatteryCurrent.subscribe(function(msg) {
		cur = msg.data;
		
		var batcur_mess = cur + " [A]";
		writeStatusMessage('bat_cur', batcur_mess);

	});

	var subRobotBatterySOC = new ROSLIB.Topic({
		ros : ros,
		name : robotBatterySOC,
		messageType: 'std_msgs/UInt8',
		throttle_rate: 5000 // milliseconds
	});
		
	var soc = 0;

	subRobotBatterySOC.subscribe(function(msg) {
		soc = msg.data;
		
		var batsoc_mess = soc + " [%]";

		$('#bat_soc').LineProgressbar({
			percentage:parseInt(soc),
			radius: '3px',
			height: '25px',
		});

	});

	var subRobotSonarSensor = new ROSLIB.Topic({
		ros : ros,
		name : robotSonarSensor,
		messageType: 'std_msgs/Float32MultiArray',
		throttle_rate: 5000 // milliseconds
	});

	subRobotSonarSensor.subscribe(function(msg) {
		sd1 = msg.data[0];
		sd2 = msg.data[1];
		sd3 = msg.data[2];
		sd4 = msg.data[3];
		sd5 = msg.data[4];
		sd6 = msg.data[5];
		sd7 = msg.data[6];
		sd8 = msg.data[7];
		sd9 = msg.data[8];
		sd10 = msg.data[9];
		
		intsd1 = parseInt(sd1);
		intsd2 = parseInt(sd2);
		intsd3 = parseInt(sd3);
		intsd4 = parseInt(sd4);
		intsd5 = parseInt(sd5);
		intsd6 = parseInt(sd6);
		intsd7 = parseInt(sd7);
		intsd8 = parseInt(sd8);
		intsd9 = parseInt(sd9);
		intsd10 = parseInt(sd10);
	
		$('#sonar1').LineProgressbar({
		  percentage:intsd1,
		  radius: '3px',
		  height: '15px',
		});

		$('#sonar2').LineProgressbar({
			percentage:intsd2,
			radius: '3px',
			height: '15px',
		});

		$('#sonar3').LineProgressbar({
			percentage:intsd3,
			radius: '3px',
			height: '15px',
		});

		$('#sonar4').LineProgressbar({
			percentage:intsd4,
			radius: '3px',
			height: '15px',
		});

		$('#sonar5').LineProgressbar({
			percentage:intsd5,
			radius: '3px',
			height: '15px',
		});

		$('#sonar6').LineProgressbar({
			percentage:intsd6,
			radius: '3px',
			height: '15px',
		});

		$('#sonar7').LineProgressbar({
			percentage:intsd7,
			radius: '3px',
			height: '15px',
		});

		$('#sonar8').LineProgressbar({
			percentage:intsd8,
			radius: '3px',
			height: '15px',
		});

		$('#sonar9').LineProgressbar({
			percentage:intsd9,
			radius: '3px',
			height: '15px',
		});

		$('#sonar10').LineProgressbar({
			percentage:intsd10,
			radius: '3px',
			height: '15px',
		});
	});

	var subAngle = new ROSLIB.Topic({
		ros : ros,
		name : robotAngle,
		messageType: 'std_msgs/Int16',
		throttle_rate: 5000 // milliseconds
	});

	subAngle.subscribe(function(msg) {
		document.getElementById("angle").value = msg.data;
	});

	var subImu = new ROSLIB.Topic({
		ros : ros,
		name : robotImu,
		messageType: 'sensor_msgs/Imu',
		throttle_rate: 5000 // milliseconds
	});

	subImu.subscribe(function(msg) {
		var quat = new THREE.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
		var rotation = new THREE.Euler().setFromQuaternion(quat, "XYZ");

		document.getElementById("imu").value = rotation.z;
	});

    
}

function pubAirPurifier(val) {

    var ap_pub = new ROSLIB.Topic({
    	ros : ros,
    	name : '/purifier_control_command',
    	messageType : 'std_msgs/UInt16',
    	throttle_rate: 1
    });

    var msg = new ROSLIB.Message({
    	data : val
    });

    ap_pub.publish(msg);
}

function run_pubAP(event) {
	if(event.target.checked)  {
		runAP = true;
		pubAirPurifier(400);
		document.getElementById("APVal").value = 400;
		document.getElementById('APVal').disabled = false;
	  } else {
		runAP = false;
		pubAirPurifier(0);
		document.getElementById("APVal").value = 0;
		document.getElementById('APVal').disabled = true;
	  }
}

function run_knobAP() {
	var val = document.getElementById("APVal").value;

	if(runAP == true)
	{
		pubAirPurifier(parseInt(val));

		console.log("run_knobAP: " + parseInt(val));
	}
}

function setSendCmd(event) {
	if(event.target.checked)  {
		sendcmd = true;
	} else {
		sendcmd = false;
	}
}

function pubUVC(val) {

    var uvc_pub = new ROSLIB.Topic({
    	ros : ros,
    	name : '/power_control_command',
    	messageType : 'zetabot_main/PowerControlMsgs',
    	throttle_rate: 1
    });

    var msg = new ROSLIB.Message({
    	port : 15,
		state : Boolean(val)
    });

    uvc_pub.publish(msg);
}

function run_pubUVC(event) {
	if(event.target.checked)  {
		pubUVC(true);
	  } else {
		pubUVC(false);;
	  }
}

function sendCAMViewTopic(event) {
	if(event.target.checked)  {
		var svt_pub = new ROSLIB.Topic({
			ros : ros,
			name : '/set_camview',
			messageType : 'std_msgs/Bool',
			throttle_rate: 1
		});
	
		var msg = new ROSLIB.Message({
			data : Boolean(true)
		});
	
		svt_pub.publish(msg);

		console.log("send CAM View topic ");

	} else {
		var svt_pub = new ROSLIB.Topic({
			ros : ros,
			name : '/set_camview',
			messageType : 'std_msgs/Bool',
			throttle_rate: 1
		});
	
		var msg = new ROSLIB.Message({
			data : Boolean(false)
		});
	
		svt_pub.publish(msg);

		console.log("stop CAM View topic ");
	}
}

function run_LEDCtrl()
{
	var ledtype1 = -1;
	var ledbar1_type = document.getElementById("front_ledbar").value;
	var colorval1 = document.getElementById("fledbar").value;
	console.log("ledbar1_type: " + ledbar1_type);
	console.log("colorval1: " + colorval1);
	if(ledbar1_type =="off") { ledtype1 = 0;}
	else if(ledbar1_type =="on") { ledtype1 = 1;}
	else if(ledbar1_type =="blink") { ledtype1 = 2;}
	else if(ledbar1_type =="fblink") { ledtype1 = 3;}
	else if(ledbar1_type =="fade") { ledtype1 = 4;}
	else if(ledbar1_type =="sweep") { ledtype1 = 5;}
	else if(ledbar1_type =="fsweep") { ledtype1 = 6;}
	else if(ledbar1_type =="stay") { ledtype1 = 0xff;}

	console.log("ledbar1_type(num): " + ledtype1);

	var colorhex1 = colorval1.substr(1,6);
	var colornum1 = parseInt(colorhex1, 16);
	console.log("colorval1 hex: " + colorhex1);
	console.log("colorval1 num: " + colornum1);

	var ledtype2 = -1;
	var ledbar2_type = document.getElementById("rear_ledbar").value;
	var colorval2 = document.getElementById("rledbar").value;
	console.log("ledbar2_type: " + ledbar2_type);
	console.log("colorval2: " + colorval2);
	if(ledbar2_type =="off") { ledtype2 = 0;}
	else if(ledbar2_type =="on") { ledtype2 = 1;}
	else if(ledbar2_type =="blink") { ledtype2 = 2;}
	else if(ledbar2_type =="fblink") { ledtype2 = 3;}
	else if(ledbar2_type =="fade") { ledtype2 = 4;}
	else if(ledbar2_type =="sweep") { ledtype2 = 5;}
	else if(ledbar2_type =="fsweep") { ledtype2 = 6;}
	else if(ledbar2_type =="stay") { ledtype2 = 0xff;}

	console.log("ledbar2_type(num): " + ledtype2);

	var colorhex2 = colorval2.substr(1,6);
	var colornum2 = parseInt(colorhex2, 16);
	console.log("colorval2 hex: " + colorhex2);
	console.log("colorval2 num: " + colornum2);

	
	if(ledbar1_type == -1) ledbar1_type = 0;
	if(ledbar2_type == -1) ledbar2_type = 0;

	var cmdval = ((((BigInt(ledtype1) << BigInt(24)) | BigInt(colornum1)) << BigInt(32))) | (((BigInt(ledtype2) << BigInt(24)) | BigInt(colornum2))); 

	console.log("cmd val: " + cmdval);
	cmdval_str = cmdval.toString();
	console.log("cmd val str: " + cmdval_str);

	var ledctrl_pub = new ROSLIB.Topic({
    	ros : ros,
    	name : robotLEDCtrl,
    	messageType : 'std_msgs/UInt64',
    	throttle_rate: 1
    });


    var msg = new ROSLIB.Message({
    	data : parseInt(cmdval_str)
    });

    ledctrl_pub.publish(msg);

}