var status_listener = new ROSLIB.Topic(
{
	ros: ros,
	name: "status",
	messageType: "std_msgs/String"
});

status_listener.subscribe(function(message)
{
	document.getElementById("status_msg").innerText = message.data;
});
