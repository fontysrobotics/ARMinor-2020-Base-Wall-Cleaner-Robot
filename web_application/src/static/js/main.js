var ros = new ROSLIB.Ros({
url : 'ws://192.168.1.104:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

/*
var odom_listener = new ROSLIB.Topic(
{
    ros: ros,
    name: "/odom",
    messageType: "nav_msgs/Odometry"
});

var robot_x_pos;
var robot_y_pos;
var d = false;
odom_listener.subscribe(function(message)
{
    if(!d)
    {
        console.log(message);
        d = true;
    }
    else
    {
        return;
    }
    robot_x_pos = message.twist.twist.linear.x * 1e7;
    robot_y_pos = message.twist.twist.linear.y * 1e7;
    //console.log("y: ", robot_y_pos);
    RenderRobot();
});

function RenderRobot()
{
    ctx.fillStyle = "#F00";
    ctx.fillRect(robot_x_pos, robot_y_pos, 5, 5);
}
*/
