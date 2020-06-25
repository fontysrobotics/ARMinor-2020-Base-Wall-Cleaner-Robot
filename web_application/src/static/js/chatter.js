var cmdChatter = new ROSLIB.Topic({
    ros : ros,
    name : '/chatter',
    messageType : 'std_msgs/String'
});

var chatter_messages = [
    new ROSLIB.Message({data: "start"}),
    new ROSLIB.Message({data: "stop"})
]

function SendMessageStart()
{
    cmdChatter.publish(chatter_messages[0]);
}

function SendMessageStop()
{
    cmdChatter.publish(chatter_messages[1]);
}
