var image_listener = new ROSLIB.Topic(
{
    ros: ros,
    name: "photographer",
    messageType: "sensor_msgs/CompressedImage"
});

image_listener.subscribe(function(message)
{
    console.log("Received image");
    var img_element = document.getElementById("img");
    img_element.setAttribute("src", "data:image/jpg;base64," + message.data);
});
