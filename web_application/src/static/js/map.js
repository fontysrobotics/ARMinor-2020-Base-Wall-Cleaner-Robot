var elInfo = document.getElementById("info");
var elHeader = document.getElementById("header");
var elData = document.getElementById("data");
var el_canvas = document.getElementById("canvas");
var ctx = el_canvas.getContext("2d");
var map_width;
var map_height;
var map_data;


var map_listener = new ROSLIB.Topic(
    {
        ros: ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid'
    }
);

map_listener.subscribe(function(message)
{
    document.getElementById("loadscreen").style.display = "none";
    document.getElementById("content").style.display = "initial";
    map_width = message.info.width;
    map_height = message.info.height;
    map_data = message.data;
    console.log(message);
    el_canvas.width = map_width;
    el_canvas.height = map_height;
    RenderMap();
});

function RenderMap()
{
    var x_pos = 0;
    var y_pos = 0;
    ctx.fillStyle = "#FFF";
    for(var i=0; i<map_data.length; i++)
    {
        if(map_data[i] == 100)
        {
            ctx.fillRect(x_pos, y_pos, 1, 1);
        }
        if(x_pos == map_width)
        {
            x_pos = 0;
            y_pos++;
        }
        x_pos++;
    };
}
