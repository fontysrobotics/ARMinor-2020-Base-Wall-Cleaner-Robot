from Datetime import Datetime
import roslibpy
from roslibpy.actionlib import ActionClient, Goal, Message

GOAL_1 = ({"x": -4.14, "y": -4.63}, 0)
{
    "target_pose": {
            "header": {
                    "frame_id": "map",
                    "stamp": Datetime.now()
                }
        }
}
GOAL_2 = ({"x": 1.34, "y": 2.61}, 0)

def RosFunc(f):
    def wrapper(*args):
        if args[0].isConnected:
            return f()
        else:
            print("Ros not connected")
    return wrapper


class RosWrapper:
    def __init__(self):
        self.ros = roslibpy.Ros(host="192.168.1.104", port=9090)
        self.client = ActionClient(self.ros, "192.168.1.104", "move_base")
        self.isConnected = False

    def Init(self):
        try:
            self.client.run()
            self.goals = [
                Goal(self.client, Message()
            ]
            self.isConnected = True
        except Exception as e:
            print(f"Failed to initialize ROSlib: {e}")

    @RosFunc
    def Test(self):
        service = roslibpy.Service(self.client, "/rosout/get_loggers", "roscpp/GetLoggers")
        request = roslibpy.ServiceRequest()
        return service.call(request)

    def Kill(self):
        self.client.terminate()

