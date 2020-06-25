import roslibpy
from roslibpy import actionlib

client = roslibpy.Ros(host="localhost", port=9090)
server = roslibpy.actionlib.SimpleActionServer(client, "/my_custom_message")

def execute(param):
    print(f"Received param: {param}")
    server.set_succeeded({"sequence": [0, 1]})

if __name__ == "__main__":
    server.start(execute)
    client.run_forever()


