from __future__ import print_function
import time

from flask import Flask
from flask import render_template
import roslibpy

app = Flask(__name__)
client = None
talker = None



@app.route("/")
def index(name=None):
    return render_template("thing.html")

@app.route("/one")
def one():
    if client.is_connected:
        talker.publish(roslibpy.Message({"data": "One"}))
    else:
        print("Client is not connected")
    return render_template("index.html")

@app.route("/two")
def two():
    if client.is_connected:
        talker.publish(roslibpy.Message({"data": "Two"}))
    else:
        print("Client is not connected")
    return render_template("index.html")

if __name__ == "__main__":
    """
    client = roslibpy.Ros(host="192.168.1.104", port=9090)
    client.run()
    talker = roslibpy.Topic(client, "/chatter", "std_msgs/String", throttle_rate=10)
    """
    app.run(host="0.0.0.0", debug=True)
    """
    talker.unadvertise()
    client.terminate()
    """
    """
    client = roslibpy.Ros(host="192.168.1.104", port=9090)
    client.on_ready(lambda: print("Client is ready"))
    client.run()

    talker = roslibpy.Topic(client, "/chatter", "std_msgs/String")
    talker.advertise()

    while not talker.is_advertised:
        time.sleep(0.5)

    for i in range(10):
        if client.is_connected:
            print("Publishing message")
            talker.publish(roslibpy.Message({"data": "Hello"}))
        else:
            print("Client is not connected")
        time.sleep(1)

    talker.unadvertise()
    client.close()
    client.terminate()
    """
