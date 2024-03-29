import logging
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)


class Photographer:
    def __init__(self):
        print("Initializing photographer")
        self.image = None
        self.captureImageFlag = False
        self.publisher = rospy.Publisher("photographer", CompressedImage)
        self.listener = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self._imageCallback)
        #rospy.init_node("photographer", anonymous=True)
        self.rate = rospy.Rate(10)
        #self.listener.spin()

    def CaptureImage(self):
        print("Capturing image")
        self.captureImageFlag = True
        while not self.image:
            self.rate.sleep()
        self.publisher.publish(self.image)
        self.captureImageFlag = False
        self.image = None

    def _imageCallback(self, data):
        if self.captureImageFlag:
            self.image = data

class WebController:
    def __init__(self):
        self.command_listener = rospy.Subscriber("/commands", String, self._commandCallback)
        self.status_publisher = rospy.Publisher("/status", String, queue_size=10)
        self.cur_command = None

    def SendStatus(self, msg):
        self.status_publisher.publish(msg)

    def GetCommand(self):
        return self.cur_command

    def _commandCallback(self, data):
        if self.cur_command != data.data:
            print("Executing command %s" % data.data)
            self.cur_command = data.data

