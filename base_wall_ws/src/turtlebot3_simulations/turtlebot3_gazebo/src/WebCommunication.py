import logging
import rospy
from sensor_msgs.msg import CompressedImage

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)


class Photographer:
    def __init__(self):
        logger.debug("Initializing photographer")
        self.image = None
        self.captureImageFlag = False
        self.publisher = rospy.Publisher("photographer", CompressedImage)
        self.listener = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self._imageCallback)
        #rospy.init_node("photographer", anonymous=True)
        self.rate = rospy.Rate(10)
        #self.listener.spin()

    def CaptureImage(self):
        logger.debug("Capturing image")
        self.captureImageFlag = True
        while not self.image:
            self.rate.sleep()
        self.publisher.publish(self.image)
        self.captureImageFlag = False
        self.image = None

    def _imageCallback(self, data):
        if self.captureImageFlag:
            self.image = data
