import message_filters

from options import parse_args
from config import InferenceConfig
import numpy as np
from PlaneSegmentor import PlaneSegmentor
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer


class Node:
    def __init__(self, options, config, camera, image_topic='/image',
                 building_mask_topic='/mask',
                 mask_publishing_topic='/plane_mask', rate=10):
        rospy.init_node("planercnn_node", anonymous=True)
        self.segmentor = PlaneSegmentor(options, config, camera)
        self.sub_image = message_filters.Subscriber(image_topic, Image)
        self.sub_mask = message_filters.Subscriber(building_mask_topic, Image)
        self.ts = message_filters.TimeSynchronizer(
            [self.sub_image, self.sub_mask], 10)
        self.ts.registerCallback(self.callback)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(mask_publishing_topic, Image, queue_size=10)

    # self.image = None
    # self.rate = rospy.Rate(rate)

    def callback(self, image_msg, building_mask_message):
        rospy.loginfo("Received image")
        image = self.bridge.imgmsg_to_cv2(image_msg)
        building_mask = self.bridge.imgmsg_to_cv2(building_mask_message)
        plane_mask = self.segmentor.segment(image, building_mask)
        rospy.loginfo("Created Mask, publishing")
        msg = self.bridge.cv2_to_imgmsg(plane_mask)
        msg.header.stamp = image_msg.header.stamp
        self.pub.publish(msg)

    def start(self):
        rospy.loginfo("Starting segmentation")
        rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('Running')
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.image))
            self.rate.sleep()


def main():
    args = parse_args()

    if args.dataset == '':
        args.keyname = 'evaluate'
    else:
        args.keyname = args.dataset
        pass
    args.test_dir = 'test/' + args.keyname

    if args.testingIndex >= 0:
        args.debug = True
        pass
    if args.debug:
        args.test_dir += '_debug'
        args.printInfo = True
        pass
    options = args
    config = InferenceConfig(options)
    camera = np.array([320, 320, 320, 240, 640, 480])
    node = Node(options, config, camera)
    node.start()


if __name__ == '__main__':
    main()
