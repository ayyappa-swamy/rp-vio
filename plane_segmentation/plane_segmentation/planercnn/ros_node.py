from options import parse_args
from config import InferenceConfig
import numpy as np
from PlaneSegmentor import PlaneSegmentor
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Node:
    def __init__(self, options, config, camera, image_topic='/image',
                 mask_publishing_topic='/plane_mask', rate=10):
        rospy.init_node("Planercnn_Node", anonymous=True)
        self.segmentor = PlaneSegmentor(options, config, camera)
        self.sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(mask_publishing_topic, Image, queue_size=10)
        self.image = None
        self.rate = rospy.Rate(rate)

    def callback(self, message):
        self.image = self.bridge.imgmsg_to_cv2(message)

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
