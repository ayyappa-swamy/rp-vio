import numpy as np

import rospy

from options import parse_args
from config import InferenceConfig
from plane_segmentor import PlaneSegmentor


def run_server(options, config, camera):
    
    segmentor = PlaneSegmentor(options,config,camera)
    pass


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
    run_server(options, config, camera)


if __name__ == '__main__':
    main()
