import numpy as np
from detect import Detector

#
class PlaneSegmentor:
    def __init__(self, options, config, camera):
        self.camera = camera
        self.config = config
        self.options = options
        self.detector = Detector(options, config, camera)
        pass

    def segment(self, rgb_image, seg_image) -> np.ndarray:
        """
        Run
        - Plane detector
        - Plane filtering
        - Mask merging

        Returns: plane segmented image
        """
        pass

    def detect_planes(self, rgb_image) -> np.ndarray:
        """
        Detect planes
        rgb_image: h x w x 3 rgb image

        Return: masks, parameters
        """
        return self.detector.run(rgb_image)

    def filter_planes(self, rgb: np.ndarray, building_seg: np.ndarray,
                      masks: np.ndarray) -> np.ndarray:
        """
        Filter masks

        Returns: filtered masks
        """
        pass

    def merge_masks(self, masks) -> np.ndarray:
        """
        Merge n masks into a segmented image
        """
        pass
