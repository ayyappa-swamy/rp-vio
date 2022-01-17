import numpy as np
from detect import Detector


#
class PlaneSegmentor:
    def __init__(self, options, config, camera, ids_file='seg_rgbs.txt',
                 ground_id=212, sky_id=24):
        self.camera = camera
        self.config = config
        self.options = options
        self.detector = Detector(options, config, camera)
        self.mapping = {}
        self.rev_mapping = -1 * np.ones((256, 256, 256), dtype=int)
        self.save_ids(ids_file)
        self.ground_id = ground_id
        self.sky_id = sky_id

    def save_ids(self, ids_file: str):
        with open(ids_file, 'r') as f:
            for line in f.readlines():
                _id, color = tuple(line[:-1].split('\t'))
                _id = int(_id)
                color = np.fromstring(color[1:-1], dtype=np.uint8, sep=',')
                self.mapping[_id] = color
                self.rev_mapping[color[0], color[1], color[2]] = _id

    def segment(self, rgb_image, seg_image) -> np.ndarray:
        """
        Run
        - Plane detector
        - Plane filtering
        - Mask merging

        Returns: plane segmented image
        """
        masks, parameters = self.detect_planes(rgb_image)
        masks = self.filter_planes(rgb_image, seg_image, masks)
        plane_mask = self.merge_masks(masks)

        return plane_mask

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
        id_image = self.rev_mapping[
            building_seg[:, :, 0], building_seg[:, :, 1], building_seg[:, :, 2]]
        sky_mask = id_image == self.sky_id
        ground_mask = id_image == self.ground_id

        for mask in masks:
            mask[sky_mask | ground_mask] = 0

        return masks

    def merge_masks(self, masks) -> np.ndarray:
        """
        Merge n masks into a segmented image
        """
        plane_seg = np.zeros((*masks.shape[1:], 3))

        for i, mask in enumerate(masks):
            plane_seg[mask == 1] = self.mapping[i + 1]

        return plane_seg
