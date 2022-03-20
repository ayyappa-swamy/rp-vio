import cv2, os
from my_vp_detect import VPDetection
import numpy as np

images_list = os.listdir('frames_rgb')
images_list.sort()

for img in images_list:
    image = cv2.imread(os.path.join('frames_rgb', img))
    vpd = VPDetection(length_thresh=5, focal_length=320, seed=911)
    vps = vpd.find_vps(image)
    image_vps = vpd.create_debug_VP_image(save_image=os.path.join('frames_vps', img))