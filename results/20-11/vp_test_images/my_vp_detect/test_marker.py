import cv2, os
from vp_detection import VPDetection
import numpy as np

seq_path = os.path.join('..', 'data', 'UrbanMAV_subset', 'processed', 'sequence3')

im1 = cv2.imread(os.path.join(seq_path, 'view1', 'image.jpg'))
im2 = cv2.imread(os.path.join(seq_path, 'view2', 'image.jpg'))

mask1 = cv2.imread(os.path.join(seq_path, 'view1', 'features', 'faces', 'facade5.png'))
mask2 = cv2.imread(os.path.join(seq_path, 'view2', 'features', 'faces', 'facade5.png'))

im1_masked = cv2.bitwise_and(im1, mask1)
im2_masked = cv2.bitwise_and(im2, mask2)

cv2.imwrite('view1_masked_facade5.png', im1_masked)
cv2.imwrite('view2_masked_facade5.png', im2_masked)

lineSegments1 = np.zeros((0, 4))
lineSegments2 = np.zeros((0, 4))

vpd1 = VPDetection()
vps1 = vpd1.find_vps(im1_masked)
im1_vps1 = vpd1.create_debug_VP_image(save_image='view1_facade5_vps.png')

for (x1, y1, x2, y2) in vpd1.lines[vpd1.clusters[2]]:
    lineSegments1 = np.vstack((lineSegments1, np.array([[x1, y1, x2, y2]])))

np.savetxt('im1_vp_lines.txt', lineSegments1, '%g')

vpd2 = VPDetection()
vps2 = vpd2.find_vps(im2_masked)
im2_vps2 = vpd2.create_debug_VP_image(save_image='view2_facade5_vps.png')

for (x1, y1, x2, y2) in vpd2.lines[vpd2.clusters[2]]:
    lineSegments2 = np.vstack((lineSegments2, np.array([[x1, y1, x2, y2]])))

np.savetxt('im2_vp_lines.txt', lineSegments2, fmt='%g')