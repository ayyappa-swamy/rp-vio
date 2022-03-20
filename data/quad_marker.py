import cv2, os
import numpy as np
from numpy.__config__ import show

images_list = os.listdir('frames_rgb')
masks_list = os.listdir('masks')
images_list.sort()
masks_list.sort()

def show_image(img, title='image'):
    cv2.imshow(title, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def draw_quad(img, masked_image, plane_id):
    mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
    print("masked image size is ", masked_image.shape)
    print("mask image size is ", mask.shape)
    # mask[np.all(masked_image == (123, 154, 0), axis=-1)] = 255
    mask[np.all(masked_image == (16, 48, 25), axis=-1)] = 255

    # dilate thresholded image - merges top/bottom 
    kernel = np.ones((3,3), np.uint8)
    dilated = cv2.dilate(mask, kernel, iterations=5)
    # cv2.imshow('threshold dilated', dilated)

    # find contours
    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    # cv2.drawContours(img, contours, 0, (255, 255, 255), 3)
    print("contours:",len(contours))
    print("largest contour has ",len(contours[0]),"points")

    # simplify contours
    epsilon = 0.05*cv2.arcLength(contours[0],True)
    approx = cv2.approxPolyDP(contours[0],epsilon,True)
    cv2.drawContours(img, [approx], 0, (255,255,255), 3)
    print("simplified contour has", len(approx), "points")
    show_image(img)
    return img

def draw_quads(image, masked_image, plane_ids=[]):
    for plane_id in plane_ids:
        image = draw_quad(image, masked_image, plane_id)

    return image

plane_ids = [7, 8, 9, 11, 15, 16, 17, 18, 20, 21, 22, 23, 24, 36, 41, 43, 47, 59, 62, 80, 85, 91, 96, 109, 119, 120, 121, 123, 125, 131]

for img, msk in zip(images_list[:5], masks_list[:5]):
    image = cv2.imread(os.path.join('frames_rgb', img))
    masked_image = cv2.imread(os.path.join('masks', msk), -1)
    # cv2.imshow('mask', masked_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    draw_quads(image, masked_image, plane_ids=plane_ids[:2])