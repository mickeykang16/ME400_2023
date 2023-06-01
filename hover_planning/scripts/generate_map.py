import numpy as np
import cv2
import os

if __name__ == '__main__':
    # current resolution 5cm per pixel
    # resolution 5cm
    img_size = [202, 92] # 102, need tp recheck
    corner_point = [41, 51]
    img = np.zeros(img_size, np.uint8) # height by width
    img[0, :] = 255
    img[corner_point[0], corner_point[1]:] = 255
    img[-1, 0:corner_point[1]+1] = 255
    img[:, 0] = 255
    img[corner_point[0]:, corner_point[1]] = 255
    img[:corner_point[0]+1, -1] = 255
    img[corner_point[0]+1:, corner_point[1]+1:] = 125
    img = 255-img
    # big_img = np.ones((100, 100), np.uint8) * 125
    # big_img[:82, :42] = img
    # img = np.flip(img , axis=0)
    cv2.imshow('tmp', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    dir_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    file_path = os.path.join(dir_path, "data/map.png")
    cv2.imwrite(file_path, img)
    