import numpy as np
import cv2

if __name__ == '__main__':
    # current resolution 5cm per pixel
    img = np.zeros((82, 42), np.uint8) # height by width
    img[0, :] = 255
    img[21, 21:] = 255
    img[-1, 0:21] = 255
    img[:, 0] = 255
    img[21:, 21] = 255
    img[:22, -1] = 255
    img[22:, 22:] = 125
    img = 255-img
    # big_img = np.ones((100, 100), np.uint8) * 125
    # big_img[:82, :42] = img
    # img = np.flip(img, axis=0)
    cv2.imshow('tmp', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite('../data/testmap.png', img)