import cv2
import numpy as np
import matplotlib.pyplot as plt

from libs.filter_image import filter_image
from libs.common import put_data
from libs.find_lanes import find_new_fit, get_both_fitx, get_road

class PolynomFinder(object):
    def __init__(self, calibrator, left=None, right=None):
        self.__left = left
        self.__right = right
        self.__calibrator = calibrator
        self.curvature = None
        self.offset = None
        self.stat = {}
        self.lefts = []
        self.rights = []
        self.bad_images = []

    def get_fitx(self, image):
        left_fit, right_fit = find_new_fit(image, self.__left, self.__right)
        left_fitx, right_fitx, data = get_both_fitx(left_fit, right_fit, image.shape)

        self.lefts.append(left_fit)
        self.rights.append(right_fit)

        if len(self.stat) == 0:
            for k in data:
                self.stat[k] = []
        for k, v in data.items():
            self.stat[k].append(v)

        return left_fitx, right_fitx, data

    def process_image(self, image):
        undistored = self.__calibrator.undistort(image)
        transform = self.__calibrator.transform(undistored)
        filtered = filter_image(transform)
        road_bv, data = get_road(filtered, self.get_fitx)

        road = self.__calibrator.un_transform(road_bv)

        res = cv2.addWeighted(undistored, 1, road, 0.3, 0)
        put_data(res, data)
        # if data["low_dist"] < 0 or data["top_dist"] < 0:
        #     self.bad_images.append(image)
        #     print("dumped", data)
        return res

def plot_poly(pf):
    lefts = np.array(pf.lefts)
    rights = np.array(pf.rights)
    fig, axes = plt.subplots(6, 1, figsize=(15, 30))
    for i in range(3):
        axes[i].plot(lefts[:,i])
        axes[i + 3].plot(rights[:,i])

