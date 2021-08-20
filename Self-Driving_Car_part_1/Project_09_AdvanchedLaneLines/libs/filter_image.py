import cv2
import numpy as np
from .common import make_gray, get_s_channel

def get_bnary_from_mask(mask):
    binary = np.zeros_like(mask)
    binary[mask] = 1
    return binary

    
def abs_sobel_thresh(image, orient='x', thresh_min=0, thresh_max=255, sobel_kernel=3):
    if len(image.shape) == 3:
        image = make_gray(image)
    if orient == 'x':
        sobel = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    else:
        sobel = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    
    return get_bnary_from_mask((scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max))

def mag_thresh(image, sobel_kernel=3, mag_thresh=(0, 255)):
    if len(image.shape) == 3:
        image = make_gray(image)
    sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    sobelm = np.sqrt(sobelx ** 2 + sobely ** 2)

    scaled_sobel = np.uint8(255*sobelm/np.max(sobelm))
    
    return get_bnary_from_mask((scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1]))
    
def dir_threshold(image, sobel_kernel=3, thresh=(0, np.pi/2)):
    if len(image.shape) == 3:
        image = make_gray(image)
    sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    sobel_angle = np.arctan2(np.absolute(sobely), np.absolute(sobelx))

    return get_bnary_from_mask((sobel_angle >= thresh[0]) & (sobel_angle <= thresh[1]))

def hls_select(image, thresh=(0, 255)):
    S = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)[:,:,2]
    
    return get_bnary_from_mask((S > thresh[0]) & (S <= thresh[1]))

def filter_image(image):
    image = get_s_channel(image)
    sybinary = abs_sobel_thresh(image, orient='y', sobel_kernel=5, thresh_min=7, thresh_max=150)
    sxbinary = abs_sobel_thresh(image, orient='x', sobel_kernel=5, thresh_min=5, thresh_max=150)

    combined_binary = np.zeros_like(sybinary)
    combined_binary[(sxbinary == 1) | (sybinary == 1)] = 1
    return combined_binary.astype(np.uint8)