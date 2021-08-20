import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import cv2

def get_windows(
    binary_warped,
    x_base,
    nwindows = 9,  
    margin = 100,
    minpix = 50):
    window_height = np.int(binary_warped.shape[0]//nwindows)

    lane_inds = []
    # Current positions to be updated later for each window in nwindows
    x_current = x_base
    # Step through the windows one by onea
    
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        ### TO-DO: Find the four below boundaries of the window ###
        win_x_low = x_current - margin  # Update this
        win_x_high = x_current + margin  # Update this

        ### TO-DO: Identify the nonzero pixels in x and y within the window ###
        good_inds = (
            (nonzeroy >= win_y_low) & 
            (nonzeroy < win_y_high) & 
            (nonzerox >= win_x_low) & 
            (nonzerox < win_x_high)
        ).nonzero()[0]
        
        # Append these indices to the lists
        lane_inds.append(good_inds)
        
        
        if len(good_inds) > minpix:
            x_current = np.int(np.mean(nonzerox[good_inds]))
        ### TO-DO: If you found > minpix pixels, recenter next window ###
        ### (`right` or `leftx_current`) on their mean position ###
        pass # Remove this when you add your function
    return lane_inds

def find_lane_pixels(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # HYPERPARAMETERS
    params = {
        # Choose the number of sliding windows
        "nwindows" : 9,
        # Set the width of the windows +/- margin
        "margin" : 100,
        # Set minimum number of pixels found to recenter window
        "minpix" : 50,        
    }

    # Set height of windows - based on nwindows above and image shape
#     window_height = np.int(binary_warped.shape[0]//nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = get_windows(binary_warped, leftx_base, **params)
    right_lane_inds = get_windows(binary_warped, rightx_base, **params)

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # Avoids an error if the above is not implemented fully
        pass

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty


def get_polynomial_fit(binary_warped):
    leftx, lefty, rightx, righty = find_lane_pixels(binary_warped)

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    return left_fit, right_fit

def get_fitx(poly_fit, ploty):
    return poly_fit[0] * ploty**2 + poly_fit[1] * ploty + poly_fit[2]

def get_ploty(shape):
    return np.linspace(0, shape[0] - 1, shape[0])

def get_curvature(poly, y):
    a = poly[0]
    b = poly[1]
    return (1 + (2 * a * y + b) ** 2) ** 1.5 / abs(2 * a)

def get_both_fitx(left_fit, right_fit, shape):
    ploty = get_ploty(shape)
    curv = min(get_curvature(p, ploty).min() for p in [left_fit, right_fit])
    left_fitx = get_fitx(left_fit, ploty)
    right_fitx = get_fitx(right_fit, ploty)
    offset = abs(left_fitx[0] + right_fitx[0] - shape[1]) / 2
    # TODO convert curvature and offset to meters
    data = {
        "curvature" : curv,
        "offset" : offset,
        # "top_dist" : right_fitx[0] - left_fitx[0],
        # "low_dist" : right_fitx[-1] - left_fitx[-1]
    }
    return left_fitx, right_fitx, data

def fit_polynomial(binary_warped):
    left_fit, right_fit = get_polynomial_fit(binary_warped)
    return get_both_fitx(left_fit, right_fit, binary_warped.shape)

def get_image_with_lane(left, right, shape, val = 255):
    shape = list(shape[:2]) + [3]
    layer = np.zeros(shape, dtype=np.uint8)
    left = left.astype(np.int)
    right = right.astype(np.int)
    for i in range(len(left)):
        for j in range(max(left[i], 0), min(right[i], shape[1])):
            layer[i, j, 1] = val
    return layer

def get_road(filtered_image, method):
    left, right, data = method(filtered_image)
    return get_image_with_lane(left, right, filtered_image.shape), data

def get_lane_indexes(poly, nonzero, nonzerox, nonzeroy, margin):
    line = poly[0] * (nonzeroy**2) + poly[1] * nonzeroy + poly[2]
    return (nonzerox > (line - margin)) & (nonzerox < (line + margin))

def find_new_fit(binary_warped, left_fit, right_fit):
    if left_fit is None:
        return get_polynomial_fit(binary_warped)
    # HYPERPARAMETER
    # Choose the width of the margin around the previous polynomial to search
    # The quiz grader expects 100 here, but feel free to tune on your own!
    margin = 100

    # Grab activated pixels
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    ### Set the area of search based on activated x-values ###
    ### within the +/- margin of our polynomial function ###
    left_lane_inds = get_lane_indexes(left_fit, nonzero, nonzerox, nonzeroy, margin)
    right_lane_inds = get_lane_indexes(right_fit, nonzero, nonzerox, nonzeroy, margin)

    # Again, extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    return left_fit, right_fit
