### Distortion coefficients
import numpy as np

import cv2
from .common import make_gray, apply_pipeline

def get_corners(image, shape, nxy):
    gray = make_gray(image)
    if gray.shape != shape:
        return None
    
    ret, corners = cv2.findChessboardCorners(gray, nxy)
    if ret:
        return corners

    return None

def get_object_points(nx, ny, count):
    objpoint = np.zeros((nx*ny,3), np.float32)
    objpoint[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)
    return [objpoint] * count

### Transform matix

def apply_shape(arr, shape):
    arr[:,0] *= shape[1]
    arr[:,1] *= shape[0]
    
    return arr.astype(np.float32)

def get_transform_src(shape):
    x1, x2, x3, x4 = 0.1, 0.37, 0.65, 0.95
    y1, y2 = 0.7, 0.95
    return apply_shape(np.array([
        [x1, y2],
        [x2, y1],
        [x3, y1],
        [x4, y2],
    ]), shape)

def get_transform_dst(shape):
    return apply_shape(np.array([
        [0, 1],
        [0, 0],
        [1, 0],
        [1, 1]
    ]), shape)

def get_transform_matrices(shape):
    src = get_transform_src(shape)
    dst = get_transform_dst(shape)
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    return M, Minv



### CameraCalibrator

class CameraCalibrator:
    def __init__(self, nx, ny, shape=(720, 1280)):
        self.image_points = []
        self.__nxy = (nx, ny)

        self.__M, self.__Minv  = get_transform_matrices(shape)

        self.__shape = shape
        self.__mtx = None
        self.__dist = None
    
    def calibrate(self, image):
        corners = get_corners(image, self.__shape, self.__nxy)
        if corners is not None:
            self.image_points.append(corners)
            return True
        return False
    
    def finish_calibrate(self):
        nx, ny = self.__nxy
        object_points = get_object_points(nx, ny, len(self.image_points))
        ret, mtx, dist, _, _ = cv2.calibrateCamera(
            object_points, 
            self.image_points, 
            self.__shape[::-1], 
            None, 
            None
        )
        assert(ret)
        self.__mtx = mtx
        self.__dist = dist
    
    def undistort(self, image):
        return cv2.undistort(image, self.__mtx, self.__dist, None, self.__mtx)
    
    def transform(self, image):
        shape = [image.shape[1], image.shape[0]]
        return cv2.warpPerspective(image, self.__M, shape)
    
    def un_transform(self, image):
        shape = [image.shape[1], image.shape[0]]
        return cv2.warpPerspective(image, self.__Minv, shape)
    
    def process_image(self, image):
        return list(apply_pipeline(image, [
            self.undistort,
            self.transform
        ]))