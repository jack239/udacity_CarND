import os
import cv2
import matplotlib.image as mpimg


def get_files_in_dir(directory):
    for file_name in os.listdir(directory):
        full_name = directory + file_name
        if os.path.isfile(full_name):
            yield full_name

def get_images_in_dir(directory):
    return [mpimg.imread(file_name) for file_name in get_files_in_dir(directory)]

def plot_images(fig, images, row_id=0, rows=1):
    start_id = row_id * len(images) + 1
    axes = []
    for i, image in enumerate(images):
        ax = fig.add_subplot(rows, len(images), start_id + i) 
        axes.append(ax)
        if len(image.shape) == 3:
            ax.imshow(image)
        else:
            ax.imshow(image, cmap='gray')
    return axes
            
def make_gray(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def get_s_channel(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)[:,:,2]

def apply_pipeline(image, functions):
    yield image
    for function in functions:
        image = function(image)
        yield image

def put_data(image, data):
    for msg_id, (key, value) in enumerate(data.items()):
        msg = "{} is {:.2f}".format(key, value)
        cv2.putText(image, msg, (100, 100 + msg_id * 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255,255), thickness=3)
