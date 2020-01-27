import cv2
import numpy as np
import time

#function to filter an image based on the hsv values we want and return the mask
def filter_image(img, hsv_lower, hsv_upper):
    gaus = cv2.GaussianBlur(img, (15, 15), 0)
    blur = cv2.blur(gaus, (10, 10))

    #img_filt = cv2.medianBlur(img, 5)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    return mask

    ###############################################################################
    ### You might need to change the parameter values to get better results
    ###############################################################################
def detect_blob(mask):
    #img = cv2.medianBlur(mask, 9)
    img = cv2.GaussianBlur(mask, (15, 15), 0)

   # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 256;
    #filter by color (on binary)
    params.filterByColor = True
    params.blobColor = 255  # this looks at binary image 0 for looking for dark areas
    # Filter by Area.
    params.filterByArea = True
    #params.minArea = 200 old value
    params.minArea = 100
    params.maxArea = 20000
    # Filter by Circularity was False. A cube's circularity is 0.785
    params.filterByCircularity = False
    #params.minCircularity = 0.65
    #params.maxCircularity = 0.9

    # Filter by Convexity
    params.filterByConvexity = False
    # Filter by Inertia
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    keypoints = detector.detect(img)
    return keypoints

def find_cube(img, hsv_lower, hsv_upper):
    """Find the cube in an image.
        Arguments:
        img -- the image
        hsv_lower -- the h, s, and v lower bounds
        hsv_upper -- the h, s, and v upper bounds
        Returns [x, y, radius] of the target blob, and [0,0,0] or None if no blob is found.
    """
    #get the mask for the image
    mask = filter_image(img, hsv_lower, hsv_upper)
    #find the keypoints in the image by running the blob_detection
    keypoints = detect_blob(mask)

    #if there aren't any cube "blobs" detected, return none.
    if keypoints == []:
        return None
    
    ###############################################################################
    # Todo: Sort the keypoints in a certain way if multiple key points get returned
    ###############################################################################
    #probably want to find the keypoint that looks most like a cube.
    #what happens when we see the cube from the side? We get 2 faces...
    #what happens if we detect a 'cube' from the background noise?

    #sort keypoint by how close they are (how big the radius is)
    #


    keypoints = sorted(keypoints, key=lambda keypoint: keypoint.size, reverse=True)
    return [keypoints[0].pt[0], keypoints[0].pt[1], keypoints[0].size]

