import numpy as np
import cv2
import argparse

# Construct the argument parser and parse the arguments
#ap = argparse.ArgumentParser()
# Image size is proportional to algorithm frame processing speed - bigger pictures cause more frame lag
#ap.add_argument("-i",
#                "--image",
#                required=True,
#                help="path to the static image that will be processed for keypoints")
#ap.add_argument("-l",
#                "--label",
#                help="string to label the object found in the camera feed" )
#ap.add_argument("-s",
#                "--source",
#                type=int,
#                help="optional argument for choosing a specific camera source (-1,0,1,2,3)")
#args = vars(ap.parse_args())


def sift_det(image, source):
    # TODO:
    # Sets up match count between images for sensitivity of detection - choose your value!
    MIN_MATCH_COUNT = 10 #20

    # If VideoCapture(feed) doesn't work, manually try -1, 0, 1, 2, 3 (if none of those work, 
    # the webcam's not supported!)

    # Reads in the image
    img1 = cv2.imread(image, 0)                      
    label = "one_way"
    # Labels the image as the name passed in    
#    if args["label"] is not None:
#        label = args["label"]
#    else:
#        # Takes the name of the image as the name
#        if image[:2] == "./":
#            label = label = (image.split("/"))[2]
#        else:
#            label = image[2:-4]

    ################################################### Set up Feature Detection

    # Create a the SIFT Detector Object
    orb = cv2.ORB_create()
#    try:
#        sift = cv2.xfeatures2d.SIFT_create()
#    except AttributeError:
#        print("Install 'opencv-contrib-python' for access to the xfeatures2d module")

    # Compute keypoints
    kp, des = orb.detectAndCompute(img1, None)

    FLANN_INDEX_KDTREE = 0
    # Option of changing 'trees' and 'checks' values for different levels of accuracy
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)    
    search_params = dict(checks = 50)                                 

    # Fast Library for Approximate Nearest Neighbor Algorithm
    # Creates FLANN object for use below
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = False)
    #flann = cv2.FlannBasedMatcher(index_params, search_params)


    frame = source

    if frame is None:             # Did we get an image at all?
        pass

    ################################################### Shape Computation
    sign_box = None
    color_box = None
    
    # TODO: (DONE)
    # What color space does OpenCV read images in, and what color space do we want process?
    # Check out cvtColor! <https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html>
    # Read in the image from the camera
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # TODO: (DONE)
    # Set up your HSV threshold bounds - [Hue, Saturation, Value]
    lower = np.array([0, 0, 0], dtype = "uint8") #0,0,100
    upper = np.array([255, 255, 10], dtype = "uint8") #255,20,130

    # TODO: (DONE)
    # Check inRange() <https://docs.opencv.org/3.0-beta/modules/core/doc/operations_on_arrays.html?highlight=inrange#invert>
    # Create mask for image with overlapping values
    mask = cv2.inRange(img,lower,upper)

    # TODO:
    # What parameters work best for thresholding? <https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html?highlight=adaptivethreshold>
    imgThresh = cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2) #3,1

    # TODO:
    # This is OpenCV's call to find all of the contours
    # Experiment with different algorithms (cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    # in the parameters of cv2.findContours!
    # <https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findContours>
    # The underscores represent Python's method of unpacking return values, but not using them
    _, contours, _ = cv2.findContours(imgThresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    # Optional TODO:
    # Optional processing of contours - do we want to remove all non-rectangle shapes from contours?
    # Read the documentation on approxPolyDP <https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html>

    # TODO:
    # Orders contours - but by what metric? Check the "key" options <https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html>
    # Ex. key = cv2.contourVectorLen() (Would order the contours by vector length - not an actual function, but this is how you would set the "key")
    # Python's "sorted" function applies a "key" set lambda function to each element within an array, this is not a traditional dictionary "key"
    contours = sorted(contours, key = cv2.contourArea, reverse = True)[1:]   # Removes contouring of display window

    if len(contours) != 0:
        # TODO:
        # Draws the max of the contours arrays with respect to the "key" chosen above
        contours_max = max(contours, key = cv2.contourArea)

        # Find bounding box coordinates
        rect = cv2.boundingRect(contours_max)
        x, y, w, h = rect

        # TODO:
        # Calculates area of detection - what detection area should be the lower bound?
        if w*h > 400:
            color_box= ((x, y), (x+w, y+h))
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 4)


    ################################################### Feature Detection
    # NOTE: NO CODE TO FILL IN HERE - BUT FEEL FREE TO CHANGE THE PARAMETERS

    kp_s, des_s = orb.detectAndCompute(frame, None)

    if(len(kp) >= 2 and len(kp_s) >= 2):
            # Uses the FLANN algorithm to search for nearest neighbors between elements of two images
            # Faster than the BFMatcher for larger datasets
            matches = bf.knnMatch(des,des_s,k=2)
            #matches = flann.knnMatch(des, des_s, k = 2)

    if des_s is None and len(matches) == 0:
        pass

    # Store all the good matches (based off Lowe's ratio test)
    good = []
    for k, pair in enumerate(matches):
        try:
            (m, n) = pair
            if m.distance < 0.75 * n.distance:
                good.append(m)
        except ValueError:
            pass

    # When there are enough matches, we convert the keypoints to floats in order to draw them later
    if len(good) >= MIN_MATCH_COUNT:
        try:
            src_pts = np.float32([ kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp_s[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        except IndexError:
            pass

        # Homography adds a degree of rotation/translation invariance by mapping the transformation
        # of points between two images
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w = img1.shape
        pts = np.float32([ [0,0],[0,h-2],[w-2,h-2],[w-2,0] ]).reshape(-1,1,2)

        if M is not None:
            dst = cv2.perspectiveTransform(pts, M)
            intDst = np.int32(dst)

            # Draws a bounding box around the area of most matched points
            sign_box = ((intDst[0][0][0], intDst[0][0][1]), (intDst[2][0][0], intDst[2][0][1]))
            cv2.rectangle(frame, (intDst[0][0][0], intDst[0][0][1]), (intDst[2][0][0], intDst[2][0][1]), (0, 0, 255), 4, cv2.LINE_AA, 0)
            cv2.putText(frame, label, (dst[0][0][0], dst[0][0][1]) , cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 0, 255), lineType = cv2.LINE_AA )

        else:
            matchesMask = None

    else:
        matchesMask = None

    draw_params = dict(matchColor = (0, 255, 0), # Draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # Draw only inliers
                       flags = 2)

    try:
        # Option of slicing the 'good' list to display a certain number of matches (ex. good[:6])
        # Take out draw_params if we do not want to draw matches
        frame = cv2.drawMatches(img1, kp, frame, kp_s, good, None, **draw_params)
    except cv2.error:
        pass

    return color_box, sign_box

#if __name__ == "__main__":
#    sift_det(args["image"])

