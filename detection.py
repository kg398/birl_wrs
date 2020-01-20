import PoseEstimation as calib
import vision_tools as vision
import numpy as np
import cv2

def placement_mat(image):
    """
    detect the different objects on the placement mat and grasping points
    note, objects could be in any location on mat - need vision

    :param image: undistorted image of placement mat
    :return:
    """

    kernel = np.ones((3, 3), np.uint8)

    crop_image, x_shift, y_shift = crop_table(image)
    # crop_image = image
    # cv2.imwrite("CroppedUndistorted_PlacementMat.jpg", crop_image)
    gray = cv2.cvtColor(crop_image, cv2.COLOR_BGR2GRAY)
    gray = shadows(gray)

    # Erosion
    # new_image = cv2.dilate(new_image, kernel, iterations=1)

    # Opening - removes noise
    new_image = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel, iterations=1)
    new_image = cv2.morphologyEx(new_image, cv2.MORPH_OPEN, kernel, iterations=2)
    # new_image = gray

    # new_image = cv2.GaussianBlur(new_image, (5, 5), 0)

    # alpha * old_image + beta

    alpha = 1  # Simple contrast control
    beta = 0  # Simple brightness control
    new_image = cv2.convertScaleAbs(new_image, alpha=alpha, beta=beta)


    # Gamma correction

    gamma = 1
    table = np.array([((i / 255.0) ** gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    new_image = cv2.LUT(new_image, table)

    # new_image = cv2.medianBlur(new_image, 5)
    # new_image = cv2.medianBlur(new_image, 5)

    cv2.imshow("new image", new_image)
    cv2.waitKey(1000)


    edged = cv2.Canny(new_image, 50, 100)
    edged = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel, iterations=1)
    # edged = cv2.morphologyEx(edged, cv2.MORPH_OPEN, kernel, iterations=1)

    cv2.imshow("edged", edged)
    cv2.waitKey()
    cv2.destroyAllWindows()




    # # Object Detection
    #
    # # blur = cv2.medianBlur(gray, 3)
    # # cv2.imshow("Blurred", blur)
    # # cv2.waitKey(0)
    #
    # Belt Detector
    x, y = circular_detector(gray, 110, 120)
    #
    # Housed Bearing
    x, y = circular_detector(gray, 50, 60) # Finds the inside circle of housed bearing
    #
    # M12 Nut
    # x, y = M12Nut(gray)
    x, y = Hexagon(edged)

    # # # Pulley
    # # x, y = pulley(gray)
    #
    # # 17mm Spacer
    # x, y = circular_detector(edged, 13, 15)
    #
    # # 9mm Spacer
    # x, y = circular_detector(edged, 6, 10)

    _, cnts, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # for c in cnts:
    #     cv2.drawContours(edged, [c], -1, (0, 0, 0), 2)
    #     cv2.imshow("removing contours", edged)
    #     cv2.waitKey(100)
    #     cv2.destroyAllWindows()
    #
    # cv2.destroyAllWindows()
    # cv2.drawContours(edged, cnts[0], -1, (255, 255, 255), 2)
    # cv2.imshow("contour0", edged)
    # cv2.waitKey()


    try:
        # correct image point for crop
        x = x + x_shift
        y = y + y_shift
    except:
        # No crop occured
        pass

    return x, y

def trial(image):

    # kernel = np.ones((3, 3), np.uint8)

    # Crop
    image, x_shift, y_shift = crop_table(image)
    cv2.imshow("original", image)

    # Blur
    image = cv2.medianBlur(image, np.ones((3, 3)))

    # pyramid mean shift
    image = cv2.pyrMeanShiftFiltering(image, 5, 30)

    # remove shadows and normalise
    image = shadows(image)

    # Gray
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


    # threshold - adaptive
    image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

    # opening removes noise
    # image = cv2.morphologyEx(image, cv2.MORPH_OPEN, np.ones((3, 3)), iterations=1)


    cv2.imshow("threshold", image)
    cv2.waitKey()


def remove_contour(cnts, image):
    edged = image
    # _, cnts, heirarchy = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    mask = np.ones(image.shape[:2], dtype="uint8") * 255

    # loop over the contours
    for c in cnts:
        cv2.drawContours(mask, [c], -1, 0, -1)
        edited_image = cv2.bitwise_and(edged, edged, mask = mask)
    cv2.imshow("Mask", mask)
    cv2.imshow("After", edited_image)
    cv2.waitKey()

    return edited_image



def normalise(image):
    """
    Normalise image for contrast and brightness
    :param image:
    :return:
    """
def crop_table(image):
    """
    crop the image from the camera to the region of interest - the table
    :param image:
    :return:
    """
    x = 760
    y = 300
    w = 600
    h = 640

    crop_img = image[y:y + h, x:x + w]
    # cv2.imshow("cropped", crop_img)
    # cv2.waitKey()

    return crop_img, x, y

def circular_detector(image, min_radius = 60, max_radius= 100):
    """
    Find circular shapes
    :param image:
    :param min_radius:
    :param max_radius:
    :return:
    """

    param = {'thresh': [120, 300], 'radius': [min_radius, max_radius]}

    circle = calib.find_circles2(image, param = param, blur = 3, num_circles=1)

    # print("Radius detected: {}".format(circle[2]))

    # x = circle[0] - circle[2]/np.sqrt(2)
    # y = circle[1] - circle[2]/np.sqrt(2)

    try:
        x = circle[0]
        y = circle[1]
    except TypeError:
        print("Object not found")
        return None

    return x, y

def belt(image):
    """
    The belt detector works best without any pre-processing of the image
    :param image:
    :return:
    """

    # Belt Detector
    x, y = circular_detector(image, 70, 80)

    return x, y

def M12Nut(image):
    """
    M12 Nut requires some image pre-processing and blob detection
    :param image:
    :return:
    """
    kernel = np.ones((5, 5), np.uint8)
    image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel, iterations=4)

    parameters = cv2.SimpleBlobDetector_Params()
    detector = cv2.SimpleBlobDetector_create(parameters=parameters)
    keypoints = detector.detect(image)
    new_image = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    for i in range(len(keypoints)):
        print("Keypoint: ", keypoints[i].pt)
    cv2.imshow("Keypoints", new_image)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()
    x, y = keypoints[0].pt

    return x, y

def Hexagon(image):
    """
    M12 nut could be easier to find with contour detection on edged image and find a hexagon
    :param image:
    :return:
    """
    return x, y


def shadows(image):
    """
    remove shadows and normalise
    :param image:
    :return:
    """

    rgb_channels = cv2.split(image) # get rgb channels in order: b, g, r

    result_channels = []
    result_norm_channels = []

    for channel in rgb_channels:

        dilated_img = cv2.dilate(channel, np.ones((7, 7), np.uint8)) # joins broken lines
        blurred_image = cv2.medianBlur(dilated_img, 21)
        diff_img = 255 - cv2.absdiff(channel, blurred_image)

        norm_img = cv2.normalize(diff_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

        result_channels.append(diff_img)
        result_norm_channels.append(norm_img)

    # result = cv2.merge(result_channels) # unnormalised result
    result_norm = cv2.merge(result_norm_channels)
    # cv2.imshow("Normalised", result_norm)
    # cv2.waitKey()
    return result_norm

def pulley(image):
    """
    Pulley colour close to paper, so needs very careful lighting and other peices to be removed from board.
    :param image:
    :return:
    """
    kernel = np.ones((3, 3), np.uint8)
    image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel, iterations=3)
    # image = cv2.medianBlur(image, 15)
    x, y = circular_detector(image, 25, 30)

    return x, y
