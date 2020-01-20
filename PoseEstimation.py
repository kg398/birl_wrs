import numpy as np
import cv2
import glob
import yaml
from scipy import linalg

from urx import urx
from math import pi
from multiprocessing import Process
import vision_tools as vision
import time


done = False
CAMERA = 0

# def camera_calibrate(cap):
#
#     """
#     Calibrate for camera distortion, and calculate distortion coefficients to use for further image processing.
#
#     :param image: image to calculate distortion coefficients from (contains chessboard)
#     :return: remapping coefficients (x and y) and region of interest
#     """
#
#     image = vision.capture_pic(cap, remap=False)
#     cv2.imwrite('CapturedImage.jpg', image)
#
#     image = cv2.medianBlur(image, 5)
#
#
#     # termination criteria
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#
#     # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
#     objp = np.zeros((6*7, 3), np.float32)
#     objp[:,:2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
#     # 7 by 6 grid of object points
#
#     # Arrays to store object points and image points from all the images.
#     objpoints = [] # 3d point in real world space
#     imgpoints = [] # 2d points in image plane.
#
#
#     # image = cv2.imread(image)
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#
#     # Find the chess board corners
#     ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)
#
#     # ret, centers = cv2.findCirclesGrid(gray, (7, 6), None)
#
#     # If found, add object points, image points (after refining them)
#     if ret is True:
#         objpoints.append(objp)
#
#         corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1,-1), criteria)
#         imgpoints.append(corners2)
#
#         # Draw and display the corners
#         image = cv2.drawChessboardCorners(image, (7, 6), corners2, ret)
#         cv2.imshow('image', image)
#         cv2.imwrite('dots.png', image)
#         cv2.waitKey(3000)
#
#         ret, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
#
#         # shape returns number of rows, columns, channels
#         height, width = image.shape[:2]  # extracts all info from image.shape except number of channels (entry 2)
#         newcameramtx, RegionOfInterest = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (width, height), 1, (width, height))
#
#         # undistort via shortcut
#         undistorted = cv2.undistort(image, camera_matrix, distortion_coefficients, None, newcameramtx)
#
#         # # undistort while obtaining remapping coefficients
#         # # find remapping function from distorted image to undistorted image
#         # mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion_coefficients, None, newcameramtx, (width, height), 5)
#         # # remap image using new mapping
#         # undistorted = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
#
#
#         cv2.imwrite('uncropped_calibrated_image.png', undistorted)
#
#         # crop the image
#         x, y, width, height = RegionOfInterest
#
#         if (width < 10) or (height < 10):
#             print("Zero Region of Interest, trying again")
#             camera_calibrate(cap)
#
#         undistorted = undistorted[y: y + height, x: x + width]
#         cv2.imwrite('calibrated_image.png', undistorted)
#
#         cv2.imshow('calibrated image', undistorted)
#         cv2.waitKey(1000)
#
#         # Reprojection error
#         mean_error = 0
#         tot_error = 0
#         for i in range(len(objpoints)):
#             imgpoints2, _ = cv2.projectPoints(objpoints[i], rotation_vectors[i], translation_vectors[i], camera_matrix, distortion_coefficients)
#             error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
#             tot_error += error
#
#         print("total error: ", mean_error / len(objpoints))
#
#         # # store calibration parameters in a yaml file
#         #
#         # # It's very important to transform the matrix to list (apparently)
#         # data = {'camera_matrix': np.asarray(camera_matrix).tolist(),
#         #         'dist_coeff': np.asarray(distortion_coefficients).tolist(),
#         #         'newcameramtx': np.asarray(newcameramtx).tolist(),
#         #         'RegionOfInterest': np.asarray(RegionOfInterest).tolist()}
#         #
#         # with open("calibration.yaml", "w") as f:
#         #     yaml.dump(data, f)
#
#         np.save('camera_matrix', camera_matrix)
#         np.save('distortion_coefficients', distortion_coefficients)
#         np.save('newcameramtx', newcameramtx)
#         np.save('RegionOfInterest', RegionOfInterest)
#
#         # return mapx, mapy, RegionOfInterest
#         return undistorted
#
#     elif ret is False:
#         print("no corners found in this image, trying again...")
#         # cv2.imshow('failed image', image)
#         # cv2.waitKey(3000)
#         cv2.destroyAllWindows()
#         camera_calibrate(cap)
#
#     cv2.destroyAllWindows()


def camera_calibrate():
    """
    Calibrate for camera distortion, and calculate distortion coefficients to use for further image processing.

    :param image: image to calculate distortion coefficients from (contains chessboard)
    :return: remapping coefficients (x and y) and region of interest
    """
    failurecounter = 0
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
    # 7 by 6 grid of object points

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.


    images = glob.glob('./CalibrationImages/*.jpg')
    # image = cv2.imread(image)

    for image in images:
        image = cv2.imread(image)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

        # If found, add object points, image points (after refining them)
        if ret is True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # # Draw and display the corners
            # image = cv2.drawChessboardCorners(image, (7, 6), corners2, ret)
            # cv2.imshow('image', image)
            # cv2.imwrite('dots.png', image)
            # cv2.waitKey(1000)
        elif ret is False:
            # print("No corners found in this image")
            failurecounter += 1
            print("failurecounter: {}".format(failurecounter))
    # cv2.destroyAllWindows()

    ret, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)


    image = cv2.imread('./CalibrationImages/image1.jpg')

    # shape returns number of rows, columns, channels
    height, width = image.shape[:2]  # extracts all info from image.shape except number of channels (entry 2)
    newcameramtx, RegionOfInterest = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients,
                                                                       (width, height), 1, (width, height))

    # undistort via shortcut
    undistorted = cv2.undistort(image, camera_matrix, distortion_coefficients, None, newcameramtx)

    cv2.imwrite('uncropped_calibrated_image.png', undistorted)

    # crop the image
    x, y, width, height = RegionOfInterest

    if (width < 10) or (height < 10):
        print("Zero Region of Interest")
        # camera_calibrate(cap)

    undistorted = undistorted[y: y + height, x: x + width]
    cv2.imwrite('calibrated_image.png', undistorted)

    cv2.imshow('calibrated image', undistorted)
    cv2.waitKey(1000)





    np.save('camera_matrix', camera_matrix)
    np.save('distortion_coefficients', distortion_coefficients)
    np.save('newcameramtx', newcameramtx)
    np.save('RegionOfInterest', RegionOfInterest)

        # return mapx, mapy, RegionOfInterest
    return undistorted

    # cv2.destroyAllWindows()


def get_calibration_images():
    """

    :param cap:
    :return:
    """
    cap = cv2.VideoCapture(CAMERA)
    cap.set(3, 1920)
    cap.set(4, 1080)
    counter = 0
    while counter < 100:
        ret, image = cap.read()
        cv2.imwrite('./CalibrationImages/image{}.jpg'.format(counter), image)
        counter += 1
        time.sleep(0.1)
    cap.release()

def image_remap(image):
    """
    remap image using distortion coefficients calculated from camera calibration stage, and stored in local file
    :param image: image to undistort
    :return: undistorted image
    """
    # with open('calibration.yaml') as f:
    #     LoadDict = yaml.load(f)

    camera_matrix = np.load("camera_matrix.npy")
    distortion_coefficients = np.load("distortion_coefficients.npy")
    newcameramtx = np.load("newcameramtx.npy")
    RegionOfInterest = np.load("RegionOfInterest.npy")

    undistorted = cv2.undistort(image, camera_matrix, distortion_coefficients, None, newcameramtx)


    # # crop the image
    # x, y, width, height = RegionOfInterest
    # undistorted = undistorted[y: y + height, x: x + width]

    cv2.imwrite('./Undistorted_images/undistorted.jpg', undistorted)

    return undistorted


def find_circle(image):
    """
    Find location of arm in camera field and calibrate coordinates so we can move arm to locations in camera field
    :param image: cv2.imread() image or image captured through cv2. Note, this function does not cv2.imread :(
    :return: 3 element vector of: (x,y,radius)
    """
    # img = cv2.imread(image)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_img = cv2.medianBlur(gray, 3)


    # param1 is the parameter passed to canny edge detector
    # param2 is the theshold for circle center detector, smaller = more false circles detected
    # function returns larger accumulator values first (ie. more likely to be circles if higher in list)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=100, param1 = 200, param2 = 50, minRadius=40, maxRadius=70)
    # output of circles is 3 element vector of: (x,y,radius)

    if circles is None:
        # print("Error: No circles found in this image: {}".format(image))
        print("Error: No circles found in this image")
        return None

    # have to index into the circles first, for some reason it is a 0 dimensional array of an array
    circles = circles[0]

    # circles = np.uint16(np.around(circles))

    print("radii: {}".format(circles[:, 2])) # [rows, columns]

    # plot circles on original image (not gray)
    for i in circles:
        # outer circle
        cv2.circle(image, (i[0], i[1]), i[2], (0, 0, 255), 2)

    cv2.imshow('detected circles', image)
    # cv2.waitKey(0)
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows()

    return circles


def find_circles2(image, num_circles=1, blur=3, overlap=True, separation=None, param = {'thresh': [120, 300], 'radius': [65, 75]}):

    # thresh[0] = param2
    # thresh[1] = param1

    # param1 is the parameter passed to canny edge detector
    # param2 is the threshold for circle center detector, smaller = more false circles detected
    # function returns larger accumulator values first (ie. more likely to be circles if higher in list)

    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    except:
        # already greyscale
        gray = image
        pass

    blurred_img = cv2.medianBlur(gray, blur)

    circles = None
    counter = 0
    old_circles = None

    while counter < param['thresh'][0] - 1:
        circles = cv2.HoughCircles(blurred_img.astype("uint8"), cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=param['thresh'][1],
                                   param2=param['thresh'][0] - counter,
                                   minRadius=param['radius'][0],
                                   maxRadius=param['radius'][1])

        if old_circles is None:
            old_circles = circles
        if overlap is False and circles is not None:
            keep_circles = np.zeros_like(old_circles)
            new_circles = circles

            for old_id, old_circle in enumerate(old_circles[0]):
                idx = []
                separation_list = linalg.norm(circles[:, :, :2] - [old_circle[:2]], axis=2)
                idx.append(np.argmin(separation_list))
                # print "old_new", np.shape(circles),np.shape(old_circles),separation_list
                keep_circles[0][old_id] = circles[0][idx]

            for circle in circles[0]:
                # print "circle",circle, "keep",keep_circles

                keep_separation = linalg.norm([[circle[:2]]] - keep_circles[:, :, :2], axis=2)
                # print "KEEP SEPERATION",keep_separation
                if (sum(np.greater(keep_separation[0], separation)) == keep_separation[0].size).astype(np.int):
                    print("COUNTER: ", counter)
                    print("Separation from original circles: ", keep_separation)
                    print(circle)
                    print((keep_separation > separation))
                    print("IT IS A COMPLETELY SEPARATE CIRCLE")
                    keep_circles = np.append(keep_circles, [[circle]], axis=1)

            circles = keep_circles

        if circles is not None and len(circles[0]) > (num_circles - 1):
            # for circle in circles[0]:
            #     print("circle: {}".format(circle))
            # print(param['thresh'][0] - counter)
            # print('All Calibration points found')
            break

        if counter == param['thresh'][0] and len(circles[0]) < num_circles:
            print("Found circles: ", circles)
        old_circles = circles

        counter = counter + 1

    if circles is None:
        print('No circles Detected, try changing param values')
        return None
    else:
        cimg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        # cimg = copy.copy(ir_img)
        for i in circles[0]:
            # draw the outer circle
            cv2.circle(cimg, (int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 2)
            # draw the center of the circle
            cv2.circle(cimg, (int(i[0]), int(i[1])), 2, (0, 255, 0), 1)

        cv2.imshow("Points Identified", cimg)
        cv2.imwrite("identified_circle.jpg", cimg)
        # if cv2.waitKey():
        #     print("Quit")
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
        print("Radius detected:", circles[0][0][2])

        return circles[0][0]

def calibrate():

    # Process(target=move_loop()).start()
    # Process(target=calibrate_loop()).start()

    print("Calibration started")

    Burt = urx.Robot()

    pos1 = [-0.20, -0.26, 0.1, pi, 0, 0]
    pos2 = [-0.20, -0.53, 0.1, pi, 0, 0]
    pos3 = [-0.42, -0.53, 0.1, pi, 0, 0]

    param = {'thresh': [120, 300], 'radius': [50, 60]}

    Burt.movel(pos1, vel=0.2, acc=0.2)
    print(Burt.getj())
    image = vision.capture_pic()
    # image = image_remap(image)
    circle1 = find_circles2(image, param = param)
    robot_position1 = Burt.getl()[:2]  # gets x and y coordinates
    robot_position1 = np.array(robot_position1)

    camera_position1 = circle1[:2]  # gets x and y coordinates
    camera_position1 = np.array(camera_position1)
    print("robot position: {}".format(robot_position1))
    print("camera position: {}".format(camera_position1))

    Burt.movel(pos2, vel=0.2, acc=0.2)
    image = vision.capture_pic()
    # image = image_remap(image)
    cv2.imshow("image", image)
    # cv2.waitKey()
    circle2 = find_circles2(image, param = param)
    robot_position2 = Burt.getl()[:2]  # gets x and y coordinates
    robot_position2 = np.array(robot_position2)

    camera_position2 = circle2[:2]  # gets x and y coordinates
    camera_position2 = np.array(camera_position2)
    print("robot position: {}".format(robot_position2))
    print("camera position: {}".format(camera_position2))

    Burt.movel(pos3, vel=0.2, acc=0.2)
    image = vision.capture_pic()
    # image = image_remap(image)
    cv2.imshow("image", image)
    # cv2.waitKey()
    circle3 = find_circles2(image, param = param)
    robot_position3 = Burt.getl()[:2]  # gets x and y coordinates
    robot_position3 = np.array(robot_position3)

    camera_position3 = circle3[:2]  # gets x and y coordinates
    camera_position3 = np.array(camera_position3)
    print("robot position: {}".format(robot_position3))
    print("camera position: {}".format(camera_position3))

    cv2.destroyAllWindows()
    
    # Get two vectors in each coordinate set
    # c = b - a
    camera_1 = camera_position1 - camera_position2
    print("camera1: ", camera_1)
    camera_2 = camera_position3 - camera_position2
    print("camera2: ", camera_2)

    robot_1 = robot_position1 - robot_position2
    robot_2 = robot_position3 - robot_position2

    camera_matrix = np.matrix([camera_1, camera_2])
    camera_matrix = camera_matrix.T
    print(camera_matrix)

    robot_matrix = np.matrix([robot_1, robot_2])
    robot_matrix = robot_matrix.T
    print(robot_matrix)

    P_CameraToRobot = robot_matrix * camera_matrix.I

    np.save('P_CameraToRobot', P_CameraToRobot)
    np.save('camera_origin', camera_position2.T)
    np.save('robot_origin', robot_position2.T)

    Burt.close()

    return

def move_cameraposition(cameraposition = [600, 200], HeightFromCalibration = 0.25, height = 0.15, rob = None):
    """
    move robot to a pixel location in camera image
    :param cameraposition: camera position
    :return:
    """
    camera_height = 0.94 # correct when changed
    SizeCalibrated = np.array([1920, 1080])

    cameraposition = np.array(cameraposition)

    # correction for calibration height
    camera_distance = cameraposition - (SizeCalibrated / 2)  # assuming image taken (and kept) at 1484x763

    # # get the magnitude of the distance
    # abs_camera_distance = np.linalg.norm(camera_distance)

    # ratio = HeightFromCalibration / camera_height
    #
    # correction_distance = camera_distance * ratio
    #
    # corrected_distance = camera_distance + correction_distance
    #
    # # #Correct for desired offset from calibration height
    # # ratio2 = - desired_HeightFromCalibration / camera_height
    # # correction_distance_x += camera_distance[0] * ratio2
    # # correction_distance_y += camera_distance[1] * ratio2
    #
    # corrected_x_camera = camera_distance[0] + correction_distance_x  # corrected distance to camera origin
    # corrected_y_camera = camera_distance[1] + correction_distance_y


    ratio = camera_distance / (camera_height - HeightFromCalibration)

    corrected_distance = camera_height * ratio

    cameraposition = corrected_distance + (SizeCalibrated / 2)

    cameraposition = cameraposition.T

    P_CameraToRobot = np.load("P_CameraToRobot.npy")
    camera_origin = np.load("camera_origin.npy")
    robot_origin = np.load("robot_origin.npy")


    camera_vector = cameraposition - camera_origin

    # camera_vector = np.matrix(camera_vector)
    robot_vector = P_CameraToRobot.dot(camera_vector)

    robot_position = robot_origin + robot_vector

    print("moving to: ", robot_position)
    robot_position = [robot_position[0], robot_position[1], height, pi, 0, 0]
    try:
        rob.movep(robot_position, vel = 0.2, acc = 0.2)
    except:
        print("robot to move not specified correctly, see move_cameraposition")
    return























