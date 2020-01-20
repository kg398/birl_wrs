import glob
import PoseEstimation as calib
import vision_tools as vision
import cv2
import detection as detect
import urx.urx as urx
import time
from multiprocessing import Process
from math import pi

CAMERA = 0

# vision.check_camera(3)

# Burt = urx.Robot(host="129.169.80.50")
# Urnie = urx.Robot(host="129.169.80.100")

try:

    # cap = cv2.VideoCapture(CAMERA)
    # ret, frame = cap.read()
    # cv2.imshow("FRAME", frame)
    # cv2.waitKey(3000)

    # # cap.open(CAMERA)
    # cap.set(3, 1920)
    # cap.set(4, 1080)
    # cap.release()
    # time.sleep(3)

    # Process(target=vision.VideoFeed, args=(cap, )).start()
    # cv2.waitKey(5000)

    # move the robot out of the way of the camera
    # print(Urnie.getj())
    # jpos = [-0.3834779898272913, -1.5607588926898401, 2.5670101642608643, -2.5767467657672327, -1.5681899229632776, -4.136947218571798]
    # Burt.movej(jpos)
    image = vision.capture_pic(remap = True)
    cv2.imwrite("CapturedImage.jpg", image)

    image = cv2.imread("PlacementMat.jpg")
    # x, y = detect.placement_mat(image)
    detect.trial(image)
    # calib.move_cameraposition([x, y], height=0.02, HeightFromCalibration=0.27, rob = Burt)

    # image = vision.capture_pic(cap, remap = False)
    # cv2.imwrite('CapturedImage.jpg', image)

    # images = glob.glob('*.jpg')


    # calib.posture_estimation(image)

    # # Read video
    # video = cv2.VideoCapture("videos/waypoints2.mp4")
    #
    # # Exit if video not opened.
    # if not video.isOpened():
    #     print
    #     "Could not open video"
    #     sys.exit()
    #
    # # Read first frame.
    # ok, frame = video.read()
    # if not ok:
    #     print('Cannot read video file')
    # else:
    #     cv2.imwrite("frame.jpg", frame)
    #     images = glob.glob("frame.jpg")
    #     calib.posture_estimation(images)
    # if not ok:
    #     print('Cannot read video file')
    # else:
    #     cv2.imwrite("frame.jpg", frame)
    #     images = glob.glob("frame.jpg")
    #     calib.posture_estimation(images)

    # calib.get_calibration_images()
    # calib.camera_calibrate()
    # calib.calibrate()

    # Process(target=calib.calibrate).start()

    # print(Urnie.getl())

    # # image = cv2.imread("./Undistorted_images/Placement_mat.jpg")

    # worker2.start()
    # worker1.start()


finally:

    # Burt.close()
    cv2.destroyAllWindows()
    # cap.release()
    # Urnie.close()
    print("Complete!")