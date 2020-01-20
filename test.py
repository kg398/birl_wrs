import glob
import PoseEstimation as calib
import vision_tools as vision
import cv2

# vision.check_camera(3)

image = vision.capture_pic(0, 0)
cv2.imwrite('CapturedImage.jpg', image)
# images = glob.glob('*.jpg')

# calib.camera_calibrate()

# should calibrate image with coefficients calculated using chessboard

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



# calib.calibrate()

cv2.imshow('corrected image', calib.image_remap(image))
cv2.waitKey()
cv2.destroyAllWindows()

# calib.move_cameraposition()