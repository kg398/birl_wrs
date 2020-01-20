import numpy as np
import cv2
from matplotlib import pyplot as plt
from scipy.spatial.distance import cdist
from scipy import linalg
#import imutils
import copy
import PoseEstimation as calib
import time

CAMERA = 0
PORTS = 3

CAL_PARAM = {'thresh': [120,200],
            'radius': [13, 15]}

# Calibration coordinates
CAL_XOFF = 0.0
CAL_YOFF = 0.0

# C1 = [-213.3+CAL_XOFF,-500.0+CAL_YOFF]
# C2 = [-569.75+CAL_XOFF,-156.5+CAL_YOFF]
# C3 = [-213.3+CAL_XOFF,-156.5+CAL_YOFF]

C1 = [-340+CAL_XOFF,-330.0+CAL_YOFF]
C2 = [-500+CAL_XOFF,-210.0+CAL_YOFF]
C3 = [-340.0+CAL_XOFF,-210.0+CAL_YOFF]

DELTA_X = C3[0]-C2[0]
DELTA_Y = C3[1]-C1[1]

def Test1():

    while True:

        print("Test1")
        time.sleep(2)
    return

def Test2():
    while True:
        print("Test2")
        time.sleep(1)
    return


def VideoFeed():
    print("Video Feed Started")
    cap = cv2.VideoCapture(CAMERA)
    cap.set(3, 1920)
    cap.set(4, 1080)

    ret = True
    while ret is True:
        ret, frame = cap.read()
        # cv2.imwrite("frame.jpg", frame)

    cap.release()
    print("Video Feed ended")
    return

def capture_pic(rotation = 0, remap = True):
    '''Capture a picture from camera.
    
    Args:
        name (str):     The name of picture
        camera (int):   Camera number (Use check_camera() to find correct camera
        rotation(int):  Number of anti-clockwise rotations for image
    
    Returns:
        bool: True if successful, False otherwise
    '''
    try:
        # cap = cv2.VideoCapture(CAMERA)
        #
        # # capture at 1080p lol
        # cap.set(3, 1920)
        # cap.set(4, 1080)

        cap = cv2.VideoCapture(CAMERA)
        # cap.open(CAMERA)
        cap.set(3, 1920)
        cap.set(4, 1080)
        # time.sleep(1)

        # Capture frame-by-frame
        # print(cv2.VideoCapture.isOpened(cap))
        ret, frame = cap.read()

        # WE SHOULDNT CROP HERE, unless it is a symmetric crop, as then the cameraposition is not in the middle, and it messes up the maths for the move to camera position maths
        # #crop
        #
        # x = 300
        # y = 300
        # w = 1500
        # h = 780

        # frame = frame[y:y + h, x:x + w]

        # frame = np.rot90(frame, rotation)

        if remap is True:
            try:
                frame = calib.image_remap(frame)
                print("Image Remapped")
            except:
                print("Warning: Camera Calibration coefficients have not yet been set!")
                pass
        else:
            pass

        cap.release()

    # Display the resulting frame
        # cv2.imwrite(name,frame)
        # plt.imshow(frame)

        return frame

    except Exception as error:
        print("Error in capturing image, in capture_pic")
        print(error)
        return False

def check_camera(ports = PORTS):
    
    '''Checks all attached ports for connected camera
    
    Args:
        ports (int): Number of ports to test
    
    Returns:
        Displays matplotlib figure with connected cameras and corresponding port numbers
    '''
    frame = {}
    for i in range(ports):
        vc = cv2.VideoCapture(i)
        if vc.isOpened():
            rval, capture = vc.read()
            frame[i] = capture
        else:
            print ('Webcam ' + str(i) + ' is not connected')
        vc.release()
    dim1 = int(np.ceil(np.sqrt(len(frame))))
    dim2 = int(np.ceil(float(len(frame))/dim1))
    dim = (str(dim1) + str(dim2))
    print (len(frame), dim)
    if len(frame)>0:
        plt.figure()
        num = 1
        for (key) in frame:
            #print key, np.shape(frame[key])
            plt.subplot(int(str(dim)+str(num))), plt.imshow(frame[key])
            plt.title(str(key)), plt.xticks([]), plt.yticks([])
            num = num + 1
    else:
        print ("No webcams detected at all")
    plt.show()

def find_circles(img, num_circles, param=CAL_PARAM, blur=3, show=True):
    gray = copy.copy(img)
    if show:
        plt.imshow(gray)
        plt.show()
    if len(np.shape(gray))>2:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    
    blurred_img = cv2.medianBlur(gray, blur)

    circles = None
    counter = 0
    #plt.imshow(blurred_img)
    print (param)
    

    while counter < param['thresh'][0]-1 :
        #print (counter)
        circles = cv2.HoughCircles(blurred_img.astype("uint8"), cv2.HOUGH_GRADIENT, 1, 20,
                                       param1=param['thresh'][1],
                                       param2=param['thresh'][0]-counter,
                                       minRadius = param['radius'][0],
                                       maxRadius = param['radius'][1])

        if circles is not None and len(circles[0])>num_circles-1:
            print (param['thresh'][0]-counter)
            print ('All Calibration points found')
            break

        counter = counter + 1

    if circles is None:
        print ('No circles Detected, try changing param values')
        return None
    else:
        cimg = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
        #cimg = copy.copy(ir_img)
        for i in circles[0,:]:
                # draw the outer circle
            cv2.circle(cimg,(int(i[0]),int(i[1])),int(i[2]),(0,0,255),1)
                # draw the center of the ci~rcle
            cv2.circle(cimg,(int(i[0]),int(i[1])),2,(0,0,255),1)

        if show==True:
            cv2.imshow("Calibration Points Identified", cimg)
            cv2.imwrite("calibrated_image.jpg", cimg)
            if cv2.waitKey():# & 0xFF == ord('q'):
                print ("Quit")
            cv2.destroyAllWindows()
        return circles, cimg

def sort_circles3(circles):
    circles_sorted = np.zeros(shape=np.shape(circles))
    # Sort so that circle points ordered clockwise from top left
    circles_sorted[0] = circles[0][np.argsort(circles[0][:,1])]
    #print (circles_sorted[0])
    circles_sorted[0][1:] = circles_sorted[0][1:][np.argsort(circles_sorted[0][1:][:,0])]
    #circles_sorted[0][2:] = circles_sorted[0][2:][np.argsort(circles_sorted[0][2:][:,1])]
    #print (circles_sorted[0])

    crop_points = [circles_sorted[0][0][1], circles_sorted[0][1][1], 
                   circles_sorted[0][0][0], circles_sorted[0][2][0]]

    return circles_sorted, crop_points

def black_out(image, crop_points):
    img = copy.copy(image)
    img[0:crop_points[0],:]=0
    img[:,0:crop_points[2]]=0
    img[crop_points[1]:,:]=0
    img[:,crop_points[3]:]=0
    return img

def run_calibration(cali_image, cal_param = CAL_PARAM, adjust=True):
    cali_img = copy.copy(cali_image)
    
    circles, cimg = find_circles2(copy.copy(cali_img), 3, param=cal_param, blur=1, overlap=False, separation=180,show=False)
    if adjust:
        while True:
            
            plt.imshow(cimg)
            plt.show()
            cal_check =input("Change Calibration?: ")
            if cal_check == "yes":
                print (cal_param)
                r1 = int(input("Radius 1: "))
                r2 = int(input("Radius 2: "))
                t1 = int(input("Thresh 1: "))
                t2 = int(input("Thresh 2: "))
                    
                cal_param = {'thresh': [t1,t2],
                             'radius': [r1, r2]}
                print ("New CAL_PARAM: ", cal_param)
                circles, cimg = find_circles2(copy.copy(cali_img), 3, param=cal_param, blur=1, 
                                              overlap=False, separation=250,show=False)
            elif cal_check=="no":
                break
    print (np.shape(circles))
    circles_sorted, crop_points = sort_circles3(circles)
    return circles_sorted, crop_points

def pix3world_cal(p1,p2,p3):
    a = p3[0]-p1[0]
    b = p3[1]-p1[1]
    c = p3[0]-p2[0]
    d = p3[1]-p2[1]
    
    if (a*d-b*c)!=0:
        inverse = (1.0/(a*d-b*c))*np.matrix([[b, -a],
                                            [-d, c]])
    else:
        inverse = np.matrix([0, 0],
                            [0, 0])
    return p1,inverse

def pix3world(p1,inverse,pixels):
    weights = inverse*np.matrix([[pixels[0]-p1[0]],
                                [pixels[1]-p1[1]]])
    # world: x, y
    return C1[0]-weights[0]*DELTA_X,C1[1]-weights[1]*DELTA_Y

def black_out(image, crop_points):
    img = copy.copy(image)
    img[0:crop_points[0],:]=0
    img[:,0:crop_points[2]]=0
    img[crop_points[1]:,:]=0
    img[:,crop_points[3]:]=0
    return img

def find_circles2(img, num_circles, param=CAL_PARAM, blur=3, overlap=True, separation=None, show=True):
    gray = copy.copy(img)
    if show:
        plt.imshow(gray)
        plt.show()
    if len(np.shape(gray))>2:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    
    blurred_img = cv2.medianBlur(gray, blur)

    circles = None
    counter = 0
    #plt.imshow(blurred_img)
    print (param)
    old_circles = None

    while counter < param['thresh'][0]-1 :
        #print (counter)
        circles = cv2.HoughCircles(blurred_img.astype("uint8"), cv2.HOUGH_GRADIENT, 1, 20,
                                       param1=param['thresh'][1],
                                       param2=param['thresh'][0]-counter,
                                       minRadius = param['radius'][0],
                                       maxRadius = param['radius'][1])
        if old_circles is None:
            old_circles = circles
        if overlap is False and circles is not None:
            keep_circles = np.zeros_like(old_circles)
            new_circles = circles
            
            for old_id, old_circle in enumerate(old_circles[0]):
                idx = []
                separation_list = linalg.norm(circles[:,:,:2]-[old_circle[:2]], axis=2)
                idx.append(np.argmin(separation_list))
                #print "old_new", np.shape(circles),np.shape(old_circles),separation_list
                keep_circles[0][old_id] = circles[0][idx]

            for circle in circles[0]:
                #print "circle",circle, "keep",keep_circles
                
                keep_separation = linalg.norm([[circle[:2]]]-keep_circles[:,:,:2], axis=2)
                #print "KEEP SEPERATION",keep_separation
                if (sum(np.greater(keep_separation[0],separation)) == keep_separation[0].size).astype(np.int):
                    print ("COUNTERR: ", counter)
                    print ("Separation from original circles: ", keep_separation)
                    print (circle)
                    print ((keep_separation > separation))
                    print ("IT IS A COMPLETELY SEPARATE CIRCLE")
                    keep_circles = np.append(keep_circles, [[circle]], axis=1)
                
            circles = keep_circles        
            
        
        if circles is not None and len(circles[0])>num_circles-1:
            for circle in circles[0]:
                print (circle)
            print (param['thresh'][0]-counter)
            print (('All Calibration points found'))
            break
        
        if counter==param['thresh'][0] and len(circles[0])<num_circles:
            print ("Found circles: ", circles)
        old_circles = circles

        counter = counter + 1

    if circles is None:
        print ('No circles Detected, try changing param values')
        return None
    else:
        cimg = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
        #cimg = copy.copy(ir_img)
        for i in circles[0,:]:
                # draw the outer circle
            cv2.circle(cimg,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),1)
                # draw the center of the ci~rcle
            cv2.circle(cimg,(int(i[0]),int(i[1])),2,(0,255,0),1)

        if show==True:
            cv2.imshow("Points Identified", cimg)
            cv2.imwrite("calibrated_image.jpg", cimg)
            if cv2.waitKey():# & 0xFF == ord('q'):
                print ("Quit")
            cv2.destroyAllWindows()
        return circles, cimg
