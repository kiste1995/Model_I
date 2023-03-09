import cv2
import numpy as np
from matplotlib import pyplot as plt

def onChange(pos):
    pass

capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

fps = capture.get(cv2.CAP_PROP_FPS)
try:
    delay = int(1000/fps)
except ZeroDivisionError:
    print("ZeroDivisionError")
    delay = 45

cv2.namedWindow('frame')
#cv2.namedWindow('cvt_frame')
cv2.namedWindow('ir_frame')
#cv2.namedWindow('ir_cvt_frame')

cv2.createTrackbar("lower_h", "ir_frame", 0, 179, onChange)
cv2.createTrackbar("lower_s", "ir_frame", 0, 255, onChange)
cv2.createTrackbar("lower_v", "ir_frame", 0, 255, onChange)
cv2.createTrackbar("upper_h", "ir_frame", 0, 179, onChange)
cv2.createTrackbar("upper_s", "ir_frame", 0, 255, onChange)
cv2.createTrackbar("upper_v", "ir_frame", 0, 255, onChange)

cv2.setTrackbarPos("lower_h", "ir_frame", 0)
cv2.setTrackbarPos("lower_s", "ir_frame", 0)
cv2.setTrackbarPos("lower_v", "ir_frame", 0)
cv2.setTrackbarPos("upper_h", "ir_frame", 179)
cv2.setTrackbarPos("upper_s", "ir_frame", 255)
cv2.setTrackbarPos("upper_v", "ir_frame", 255)
"""
cv2.createTrackbar("lower_cvt_h", "ir_cvt_frame", 0, 179, onChange)
cv2.createTrackbar("lower_cvt_s", "ir_cvt_frame", 0, 255, onChange)
cv2.createTrackbar("lower_cvt_v", "ir_cvt_frame", 0, 255, onChange)
cv2.createTrackbar("upper_cvt_h", "ir_cvt_frame", 0, 179, onChange)
cv2.createTrackbar("upper_cvt_s", "ir_cvt_frame", 0, 255, onChange)
cv2.createTrackbar("upper_cvt_v", "ir_cvt_frame", 0, 255, onChange)

cv2.setTrackbarPos("lower_cvt_h", "ir_cvt_frame", 0)
cv2.setTrackbarPos("lower_cvt_s", "ir_cvt_frame", 0)
cv2.setTrackbarPos("lower_cvt_v", "ir_cvt_frame", 0)
cv2.setTrackbarPos("upper_cvt_h", "ir_cvt_frame", 179)
cv2.setTrackbarPos("upper_cvt_s", "ir_cvt_frame", 255)
cv2.setTrackbarPos("upper_cvt_v", "ir_cvt_frame", 255)
"""
while True:
    ret, frame = capture.read()

    lower_h = cv2.getTrackbarPos("lower_h", "ir_frame")
    lower_s = cv2.getTrackbarPos("lower_s", "ir_frame")
    lower_v = cv2.getTrackbarPos("lower_v", "ir_frame")
    upper_h = cv2.getTrackbarPos("upper_h", "ir_frame")
    upper_s = cv2.getTrackbarPos("upper_s", "ir_frame")
    upper_v = cv2.getTrackbarPos("upper_v", "ir_frame")
    """
    lower_cvt_h = cv2.getTrackbarPos("lower_cvt_h", "ir_cvt_frame")
    lower_cvt_s = cv2.getTrackbarPos("lower_cvt_s", "ir_cvt_frame")
    lower_cvt_v = cv2.getTrackbarPos("lower_cvt_v", "ir_cvt_frame")
    upper_cvt_h = cv2.getTrackbarPos("upper_cvt_h", "ir_cvt_frame")
    upper_cvt_s = cv2.getTrackbarPos("upper_cvt_s", "ir_cvt_frame")
    upper_cvt_v = cv2.getTrackbarPos("upper_cvt_v", "ir_cvt_frame")
    """

    if ret:
        blurred_frame = cv2.GaussianBlur(frame, ksize= (3, 3), sigmaX= 0, sigmaY= 0)
        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        clahe = cv2.createCLAHE(clipLimit= 1.3, tileGridSize= (5, 5))
        clahe_s = clahe.apply(s)
        hsv_merge = cv2.merge((h, clahe_s, v))
        print(clahe_s)

        cvt_frame = cv2.cvtColor(hsv_merge, cv2.COLOR_HSV2BGR)

        ir_mark_1 = cv2.inRange(hsv, (lower_h, lower_s, lower_v), (upper_h, upper_s, upper_v))
        ir_frame = cv2.bitwise_and(hsv, hsv, mask= ir_mark_1)
        ir_frame = cv2.cvtColor(ir_frame, cv2.COLOR_HSV2BGR)

        #ir_mark_2 = cv2.inRange(hsv_merge, (lower_cvt_h, lower_cvt_s, lower_cvt_v), (upper_cvt_h, upper_cvt_s, upper_cvt_v))
        #ir_cvt_frame = cv2.bitwise_and(hsv_merge, hsv_merge, mask= ir_mark_2)
        #ir_cvt_frame = cv2.cvtColor(ir_cvt_frame, cv2.COLOR_HSV2BGR)

        cv2.imshow("frame", frame)
        #cv2.imshow("cvt_frame", cvt_frame)
        cv2.imshow("ir_frame", ir_frame)
        #cv2.imshow("ir_cvt_frame", ir_cvt_frame)

    else:
        print("can't open video!!")

    #print("FPS: %f, Delay: %dms" %(recog.fps, recog.delay))

    if cv2.waitKey(40) == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()