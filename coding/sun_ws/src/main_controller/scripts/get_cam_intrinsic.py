import cv2
import numpy

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d, %d" % (x, y)
        cv2.circle(img, (x, y), 3, (120, 0, 0), thickness = -1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                     1, (0,0,255), thickness = 1)

cap = cv2.VideoCapture(1)
while 1:
    ret, frame = cap.read()
    cv2.imshow("window", frame)
    if cv2.waitKey(1) & 0xff == ord('q'): 
        cv2.imwrite("camera.jpg", frame)
        img = cv2.imread('camera.jpg')
        cv2.imshow("image", img)
        cap.release()
        break
while 1:
    cv2.namedWindow("image")
    cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
    cv2.imshow("image", img)
    if cv2.waitKey(1)&0xFF==27:
        break
cv2.destoryAllWindows()
