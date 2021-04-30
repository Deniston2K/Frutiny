import cv2
import os 
import time

cap = cv2.VideoCapture(0)
result = True
while(result):
    
    ret,frame = cap.read()
    cv2.imwrite("img.jpg",frame)
    result =False
cap.release()
cv2.destroyAllWindows()
# USAGE
# python object_size.py --image images/example_01.png --width 0.955
# python object_size.py --image images/example_02.png --width 0.955
# python object_size.py --image images/example_03.png --width 3.5

# import the necessary packages

from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2
import serial
import firebase
from time import sleep
import time


url = 'https://frutiny-b889b-default-rtdb.firebaseio.com/'
firebase = firebase.FirebaseApplication(url)

reference_obj = 0.955
path = 'img.jpg'

def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()
count=1


# construct the argument parse and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-i", "--image", required=False,
#	help="path to the input image")
#ap.add_argument("-w", "--width", type=float, required=False,
#	help="width of the left-most object in the image (in inches)")
#args = vars(ap.parse_args())


# load the image, convert it to grayscale, and blur it slightly
image = cv2.imread(path)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (7, 7), 0)
#cv2.imshow("gray",gray)

# perform edge detection, then perform a dilation + erosion to
# close gaps in between object edges
edged = cv2.Canny(gray, 50, 100)
edged = cv2.dilate(edged, None, iterations=1)
edged = cv2.erode(edged, None, iterations=1)

#cv2.imshow("edged",edged)

# find contours in the edge map
cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

# sort the contours from left-to-right and initialize the
# 'pixels per metric' calibration variable
(cnts, _) = contours.sort_contours(cnts)
pixelsPerMetric = None

# loop over the contours individually
counter = 1
for c in cnts:
    # if the contour is not sufficiently large, ignore it
    if cv2.contourArea(c) < 100:
        continue

    # compute the rotated bounding box of the contour
    orig = image.copy()
    box = cv2.minAreaRect(c)
    box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
    box = np.array(box, dtype="int")

    # order the points in the contour such that they appear
    # in top-left, top-right, bottom-right, and bottom-left
    # order, then draw the outline of the rotated bounding
    # box
    box = perspective.order_points(box)
    cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)

    # loop over the original points and draw them
    for (x, y) in box:
        cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)

    # unpack the ordered bounding box, then compute the midpoint
    # between the top-left and top-right coordinates, followed by
    # the midpoint between bottom-left and bottom-right coordinates
    (tl, tr, br, bl) = box
    (tltrX, tltrY) = midpoint(tl, tr)
    (blbrX, blbrY) = midpoint(bl, br)

    # compute the midpoint between the top-left and top-right points,
    # followed by the midpoint between the top-righ and bottom-right
    (tlblX, tlblY) = midpoint(tl, bl)
    (trbrX, trbrY) = midpoint(tr, br)

    # draw the midpoints on the image
    cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
    cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
    cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
    cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)

    # draw lines between the midpoints
    cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),(255, 0, 255), 2)
    cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),(255, 0, 255), 2)

    # compute the Euclidean distance between the midpoints
    dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
    dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))

    # if the pixels per metric has not been initialized, then
    # compute it as the ratio of pixels to supplied metric
    # (in this case, inches)
    if pixelsPerMetric is None:
        pixelsPerMetric = dB / reference_obj

    #compute the size of the object
    dimA = dA / pixelsPerMetric
    dimB = dB / pixelsPerMetric
    height= dimA*25.4*10
    width= dimB*25.4*10/2
    r =height/2
    v3 =(1.33) *(3.14*(r*r*r))
    volume = v3*(16.38)/10000
    #density=0.93g/cm^3 of apple
    #density=15.321 g/inch^3
    density=0.0093
    mass=0
    mass=density*volume
    weight= mass*(9.81)+100
    result_1=0
    
    tgc=0
    while True:  
        if ser.in_waiting > 0:
            for i in range (1,3):
                line = ser.readline().decode('utf-8').rstrip()
                if i == 1:
                    result_1= firebase.put("/fruitiny_gas","Gas_val {}".format(count),line)
                    print("Gas_value : {}".format(result_1))
                    sleep(2)
                    
                    

                elif i ==2:
                    result= firebase.put("/fruitiny_ir","Ir_val {}".format(count), line)
                    print("Ir_value : {}".format(result))
                    sleep(2)
                    count=count+1
                    break
                    
                else:
                    continue
            break
        
    tgc = result_1.encode("utf-8")
    ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )
    
    height_1=str(int(height))
    size=height_1
    weight_1=str(int(weight))
    
    tgc_filter = filter(str.isdigit, tgc)
    tgc_string = "".join(tgc_filter)
    tgc_string = tgc_string[:-2]

    
    EndCom = "\xff\xff\xff"
    ser.write('size.val=' +size +EndCom)
    ser.write('weight.val=' +weight_1 +EndCom)
    ser.write('tgc.val=' +tgc_string +EndCom)
    while ser.readable():
      string = "*"
      string = ser.readline()
      time.sleep(1)
      EndCom = "\xff\xff\xff"
      grade = "1020"
      tgc = "920"
      quality = "249"
      ser.write('colourgrade.val=' +grade +EndCom)
      ser.write('quality.val=' +quality +EndCom)
      break
                
    if counter ==1:
        count=1
        volume=volume/10
        print("Height :  " + str(height)+"mm"+" and "+"Width : " + str(width)+"mm")
        print("Volume : "+ str(volume)+" mm^3")
        print("weight : "+ str(weight)+" gm")
        result= firebase.put("/fruitiny_height","height_val {}".format(count),height)
        sleep(2)
        result= firebase.put("/fruitiny_Width","Width_val {}".format(count),width)
        sleep(2)
        result= firebase.put("/fruitiny_volume","volume_val {}".format(count),volume)
        sleep(2)
        result= firebase.put("/fruitiny_weight","weight_val {}".format(count),weight)
        sleep(2)
        count=+1
        break
    # draw the object sizes on the image
    cv2.putText(orig, "{:.1f}in".format(dimA),
        (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (255, 255, 255), 2)
    cv2.putText(orig, "{:.1f}in".format(dimB),
        (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (255, 255, 255), 2)
    counter =counter+1

    # show the output image
    cv2.imshow("Image", orig)
    cv2.waitKey(100)
    



print("second file executed ")