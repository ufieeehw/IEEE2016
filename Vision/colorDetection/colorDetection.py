import cv2
import numpy


def colorDetection(image):
    #Converts image HSV type image
    hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    testImage = image
    #Ranges for color detection
    lowerYellow = numpy.array([20, 100, 100])
    upperYellow = numpy.array([30,255, 255])
    lowerBlue = numpy.array([85,100,100])
    upperBlue = numpy.array([124,255,255])
    lowerRed = numpy.array([0,100,100])
    upperRed = numpy.array([19,255,255])

    #Ranges applied to the hsvImage
    yMask = cv2.inRange(hsvImage, lowerYellow, upperYellow)
    bMask = cv2.inRange(hsvImage, lowerBlue, upperBlue)
    rMask = cv2.inRange(hsvImage, lowerRed, upperRed)

    #Finding the contours on the image
    yContours, yHierarchy = cv2.findContours(yMask, cv2.cv.CV_RETR_TREE,
                                cv2.cv.CV_CHAIN_APPROX_SIMPLE)
    bContours, bHierarchy = cv2.findContours(bMask, cv2.cv.CV_RETR_TREE,
                                cv2.cv.CV_CHAIN_APPROX_SIMPLE)
    rContours, rHierarchy = cv2.findContours(rMask, cv2.cv.CV_RETR_TREE,
                                cv2.cv.CV_CHAIN_APPROX_SIMPLE)

    #Given at least one yellow contour
    if len(yContours) > 0:
        # Find the index of the largest contour
        yAreas = [cv2.contourArea(i) for i in yContours]
        yMaxIndex = numpy.argmax(yAreas)
        yCnt = yContours[yMaxIndex]
        #Find coordinate for boundary rectangle
        yx,yy,yw,yh = cv2.boundingRect(yCnt)
        #Draw rectangle
        cv2.rectangle(testImage,(yx-15,yy-15),(yx+yw+15,yy+yh+15),(0,255,255),0)

    #Given at least one blue contour
    if len(bContours) > 0:
        # Find the index of the largest contour
        bAreas = [cv2.contourArea(i) for i in bContours]
        bMaxIndex = numpy.argmax(bAreas)
        bCnt = bContours[bMaxIndex]
        #Find coordinate for boundary rectangle
        bx,by,bw,bh = cv2.boundingRect(bCnt)
        #Draw rectangle
        cv2.rectangle(testImage,(bx-15,by-15),(bx+bw+15,by+bh+15),(255,0,0),0)

    #Given at least one red contour
    if len(rContours) > 0:
        # Find the index of the largest contour
        rAreas = [cv2.contourArea(i) for i in rContours]
        rMaxIndex = numpy.argmax(rAreas)
        rCnt = rContours[rMaxIndex]
        #Find coordinate for boundary rectangle
        rx,ry,rw,rh = cv2.boundingRect(rCnt)
        #Draw rectangle
        cv2.rectangle(testImage,(rx-15,ry-15),(rx+rw+15,ry+rh+15),(0,0,255),0)


    # #Displaying the masks individually and the final image
    # cv2.imshow('Yellow Mask', yMask)
    # cv2.imshow('Blue Mask', bMask)
    # cv2.imshow('Red Mask', rMask)
    # cv2.imshow('Altered image', testImage)
    return testImage

def main():
    #Default Camera (cv2.videocapture(-1) the parameter indexes your cameras)


    camera = cv2.VideoCapture(-1)

    while camera.isOpened():
        _, image = camera.read()
        cv2.imshow('Original', image)

        rectImg = colorDetection(image)
        cv2.imshow('Color Detector', rectImg)
        cv2.waitKey(5)




if __name__ == '__main__':
    main()
