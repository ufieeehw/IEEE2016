import cv2
import numpy


def colorDetection(image, lowerBound, upperBound, erosionValue = 15):
    #Converts image to HSV type image
    hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    testImage = image   #ONLY FOR TEST PURPOSES

    #Ranges for color detection
    lower = numpy.array(lowerBound)
    upper = numpy.array(upperBound)

    #Ranges applied to hsvImage
    mask = cv2.inRange(hsvImage, lower, upper)

    #Remove some noise
    kernel = numpy.ones((5,5),numpy.uint8)
    noiseFilter1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    noiseFilter2 = cv2.morphologyEx(noiseFilter1, cv2.MORPH_CLOSE, kernel)
    cv2.imshow('ColorMask', noiseFilter2)  #ONLY FOR TEST PURPOSES

    #Finding the contours of the image
    contours = cv2.findContours(noiseFilter2, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]

    croppedImages = []
    centroids = []
    #Given at least one yellow contour
    if len(contours) > 0:
        #creates array of measurements from contours
        locations = [cv2.boundingRect(cnt) for cnt in contours]
        #Find the index of the largest contour
        for (x,y,w,h) in locations:
            for cnt in contours:
                if cv2.contourArea(cnt) > 20:
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    centroid = (cx, cy)
                    cv2.circle(testImage, centroid, 5, (0,0,255), -1)
                    centroids.append(centroid)
                    rectX = x - erosionValue
                    rectY = y - erosionValue
                    rectW = w + erosionValue
                    rectH = h + erosionValue
                    print rectX
                    print rectY
                    print rectW
                    print rectH
                    if rectX > 0 and rectY > 0: #and rectW < rows and rectH < cols:
                        cImage = testImage[rectX:rectY,rectW:rectH]
                        cImage = testImage[rectY:rectH, rectX:rectW]
                    #cv2.imshow("TEST", testImage)

                        #cv2.imshow("TestWindow", cImage)
                    cv2.waitKey(8)

                    # croppedImages.append(cImage)


    return croppedImages, centroids, testImage

def main():
    #Default Camera (cv2.videocapture(-1) the parameter indexes your cameras)
    camera = cv2.VideoCapture(-1)
    while camera.isOpened():
        _, image = camera.read()
        cv2.imshow('Original', image)
        lowerBlue = numpy.array([0,100,100])
        upperBlue = numpy.array([50,255,255])
        #image = cv2.imread("/home/chris/Desktop/opencvtest/TopView.JPG")
        rectImg, centroids, circleImg = colorDetection(image, lowerBlue, upperBlue)

        # cv2.circle(rectImg[0],centroids[0], 5, (0,0,255), -1)
        # cv2.rectangle(rectImg[0],(x-15,y-15),(x+w+15,y+h+15),(0,0,0),0)

        cv2.imshow('Color Detector', circleImg)

        #break loop when ESC is pressed
        #cv2.waitKey(0)
        k = cv2.waitKey(5)
        if k == 27:
            break

    #close all windows to end program
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
