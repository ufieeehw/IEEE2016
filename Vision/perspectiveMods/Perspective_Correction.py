import cv2
import numpy as np

'''
#############################################################
#                IEEE Hardware Team 2016                    #
#   Created by: Christine Moore                                #
#   Email:      christinemoore321@gmail.com                       #
#                                                           #
#   Created for:    perspective correction and transfrom    #
#   Created Date:   October 8th, 2015                    #
#
#############################################################
'''

corners = []
line_segments = []
x_values = []
y_values = []

# function find the corners from the intersection
# of the lines detected as the edge of the object to be transformed
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return -1, -1

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

    
# read in image
img = cv2.imread('Images/card.png')

# convert it to bw
bw_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# actual run canny edge 
# parameters are subject to change/testing
# 2nd param in minVal
# 3rd param is max Val
# they form the interval for acceptable intensity gradients
edge_img = cv2.Canny(bw_img, 100, 200)

# 2nd input is rho
# 3rd param is theta
# 4th param is min accpetance value for a line
# 3rd and 4th param is going to be the major difference
lines = cv2.HoughLines(edge_img,1,np.pi/80,120)
for rho,theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    point1 = [x1, y1]
    point2 = [x2, y2]

    segment = [point1, point2]
    line_segments.append(segment)
    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

for lineSeg1 in line_segments:
    for lineSeg2 in line_segments:
        # detecting areas where the line intersects
        x, y = line_intersection(lineSeg1, lineSeg2)
        if x != -1 and y != -1:
            cv2.circle(img,(x,y), 10, (0,255,255), -1)
            cornerPoint = [x, y]
            boolX = x in x_values
            boolY = y in y_values
            if (x > 0 and y > 0 and boolX == False and boolY == False):
                corners.append(cornerPoint)
                x_values.append(x)
                y_values.append(y)

isQuad = True
#check to make sure detected shape is a quadrilateral


if (len(corners) < 4):
    print 'object detected is not a quadrilateral'
    isQuad = False

# get mass center
leftmostX = 100000000000000
rightmostX = 0
topmostY = 100000000000000
bottommostY = 0

# finding left and right points and top and bottom points
# allows us to to average them to find center of mass
for currentCorner in corners:
    x = currentCorner[0]
    y = currentCorner[1]
    if x > rightmostX and x > 0:
        rightmostX = x
    if x < leftmostX and x > 0:
        leftmostX = x
    if y > bottommostY and y > 0:
        bottommostY = y
    if y < topmostY and y > 0:
        topmostY = y

# average them to find center of mass
centerX = (leftmostX + rightmostX) / 2
centerY = (topmostY + bottommostY) / 2

# just a display for the center point
cv2.circle(img,(centerX, centerY), 10, (255,0,255), -1)

# topcorners 
topCorners = []
# bottom Corners = []
bottomCorners = []

# topleft corner
TL = []
# topright corner
TR = []
# bottom left corner
BL = []
# bottom right corner
BR = []

for currentCorner in corners:
    if currentCorner[1] > centerY:
        bottomCorners.append(currentCorner)
    elif currentCorner[1] < centerY:
        topCorners.append(currentCorner)

# determine top left and top right corners
if topCorners[0][0] < topCorners[1][0]:
    TL = (topCorners[0])
    TR = (topCorners[1])
elif topCorners[1][0] < topCorners[0][0]:
    TL = (topCorners[1])
    TR = (topCorners[0])

if bottomCorners[0][0] < bottomCorners[1][0]:
    BL = (bottomCorners[0])
    BR = (bottomCorners[1])
elif bottomCorners[1][0] < bottomCorners[0][0]:
    BL = (bottomCorners[1])
    BR = (bottomCorners[0])

sortedCorners = []
newQuadPoints = []

sortedCorners.append(TL)
sortedCorners.append(TR)
sortedCorners.append(BR)
sortedCorners.append(BL)

print sortedCorners

height = 300
width = 220

a = np.zeros(shape=(height, width))
newQuadPoints.append([0, 0])
newQuadPoints.append([width, 0])
newQuadPoints.append([width, height])
newQuadPoints.append([0, height])

sortedCorners = np.float32(sortedCorners)
newQuadPoints = np.float32(newQuadPoints)

transformMatrix = cv2.getPerspectiveTransform(sortedCorners, newQuadPoints)

warpedImage = cv2.warpPerspective(img, transformMatrix, (width, height))

cv2.imshow("img", warpedImage)
cv2.waitKey(0)
cv2.destroyAllWindows()

