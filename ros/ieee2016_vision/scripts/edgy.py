import cv2
import numpy as np

image = cv2.imread('QR_test.jpg', 1)

# Remove noise from the colored image
image = cv2.fastNlMeansDenoisingColored(image,None,10,10,7,21)

# Convert image from BGR to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# Set lower and upper bound of blue color value (HSV)
lower_bound = np.array([50, 50, 0])
upper_bound = np.array([210, 200, 100])


# Find the HSV colors within the specified boundaries and create a mask of what was detected
mask = cv2.inRange(hsv, lower_bound, upper_bound)
output = cv2.bitwise_and(image, image, mask = mask)
output2 = cv2.bitwise_and(image, image, mask = mask)


# Convert the colors to B/W, blur it, and apply binary threshold
output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
output = cv2.GaussianBlur(output, (3, 3), 0)
ret,output = cv2.threshold(output, 15, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# ret,output2 = cv2.threshold(output, 15, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)


# Use Canny detection to find edges
output = cv2.Canny(output, 30, 250)

# Morphological transformations on canny image to try to form detectable rectangles
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel, iterations=2) #MORPH_CLOSE = dilation then erosion

kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (33,33))
# kernel2 = kernel
# output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
# Retains only the boundaries of the detected edges
output = cv2.morphologyEx(output, cv2.MORPH_GRADIENT, kernel) 

output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel2, iterations=2)



# Finds the contours of the image
_, cnts, _ = cv2.findContours(output.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

coor = []
all_coors = []

# Loop over the contours
for c in cnts:
    # Approximate the contour
    peri = 0.09*cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, peri, True)
    # print cv2.contourArea(c)

    # Only draw contours if the area is in the specified range.
    if 20000 > cv2.contourArea(c) > 10000:
        # If the approximated contour has four points, then assume that the
        # contour is a block -- a block is a rectangle and thus has four vertices.
        if len(approx) == 4:
            # Draws rectangle around contours
            cv2.drawContours(image, [approx], -1, (0, 255, 0), 4) 

            # Calculates the moments of the contours
            moments = cv2.moments(approx)

            # Finds the x and y coordinates of the quadrilateral's centroid
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])

            # Store the coordinates into a list
            coor.append(cx)
            coor.append(cy)

            # Put a dot where the centroid is
            cv2.circle(image, tuple(coor), 10, (0, 255, 0), -1)

            # Create a list for all found centroids
            all_coors.append(coor)
            coor = []

# all_coors.append([690, 496])
# print all_coors

# Sort all center coordinates by their x-coordinate (ascending)
for x in all_coors:
    all_coors = sorted(all_coors, key = lambda x: x[0], reverse=False)

if len(all_coors) == 0:
    print 'No blocks detected'
elif len(all_coors) == 1:
    print all_coors[0]    
else:    
    # print all_coors
    # print all_coors_copy

    # Variables that store the two left-most blocks
    left_most = all_coors[0]
    left_most2 = all_coors[1]

    #Finds the upper left block. If none, finds the left-most block.
    if abs(left_most[0] - left_most2[0]) >= 100:
        print left_most
    elif left_most[1] > left_most2[1]:
        print left_most2  

# Show the image
# cv2.imshow("processed", output)
# cv2.imshow("final", output2)
cv2.waitKey(0)
