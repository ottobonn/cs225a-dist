import cv2
import numpy as np
import json

input_image = 'robert.png'
im = cv2.imread('imgs/' + input_image)
im = cv2.resize(im, (im.shape[0], im.shape[1]))
gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(im, (3, 3), 1)
canny = cv2.Canny(blurred, 0, 255)
ret, thresh = cv2.threshold(canny, 50, 255, 0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

i = 0
while (i < len(contours)):
    if len(contours[i]) < 3:
        del contours[i]
    else:
        i += 1

######## output contours as JOSN file ####### 

# find the max position to normalize the postions
max_pos = 0
for l in contours:
    for p in l:
        p1 = p[0]
        if p1[0] > max_pos:
            max_pos = p1[0]
        if p1[1] > max_pos:
            max_pos = p1[1]

# build JSON object
outputJSON = {}
outputJSON['version'] = '1.0.0'
sequence = []
for l in contours:
    cell = {}
    cell['tool'] = 0
    points = []
    for p in l:
        p1 = p[0]
        points.append([float(p1[0])/max_pos, float(p1[1])/max_pos])
    cell['points'] = points
    sequence.append(cell)
outputJSON['sequence'] = sequence

with open('robert.json', 'w') as outfile:
    json.dump(outputJSON, outfile)

######## end output JSON #####################

# print contours
#output = np.zeros(im.shape, np.uint8)
#cv2.drawContours(output, contours, -1, (0, 255, 0), 1)
#cv2.imshow('edges', output)
#cv2.waitKey(10000)
#cv2.destroyAllWindows()
#cv2.imwrite('edges/' + input_image, output)
