import cv2
import numpy as np
import json
import sys
from functools import reduce
import time

MAX = 0

def calculateDist(point0,point1):

    return (point1[0]-point0[0])**2 + (point1[1]-point0[1])**2

## Inputs:
#   Nodes: a vector of [start_point end_point] of all lines in found contour
#   loc: current position
#   side: begin side( 0 ) or end side( 1 )
def bestRouteHelper(nodes, availIndices, currIndex, currSubIndex, route, cumDist):
    n = len(availIndices)
    #Base Case: nodes is empty (all nodes added to route)
    if n == 0:
        return cumDist
    minDist         = MAX
    minNextIndexOfIndex    = None
    minNextSubIndex = None

    #find next closest node
    for i in range(n):
        nextIndex = availIndices[i]
        for nextSubIndex in (0,1):
            d = calculateDist(nodes[currIndex][1-currSubIndex], nodes[nextIndex][nextSubIndex])
            if d < minDist:
                minDist = d
                minNextIndexOfIndex = i
                minNextSubIndex = nextSubIndex



    minNextIndex = availIndices.pop(minNextIndexOfIndex)
    route.append((minNextIndex,minNextSubIndex))
    cumDist += minDist
    finalDist = bestRouteHelper(nodes, availIndices, minNextIndex, minNextSubIndex, route, cumDist)
    availIndices.insert(minNextIndexOfIndex, minNextIndex)
    return finalDist

## Input:
# contours: contour output from findContour.
#
# Determines the best order for the robot to traverse through the contours
# in order to minimize distance travelled, and outputs the new contour
# Modifications to contour includes:
# - re-ordering the contours
# - reversing a contour
def bestRoute(contours):
    nodes = []
    n = len(contours)
    min = MAX
    route = None
    indices = list(range(n))
    for i in range(n):
        nodes.append((contours[i][0][0],contours[i][-1][0]))

    for i in range(n):
        currIndex=indices.pop(i)
        for currSubIndex in (0,1):
            routeSoFar = [(currIndex, currSubIndex)]
            d = bestRouteHelper(nodes, indices, currIndex, currSubIndex, routeSoFar, 0)
            if d<min:
                min = d
                route = routeSoFar
        indices.insert(i,currIndex)

    newContours = []
    for i in range(n):
        index,subIndex = route[i]
        if subIndex == 0:
            newContours.append( contours[i] )
        else:
            newContours.append( contours[i][::-1] )
    #print(newContours)
    return newContours

## Inputs:
# image: one color layer of the image. Data type: Matrix
# image_width: width of the image
# image_height: height of the image
#
# This function parses the color layer into vertical lines of consequtive points.
# Then calls bestRoute to return the planned path for drawing

def colorPath(image,image_width,image_height):
    path = []
    line = []
    for col in range(image_width):
        start = 0
        for row in range(image_height): # straight lines from up to down
            if image[row][col]=1: # start and inside the line
                start = 1
                line.append([row,col])
            else if image[row][col]=0 && start = 1: # end of line
                # only draw lines that are >=3 pixels. avoiding sharp shapes.
                if len(line)>=3:
                    path.append(line)
                
                line = []
                start = 0

    # Call same path planning function as contour, since already parsed shape in to lines
    return bestRoute(path)






def main(input_image, output_contour_dir, output_json_dir):
    im = cv2.imread(input_image)
    resize_width = 380
    resize_height = int(im.shape[1] * resize_width / im.shape[0])
    im = cv2.resize(im, (resize_width, resize_height))
    #gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(im, (3, 3), 1)
    canny = cv2.Canny(blurred, 0, 255)
    ret, thresh = cv2.threshold(canny, 50, 255, 0)
    #contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Use this for Python2
    im2,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)      # Use this for Python3
    ## filter the trivial piece of contours
    filter_size = 5
    i = 0
    while (i < len(contours)):
        if len(contours[i]) < 5:
            del contours[i]
        else:
            i += 1
    ######## find best route #######

    global MAX
    MAX = im.shape[0]**2 + im.shape[1]**2
    # test contour:
    #temp = [([[1,1]],[[1,1]],[[10,10]]),([[2,2]],[[1,1]],[[1,1]],[[9,9]]),([[3,3]],[[1,1]],[[1,1]],[[8,8]])]
    #newContours = bestRoute(temp)
    newContours = bestRoute(contours)
    contours = newContours
    # example call of color path planning:
    #contours = def colorPath(im[0],image_width,image_height):

    ######## output contours as JSON file #######

    # find the max position to normalize the positions
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

    with open(output_json_dir, 'w') as outfile:
        json.dump(outputJSON, outfile)
    
    # check if the output json works
    #contours_out = []
    #with open('robert.json') as jsonfile:
    #    trajectory = json.load(jsonfile)
    #    for toolpath in trajectory["sequence"]:
    #        points = np.array(toolpath["points"])
    #        points_out = []
    #        for p in points:
    #            points_out.append([[int(max_pos*p[0]), int(max_pos*p[1])]])
    #        contours_out.append(np.array(points_out))
    #contours_out = np.array(contours_out)

    ######## end output JSON #####################

    ### save the contours to a picture
    output = np.zeros(im.shape, np.uint8)
    cv2.drawContours(output, contours, -1, (0, 255, 0), 1)
    #cv2.drawContours(output, contours_out, -1, (0, 255, 0), 1)
    #cv2.imshow('edges', output)
    #cv2.waitKey(10000)
    #cv2.destroyAllWindows()
    cv2.imwrite(output_contour_dir, output)

    return 0





if __name__ == "__main__":
    if len(sys.argv) == 4:
        sys.exit(main(sys.argv[1], sys.argv[2], sys.argv[3]))
    else:
        #print "Usage: ./edge.py <input_image> <output_contour_image> <output_json_file>"   # Use this for Python 2
        print ("Usage: ./edge.py <input_image> <output_contour_image> <output_json_file>")  # Use this for Python 3

