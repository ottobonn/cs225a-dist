import cv2
import numpy as np
import json
import sys

MAX = 0

def calculateDist(point0,point1):

    return np.linalg.norm(point0-point1)

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
            if image[row][col] == 1: # start and inside the line
                start = 1
                line.append([row,col])
            elif image[row][col] == 0 and start == 1: # end of line
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
    resize_height = im.shape[1] * resize_width / im.shape[0]
    im = cv2.resize(im, (resize_width, resize_height))
    blurred = cv2.GaussianBlur(im, (3, 3), 1)
    print blurred.shape
    ######## output contours as JOSN file ####### 

    # find the max position to normalize the postions
    max_pos = max(resize_width, resize_height)

    # build JSON object

    # CMYK colors:
    # Cyan: (0, 0, 255) - 1
    # Magenta: (255, 0, 0) - 2
    # Yellow: (255, 255, 0) - 3
    # Key: (0, 0, 0) - 0
    
    C = [0, 0, 255]
    M = [255, 0, 0]
    Y = [255, 255, 0]
    K = [0, 0, 0]
    colors = [K, C, M, Y]

    JSONs = []
    sequences = []
    matrixes = []
    
    for i in range(len(colors)):
        JSONs.append({})
        JSONs[i]['version'] = '1.0.0'
        sequences.append([])
        matrixes.append(np.zeros((resize_width, resize_height)))

    for i in range(resize_width):
        for j in range(resize_height):
            c = blurred[i][j]
            # c = im[i][j]
            match_c = 0
            dist = np.linalg.norm(c-colors[0])
            for k in range(1, len(colors)):
                dist_k = np.linalg.norm(c - colors[k])
                if dist_k < dist:
                    match_c = k
                    dist = dist_k
            matrixes[match_c][i][j] = 1
    
    for c in range(len(colors)):
        paths = colorPath(matrixes[c], resize_width, resize_height)
        for l in paths:
            cell = {}
            cell['tool'] = c
            cell['points'] = l/float(max_pos)
            sequence.append(cell)
            JSONs[i]['sequence'] = sequence

        with open(output_json_dir+str(c)+'.json', 'w') as outfile:
            json.dump(JSONs[c], outfile)
    ######## end output JSON #####################

    ### save the contours to a picture
    #output = np.zeros(im.shape, np.uint8)
    #cv2.drawContours(output, contours, -1, (0, 255, 0), 1)
    #cv2.drawContours(output, contours_out, -1, (0, 255, 0), 1)
    #cv2.imshow('edges', output)
    #cv2.waitKey(10000)
    #cv2.destroyAllWindows()
    #cv2.imwrite(output_contour_dir, output)

    return 0

if __name__ == "__main__":
    if len(sys.argv) == 4:
        sys.exit(main(sys.argv[1], sys.argv[2], sys.argv[3]))
    else:
        print "Usage: ./edge.py <input_image> <output_contour_image> <output_json_file>"





