import cv2
import numpy as np
import json
import sys
import glob, os
##################################################
MAX = 0
MARKER_SIZE = 10
THRESHOLD = 101 #line space

# Colors:
# Blue: (0, 0, 255) - 1
# Red: (255, 0, 0) - 2
# Yellow: (255, 255, 0) - 3
# Key: (0, 0, 0) - 0 (black)
K = [0, 0, 0]
C = [0, 0, 255]
M = [255, 0, 0]
Y = [255, 255, 0]
#W = [255, 255, 255]
COLOR_THRESH = 180
colors = [K, C, M, Y]
###################################################

def calculateDist(point0,point1):
    return (point1[0]-point0[0])**2 + (point1[1]-point0[1])**2
#return np.linalg.norm(point0-point1)

## Inputs:
#   Nodes: a vector of [start_point end_point] of all lines in found contour
#   loc: current position
#   side: begin side( 0 ) or end side( 1 )
def bestRouteHelper(nodes, currNode, currStartSide, route, cumDist):
    n = len(nodes)
    #Base Case: nodes is empty (all nodes added to route)
    if n == 0:
        return cumDist
    minDist         = MAX
    minNextNodeIndex = None
    minNextStartSide = None

#    #find next closest node
    for i in range(n):
        nextNode = nodes[i]
        for nextStartSide in (0,1):
            d = calculateDist(currNode[1-currStartSide], nextNode[nextStartSide])
            if d < minDist:
                minDist = d
                minNextNodeIndex = i
                minNextStartSide = nextStartSide



    minNextNode = nodes.pop(minNextNodeIndex)

    if minDist < THRESHOLD:
        #route[-1] += [minNextNode[minNextStartSide],minNextNode[1-minNextStartSide]]
        mean0 = minNextNode[minNextStartSide][0]+currNode[1-currStartSide][0]
        mean1 = minNextNode[minNextStartSide][1]+currNode[1-currStartSide][1]
        mean = [int(mean0/2), int(mean1/2)]
        route[-1] = route[-1][:-1]+[mean]+[minNextNode[1-minNextStartSide]]

    else:
        route.append([minNextNode[minNextStartSide],minNextNode[1-minNextStartSide]])
        cumDist += minDist
    finalDist = bestRouteHelper(nodes, minNextNode, minNextStartSide, route, cumDist)
    nodes.insert(minNextNodeIndex, minNextNode)
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
    print(n)
    min = MAX
    minRoute = []
    for i in range(n):
        nodes.append((contours[i][0],contours[i][-1]))
    
    for i in range(n):
        currNode=nodes.pop(i)
        for startSide in (0,1):
            route = [ [currNode[startSide],currNode[1-startSide]] ]
            d = bestRouteHelper(nodes, currNode, startSide, route, 0)
            if d<min:
                min = d
                minRoute = route
        nodes.insert(i,currNode)

#    for i in range(len(minRoute)):
#        newLine = []
#        for j in range(len(lines)-1):


#        minRoute[i] = newLine
    print("Generated route has "+str(len(minRoute))+" contours.")
    return minRoute


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
    for col in range(image_width)[::MARKER_SIZE]:
        start = 0
        for row in range(image_height): # straight lines from up to down
            if image[row][col] == 1: # start and inside the line
                start = 1
                line.append([col,row])
            if (image[row][col] == 0 or row==(image_height-1) ) and start == 1: # end of line
                # only draw lines that are >=3 pixels. avoiding sharp shapes.
                if len(line)>=3:
                    path.append(line)
                
                line = []
                start = 0
    vertical = bestRoute(path)


    path = []
    line = []
    for row in range(image_height)[::MARKER_SIZE]:
        start = 0
        for col in range(image_width): # straight lines from up to down
            if image[row][col] == 1: # start and inside the line
                start = 1
                line.append([col,row])
            if (image[row][col] == 0 or col==(image_width-1) ) and start == 1: # end of line
                # only draw lines that are >=3 pixels. avoiding sharp shapes.
                if len(line)>=3:
                    path.append(line)
                
                line = []
                start = 0
    horizontal = bestRoute(path)
# Call same path planning function as contour, since already parsed shape in to lines
    return vertical + horizontal





def main(input_image, output_contour_dir, output_json_dir):
    im = cv2.imread(input_image)
    im = cv2.flip(im,0)
    print(im.shape)
    resize_width = 380
    resize_height = int(im.shape[1] * resize_width / im.shape[0])
    im = cv2.resize(im, (resize_width, resize_height))
    blurred = cv2.GaussianBlur(im, (3, 3), 1)
    print(blurred.shape)
    
    global MAX
    MAX = im.shape[0]**2 + im.shape[1]**2
    ######## output contours as JOSN file #######
    
    # find the max position to normalize the postions
    max_pos = max(resize_width, resize_height)
    
    # build JSON object
    

    
    JSONs = {}
    JSONs['version'] = '1.0.0'
    matrixes = []
    
    for i in range(len(colors)):
        #JSONs.append({})
        #JSONs[i]['version'] = '1.0.0'
        #sequences.append([])
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
            if(dist<COLOR_THRESH):
                matrixes[match_c][i][j] = 1
    
    for c in range(len(colors)):
        paths = colorPath(matrixes[c], resize_width, resize_height)
        sequences = []
        for l in paths:
            cell = {}
            cell['tool'] = c
            points = []
            for p in l:
                #p1 = p[0]
                points.append([float(p[0])/max_pos, float(p[1])/max_pos])
            #cell['points'] = [[[i/float(max_pos) for i in j] for j in k] for k in l]
            #cell['points'] = [i/float(max_pos)for k in l]
            cell['points'] = points
            sequences.append(cell)
        JSONs['sequence'] = sequences
        
        with open(output_json_dir+'.json', 'a') as outfile:
            json.dump(JSONs, outfile)
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
        print("Usage: ./edge.py <input_image> <output_contour_image> <output_json_file>")
