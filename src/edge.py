import cv2
import numpy as np
import json
import sys

def main(input_image, output_contour_dir, output_json_dir):
    im = cv2.imread(input_image)
    resize_width = 380
    resize_height = im.shape[1] * resize_width / im.shape[0]
    im = cv2.resize(im, (resize_width, resize_height))
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(im, (3, 3), 1)
    canny = cv2.Canny(blurred, 0, 255)
    ret, thresh = cv2.threshold(canny, 50, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    ## filter the trivial piece of contours
    filter_size = 5
    i = 0
    while (i < len(contours)):
        if len(contours[i]) < 5:
            del contours[i]
        else:
            i += 1

    ######## output contours as JOSN file ####### 

    # find the max position to normalize the postions
    max_pos = max(resize_width, resize_height)

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
        print "Usage: ./edge.py <input_image> <output_contour_image> <output_json_file>"





