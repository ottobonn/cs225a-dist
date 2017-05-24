import cv2
import numpy as np
import json
import sys
import glob, os


def main(input_image, output_contour_dir, output_json_dir):
    im = cv2.imread(input_image)
    print(im.shape)
    resize_width = 380
    resize_height = int(im.shape[1] * resize_width / im.shape[0])
    im = cv2.resize(im, (resize_width, resize_height))
    blurred = cv2.GaussianBlur(im, (3, 3), 1)
    print(blurred.shape)
    
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
    
    JSONs = {}
    JSONs['version'] = '1.0.0'
    matrixes = []
    
    preview_img = np.zeros((resize_width, resize_height, 3))


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
            matrixes[match_c][i][j] = 1

            preview_img[i, j, :] = colors[match_c]
    
    # for c in range(len(colors)):
    #     paths = colorPath(matrixes[c], resize_width, resize_height)
    #     for l in paths:
    #         cell = {}
    #         cell['tool'] = c
    #         cell['points'] = l/float(max_pos)
    #         sequence.append(cell)
    #         JSONs[i]['sequence'] = sequence

    #     with open(output_json_dir+str(c)+'.json', 'w') as outfile:
    #         json.dump(JSONs[c], outfile)

    ######## end output JSON #####################


    ######## preview

    cv2.imshow('preview', preview_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    ### save the contours to a picture

    # for c in range(len(colors)):
    #     paths = colorPath(matrixes[c], resize_width, resize_height)
    #     sequences = []
    #     for l in paths:
    #         cell = {}
    #         cell['tool'] = c
    #         points = []
    #         for p in l:
    #             #p1 = p[0]
    #             points.append([float(p[0])/max_pos, float(p[1])/max_pos])
    #         #cell['points'] = [[[i/float(max_pos) for i in j] for j in k] for k in l]
    #         #cell['points'] = [i/float(max_pos)for k in l]
    #         cell['points'] = points
    #         sequences.append(cell)
    #     JSONs['sequence'] = sequences
        
    #     with open(output_json_dir+str(c)+'.json', 'w') as outfile:
            # json.dump(JSONs, outfile)
######## end output JSON #####################



    return 0


if __name__ == "__main__":
    if len(sys.argv) == 4:
        sys.exit(main(sys.argv[1], sys.argv[2], sys.argv[3]))
    else:
        print "Usage: ./color_separation.py <input_image> <output_contour_image> <output_json_file>"





