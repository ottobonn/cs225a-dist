#!/usr/bin/env python2

import json
import sys
import matplotlib.pyplot as plt
import numpy as np
import time

def main(filename):
    with open(filename) as jsonfile:
        trajectory = json.load(jsonfile)
    #plt.ion()
        plt.figure()
        
        #plt.show()
        #x =np.array([])
        #y =np.array([])
        for toolpath in trajectory["sequence"]:
            points = np.array(toolpath["points"])
            color = toolpath["tool"]
            #x = np.append(x , points[:,0])
            
            #y = np.append(y ,points[:,1])
            x = points[:,0]
            y = points[:,1]
            if color == 0:
                plt.plot(x, y, markersize=3, color="k",marker="*")
            elif color == 1:
                plt.plot(x, y, markersize=3, color="b",marker="*")
            elif color == 2:
                plt.plot(x, y, markersize=3, color="r",marker="*")
            elif color == 3:
                plt.plot(x, y, markersize=3, color="g",marker="*")
            elif color == 4:
                plt.plot(x, y, markersize=3, color="y",marker="*")
            elif color == 5:
                plt.plot(x, y, markersize=3, color="#ffa500",marker="*")
#plt.draw()
#input("Press enter...")
            #time.sleep()
        plt.show()
            #plt.draw()
        plt.xlim(-0.1, 1.1)
        plt.ylim(-0.1, 1.1)


    return 0


if __name__ == "__main__":
    if len(sys.argv) == 2:
        sys.exit(main(sys.argv[1]))
    else:
        print("Usage: ./preview.py <json filename>")
