#!/usr/bin/env python2

import json
import sys
import matplotlib.pyplot as plt
import numpy as np

def main(filename):
    with open(filename) as jsonfile:
        trajectory = json.load(jsonfile)
        plt.figure()
        for toolpath in trajectory["sequence"]:
            points = np.array(toolpath["points"])
            plt.plot(points[:,0], points[:,1], "-o")
        plt.xlim(-0.1, 1.1)
        plt.ylim(-0.1, 1.1)
        plt.show()
    return 0

if __name__ == "__main__":
    if len(sys.argv) == 2:
        sys.exit(main(sys.argv[1]))
    else:
        print "Usage: ./preview.py <json filename>"
