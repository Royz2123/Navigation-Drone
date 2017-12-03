import cv2
import sys
import numpy as np


def print_usage():
    print """Invalid arguments:

    Usage: mvmnt.py    [im1]     [im2]
    """

def main():
    if len(sys.argv) < 3:
        print_usage()
        return

    # read the two images
    im1 = cv2.imread(sys.argv[1])
    im2 = cv2.imread(sys.argv[2])

    # normalize
    im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2HSV)
    im2 = cv2.cvtColor(im2, cv2.COLOR_BGR2HSV)

    # estimate the transform between them
    transform = cv2.estimateRigidTransform(
        im1,
        im2,
        fullAffine=True
    )

    print(transform)






if __name__ == "__main__":
    main()
