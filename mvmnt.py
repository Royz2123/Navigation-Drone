import cv2
import sys
import numpy as np
import time


ESC_KEY = 27

# MODES
IM_MODE = 1
VID_MODE = 0


def print_usage():
    print """Invalid arguments:

    Usage: mvmnt.py    [mode={0: vid, 1: ims}]
                    VID_MODE:    [input {num, path}]
                    IM_MODE:     [im1]     [im2]
    """

def print_transform(im1, im2):
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


def main():
    if len(sys.argv) < 2:
        print_usage()
        return

    if int(sys.argv[1]) == IM_MODE:
        if len(sys.argv) < 4:
            print_usage()
            return

        # read the two images
        im1 = cv2.imread(sys.argv[2])
        im2 = cv2.imread(sys.argv[3])

        # compute transform
        print_transform(im1, im2)

    else:
        # define the two images
        new_im = None
        old_im = None

        print sys.argv[2]

        cap = cv2.VideoCapture(sys.argv[2])
        while cap.isOpened():
            # Take each frame
            ret, new_im = cap.read()

            # compare to last frame if exists
            if old_im is not None:
                # Disply two images
                cv2.imshow("new_im", new_im)
                cv2.imshow("old_im", old_im)

                # Wait
                k = cv2.waitKey(0)
                if k == ESC_KEY:
                    break

                # Compute transform
                print_transform(old_im, new_im)

            # Move onto the next frame
            old_im = new_im

        cap.release()
        cv2.destroyAllWindows()








if __name__ == "__main__":
    main()
