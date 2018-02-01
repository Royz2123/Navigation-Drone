import cv2
import sys
import numpy as np
import time
import math
import os
import traceback
from matplotlib import pyplot as plt

# ESCAPE KEY
ESC_KEY = 27

# MODES
IM_MODE = 1
VID_MODE = 0

# camera info
CAMERA_RATIO_X = 1.25
CAMERA_RATIO_Y = ( 9 * CAMERA_RATIO_X ) / 16

global total_y,total_x
total_y = 0
total_x = 0


def clear_screen():
    if os.name == "nt":
        os.system("cls")
    else:
        os.system('clear')

def print_usage():
    print """Invalid arguments:

    Usage: mvmnt.py    [mode={0: vid, 1: ims}]
                    VID_MODE:    [input {num, path}]
                    IM_MODE:     [im1]     [im2]
    """


def debug_changes(deltaTheta, deltaScale, deltaX, deltaY):
    clear_screen()
    print (
        """
        Current movement:
        deltaTheta: %f\t\tdeltaScale: %f\n
        deltaX: %f\t\tdeltaY: %f\n
        """ % (
            deltaTheta, deltaScale, deltaX, deltaY
        )
    )

def do_transform(im1, im2):
    # normalize
    im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2HSV)
    im2 = cv2.cvtColor(im2, cv2.COLOR_BGR2HSV)


    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
                'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']

    # estimate the transform between them
    res = cv2.matchTemplate(im2,im1,eval(methods[1]))
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    top_left = max_loc
    bottom_right = (top_left[0], top_left[1])
    cv2.rectangle(im1,top_left, bottom_right, 255, 2)
    plt.subplot(121),plt.imshow(res,cmap = 'gray')
    plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(im1,cmap = 'gray')
    plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
    plt.suptitle('cv2.TM_CCOEFF_NORMED')
    plt.show()

    return transform


def add_text(im, txt, pos ,maz=255):
    cv2.putText(
        im,
        txt,
        pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, maz),
        2,
        cv2.LINE_AA
    )

def add_line(im, x, y, degree,maz=1):
    if degree == 90 or degree == -90 or degree == 270 or degree == -270:
        degree+=1
    a=1
    if degree < 0:
        a = -1;
    cv2.line(
        im,
        (x, y),
        (x-100,int(y-a*100*math.tan(degree*math.pi/180))),
        (255, 0, maz),
        5
    )
# TODO: Maybe use kabsch algorithm


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
        transform = do_transform(im1, im2)
        print(transform)
    else:
        # define the two images
        new_im = None
        old_im = None
        frame_num = 0
        pause = False

        # define the current position
        curr_pos = {
            "translation" : [0, 0, 95],
            "rotation" : 0,
        }

        # try to see if this is a webcom video
        vid = sys.argv[2]
        live_mode = 0
        try:
            live_mode = 1
            vid = int(vid)
        except:
            pass

        # work with video
        cap = cv2.VideoCapture(vid)

        # Define the codec and create VideoWriter object
        size = (
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        )
        fourcc = cv2.VideoWriter_fourcc(*'MPEG')  # 'x264' doesn't work
        out = cv2.VideoWriter('output.avi',fourcc, 20.0, size)

        try:
            while cap.isOpened():
                if not pause:
                    # Take each frame
                    ret, new_im = cap.read()

                    # check if the movie is still going
                    if not ret:
                        break

                    # compare to last frame if exists
                    if old_im is not None:
                        # Compute transform
                        try:
                            # get the transform on a new matrix
                            do_transform(old_im, new_im)
                            # show the new image

                            pass
                            # Display two images
                        except Exception as e:
                            print "%s:\t%s" % (frame_num, e)
                            traceback.print_exc()
                            pass

                    # Move onto the next frame
                    old_im = new_im
                    frame_num += 1

                ch = cv2.waitKey(1)
                if ch & 0xFF == ord('q'):
                    break
                elif ch & 0xFF == ord('x'):
                    global total_y,total_x
                    total_y = 0
                    total_x = 0
                    print (total_y);
                    curr_pos = {
                        "translation" : [0, 0, 50],
                        "rotation" : 0,
                    }
                elif ch & 0xFF == ord(' '):
                    pause = not pause
        finally:
            cap.release()
            out.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
