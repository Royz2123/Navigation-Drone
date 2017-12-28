import cv2
import sys
import numpy as np
import time
import math
import os

# ESCAPE KEY
ESC_KEY = 27

# MODES
IM_MODE = 1
VID_MODE = 0

# camera info
CAMERA_RATIO_X = 1.25
CAMERA_RATIO_Y = ( 9 * CAMERA_RATIO_X ) /16

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

    # estimate the transform between them
    transform = cv2.estimateRigidTransform(
        im1,
        im2,
        fullAffine=False
    )

    if transform is None:
        raise RuntimeError("Couldn't find the transform")

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

# TODO: Maybe use kabsch algorithm
def process_transform(curr_pos, old_im, new_im):
    transform = do_transform(old_im, new_im)

    # work on transform
    A = [transform[0][:2], transform[1][:2]]
    B = [transform[0][2], transform[1][2]]

    # find current parms
    deltaTheta = math.atan(A[0][1] / A[0][0])
    deltaScale = (A[0][0] / math.cos(deltaTheta))
    deltaX = B[0]
    deltaY = B[1]
    debug_changes(deltaTheta, deltaScale, deltaX, deltaY)

    # find real drone movement
    old_height = curr_pos["translation"][2]
    new_height = old_height / deltaScale
    rel_X = (deltaX / new_im.shape[1]) * (new_height * CAMERA_RATIO_X)
    ret_Y = (deltaY / new_im.shape[0]) * (new_height * CAMERA_RATIO_Y)

    # update rotation and translation
    curr_pos["rotation"] += deltaTheta
    curr_pos["translation"][0] += rel_X
    curr_pos["translation"][1] += ret_Y
    curr_pos["translation"][2] = 95#new_height

    # add text to this im
    disp_img = new_im.copy()
    add_text(disp_img, "X SHIFT: %s" % curr_pos["translation"][0], (20, 120))
    add_text(disp_img, "Y SHIFT: %s" % curr_pos["translation"][1], (20, 150))
    add_text(disp_img, "Z SHIFT: %s" % curr_pos["translation"][2], (20, 180))
    add_text(disp_img, "ROTATION: %s (DEGREES)" % curr_pos["rotation"], (20, 210))
    add_text(disp_img, "+", (disp_img.shape[1]/2, disp_img.shape[0]/2),10)

    # return the displaed image
    return disp_img


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
                            disp_img = process_transform(
                                curr_pos,
                                old_im,
                                new_im
                            )

                            # show the new image
                            cv2.imshow('frame', disp_img)

                            # Display two images
                            out.write(disp_img)
                        except Exception as e:
                            print "%s:\t%s" % (frame_num, e)
                            pass

                    # Move onto the next frame
                    old_im = new_im
                    frame_num += 1

                ch = cv2.waitKey(1)
                if ch & 0xFF == ord('q'):
                    break
                elif ch & 0xFF == ord('x'):
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
