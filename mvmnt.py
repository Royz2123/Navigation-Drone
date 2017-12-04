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

def do_transform(im1, im2):
    # normalize
    im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2HSV)
    im2 = cv2.cvtColor(im2, cv2.COLOR_BGR2HSV)

    # estimate the transform between them
    transform = cv2.estimateRigidTransform(
        im1,
        im2,
        fullAffine=True
    )

    if transform is None:
        raise RuntimeError("Couldn't find the transform")

    return transform


def add_text(im, txt, pos):
    cv2.putText(
        im,
        txt,
        pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2,
        cv2.LINE_AA
    )
    

def calc_transform(transform, ):
    
    

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

        # define the current position
        curr_pos = {
            "translation" : [0, 0, 50],
            "rotation" : 0,
        }
        
        # try to see if this is a webcom video
        vid = sys.argv[2]
        try:
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

        while cap.isOpened():
            # Take each frame
            ret, new_im = cap.read()

            # check if the movie is still going
            if not ret:
                break

            # compare to last frame if exists
            if old_im is not None:
                # Compute transform
                try:
                    transform = do_transform(old_im, new_im)

                    # work on transform
                    A = [transform[0][:2], transform[1][:2]]
                    B = [transform[0][2], transform[1][2]]
                    
                    # find scaling and rotation
                    U, S, V = np.linalg.svd(a, full_matrices=True)
                                        
                    # update rotation
                    # Maybe use kabsch algorithm
                    
                    # update translation
                    scaling = S[0][0]
                    curr_pos["translation"][0] += B[0]
                    curr_pos["translation"][1] += B[1]
                    curr_pos["translation"][2] *= scaling

                    # add text to this im
                    disp_img = new_im.copy()
                    add_text(disp_img, "X SHIFT: %s (PIXELS)" % curr_pos["translation"][0], (20, 120))
                    add_text(disp_img, "Y SHIFT: %s (PIXELS)" % curr_pos["translation"][1], (20, 150))
                    add_text(disp_img, "Z SHIFT: %s (PIXELS)" % curr_pos["translation"][2], (20, 180))
                    add_text(disp_img, "ROTATION: %s (DEGREES)" % curr_pos["rotation"], (20, 210))

                    # Display two images
                    out.write(disp_img)
                except Exception as e:
                    print e
                    pass

            # Move onto the next frame
            old_im = new_im

        cap.release()
        out.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
