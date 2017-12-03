import cv2
import sys


def print_usage():
    print """
        Usage: mvmnt.py    [im1]     [im2]
    """

def main():
    if len(sys.argv) < 3:
        print_usage()
    
    # read the two images
    im1 = cv2.imread(sys.argv[1])
    im2 = cv2.imread(sys.argv[2])
    
    # estimate the transform between them
    transform = cv2.estimateRigidTransform(im1, im2, fullAffine=True)
    
    print(transform)






if __name__ == "__main__":
    main()