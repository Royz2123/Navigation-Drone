OpenCV Axes
===========

OpenCV defines the axes in the following way:

For images:

     +---------- X
     |
     |
     |
     
     Y

For 3D operations: (cv::solvePnP)

     Y
      
     |
     |
     |
     |
     +---------- X
    /
   / 
     
 Z 

However, the forward direction for the camera is +Z!
A camera that looks forward will look like this:

              Y
   
              |   Z
              |  /
              | /
              |/
   X ---------+
     

Control Axes
============

We define the drone control axes to be:
(The drone is facing the Z axis)

     Y
     |
     |   Z
     |  /
     | /
     |/
     +---------- X

W is CW yaw rotation.

The drone defines the control axes to be:
(The drone is facing the Z axis)

              Z
   
              |   Z
              |  /
              | /
              |/
   X ---------+

W is CCW yaw rotation.

The drone input vector is (X, Y, Z, W).

Open Square
===========

The order of points in the open square pattern is:

     2      3

     ########
     #      #
     #      #
     #      #
             
	 1      4

Doubles
=======

We prefer working with floats and not with doubles.
Maybe one day when we encouter a bug that is caused
by the low precision of floats, we'll reconsider
this decision.
