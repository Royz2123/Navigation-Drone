#include "location.hpp"

#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

using std::vector;

using cv::Mat;
using cv::Scalar;
using cv::Point;
using cv::Point2f;
using cv::Point2i;
using cv::Point3f;
using cv::Vec3f;
using cv::Vec4f;
using cv::Vec4i;
using cv::Vec4b;
using cv::Size;
using cv::Size2i;
using cv::Range;
using cv::Rect2i;
using cv::Affine3d;
using cv::Affine3f;
using cv::VideoCapture;

#define QQQ do {std::cerr << "QQQ " << __FUNCTION__ << " " << __LINE__ << std::endl;} while(0)


float rotationYaw(const Mat &rotation)
{
	float zx = rotation.at<float>(0, 2);
	float zz = rotation.at<float>(2, 2);
	float atanY = zx;
	float atanX = zz;

	float yawCCW = std::atan2(atanY, atanX);

	return -yawCCW;
}

float deltaPos;
Vec4f calculateControlErrors(Vec3f currPos, Mat currRotation, Vec3f targetPos)
{
	float yawCW = rotationYaw(currRotation);
	deltaPos = currPos[1] - targetPos[1];
	Affine3f yawRotation{Vec3f{0, 1, 0} * -yawCW};

	Point3f droneWorldForward = yawRotation * Vec3f{0, 0, -1};
	Point3f droneWorldRight = yawRotation * Vec3f{1, 0, 0};
	Point3f droneWorldUp = yawRotation * Vec3f{0, 1, 0};

	Point3f target = targetPos - currPos;

	Vec4f errors {
		target.dot(droneWorldRight),
		target.dot(droneWorldUp),
		target.dot(droneWorldForward),
		0 - yawCW
	};

	return errors;
}

Vec4f simpleControlErrors(Vec3f currPos,  Vec3f targetPos)
{
	Point3f target = targetPos - currPos;

	Vec4f errors {
		target.x,
		target.y,
		target.z,
		0.,
	};
	return errors;
}

bool bigChange(Vec3f currPos, Vec3f lastPos, int errorChange){
	return (currPos - lastPos).dot(currPos - lastPos) > errorChange;
}
