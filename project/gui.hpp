#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>
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



// GUI VIZUALIZATION FUNCTIONS

void showFrameRateInTitle(const char* window);
void printTimeSinceLastCall(const char* message);
Point2i orthoProject(Point3f point, Affine3f cameraMatrix);
void drawFlightViz(
    Mat &screenBuffer,
		Mat &cameraFeed,
		Affine3f droneTransform,
		float frameRate,
		Point3f droneTarget,
		Vec4f droneControlErrors,
		bool flightModeOn,
    bool lockMode
);

void drawViewOrthographicXZ(
		Mat &screenBuffer,
		Size2i size,
		const char* name,
		Affine3f droneTransform,
		Point3f droneTarget,
		Vec4f droneControlErrors,
		Affine3f transform,
		bool drawPlaneIcon = false
);
