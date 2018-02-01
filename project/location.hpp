#pragma once

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

float rotationYaw(const Mat &rotation);
bool bigChange(Vec3f currPos, Vec3f lastPos, int errorChange = 7500);
Vec4f simpleControlErrors(Vec3f currPos,  Vec3f targetPos);
Vec4f calculateControlErrors(Vec3f currPos, Mat currRotation, Vec3f targetPos);
