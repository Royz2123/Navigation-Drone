#pragma once

#include "quad_serial.hpp"

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

struct ChannelBounds
{
	int min;
	int zero;
	int max;
};

int lerp(float t, int a, int b);
int transformIntoChannel(float normalizedValue, ChannelBounds bounds);
void printChannels(Vec4b channels);
int throttleCalculate(float a);
Vec4b controlsToDroneChannels(Vec4f normalizedInput);
void controlDrone(
	QuadSerial &serial,
	Vec4f pidContol,
	bool devoMode,
	bool flightMode
);
