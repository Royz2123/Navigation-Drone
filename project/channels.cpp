#include "channels.hpp"

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


int lerp(float t, int a, int b)
{
	return (int)((1 - t) * a + t * b);
}

int transformIntoChannel(float normalizedValue, ChannelBounds bounds)
{
	float t = normalizedValue;
	if (t >= 0) {
		return lerp(t, bounds.zero, bounds.max);
	}
	else {
		return lerp(-t, bounds.zero, bounds.min);
	}
}

int throttleCalculate(float a) {
		// a is between [-1, 1]
		int channel = (int)(a * 64 + 63);

		// normalize
		channel = ( channel > 127 ) ? 127 : channel;
		channel = ( channel < 0 ) ? 0 : channel;

		return channel;
}

Vec4b controlsToDroneChannels(Vec4f normalizedInput)
{
  // Drone channels:
	// left and up are good
	// back and forward are also good as they are:
	// since our "+" is back, and the drone is switched, no need to change
	// anything.

	//ChannelBounds rightBounds left-127, mid - 64, right - 0;
	//ChannelBounds forwardBounds -forward -127, mid - 64, backward - 0;
	//ChannelBounds upBounds{0, 50, 100};
	//ChannelBounds clockwiseBounds{127, 64, 0};

	int right = throttleCalculate(-normalizedInput[0]);
	int up = throttleCalculate(normalizedInput[1]);
	int forward = throttleCalculate(-normalizedInput[2]);
  int clockwise = 64;

	//int clockwise = transformIntoChannel(normalizedInput[3], clockwiseBounds);

	// NOTE(Andrey): IMPORTANT: We change the order of the values here!
	return {(uchar)right, (uchar)forward, (uchar)up, (uchar)clockwise};
}

void printChannels(Vec4b channels){
	std::cout << "CHANNEL: right: " << (int)channels[0];
	std::cout << ", forward: " << (int)channels[1];
	std::cout << ", up: " << (int)channels[2];
	std::cout << ", clockwise: " << (int)channels[3] << std::endl;
}

void controlDrone(
	QuadSerial &serial,
	Vec4f pidControl,
	bool devoMode,
	bool flightModeOn
) {
	Vec4b channelControls = controlsToDroneChannels(pidControl);

	if (!flightModeOn) {
		channelControls = Vec4b{64, 64, 0, 64};
	}

	printChannels(channelControls);

	if (serial.isOpened()) {
		bool success;

		// TODO(Andrey): Work with floats instead
		if (devoMode) {
			success = serial.sendDevo(
					channelControls[0],
					channelControls[1],
					channelControls[2],
					channelControls[3]);
		}
		else{
			success = serial.send(
					((int)channelControls[0] - 64) * 2,
					((int)channelControls[1] - 64) * 2,
					((int)channelControls[2] - 64) * 2,
					((int)channelControls[3] - 64) * 2);
		}

		if (!success)
			std::cout << "Failed to send to Arduino" << std::endl;
	}
}
