#include "gui.hpp"
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

void showFrameRateInTitle(const char* window)
{
	static int64 freq = static_cast<int>(cv::getTickFrequency());
	static int64 captureLength = freq / 10;

	static int64 start = cv::getTickCount();
	static int frames = 0;

	frames++;

	int64 curr = cv::getTickCount();

	if ((curr - start) >= captureLength) {
		int fps = frames * (freq / (curr - start));
		cv::setWindowTitle(window, std::to_string(fps));
		start = curr;
		frames = 0;
	}
}

void printTimeSinceLastCall(const char* message)
{
	static int64 freq = static_cast<int>(cv::getTickFrequency());
	static int64 last = cv::getTickCount();

	int64 curr = cv::getTickCount();
	int64 delta = curr - last;
	double deltaMs = (double)delta / freq * 1000;
	printf("%s: %.2f\n", message, deltaMs);

	last = curr;
}

void drawFlightViz(Mat &screenBuffer,
		Mat &cameraFeed,
		Affine3f droneTransform,
		float frameRate,
		Point3f droneTarget,
		Vec4f droneControlErrors,
		bool flightModeOn,
		bool lockMode
){
float temp = droneControlErrors[2];
droneControlErrors[2]= droneControlErrors[1];
droneControlErrors[1] = temp;
	Mat q1, q2, q3, q4;

	q1 = cameraFeed;

	Rect2i panelSize{0, 0, q1.cols, q1.rows};

	Mat rotationTop = (cv::Mat_<float>(3, 3) <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1);
	float scaleTop = 1;
	Vec3f shiftTop{(float)panelSize.width / 2, 0, 180+(float)panelSize.height / 2};

	Mat rotationBack = (cv::Mat_<float>(3, 3) <<
		1, 0, 0,
		0, 0, 1,
		0, -1, 0);
	float scaleBack = 1;
	Vec3f shiftBack{(float)panelSize.width / 2, 0, -300 + (float)panelSize.height * 3 / 4};

	Mat rotationLeft = (cv::Mat_<float>(3, 3) <<
		0, 0, 1,
		1, 0, 0,
		0, -1, 0);
	float scaleLeft = 1;
	Vec3f shiftLeft{(float)panelSize.width / 2, 0, -300 + (float)panelSize.height * 3 / 4};

	drawViewOrthographicXZ(q2, panelSize.size(), "TOP" , droneTransform, droneTarget, droneControlErrors, Affine3f{rotationTop * scaleTop, shiftTop}, true);
	drawViewOrthographicXZ(q3, panelSize.size(), "LEFT", droneTransform, droneTarget, droneControlErrors, Affine3f{rotationLeft * scaleLeft, shiftLeft});
	drawViewOrthographicXZ(q4, panelSize.size(), "BACK", droneTransform, droneTarget, droneControlErrors, Affine3f{rotationBack * scaleBack, shiftBack});


	if (lockMode) {
		auto text = "lockMode ACTIVATED";
		auto font = cv::FONT_HERSHEY_SIMPLEX;
		auto fontScale = 0.65;
		auto thickness = 2;

		int tmpBaseline;
		Size textSize = cv::getTextSize(text, font, fontScale, thickness, &tmpBaseline);

		cv::putText(q2,
				text, Point{(q2.cols - textSize.width) / 2, 50 + textSize.height},
				font, fontScale, Scalar{0, 0, 255}, thickness);
	}

	if (!flightModeOn) {
		auto text = "Flight Mode OFF. Press X to begin flight.";
		auto font = cv::FONT_HERSHEY_SIMPLEX;
		auto fontScale = 0.65;
		auto thickness = 2;

		int tmpBaseline;
		Size textSize = cv::getTextSize(text, font, fontScale, thickness, &tmpBaseline);

		cv::putText(q2,
				text, Point{(q2.cols - textSize.width) / 2, 20 + textSize.height},
				font, fontScale, Scalar{0, 0, 255}, thickness);
	}

	Size2i totalSize{panelSize.width * 2 + 1, panelSize.height * 2 + 1};

	screenBuffer.create(totalSize, CV_8UC3);
	screenBuffer.setTo(Scalar{255, 255, 255});

	q1.copyTo(Mat{screenBuffer, Rect2i{0, 0, panelSize.width, panelSize.height}});
	q2.copyTo(Mat{screenBuffer, Rect2i{panelSize.width + 1, 0, panelSize.width, panelSize.height}});
	q3.copyTo(Mat{screenBuffer, Rect2i{0, panelSize.height + 1, panelSize.width, panelSize.height}});
	q4.copyTo(Mat{screenBuffer, Rect2i{panelSize.width + 1, panelSize.height + 1, panelSize.width, panelSize.height}});
}


void drawViewOrthographicXZ(
		Mat &screenBuffer,
		Size2i size,
		const char* name,
		Affine3f droneTransform,
		Point3f droneTarget,
		Vec4f droneControlErrors,
		Affine3f transform,
		bool drawPlaneIcon)
{
	screenBuffer.create(size.height, size.width, CV_8UC3);
	screenBuffer.setTo(Scalar{51, 51, 51});

	Point3f xBegin{-100 * 1000, 0, 0};
	Point3f xEnd = -xBegin;
	Point3f yBegin{0, -100 * 1000, 0};
	Point3f yEnd = -yBegin;
	Point3f zBegin{0, 0, -100 * 1000};
	Point3f zEnd = -zBegin;

	Scalar axesColor{242, 158, 106};

	cv::line(screenBuffer, orthoProject(xBegin, transform), orthoProject(xEnd, transform), axesColor);
	cv::line(screenBuffer, orthoProject(yBegin, transform), orthoProject(yEnd, transform), axesColor);
	cv::line(screenBuffer, orthoProject(zBegin, transform), orthoProject(zEnd, transform), axesColor);

	Point2i planePosition = orthoProject(droneTransform.translation(), transform);
	cv::circle(screenBuffer, planePosition, 3, Scalar{255, 255, 255}, 2);


	cv::putText(screenBuffer, name, {size.width - 50, 27}, cv::FONT_HERSHEY_PLAIN, 1, axesColor);

	if (drawPlaneIcon) {
		Mat currRotation{droneTransform.rotation()};

		float planeRotation = rotationYaw(Mat{droneTransform.rotation()});

		// http://www.challengers101.com/images/CompRose1.jpg
		vector<Point2i> planeIconHalf1 = {
			{0,  -141},
			{0,  -108},
			{9,  -90},
			{12, -71},
			{14, -53},
			{75, 6},
			{76, 13},
			{75, 26},
			{12, -6},
			{9,  49},
			{30, 71},
			{33, 80},
			{31, 89},
			{4,  76},
			{0,  82},
		};

		vector<Point2i> planeIconHalf2 = planeIconHalf1;
		std::reverse(planeIconHalf2.begin(), planeIconHalf2.end());

		for (auto &p : planeIconHalf2) {
			p = Point2i{-p.x, p.y};
		}

		vector<Point2i> planeIcon = planeIconHalf1;
		planeIcon.insert(planeIcon.begin(), planeIconHalf2.begin(), planeIconHalf2.end());

		for (auto &p : planeIcon) {
			float cosT = std::cos(planeRotation);
			float sinT = std::sin(planeRotation);
			p = Point2i{(int)(p.x * cosT - p.y * sinT), (int)(p.x * sinT + p.y * cosT)};

			p /= 2;
			p += planePosition;
		}

		vector<vector<Point2i>> planeIconWrapper = {planeIcon};

		cv::polylines(screenBuffer, planeIconWrapper, false, Scalar{255, 255, 255});
	}

	Point2i droneTargetPosition = orthoProject(droneTarget, transform);
	int rectSide = 10;
	cv::rectangle(screenBuffer, Rect2i{droneTargetPosition.x - rectSide / 2, droneTargetPosition.y - rectSide / 2, rectSide, rectSide}, Scalar{66, 66, 244}, 2);

	Point3f controlErrors{droneControlErrors[0], droneControlErrors[1], -droneControlErrors[2]};
	//controlErrors2D = droneTransform * controlErrors;

	Point2i dronePosition = orthoProject(droneTransform.translation(), transform);
	Point2i shiftedControlErrors = orthoProject(droneTransform * controlErrors, transform);
	cv::arrowedLine(screenBuffer, dronePosition, shiftedControlErrors, Scalar{66, 66, 244}, 2);
}

Point2i orthoProject(Point3f point, Affine3f cameraMatrix)
{
	point = cameraMatrix * point;
	return Point2i{(int)point.x, (int)point.z};
}
