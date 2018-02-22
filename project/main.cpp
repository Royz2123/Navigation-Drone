#include "channels.hpp"
#include "pid.hpp"
#include "gui.hpp"
#include "location.hpp"
#include "detection.hpp"
#include "quad_serial.hpp"

#include <pthread.h>
#include <mutex>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>         // std::thread

#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <opencv2/opencv.hpp>

using std::vector;

using cv::Mat;
using cv::Scalar;
using cv::Point;
using cv::Point2f;
using cv::Point2i;
using cv::Point3f;
using cv::Point3i;
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

#define VIZ_STEP if (++stepViz.currentStep == stepViz.displayedStep)

#define VIZ_MAT stepViz.vizMat

#define FACE_MODE 0
#define HET_MODE 1
#define NAV_MODE 2
#define CORR_MODE 3

#define NUM_THREADS 2

#define AXIS_COUNT 3

#define BATTERY_ADDITION 50
#define BATTERY_FACTOR 1000.0

#define SCALING 1.5

#define MAX_MOVEMENT 40

#define TX_TYPE 0
#define SONY_TYPE 1

// TODO: Remove Direction
enum Direction
{
	DIRECTION_UP = 1 << 0,
	DIRECTION_RIGHT = 1 << 1,
	DIRECTION_DOWN = 1 << 2,
	DIRECTION_LEFT = 1 << 3,
	DIRECTION_ALL = 0xF,
};

// Function declerations
void heavyDetect();
void mainLoop();
std::string vecToString(Vec4f vect);
bool badMovement(
	vector<Point2f>& cameraSquare,
	vector<Point2f>& oldCameraSquare
);
void squareToPosition(
	Vec3f dronePosition,
	vector<Point3f>& worldSquare,
	vector<Point2f>& cameraSquare,
	bool cameraType = TX_TYPE
);
void smoothPos(
	Vec3f smoothPosition,
	Vec3f dronePosition,
	float smoothingFactor,
	bool first_run
);


// Global variables
int cameraIndex = 0;
int detectMode = NAV_MODE;

int main(int argc, char* argv[]) {
	// Handle args
	if (argc - 1 > 0)
		cameraIndex = atoi(argv[1]);

	// Set flight mode
	if (argc - 2 > 0) {
		if (strcmp(argv[2], "face") == 0) {
			detectMode = FACE_MODE;
		} else if (strcmp(argv[2], "het") == 0) {
			detectMode = HET_MODE;
		} else if (strcmp(argv[2], "corr") == 0) {
			detectMode = CORR_MODE;
		} else {
			detectMode = NAV_MODE;
		}
	}

	// Call mainLoop, detection will be based on flight mode
	mainLoop();
	return 0;
}


void mainLoop() {
	//
	// In Pattern mode, the drone stabilizes in front of one of the following patterns:
	// a) in front of a black pattern that looks like ח (HET_MODE)
	// b) in front of a face (FACE_MODE)
	//
	// TX is the wireless camera we were using, and which required special handling:
	// In TX mode the image is flipped vertically, a small black area near
	// the borders is removed, and simple de-interlacing is applied.
	//
	bool txMode = true;

	//
	// DEVO mode is for the older remotes that use a physical Walkera DEVO
	// drone controller. Leave this off of the small new remotes.
	//
	bool devoMode = false;

	// set world measurement
	const int worldSquareSize = 15;
	int distance = 120;
	vector<Point3f> worldSquare = {
		{0, 0, 0},
		{0, (float)-worldSquareSize, 0},
		{(float)worldSquareSize, (float)-worldSquareSize, 0},
		{(float)worldSquareSize, 0, 0}
	};
	vector<Point2f> cameraSquare;
	vector<Point2f> oldCameraSquare;

	Point3f droneTarget = {worldSquareSize, 0, (float)distance};

	Affine3f droneTransform;

	// Serial stuff
	QuadSerial serial;
	serial.openDefault();
	serial.printErrorIfNotConnected();

	vector<Pid> pidControllers = {
		{5, 5, 55, "x"},
		{5, 30, 55, "y"},
		{5, 5, 40, "z"},
		{1, 0, 0, "r"},
	};
	int smoothing_factor_slider = 40;
	int batteryFactor = 50;

	cv::namedWindow("config", cv::WINDOW_NORMAL);

	for(int i=0;i<(int)pidControllers.size();i++){
		cv::createTrackbar("kP "+pidControllers[i].name, "config", &pidControllers[i].kp, 100);
		cv::createTrackbar("kI "+pidControllers[i].name, "config", &pidControllers[i].ki, 100);
		cv::createTrackbar("kD "+pidControllers[i].name, "config", &pidControllers[i].kd, 100);
	}
	cv::createTrackbar("Smoothing Factor", "config", &smoothing_factor_slider, 100);
	cv::createTrackbar("Battery Factor", "config", &batteryFactor, 2 * BATTERY_ADDITION);
	cv::createTrackbar("Distance", "config", &distance, 200);

	// Face cascade for face mode
	CascadeClassifier face_cascade;
	if (detectMode == FACE_MODE) {
		// Cascade stuff
		std::string face_cascade_name = (
			"../../../../OpenCV/data/haarcascades/haarcascade_frontalface_alt.xml"
		);
		if (!face_cascade.load(face_cascade_name)) {
			printf("Failed loading the Cascade\n");
			return;
		}
	}

	Mat logo = Mat::zeros(100, 100, CV_8U);

	{
		int margin = 10;
		int lineWidth = 20;

		Mat{logo, Range{margin, 100 - margin}, Range{margin, 100 - margin}}.setTo(255);
		Mat{logo, Range{margin + lineWidth, 100}, Range{margin + lineWidth, 100 - margin - lineWidth}}.setTo(0);
	}

	// set up video capture
	VideoCapture cap{cameraIndex};
	if (!cap.isOpened()) {
		std::cerr << "Error: Couldn't capture camera number " << cameraIndex << '\n';
		return;
	}
	cap.set(cv::CAP_PROP_FPS, 60);

	Mat templ;
	Mat frame;
	Mat prevFrame;

	// Scaling of the picture
	int scaling = (detectMode == FACE_MODE) ? SCALING : 1;

	// smoothing variables
	Vec3f dronePosition{0, 0, 0};
	Vec3f smoothPosition{0, 0, 0};
	float smoothingFactor = 0.3;

	bool first_run = true;

	int64 lastFrameTickCount = 0;

	bool flightModeOn = false;
	bool paused = false;
	bool lockMode = false;
	int pressedKey = 0;

	while (pressedKey != 'q') {
		if (!paused) {
			if(!first_run) {
				prevFrame = frame.clone();
			}
			cap >> frame;

			if (txMode) {
				frame = frame(Range{5, frame.rows}, Range{0, frame.cols - 7});

				// NOTE(Andrey): From experiments we found the odd lines are
				// more recent in both TX05 and TX03.

				for (int i = 0; i < frame.rows - 1; i += 2) {
					frame.row(i + 1).copyTo(frame.row(i));
				}
			}
		}

		VIZ_STEP
		{
			VIZ_MAT = frame.clone();
		}

		// update timings
		float deltaTime = (float)(cv::getTickCount() - lastFrameTickCount) / cv::getTickFrequency();
		lastFrameTickCount = cv::getTickCount();

		// Find the transform of the drone
		bool found;
		if(detectMode == NAV_MODE) {
			// New technique, find vector directly
			found = findTransform(frame, prevFrame, dronePosition);
		}
		else if (detectMode == CORR_MODE) {

		}
		else {
			// Old techniques, find camera square and convert to position
			// Find Square first (by face or by tiles)
			if(detectMode == FACE_MODE) {
				// Note: If we want this to work we need some resizing
				findFace(frame, templ, face_cascade);
				found = findCorr(frame, templ, cameraSquare);
			} else {
				found = findOpenSquare(frame, cameraSquare);
			}

			// set square in case of problems, e.g: flickering, not found..
			if (!found || (lockMode && badMovement(cameraSquare, oldCameraSquare))) {
				cameraSquare = oldCameraSquare;
			}
			oldCameraSquare = cameraSquare;

			// Normalize square by scaling
			for (unsigned int i = 0; i < cameraSquare.size(); i++) {
				cameraSquare[i].x *= scaling;
				cameraSquare[i].y *= scaling;
			}

			// convert square to smoothed position using solvePnP
			smoothingFactor = smoothing_factor_slider / 100.0f;
			squareToPosition(dronePosition, worldSquare, cameraSquare);
			droneTransform.translation(dronePosition);
			smoothPos(smoothPosition, dronePosition, smoothingFactor, first_run);
		}

		// update distance from trackbar
		droneTarget.z = distance;

		VIZ_STEP
		{
			VIZ_MAT = frame.clone();

			for (unsigned int i = 0; i < cameraSquare.size(); i++) {
				Scalar color = (i == cameraSquare.size() - 1) ? Scalar{66, 66, 244} : Scalar{66, 244, 66};
				cv::line(VIZ_MAT, cameraSquare[i], cameraSquare[(i + 1) % cameraSquare.size()], color, 3);
			}
		}

		// Trying for now without yaw since it doesn't help us
		Vec4f controlErrors = simpleControlErrors(smoothPosition, droneTarget);
		Vec4f pidControl;

		std::cout << "ERRORS: " << controlErrors <<std::endl;

		if (found && flightModeOn) {
			for (int i = 0; i < 4; i++) {
				pidControllers[i].update(deltaTime, controlErrors[i]);
				pidControl[i] = pidControllers[i].calc();
				std::cout << " integral " << pidControllers[i].name << " - " << pidControllers[i].i;
			}
			pidControl[1]+=(pidControllers[1].ki * (batteryFactor - BATTERY_ADDITION))/BATTERY_FACTOR;
			std::cout << std::endl;
		}
		else {
			for (int i = 0; i < 4; i++) {
				pidControl[i] = 0;
			}
		}
		std::cout << "PID: " << pidControl << std::endl;
		controlDrone(serial, pidControl, devoMode, flightModeOn);

		//
		// Draw GUI
		//
		{
			Mat displayedFrame = VIZ_MAT.clone();

			if (displayedFrame.empty()) {
				displayedFrame = Mat{32, 32, CV_8UC3, 0};
			}
			else if (displayedFrame.type() == CV_8U) {
				cv::cvtColor(displayedFrame, displayedFrame, cv::COLOR_GRAY2BGR);
			}

			drawFlightViz(displayedFrame,
				displayedFrame,
				droneTransform,
				59.9,
				droneTarget,
				pidControl * 100,
				flightModeOn,
				lockMode
			);

			// Visulize
			// picture
			cv::imshow("w", displayedFrame);
			showFrameRateInTitle("w");
			cvMoveWindow("w", 550, 0);

			// trackbars
			cv::imshow("config", logo);
			cvMoveWindow("config", 0, 0);

			// errors
			cv::Mat errors = cv::Mat::zeros(250,500,CV_8UC3);
			errors.setTo(255);
			cv::putText(
				errors,
				"Errors:" + vecToString(controlErrors),
				cv::Point(10,30),
				CV_FONT_HERSHEY_DUPLEX,
				0.6,
				cv::Scalar(0,0,0),1,8,false
			);
			cv::putText(
				errors,
				"Pid:   " + vecToString(pidControl),
				cv::Point(10,100),
				CV_FONT_HERSHEY_DUPLEX,
				0.6,
				cv::Scalar(0,0,0),1,8,false
			);
			cv::imshow("errors", errors);
			cvMoveWindow("errors", 0, 600);
		}

		pressedKey = cv::waitKey(1);

		const int BACK_BUTTON = 269025062;
		const int ENTER = 13;

		switch (pressedKey) {
			case 'l':
				lockMode = !lockMode;
				break;
			case ' ':
				paused = !paused;
				break;
			case ENTER:
			case 'x':
				flightModeOn = !flightModeOn;
				for (auto &pid : pidControllers)
					pid.i = 0;
				break;
			case BACK_BUTTON:
			case 'i':
				// NOTE: Reset integral
				for (auto &pid : pidControllers)
					pid.i = 0;
				break;
		}

		//
		// Prepare StepViz for next cycle
		//
		{
			stepViz.currentStep = -1;
			stepViz.vizMat.setTo(0xCC);

			if (pressedKey >= '0' && pressedKey <= '9') {
				if (pressedKey == '0') {
					stepViz.displayedStep = 9;
				}
				else {
					stepViz.displayedStep = pressedKey - '1';
				}
			}
			else if (pressedKey == '-') {
				if (stepViz.displayedStep > 0) {
					stepViz.displayedStep--;
				}
			}
			else if (pressedKey == '=') {
				stepViz.displayedStep++;
			}
		}

		// change from first_run
		first_run = false;
	}
}



// Debug vector to string
std::string vecToString(Vec4f vect){
	std::string strVect = "";
	for(int i = 0; i < 4; i++) {
		strVect += std::to_string(vect[i]) + ", ";
	}
	return strVect;
}


bool badMovement(
	vector<Point2f>& cameraSquare,
	vector<Point2f>& oldCameraSquare
) {
	for(unsigned int i=0; i < cameraSquare.size(); i++){
		if (
			abs(cameraSquare[i].x - oldCameraSquare[i].x) > MAX_MOVEMENT
			|| abs(cameraSquare[i].y - oldCameraSquare[i].y) > MAX_MOVEMENT
		) {
			return true;
		}
	}
	return false;
}


void squareToPosition(
	Vec3f dronePosition,
	vector<Point3f>& worldSquare,
	vector<Point2f>& cameraSquare,
	bool cameraType
) {
	// set up camera numbers
	Mat sonyEyeCameraMatrix = (cv::Mat_<float>(3, 3) <<
		5.38614e+02, 0., 3.10130e+02,
		0., 5.38112e+02, 2.27066e+02,
		0., 0., 1.);
	Mat txCameraMatrix = (cv::Mat_<float>(3, 3) <<
		6.7628576774457656e+02, 0., 3.0519865395809290e+02,
		0., 6.7561534030641053e+02, 2.4692172053127743e+02,
		0., 0., 1.);
	Mat cameraMatrix = (cameraType==TX_TYPE)?txCameraMatrix:sonyEyeCameraMatrix;

	// NOTE(Andrey): A down facing camera
	// Camera transform variables
	Affine3f droneTransform;
	Affine3f cameraTransform;
	Affine3f droneCameraTransform{Vec3f{1, 0, 0} * (CV_PI / 2)};
	Affine3f invDroneCameraTransform = droneCameraTransform.inv();

	// init variables
	Mat rvec;
	Mat tvec;

	// for now lets ignore the rvec
	// Need to see if we can modify for trapezim, Eli said it's bad
	cv::solvePnP(
		worldSquare,
		cameraSquare,
		cameraMatrix,
		Mat{},
		rvec,
		tvec
	);

	// The square doesn't move, we are moving.
	cameraTransform = Affine3f{Affine3d{rvec, tvec}}.inv();
	// We found the camera, but we want to find the drone
	droneTransform = cameraTransform * invDroneCameraTransform;

	dronePosition = droneTransform.translation();

	// now we want to normalize in terms of neg/pos
	// we found that up = -, left = -, back = -
	// for comfort, we want right, up, and back to be +
	dronePosition = {
		dronePosition[0],
		-dronePosition[1],
		-dronePosition[2]
	};
}

void smoothPos(
	Vec3f smoothPosition,
	Vec3f dronePosition,
	float smoothingFactor,
	bool first_run
) {
	// Smooth out the position
	if (first_run) {
		smoothPosition = dronePosition;
	} else {
		smoothPosition = (
			smoothingFactor * smoothPosition
			+ (1 - smoothingFactor) * dronePosition
		);
	}
}
