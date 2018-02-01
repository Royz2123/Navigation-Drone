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
#define NUM_THREADS 2

#define AXIS_COUNT 3

#define BATTERY_ADDITION 50
#define BATTERY_FACTOR 1000.0

#define SCALING 1.5

#define MAX_MOVEMENT 40

// TODO: Remove Direction
enum Direction
{
	DIRECTION_UP = 1 << 0,
	DIRECTION_RIGHT = 1 << 1,
	DIRECTION_DOWN = 1 << 2,
	DIRECTION_LEFT = 1 << 3,
	DIRECTION_ALL = 0xF,
};

/*
MULTITHREAD MAIN

int main()
{
  std::thread first (foo);     // spawn new thread that calls foo()
  std::thread second (bar,0);  // spawn new thread that calls bar(0)

  std::cout << "main, foo and bar now execute concurrently...\n";

  // synchronize threads:
  first.join();                // pauses until first finishes
  second.join();               // pauses until second finishes

  std::cout << "foo and bar completed.\n";

  return 0;
}
*/

std::string vecToString(Vec4f vect){
	std::string strVect = "";
	for(int i = 0; i < 4; i++) {
		strVect += std::to_string(vect[i]) + ", ";
	}
	return strVect;
}

// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------

// declerations
void heavyDetect();
void mainLoop();

//globals
int cameraIndex = 0;
int detectMode = HET_MODE;

Mat image;
Mat templ;

// synchronization tools
bool flagCS1[2] = {false, false};
bool flagCS2[2] = {false, false};
int turnCS1;
int turnCS2;
std::mutex image_lock;
std::mutex templ_lock;

std::thread threads[2];

int main(int argc, char* argv[]) {
	// Handle args
	if (argc - 1 > 0)
		cameraIndex = atoi(argv[1]);

	if (argc - 2 > 0){
		std::string req_mode = argv[2];
		if (req_mode.compare("face") == 0)
			detectMode = FACE_MODE;
	}

	// check detectMode
	if (detectMode == HET_MODE) {
		mainLoop();
	} else {
		std::thread th1 (mainLoop);
	  std::thread th2 (heavyDetect);

	  th1.join();
	  th2.join();
	}
	return 0;
}


// Updates the answer matrix with the new correlation
void heavyDetect() {
	// Job of heavyDetect is to update the templ matrix with the new correlation
	// This should be done with minimum wait time for the main loop.
	// every global they modify should be locked for the minimum wait time.

	// Load Cascade for detection
	CascadeClassifier face_cascade;
	if (detectMode == FACE_MODE) {
		// Cascade stuff
		std::string face_cascade_name = (
			"../../../OpenCV/data/haarcascades/haarcascade_frontalface_alt.xml"
		);
		if (!face_cascade.load(face_cascade_name)) {
			printf("Failed loading the Cascade\n");
			return;
		}
	}

	Mat my_image;
	Mat my_templ;

	while(true) {
		usleep(100000);
		// we want to keep the main loop waiting as less as we can.
		// Dont use Matrices directly, copy them to our own variables

		// quickly clone latest frame
		/*
		flagCS1[0] = true;
		turnCS1 = 1;
		while (flagCS1[1] == true && turnCS1 == 1);
		my_image = image.clone();
		flagCS1[0] = false;

		// find template
		findFace(my_image, my_templ, face_cascade);

		// quickly update template
		flagCS2[0] = true;
		turnCS2 = 1;
		while (flagCS2[1] == true && turnCS2 == 1);
		templ = my_templ.clone();
		flagCS2[0] = false;
		*/

		/*
		my_image = image.clone();
		findFace(my_image, my_templ, face_cascade);
		templ = my_templ.clone();
		*/

		// quickly clone latest frame
		image_lock.lock();
		my_image = image.clone();
		image_lock.unlock();

		// find template
		findFace(my_image, my_templ, face_cascade);

		// quickly update template
		templ_lock.lock();
		templ = my_templ.clone();
		templ_lock.unlock();

	}
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
	Point3f droneTarget = {worldSquareSize, 0, distance};

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

	Mat frame;

	// set up camera numbers
	Mat sonyEyeCameraMatrix = (cv::Mat_<float>(3, 3) <<
		5.38614e+02, 0., 3.10130e+02,
		0., 5.38112e+02, 2.27066e+02,
		0., 0., 1.);
	Mat txCameraMatrix = (cv::Mat_<float>(3, 3) <<
		6.7628576774457656e+02, 0., 3.0519865395809290e+02,
		0., 6.7561534030641053e+02, 2.4692172053127743e+02,
		0., 0., 1.);
	Mat cameraMatrix = txCameraMatrix;

	// NOTE(Andrey): A down facing camera
	Affine3f droneCameraTransform{Vec3f{1, 0, 0} * (CV_PI / 2)};
	Affine3f invDroneCameraTransform = droneCameraTransform.inv();

	// face cascade
	CascadeClassifier face_cascade;
	if (detectMode == FACE_MODE) {
		// Cascade stuff
		std::string face_cascade_name = (
			"../../../OpenCV/data/haarcascades/haarcascade_frontalface_alt.xml"
		);
		if (!face_cascade.load(face_cascade_name)) {
			printf("Failed loading the Cascade\n");
			return;
		}
	}

	Affine3f droneTransform;

	// smoothing variables
	Vec3f smoothPosition{0, 0, 0};
	float smoothingFactor = 0.3;
	vector<Point2f> oldCameraSquare;

	bool first_run = true;

	int64 lastFrameTickCount = 0;

	bool flightModeOn = false;
	bool paused = false;
	bool lockMode = false;
	int pressedKey = 0;

	while (pressedKey != 'q') {
		if (!paused) {
			cap >> frame;

			if (txMode) {

				//
				// Deinterlace and flip
				//

				// TODO: Do solve PnP in the original frame

				frame = frame(Range{5, frame.rows}, Range{0, frame.cols - 7});

				// NOTE(Andrey): From experiments we found the odd lines are
				// more recent in both TX05 and TX03.

				for (int i = 0; i < frame.rows - 1; i += 2) {
					frame.row(i + 1).copyTo(frame.row(i));
				}

				//const int horizontalAndVertical = -1;
				//cv::flip(frame, frame, horizontalAndVertical);
			}
		}

		VIZ_STEP
		{
			VIZ_MAT = frame.clone();
		}

		// update timings
		float deltaTime = (float)(cv::getTickCount() - lastFrameTickCount) / cv::getTickFrequency();
		lastFrameTickCount = cv::getTickCount();

		// normalize for detection
		if (detectMode == FACE_MODE) {
			cv::resize(
				frame,
				frame,
				cv::Size(frame.cols/SCALING, frame.rows/SCALING),
				0, 0, cv::INTER_CUBIC
			);
		}

		// clone frame for face detection & detect pattern
		bool found;
		if (detectMode == HET_MODE) {
			image = frame.clone();
			found = findOpenSquare(frame, cameraSquare);
		} else {
			/*
			flagCS1[1] = true;
			turnCS1 = 0;
			while (flagCS1[0] == true && turnCS1 == 0);
			image = frame.clone();
			flagCS1[1] = false;

			flagCS2[1] = true;
			turnCS2 = 0;
			while (flagCS2[0] == true && turnCS2 == 0);
			found = findCorr(frame, templ, cameraSquare);
			flagCS2[1] = false;
			*/

			/*
			image = frame.clone();
			found = findCorr(image, templ, cameraSquare);
			*/

			image_lock.lock();
			image = frame.clone();
			image_lock.unlock();

			templ_lock.lock();
			found = findCorr(frame, templ, cameraSquare);
			templ_lock.unlock();

			/*
			image = frame.clone();
			found = findFace(image, cameraSquare, face_cascade);
			*/

		}
		// update distance from trackbar
		droneTarget.z = distance;


		// check cameraSquare
		if (lockMode) {
			// make sure oldCameraSquare has been defined
			if (!found) {
				cameraSquare = oldCameraSquare;
			} else {
				// look for extreme movement, caused by errors and such
				bool badMovement = false;
				for(int i=0; i < 4; i++){
					if (
						abs(cameraSquare[i].x - oldCameraSquare[i].x) > MAX_MOVEMENT
						|| abs(cameraSquare[i].y - oldCameraSquare[i].y) > MAX_MOVEMENT
					) {
						badMovement = true;
					}
				}
				// work with badMovement
				if (badMovement) {
					cameraSquare = oldCameraSquare;
				} else {
					oldCameraSquare = cameraSquare;
				}
			}
		} else if(found) {
			oldCameraSquare = cameraSquare;
		}

		std::cout << std::endl << "OLD: " << cameraSquare << std::endl;
		std::cout << "NEW: " << oldCameraSquare << std::endl <<std::endl;


		VIZ_STEP
		{
			VIZ_MAT = frame.clone();

			for (unsigned int i = 0; i < cameraSquare.size(); i++) {
				Scalar color = (i == cameraSquare.size() - 1) ? Scalar{66, 66, 244} : Scalar{66, 244, 66};
				cv::line(VIZ_MAT, cameraSquare[i], cameraSquare[(i + 1) % cameraSquare.size()], color, 3);
			}
		}

		// normalize back from detection
		if (detectMode == FACE_MODE) {
			// normalize
			for (unsigned int i = 0; i < cameraSquare.size(); i++) {
				cameraSquare[i].x *= SCALING;
				cameraSquare[i].y *= SCALING;
			}
		}

		if (!found) {
			std::cerr << "NOT FOUND" << std::endl;
		} else {
			// for now lets ignore the rvec
			// Need to see if we can modify for trapezim, Eli said it's bad
			Mat rvec;
			Mat tvec;

			cout << "PNP INPUT: " << cameraSquare << std::endl;
			std::cout << "HELLOOOO" << std::endl;


			found = cv::solvePnP(worldSquare,
				cameraSquare,
				cameraMatrix,
				Mat{},
				rvec,
				tvec
			);

			Affine3f cameraTransform = Affine3f{Affine3d{rvec, tvec}};
			// The square doesn't move, we are moving.
			cameraTransform = cameraTransform.inv();
			// We found the camera, but we want to find the drone
			droneTransform = cameraTransform * invDroneCameraTransform;

			Vec3f pos = droneTransform.translation();

			// now we want to normalize in terms of neg/pos
			// we found that up = -, left = -, back = -
			// for comfort, we want right, up, and back to be +
			pos = {pos[0], -pos[1], -pos[2]};
			std::cout << "NORM SHARP POS: " << pos << std::endl;

			// Check if drastic change (from error or something)
			if (first_run)
				smoothPosition = pos;

			//else if (!bigChange(pos, smoothPosition)){
				// Smooth the positon based on last pos
			//	smoothPosition = smoothingFactor * smoothPosition + (1 - smoothingFactor) * pos;
			//}
			smoothPosition = smoothingFactor * smoothPosition + (1 - smoothingFactor) * pos;

			// Set new translation
			droneTransform.translation(smoothPosition);

			// change from first_run
			first_run = false;
		}
		smoothingFactor = smoothing_factor_slider / 100.0f;

		std::cout << "NORM SMOOTH POS: " << smoothPosition << std::endl;
    std::cout << "TARGET POS: " << droneTarget << std::endl;

		// Trying for now without yaw since it doesn't help us
		// Vec4f controlErrors = calculateControlErrors(droneTransform.translation(), Mat{droneTransform.rotation()}, droneTarget);
		Vec4f controlErrors = simpleControlErrors(
			smoothPosition,
			droneTarget
		);
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
	}
}
