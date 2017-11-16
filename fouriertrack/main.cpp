#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using std::vector;

using cv::VideoCapture;
using cv::Mat;
using cv::Rect;
using cv::Point;
using cv::Size;

int main(int argc, char *argv[])
{
	int cameraIndex = (argc - 1 > 0) ? atoi(argv[1]) : 0;
	VideoCapture capture{cameraIndex};
	if (!capture.isOpened()) {
		std::cerr << "Error: Couldn't capture camera number " << cameraIndex << '\n';
		return -1;
	}
	capture.set(cv::CAP_PROP_FPS, 60);

	Mat A = cv::imread("A.png", cv::IMREAD_GRAYSCALE);
	Mat B = cv::imread("B.png", cv::IMREAD_GRAYSCALE);

    int pressedKey = 0;
    while (pressedKey != 'q') {
        Mat frame;
        capture >> frame;
		cv::pyrDown(frame, frame);
		cv::cvtColor(frame, frame, CV_BGR2GRAY);
		cv::adaptiveThreshold(frame, frame, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, -3);

		if (A.empty() || B.empty()) {
			A = Mat::zeros(frame.size(), CV_8U);
			B = Mat::zeros(frame.size(), CV_8U);
		}

		Mat result = A - B;

		float scales[] = {2, 1.5, 1, 0.75, 0.5};
		for (float scaleFactor : scales) {
			Mat affineMatrix = (cv::Mat_<float>(2, 3) <<
					scaleFactor, 0, (1 - scaleFactor) / 2 * B.cols,
					0, scaleFactor, (1 - scaleFactor) / 2 * B.rows);
			std::cout << scaleFactor << std::endl;
			cv::imshow("croppedB", B);
			cv::waitKey(0);
			Mat scaledB;
			cv::warpAffine(B, scaledB, affineMatrix, B.size());
			cv::imshow("croppedB", scaledB);
			Mat croppedB = Mat{B, Rect{B.cols / 3, B.rows / 3, B.cols / 3, B.rows / 3}};
			cv::resize(croppedB, croppedB, croppedB.size(), scaleFactor, scaleFactor);
			cv::waitKey(0);
			cv::matchTemplate(A, croppedB, result, cv::TM_CCORR_NORMED);
		}


		Mat display;
		Mat smallA, smallB;
		cv::pyrDown(A, smallA);
		cv::pyrDown(B, smallB);
		cv::vconcat(smallA, smallB, display);
		cv::hconcat(A - B, display, display);

		double maxValue;
		Point maxLoc;
		cv::minMaxLoc(result, nullptr, &maxValue, nullptr, &maxLoc);
		Mat colors = 255 - result / maxValue * 255;
		colors.convertTo(colors, CV_8U);
		cv::applyColorMap(colors, colors, cv::COLORMAP_OCEAN);
		colors.row(maxLoc.y).setTo(cv::Scalar{0});
		colors.col(maxLoc.x).setTo(cv::Scalar{0});
		if (maxLoc.y + 1 < colors.rows)
			colors.row(maxLoc.y + 1).setTo(cv::Scalar{255, 255, 255});
		if (maxLoc.x + 1 < colors.cols)
			colors.col(maxLoc.x + 1).setTo(cv::Scalar{255, 255, 255});
		cv::imshow("other", colors);

        cv::imshow("w", display);
        pressedKey = cv::waitKey(0);

		if (pressedKey == 'a') {
			A = frame.clone();
			cv::imwrite("A.png", A);
		}
		else if (pressedKey == 'b') {
			B = frame.clone();
			cv::imwrite("B.png", B);
		}
    }
    
    return 0;
}
