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
using cv::CascadeClassifier;

#define QQQ do {std::cerr << "QQQ " << __FUNCTION__ << " " << __LINE__ << std::endl;} while(0)

struct StepVizData{
	int displayedStep = 0;
	int currentStep = -1;
	cv::Mat vizMat;
};

extern StepVizData stepViz;


bool findFace(
	Mat& image,
	Mat& templ,
  CascadeClassifier face_cascade
);
bool findFace(
	Mat& image,
	vector<Point2f>& square,
  CascadeClassifier face_cascade
);
bool findCorr(
	Mat& image,
	Mat& templ,
	vector<Point2f>& square,
	int match_method = 0.
);
void rectToVect(cv::Rect rect, vector<Point2f>& vect);
void binarizeImageInv(const Mat &src, Mat &dst);
bool findOpenSquare(Mat image, vector<Point2f> &square);
void findRailMarkers(const Mat &image, vector<Point2f> &outMarkers);
void findFloorTileMarkers(
  const Mat &image,
  vector<Point2f> &outMarkers,
  int thresholdC = 7,
  int houghThreshold = 30
);
