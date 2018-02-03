#include "location.hpp"
#include "detection.hpp"

#include <mutex>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/imgcodecs.hpp"
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
using cv::Vec3f;
using cv::Vec4f;
using cv::Vec4i;
using cv::Vec4b;
using cv::Size;
using cv::Size2i;
using cv::Range;
using cv::Rect;
using cv::Rect2i;
using cv::Affine3d;
using cv::Affine3f;
using cv::VideoCapture;

#define QQQ do {std::cerr << "QQQ " << __FUNCTION__ << " " << __LINE__ << std::endl;} while(0)

#define VIZ_STEP if (++stepViz.currentStep == stepViz.displayedStep)

#define VIZ_MAT stepViz.vizMat

StepVizData stepViz;

void binarizeImageInv(const Mat &src, Mat &dst)
{
	Mat &image = dst;
	image = src.clone();

	if (image.type() == CV_8UC3) {
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}

	CV_Assert(image.type() == CV_8U);

	cv::blur(image, image, Size{4, 4});

	//double minColor;
	//cv::minMaxIdx(image, &minColor);
	//cv::threshold(image, image, minColor + 50, 255, CV_THRESH_BINARY_INV);

	int meanColor = cv::mean(image)[0] - 70;
	int thresh = (meanColor > 0) ? meanColor : 0;

	cv::threshold(image, image, thresh, 255, CV_THRESH_BINARY_INV);

	//cv::threshold(image, image, 0, 255, CV_THRESH_OTSU);
	//cv::bitwise_not(image, image);

	cv::morphologyEx(image,
			image,
			cv::MORPH_CLOSE,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, Size{7, 7}));

	cv::morphologyEx(image,
			image,
			cv::MORPH_OPEN,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, Size{7, 7}));

	VIZ_STEP
	{
		VIZ_MAT = image.clone();
	}
}


// This function:
// 1) finds the transform between the 2 frames
// 2) apply transform to the new drone position
bool findTransform(
	Mat& frame,
	Mat& prevFrame,
	vector<Point2f>& oldCameraSquare,
	vector<Point2f>& cameraSquare
) {
	// set the two vectors (scaling and translation)


}



bool findFace(
	Mat& image,
	Mat& templ,
  CascadeClassifier face_cascade
) {
	if(image.empty()){
    std::cout << "No image found" << std::endl;
    return false;
	}

  // Init vars
	vector<Rect> faces;
	Mat frame_gray;

  // Picture stuff
  cv::cvtColor(image, frame_gray, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(frame_gray, frame_gray);

  // Detect faces
  face_cascade.detectMultiScale(
		frame_gray,
		faces,
		1.1,
		2,
		0 | cv::CASCADE_SCALE_IMAGE,
		Size(30, 30)
	);

  // Not found? nothing we can do...
	if (!faces.size())
	  return false;

	// TODO: Check if first face is the best face
	// biggest? most similat? for now go for index 0
  templ = image(faces[0]);
	return true;
}

bool findFace(
	Mat& image,
	vector<Point2f>& square,
  CascadeClassifier face_cascade
) {
	if(image.empty()){
    std::cout << "No image found" << std::endl;
    return false;
	}

  // Init vars
	vector<Rect> faces;
	Mat frame_gray;

  // Picture stuff
  cv::cvtColor(image, frame_gray, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(frame_gray, frame_gray);

  // Detect faces
  face_cascade.detectMultiScale(
		frame_gray,
		faces,
		1.1,
		2,
		0 | cv::CASCADE_SCALE_IMAGE,
		Size(30, 30)
	);

  // Not found? nothing we can do...
	if (!faces.size())
	  return false;

	// TODO: Check if first face is the best face
	// biggest? most similat? for now go for index 0
	// save location
	rectToVect(
		Rect(
			Point(faces[0].x, faces[0].y),
			Point(faces[0].x + faces[0].width, faces[0].y + faces[0].height)
		),
		square
	);
	return true;
}

bool findCorr(
	Mat& image,
	Mat& templ,
	vector<Point2f>& square,
	int match_method
){
	// check if we have a template to work with
	if (templ.empty()) {
		return false;
  }

	Mat result;
	// Match template
 	matchTemplate(image, templ, result, match_method);
  normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, Mat());

  // Find best match
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

  // For SQDIFF and SQDIFF_NORMED, the best matches are lower values.
	// For all the other methods, the higher the better
  if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED)
		matchLoc = minLoc;
  else
  	matchLoc = maxLoc;

  // save location
	rectToVect(
		Rect(
			matchLoc,
			Point(matchLoc.x + templ.cols , matchLoc.y + templ.rows)
		),
		square
	);

	// return successful finding
  return true;
}

// Turn a Rect into a vector
void rectToVect(Rect rect, vector<Point2f>& vect) {
	vect.resize(0);
	vect.push_back(Point2f(
		rect.x,
		rect.y + rect.height)
	);
	vect.push_back(Point2f(
		rect.x,
		rect.y
	));
	vect.push_back(Point2f(
		rect.x + rect.width,
		rect.y
	));
	vect.push_back(Point2f(
		rect.x + rect.width,
		rect.y + rect.height
	));
}


//                    ####
// Find the pattern:  #  #  in the image.
//                    #  #

bool findOpenSquare(Mat image, vector<Point2f> &square)
{
	square.resize(0);

	binarizeImageInv(image, image);

	//
	// Find the contours
	//

	vector<vector<Point2i>> contours;

	{
		Mat imageCopy = image.clone();

		cv::findContours(imageCopy,
				contours,
				cv::noArray(),
				CV_RETR_LIST,
				CV_CHAIN_APPROX_NONE);
	}

	if (contours.size() == 0) {
		return false;
	}

	//
	// Select contour with largest area
	//

	int largestContourIndex = -1;

	{
		double maxArea = -1;

		for (int i = 0; i < (int)contours.size(); i++) {
			double currArea = cv::contourArea(contours[i]);

			if (currArea > maxArea) {
				maxArea = currArea;
				largestContourIndex = i;
			}
		}
	}

	vector<Point2i> chosenContour = contours[largestContourIndex];

	VIZ_STEP
	{
		VIZ_MAT = Mat::zeros(image.size(), CV_8U);

		cv::drawContours(VIZ_MAT, contours, largestContourIndex, 255, -1);
	}

	//
	// Find bounding square
	//

	cv::RotatedRect boundingRect = cv::minAreaRect(chosenContour);
	auto size = boundingRect.size;
	boundingRect.size.width = boundingRect.size.height = (size.width + size.height) / 2;
	square.resize(4);
	boundingRect.points(&square[0]);

	//
	// Reorder square points into CW order
	//

	Point2f centroid = (square[0] + square[1] + square[2] + square[3]) / 4;

	std::sort(square.begin(),
			square.end(),
			[centroid](const Point2f &p1, const Point2f &p2) {
			Point2f v1 = p1 - centroid;
			Point2f v2 = p2 - centroid;
			return std::atan2(-v1.y, v1.x) > std::atan2(-v2.y, v2.x);
			});

	//
	// Find the missing edge.
	// The missing edge is the edge whose midpoint
	// is farther from any contour point than the midpint
	// of any other edge.
	//

	float maxMinDistance = -1;
	int maxMinDistanceIndex = -1;

	for (unsigned int i = 0; i < square.size(); i++) {
		Point2f edgeCenter = (square[i] + square[(i + 1) % square.size()]) / 2;

		float currMinDistance = FLT_MAX;

		for (unsigned int k = 0; k < chosenContour.size(); k++) {
			float dx = edgeCenter.x - chosenContour[k].x;
			float dy = edgeCenter.y - chosenContour[k].y;

			float sqrDist = dx * dx + dy * dy;

			currMinDistance = std::min(currMinDistance, sqrDist);
		}

		if (currMinDistance > maxMinDistance) {
			maxMinDistance = currMinDistance;
			maxMinDistanceIndex = i;
		}
	}

	std::rotate(square.begin(), square.begin() + (maxMinDistanceIndex + 1) % square.size(), square.end());

	return true;
}

void findRailMarkers(const Mat &image, vector<Point2f> &outMarkers)
{
	Mat binary;
	binarizeImageInv(image, binary);

	//
	// Find the contours
	//

	vector<vector<Point2i>> contours;
	vector<Vec4i> hierarchy;

	{
		Mat binaryCopy = binary.clone();

		cv::findContours(binaryCopy,
				contours,
				hierarchy,
				CV_RETR_TREE,
				CV_CHAIN_APPROX_NONE);
	}

	//
	// Select contour with largest area
	//

	int largestContour = -1;

	{
		double maxArea = 0;

		for (int i = 0; i < (int)contours.size(); i++) {
			double currArea = cv::contourArea(contours[i]);

			if (currArea > maxArea) {
				maxArea = currArea;
				largestContour = i;
			}
		}
	}

	//
	// Find all inner contours of the largest contour
	//

	vector<vector<Point2i>> markerContours;

	if (largestContour >= 0) {
		int nextMarker = hierarchy[largestContour][2];

		while (nextMarker >= 0) {
			markerContours.push_back(contours[nextMarker]);
			nextMarker = hierarchy[nextMarker][0];
		}
	}

	//
	// The markers are the centroids of those contours
	//

	outMarkers.resize(0);

	{
		for (auto &contour : markerContours) {
			Point2f centroid;
			Point2f sum{0, 0};

			for (auto point : contour) {
				sum += Point2f{point};
			}

			centroid = Point2f{sum.x / contour.size(), sum.y / contour.size()};
			outMarkers.push_back(centroid);
		}
	}

	VIZ_STEP
	{
		VIZ_MAT = image.clone();

		cv::drawContours(VIZ_MAT, contours, largestContour, Scalar{66, 66, 244}, 2);
		cv::drawContours(VIZ_MAT, markerContours, -1, Scalar{66, 244, 66}, 2);

		for (auto m : outMarkers) {
			cv::circle(VIZ_MAT, m, 3, Scalar{244, 66, 66}, -1);
		}
	}
}

void findFloorTileMarkers(const Mat &image, vector<Point2f> &outMarkers, int thresholdC, int houghThreshold)
{
	CV_Assert(image.type() == CV_8U || image.type() == CV_8UC3);

	Mat gray;
	if (image.type() != CV_8U) {
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}
	else {
		gray = image.clone();
	}

	cv::blur(gray, gray, Size{3, 3});

	Mat binary;

	cv::adaptiveThreshold(
			gray,
			binary,
			255,
			cv::ADAPTIVE_THRESH_MEAN_C,
			cv::THRESH_BINARY_INV,
			19 * 2 + 1,
			thresholdC);

	VIZ_STEP
	{
		VIZ_MAT = binary.clone();
	}

	vector<Vec4i> lines;
	cv::HoughLinesP(binary, lines, 1, CV_PI / 180, houghThreshold, 60, 10);

	vector<Point2f> intersections;

	for (size_t i = 0; i < lines.size(); i++) {
		for (size_t k = 0; k < lines.size(); k++) {
			//
			// Given a segment between points P and Q,
			// we represent it as a base point P and vector V = Q - P.
			// The segment consists of all points P + t * V where t is in [0, 1].
			//
			// We have two lines:
			// Pa + x * Va
			// Pb + y * Vb
			//
			// Their intersection gives us two equations:
			// Pa.x + x * Va.x = Pb.x + y * Vb.x
			// Pa.y + x * Va.y = Pb.y + y * Vb.y
			//
			// We transform this into the following matrix:
			// [ (Va.x) (-Vb.x) | (Pb.x - Pa.x) ]
			// [ (Va.y) (-Vb.y) | (Pb.y - Pa.y) ]
			//
			// We solve this matrix using Cramer's Rules, as described here:
			// https://www.cliffsnotes.com/study-guides/algebra/algebra-ii/linear-sentences-in-two-variables/ --
			// linear-equations-solutions-using-determinants-with-two-variables
			//
			// The determinant tells us if there is a solution, and we know it is valid if
			// x and y are both in range [0, 1] (on the segment).
			//
			// We use the dot product of Va and Vb to calculate the
			// cosine of the intersection angle of the segments.
			//
			// Finally, the intersection point is Pa + x * Va. (which is also Pb + y * Vb)
			//

			auto &la = lines[i];
			auto &lb = lines[k];

			Point2f pa{(float)la[0], (float)la[1]};
			Point2f pb{(float)lb[0], (float)lb[1]};

			Point2f va{(float)(la[2] - la[0]), (float)(la[3] - la[1])};
			Point2f vb{(float)(lb[2] - lb[0]), (float)(lb[3] - lb[1])};

			float determinant = va.x * (-vb.y) - va.y * (-vb.x);
			float determinantX = (pb.x - pa.x) * (-vb.y) - (pb.y - pa.y) * (-vb.x);
			float determinantY = va.x * (pb.y - pa.y) - va.y * (pb.x - pa.x);

			if (std::abs(determinant) <= 0.001)
				continue;

			float x = determinantX / determinant;
			float y = determinantY / determinant;

			float magnitudeA = std::sqrt(va.x * va.x + va.y * va.y);
			float magnitudeB = std::sqrt(vb.x * vb.x + vb.y * vb.y);

			const float padding = 30 * 2;

			float minX = -padding / magnitudeA;
			float maxX = 1 + padding / magnitudeA;

			float minY = -padding / magnitudeB;
			float maxY = 1 + padding / magnitudeB;

			if (x < minX || x > maxX || y < minY || y > maxY)
				continue;

			float dot = va.x * vb.x + va.y * vb.y;
			float cosAngle = dot / (magnitudeA * magnitudeB);

			if (std::abs(cosAngle) > 0.3)
				continue;

			Point2f intersection = pa + x * va;

			intersections.push_back(intersection);
		}
	}

	vector<bool> isMerged(intersections.size(), false);
	vector<Point2f> finalPoints;

	const int sqrMergeThreshold = 20 * 20;

	for (size_t i = 0; i < intersections.size(); i++) {
		if (isMerged[i])
			continue;

		Point2f sum = intersections[i];
		int count = 1;

		for (size_t k = i + 1; k < intersections.size(); k++) {
			if (isMerged[k])
				continue;

			Point2i diff = intersections[i] - intersections[k];
			int sqrDistance = diff.x * diff.x + diff.y * diff.y;

			if (sqrDistance > sqrMergeThreshold)
				continue;

			sum += intersections[k];
			count++;
			isMerged[k] = true;
		}

		finalPoints.push_back(sum / count);
	}

	outMarkers = finalPoints;

	VIZ_STEP
	{
		VIZ_MAT = image.clone();

		for (size_t i = 0; i < lines.size(); i++) {
			Vec4i l = lines[i];
			cv::line(VIZ_MAT, Point{l[0], l[1]}, Point{l[2], l[3]}, Scalar{0,0,255});
		}

		for (auto point : finalPoints) {
			cv::circle(VIZ_MAT, point, 5, Scalar{255, 0, 0}, -1);
    }
	}
}
