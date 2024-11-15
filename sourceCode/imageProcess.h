#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

/*
	target : get the vector of x points and y points of the contour.
*/
template <typename T, typename V>
void polyLineSpilt(const std::vector<cv::Point_<T>> &contour, std::vector<V> &contourPx, std::vector<V> &contourPy)
{
	contourPx.resize(contour.size());
	contourPy.resize(contour.size());
	for (size_t i = 0; i < contour.size(); ++i)
		contourPx[i] = (V)(contour[i].x), contourPy[i] = (V)(contour[i].y);
}

/*
	target : merge the x points and y points to the 2D contour
*/
template <typename T, typename V>
void polyLineMerge(std::vector<cv::Point_<T>> &contour, const std::vector<V> &contourPx, 
	const std::vector<V> &contourPy)
{
	assert(contourPx.size() == contourPy.size());
	contour.resize(contourPx.size());
	for (size_t i = 0; i < contour.size(); ++i)
		contour[i].x = (T)(contourPx[i]), contour[i].y = (T)(contourPy[i]);
}

/*
	target : change the type of the source to dest.
*/
template <typename T, typename V>
void changeType(const std::vector<cv::Point_<T>> &source, std::vector<cv::Point_<V>> &dest)
{
	dest.resize(source.size());
	
	for (size_t i = 0; i < source.size(); ++i)
		dest[i].x = (V)(source[i].x), dest[i].y = (V)(source[i].y);
}

void binalizeImg(const cv::Mat &img, cv::Mat &mask);

void findContour(const cv::Mat &img, std::vector<cv::Point> &maxContour);

void resample(const std::vector<cv::Point> &maxContour, std::vector<cv::Point> &outputContour, const int nPoints, float &_contourLenth);

void drawImage(const cv::Mat &img, const std::vector<cv::Point> &contour);
void drawImage(const cv::Mat &img, const std::vector<cv::Point2f> &contour);