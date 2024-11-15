#include "imageProcess.h"

/*
	input : 
----	img : the image will be binalize.
----	mask : the result of binalization.
*/
void binalizeImg(const cv::Mat &img, cv::Mat &mask)
{
	cv::Mat grayImg;
	cv::cvtColor(img, grayImg, cv::COLOR_RGBA2GRAY);
	cv::threshold(grayImg, mask, 250, 255, cv::THRESH_BINARY);
}

/*
	input :
----	img : the image we want to take contour.
----	maxContour : the boundary of the object in the image.
*/
void findContour(const cv::Mat &img, std::vector<cv::Point> &maxContour)
{
	cv::Mat mask;
	binalizeImg(img, mask);

	cv::imshow("binary image", mask);
	cv::waitKey(0);


	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(mask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
	
	auto maxNum = 0, maxIndex = 0;
	
	// hierarchy : output the tree structure of every contours, 
	// each element is represented as [next, prev, firstChild, parent]
	for (size_t i = 0; i < hierarchy.size(); ++i)
	{
		if (hierarchy[i][3] == -1)
			continue;
		if (maxNum < contours[i].size())
		{
			maxNum = contours.size();
			maxIndex = i;
		}
	}
	
	// draw the result
	mask = cv::Scalar::all(0);
	cv::drawContours(mask, contours, maxIndex, cv::Scalar::all(255), 3);
	cv::imshow("contour", mask);
	cv::waitKey(0);

	maxContour.assign(contours[maxIndex].begin(), contours[maxIndex].end());
	// conter-clockwise
	/*int sz = maxContour.size() - 1;
	for (int i = 0; i < maxContour.size() / 2; ++i)
	{
		auto tmp = maxContour[i];
		maxContour[i] = maxContour[sz - i];
		maxContour[sz - i] = tmp;
	}*/
	//std::cout << maxContour << std::endl;
}

/*
	target : resample the boundary for traingulation
	input :
----	maxContour : the boundary of the object in the image.
----	ouputContour : the resampled contour for traingulation.
----	nPoints : the number of resample points.
*/
void resample(const std::vector<cv::Point> &maxContour, std::vector<cv::Point> &outputContour, const int nPoints, float &_contourLength)
{

	std::vector<double> pointX, pointY;
	polyLineSpilt(maxContour, pointX, pointY);
	assert(pointX.size() == pointY.size() && pointX.size() > 0);

	std::vector<cv::Point2d> resamplePoints(nPoints);
	resamplePoints[0].x = pointX[0];
	resamplePoints[0].y = pointY[0];
	std::vector<cv::Point2i> points;
	polyLineMerge(points, pointX, pointY);

	// ©Pªø
	auto contourLenth = cv::arcLength(points, false);
	_contourLength = contourLenth;
	//std::cout << "contourLength : " << contourLenth << std::endl;
	auto resampleSize = contourLenth / (double)nPoints;
	auto index = 0, i = 0;
	auto dist = 0.0;
	auto lastDist = 0.0;
	while (i < nPoints)
	{
		assert(index < points.size() - 1);
		lastDist = cv::norm(points[index] - points[index + 1]);
		dist += lastDist;
		if (dist >= resampleSize)
		{
			auto _d = lastDist - (dist - resampleSize);
			cv::Point2d derivation(points[index + 1].x - points[index].x, points[index + 1].y - points[index].y);
			derivation = derivation * (1.0 / cv::norm(derivation));

			assert(i < resamplePoints.size());
			resamplePoints[i] = cv::Point2d(points[index].x, points[index].y) + derivation * _d;
			i++;

			dist = lastDist - _d; // remaining dist

			while (dist - resampleSize > 1e-3)
			{
				assert(i < resamplePoints.size());
				resamplePoints[i] = resamplePoints[i - 1] + derivation * resampleSize;
				dist -= resampleSize;
				++i;
			}
		}
		++index;
	}
	changeType(resamplePoints, outputContour);
}

/*
	target : draw the resampled points on the image.
	input :
----	img : the black image.
----	contour : the contour should be drew on the img.
*/
void drawImage(const cv::Mat &img, const std::vector<cv::Point> &contour)
{
	uint8_t *data = (uint8_t *)img.data;
	auto ch = img.channels();
	auto step = img.step1();
	uint8_t *color = new uint8_t(ch);
	memset(color, 255, ch * sizeof(uint8_t));
	for (size_t i = 0; i < contour.size(); ++i)
	{
		cv::Point2i p = contour[i];
		int index = p.y * step + p.x * ch;
		memcpy(data + index, color, ch * sizeof(uint8_t));
	}

	cv::imshow("1", img);
	cv::waitKey(0);
}

void drawImage(const cv::Mat &img, const std::vector<cv::Point2f> &contour)
{
	uint8_t *data = (uint8_t *)img.data;
	auto ch = img.channels();
	auto step = img.step1();
	uint8_t *color = new uint8_t(ch);
	memset(color, 255, ch * sizeof(uint8_t));
	for (size_t i = 0; i < contour.size(); ++i)
	{
		cv::Point2i p = contour[i];
		int index = p.y * step + p.x * ch;
		memcpy(data + index, color, ch * sizeof(uint8_t));
	}

	cv::imshow("1", img);
	cv::waitKey(0);
}