#pragma once
#if (_MSC_VER >= 1915)
#define no_init_all deprecated
#endif
#include <time.h> // time seed
#include <stdlib.h>     /* srand, rand */
#include <iostream>
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "shader.h"

#include "ABary.h"
#include "ARectangle.h"
#include "ATriangle.h"
#include "InedexEdge.h"
#include "CGALTriangulation.h"

// glm
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include "NANOFLANNWrapper.h"
#include "PoissonGenerator.h"
#include "UtilityFunctions.h"

#include "imageProcess.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

struct Vertex {
	// position
	glm::vec4 Position;
	// texCoords
	glm::vec2 TexCoords;
};


void UniformResample(std::vector<glm::vec2> oriCurve, std::vector<glm::vec2>& resampleCurve, float resampleGap, float &curveLength)
{
	resampleCurve.clear();
	oriCurve.push_back(oriCurve[0]);
	curveLength = 0;
	for (int i = 1; i < oriCurve.size(); ++i)
		curveLength += glm::distance(oriCurve[i], oriCurve[i - 1]);

	int segmentNum = (int)(std::round(curveLength / resampleGap)); // rounding
	resampleGap = curveLength / (float)segmentNum;

	int iter = 0;
	float sumDist = 0.0;
	resampleCurve.push_back(oriCurve[0]);
	while (iter < oriCurve.size() - 1)
	{
		float currentDist = glm::distance(oriCurve[iter], oriCurve[iter + 1]);
		sumDist += currentDist;

		if (sumDist > resampleGap)
		{
			float vectorLength = currentDist - (sumDist - resampleGap);
			glm::vec2 pt1 = oriCurve[iter];
			glm::vec2 pt2 = oriCurve[iter + 1];
			glm::vec2 directionVector = glm::normalize(pt2 - pt1);

			glm::vec2 newPoint1 = pt1 + directionVector * vectorLength;
			resampleCurve.push_back(newPoint1);

			sumDist = currentDist - vectorLength;

			while (sumDist - resampleGap > 1e-8)
			{
				glm::vec2 insertPt2 = resampleCurve[resampleCurve.size() - 1] + directionVector * resampleGap;
				resampleCurve.push_back(insertPt2);
				sumDist -= resampleGap;
			}
		}
		iter++;
	}



	float eps = std::numeric_limits<float>::epsilon();
	glm::vec2 lastPt = oriCurve[oriCurve.size() - 1];
	if (glm::distance(resampleCurve[resampleCurve.size() - 1], lastPt) > (resampleGap - eps)) { resampleCurve.push_back(lastPt); }

	if (glm::distance(resampleCurve[resampleCurve.size() - 1], resampleCurve[0]) < resampleGap * 0.5)
		resampleCurve.pop_back();

	//resampleCurve.push_back(oriCurve[oriCurve.size() - 1]);
}

bool IsClockwise(std::vector<glm::vec2> poly)
{
	float sumValue = 0;
	for (int i = 0; i < poly.size(); ++i)
	{
		glm::vec2 curPt = poly[i];
		glm::vec2 nextPt;
		if (i == poly.size() - 1)
		{
			nextPt = poly[0];
		}
		else
		{
			nextPt = poly[i + 1];
		}
		sumValue += ((nextPt.x - curPt.x) * (nextPt.y + curPt.y));
	}
	if (sumValue >= 0) 
		return false; 

	return true;
}

// find the edge "e" whether in the edgeList, return -1 if not.
int findEdge(InedexEdge e, const std::vector<InedexEdge> &edgeList)
{
	for (int i = 0; i < edgeList.size(); ++i)
	{
		if (e._index1 == edgeList[i]._index1 &&
			e._index2 == edgeList[i]._index2)
			return i;
		if (e._index1 == edgeList[i]._index2 &&
			e._index2 == edgeList[i]._index1)
			return i;
	}
	return -1;
}

int getUnsharedVertexIndex(ATriangle tri, InedexEdge edge)
{
	if (tri.idx1 != edge._index1 && tri.idx1 != edge._index2)
		return tri.idx1;
	if (tri.idx2 != edge._index1 && tri.idx2 != edge._index2)
		return tri.idx2;
	if (tri.idx3 != edge._index1 && tri.idx3 != edge._index2)
		return tri.idx3;
	return -1;
}



bool insidePoly(std::vector<glm::vec2> poly, float x, float y)
{
	int polySize = poly.size();
	bool oddNodes = false;
	int i;
	int j = polySize - 1;
	for (i = 0; i < polySize; ++i)
	{
		if ((poly[i].y < y && poly[j].y >= y ||
			poly[j].y < y && poly[i].y >= y)
			&& (poly[i].x <= x || poly[j].x <= x))
		{
			oddNodes ^= (poly[i].x + (y - poly[i].y) / (poly[j].y - poly[i].y) * (poly[j].x - poly[i].x) < x);
		}
		j = i;
	}
	return oddNodes;
}
void getCentroidTri(const cv::Mat &img, const std::vector<ATriangle> &triangles, 
	const std::vector<glm::vec2> &resamplePts, glm::vec2 &centroid, int &centroidTriIdx)
{
	// calculate true traingle
	std::vector<std::vector<glm::vec2>> trueTraingle;
	for (int i = 0; i < triangles.size(); ++i)
	{
		std::vector<glm::vec2> tri(3);
		tri[0] = resamplePts[triangles[i].idx1];
		tri[1] = resamplePts[triangles[i].idx2];
		tri[2] = resamplePts[triangles[i].idx3];
		trueTraingle.push_back(tri);
	}

	cv::Mat b_img = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	for (int i = 0; i < triangles.size(); ++i)
	{
		if (insidePoly(trueTraingle[i], centroid.x, centroid.y))
		{
			centroidTriIdx = i;
			std::cout << "centroid tri idx : " << centroidTriIdx << std::endl;
			cv::Point p1(trueTraingle[i][0].x, trueTraingle[i][0].y);
			cv::Point p2(trueTraingle[i][1].x, trueTraingle[i][1].y);
			cv::Point p3(trueTraingle[i][2].x, trueTraingle[i][2].y);
			cv::line(b_img, p1, p2, cv::Scalar(255, 255, 255), 2);
			cv::line(b_img, p2, p3, cv::Scalar(255, 255, 255), 2);
			cv::line(b_img, p3, p1, cv::Scalar(255, 255, 255), 2);
			/*cv::imshow("center tri", b_img);
			cv::waitKey(0);
			return;*/
		}
	}
	std::cout << " no find centroid ?!!" << std::endl;
}


void ConputeBarycentric(const std::vector<glm::vec2> &FullContour, const std::vector<glm::vec2> &resampleContour,
	const std::vector<ATriangle> &triangles, const glm::vec2 &centeoid, int centroidTriId,
	std::vector<ABary> &baryCoords, std::vector<int> &contour2triangles, ABary &centroidBary, 
	const std::vector<glm::vec2> &tesselateArt ,std::vector<ABary> &tesselateBary, std::vector<int> &tesselate2Tri)
{
	std::vector<std::vector<glm::vec2>> actualTriangles;
	for (int i = 0; i < triangles.size(); ++i)
	{
		std::vector<glm::vec2> tri(3);
		tri[0] = resampleContour[triangles[i].idx1];
		tri[1] = resampleContour[triangles[i].idx2];
		tri[2] = resampleContour[triangles[i].idx3];
		actualTriangles.push_back(tri);
	}
	
	// map all contour points to different triangles, use the centroid of the triangle to compute the bary coordinate of each points.
	for (int i = 0; i < FullContour.size(); ++i)
	{
		int triIdx = -1;
		ABary bary;
		for (int j = 0; j < triangles.size(); ++j)
		{
			// find the triangle that the points belong to.		
			if (UtilityFunctions::InsidePolygon(actualTriangles[j], FullContour[i].x, FullContour[i].y))
			{
				triIdx = j;
				break;
			}
		}
		// find the triangle that the point closest to.
		if (triIdx == -1)
		{
			std::cout << "art error !!" << std::endl;

			triIdx = -1;
			float dist = 100000000;
			for (int j = 0; j < triangles.size(); ++j)
			{
				float d = UtilityFunctions::DistanceToClosedCurve(actualTriangles[j], FullContour[i]);
				if (d < dist)
				{
					dist = d;
					triIdx = j;
				}
			}
		}
		bary = UtilityFunctions::Barycentric(FullContour[i], actualTriangles[triIdx][0], actualTriangles[triIdx][1], actualTriangles[triIdx][2]);

		baryCoords.push_back(bary);
		contour2triangles.push_back(triIdx);
	}

	// centroid bary
	centroidBary = UtilityFunctions::Barycentric(centeoid, 
		actualTriangles[centroidTriId][0], 
		actualTriangles[centroidTriId][1], 
		actualTriangles[centroidTriId][2]);

	for (int i = 0; i < tesselateArt.size(); ++i)
	{
		int triIdx = -1;
		ABary bary;
		for (int j = 0; j < triangles.size(); ++j)
		{
			
			if (UtilityFunctions::InsidePolygon(actualTriangles[j], tesselateArt[i].x, tesselateArt[i].y))
			{
				triIdx = j;
				break;
			}
		}
		if (triIdx == -1)
		{
			std::cout << "tesselate error !!" << std::endl;

			triIdx = -1;
			float dist = 100000000;
			for (int j = 0; j < triangles.size(); ++j)
			{
				float d = UtilityFunctions::DistanceToClosedCurve(actualTriangles[j], tesselateArt[i]);
				if (d < dist)
				{
					dist = d;
					triIdx = j;
				}
			}
		}
		bary = UtilityFunctions::Barycentric(tesselateArt[i], actualTriangles[triIdx][0], actualTriangles[triIdx][1], actualTriangles[triIdx][2]);
		tesselateBary.push_back(bary);
		tesselate2Tri.push_back(triIdx);
	}

}


void SetContour(std::vector<std::vector<int>>& contourX, const cv::Mat& img)
{
	contourX.clear();
	int height = img.rows;
	int width  = img.cols;
	bool start = false, end = false;
	
	std::vector<int> dataOfRaw;
	for (unsigned int i = 0; i < height; i++)
	{
		for (unsigned int j = 0; j < width; ++j)
		{
			if ((int)img.at<uchar>(i, j) != 0)
			{
				if (!start)		// left point of shape
				{
					start = true;
					dataOfRaw.push_back(j);
				}
				
			}
			else
			{
				if (start && !end)	// end point of shape
				{
					end = true;
					dataOfRaw.push_back(j);
				}
			}
			
			if (start && end)
			{
				start = false;
				end = false;
			}
		}
		if (dataOfRaw.size() % 2 == 1)
			dataOfRaw.push_back(width);

		contourX.push_back(dataOfRaw);
		dataOfRaw.clear();
		start = false;
		end = false;
	}
}

/*
	can modify the "nPoints" to get proper result.
*/
int main(int argc, char **argv)
{
	// --------- saved data ---------------------------------
	// contour
	std::vector<glm::vec2> _smoothContour;
	std::vector<glm::vec2> _resampledContour;
	// edge
	std::vector<InedexEdge> _negSpaceEdges;
	std::vector<InedexEdge> _auxiliaryEdge;
	std::vector<InedexEdge> _edgeList;
	std::vector<std::vector<int>> _edgeToTri;
	// triangle
	std::vector<ATriangle> _triangles;
	// about boundary
	int _skinPointNum = 0;
	float _contourLength = 0;
	// centroid
	glm::vec2 _centroid;
	int _centroidTriIdx = 0;
	ABary _centroidBary;
	// bary coord
	std::vector<ABary> _baryCoord;
	std::vector<int> _art2Triangles;
	// tesselate art
	std::vector<glm::vec2> _tesselateArt;
	std::vector<int> _tesselateArt2Triangles;
	std::vector<ABary> _tesselateBary;
	std::vector<ATriangle> _artTris;;
	// about rotate
	std::vector<glm::vec2> _normFromCentroidArray;
	std::vector<glm::vec2> _rotateArray;
	// GL data
	std::vector<Vertex>	_vertices;
	// bounding box
	float _xMin = 0, _yMin = 0;
	float xMax = 0, yMax = 0;	// tmp data, for get the height & width
	
	

	// read data
	std::string dir = "DataSet\\6\\";
	std::string file = "001";
	std::string input(dir + "image\\" + file + ".png");

	cv::Mat img = cv::imread(input, cv::IMREAD_UNCHANGED);
	if (img.empty())
	{
		std::cout << "didn't read image !!";
		return -1;
	}

	// ---------------------------------------------------
	// convert to RGBA format
	if (img.type() == CV_8UC1)
		cv::cvtColor(img, img, cv::COLOR_GRAY2BGRA);
	else if (img.type() == CV_8UC3)
		cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
	else if (img.type() == CV_8UC4)
		std::cout << "rgba img" << std::endl;
	else
		std::cout << "no image" << std::endl;

	cv::copyMakeBorder(img, img, 10, 10, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));
	cv::resize(img, img, cv::Size(550, 500), 0, 0, cv::INTER_LINEAR);
	
	int imgLength = (img.cols > img.rows) ? img.cols : img.rows;
	
	std::cout << "img length : " << imgLength << std::endl;

	std::vector<cv::Point> contour;
	std::vector<cv::Point2f> resampledContour;

	// ------------- find the smooth contour boundary ---------------	
	findContour(img, contour);

	// smooth contour  (clock wise points)
	for (int i = 0; i < contour.size(); ++i)
		_smoothContour.push_back(glm::vec2(contour[i].x, contour[i].y));
	
	// ------------- get contourX for placement
	/*cv::Mat im = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	std::vector< std::vector<cv::Point>> c;
	c.push_back(contour);
	cv::fillPoly(im, c, cv::Scalar(255) );*/
	

	/*SetContour(_contourX, im);
	cv::Mat bim = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	for (int i = 0; i < _contourX.size(); ++i)
	{
		for (int j = 0; j < _contourX[i].size(); ++j)
		{
			cv::circle(bim, cv::Point(_contourX[i][j], i), 0.001, cv::Scalar(255));
		}
	}
	cv::imshow("_contourX", bim);
	cv::waitKey(0);*/

	// -------------- resample curve ---------------------
	float fVal = imgLength / 500;	// 500 : upscaleFactor
	fVal *= fVal;
	int nPoints = 150 * fVal;
	float resampleGap = std::sqrt(float(nPoints)) / float(nPoints) * imgLength;
	float rGap = (float)(resampleGap * 1.3);  // 1.3 : _boundary_sampling_factor

	UniformResample(_smoothContour, _resampledContour, rGap, _contourLength);
	for (int i = 0; i < _resampledContour.size(); ++i)
		resampledContour.push_back(cv::Point2f(_resampledContour[i].x, _resampledContour[i].y));
	
	std::cout << "contour length : " << _contourLength << std::endl;
	
	cv::Mat black_img = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	//drawImage(black_img, resampledContour);
	for (unsigned int i = 0; i < resampledContour.size(); ++i)
	{
		cv::circle(black_img, resampledContour[i], 3, cv::Scalar(255));
	}
	cv::imshow("re", black_img);
	cv::waitKey(0);

	// --------- add points inside the boundary to build mesh ---------
	float reampleGap = std::sqrt(float(nPoints)) / float(nPoints) * imgLength;
	PoissonGenerator::DefaultPRNG PRNG;
	int seed = 871709;
	srand(seed);
	PRNG = PoissonGenerator::DefaultPRNG(seed);

	const auto points = PoissonGenerator::GeneratePoissonPoints(nPoints, PRNG);
	float sc = imgLength * std::sqrt(2.0f);
	for (auto pt = points.begin(); pt != points.end(); pt++)
	{
		float x = pt->x * sc;
		float y = pt->y * sc;
		cv::Point2f p(x, y);
		glm::vec2 gp(x, y);
		//std::cout << x << " " << y << std::endl;
		// inside polygon
		if (cv::pointPolygonTest(contour, p, false) == 1)
		{
			float d = UtilityFunctions::DistanceToClosedCurve(_smoothContour, gp);
			if (d > reampleGap)
			{
				resampledContour.push_back(p);
				_resampledContour.push_back(gp);
			}
		}
	}
	//drawImage(black_img, resampledContour);
	for (unsigned int i = 0; i < resampledContour.size(); ++i)
	{
		cv::circle(black_img, resampledContour[i], 3, cv::Scalar(255));
	}
	cv::imshow("re", black_img);
	cv::waitKey(0);


	// ----------check data --------------
	for (int i = 0; i < _resampledContour.size(); ++i)
	{
		assert(_resampledContour[i].x == resampledContour[i].x && _resampledContour[i].y == resampledContour[i].y);
	}

	// ---------- skin number ------------
	_skinPointNum = _resampledContour.size();

	// ------------ triangulation -------------- 
	std::vector<int> indices;
	NANOFLANNWrapper* knn = new NANOFLANNWrapper;
	knn->_leaf_max_size = 4;
	knn->SetPointData(_resampledContour);
	knn->CreatePointKDTree();
	
	cv::Rect rect(0, 0, imgLength, imgLength);
	cv::Subdiv2D subdiv(rect);
	for (int i = 0; i < resampledContour.size(); ++i)
		subdiv.insert(cv::Point2f(resampledContour[i]));

	cv::Mat tmpImg = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
	
	/*cv::Mat obimg = cv::Mat::zeros(img.rows, img.cols, CV_8UC4);
	obimg.setTo(cv::Scalar(255, 255, 255, 0));*/

	float eps = 1e-1;
	std::vector<cv::Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	int triID = 0;
	for (int i = 0; i < triangleList.size(); ++i)
	{
		glm::vec2 pt1(triangleList[i][0], triangleList[i][1]);
		glm::vec2 pt2(triangleList[i][2], triangleList[i][3]);
		glm::vec2 pt3(triangleList[i][4], triangleList[i][5]);

		// we don't know the order of trianglelist, so need knn to find the pt's idx
		int idx1 = knn->GetClosestIndices(pt1, 1)[0];
		int idx2 = knn->GetClosestIndices(pt2, 1)[0];
		int idx3 = knn->GetClosestIndices(pt3, 1)[0];
		
		glm::vec2 randPt1 = _resampledContour[idx1];
		glm::vec2 randPt2 = _resampledContour[idx2];
		glm::vec2 randPt3 = _resampledContour[idx3];
		
		/*std::cout << "p1 : " << randPt1.x << " " << randPt1.y << std::endl;
		std::cout << "p2 : " << randPt2.x << " " << randPt2.y << std::endl;
		std::cout << "p3 : " << randPt3.x << " " << randPt3.y << std::endl;
		
		std::cout << "t1 : " << pt1.x << " " << pt1.y << std::endl;
		std::cout << "t2 : " << pt2.x << " " << pt2.y << std::endl;
		std::cout << "t3 : " << pt3.x << " " << pt3.y << std::endl;*/

		if (glm::distance(pt1, randPt1) > eps)
			continue;
		if (glm::distance(pt2, randPt2) > eps)
			continue;
		if (glm::distance(pt3, randPt3) > eps)
			continue;


		glm::vec2 centPt = (pt1 + pt2 + pt3) / 3.0f;
		if (cv::pointPolygonTest(contour, cv::Point2f(centPt.x, centPt.y), true) <= 0)	// outside the contour
		{
			if (std::abs(idx1 - idx2) > 2)
			{
				InedexEdge e(idx1, idx2);
				e._dist = glm::distance(_resampledContour[e._index1], _resampledContour[e._index2]);
				_negSpaceEdges.push_back(e);
			}

			if (std::abs(idx2 - idx3) > 2)
			{
				InedexEdge e(idx2, idx3);
				e._dist = glm::distance(_resampledContour[e._index1], _resampledContour[e._index2]);
				_negSpaceEdges.push_back(e);
			}
				
			if (std::abs(idx3 - idx1) > 2)
			{
				InedexEdge e(idx3, idx1);
				e._dist = glm::distance(_resampledContour[e._index1], _resampledContour[e._index2]);
				_negSpaceEdges.push_back(e);
			}

			continue;
		}

		std::vector<glm::vec2> triPt = { pt1, pt2, pt3 };
		if (IsClockwise(triPt))
		{
			ATriangle tri(idx1, idx2, idx3);
			_triangles.push_back(tri);
			
			// ----- set edge data & edge to triangle
			// edge1
			InedexEdge e1(idx1, idx2);
			e1._dist = glm::distance(_resampledContour[e1._index1], _resampledContour[e1._index2]);
			int id = findEdge(e1, _edgeList);
			if (id < 0)
			{
				_edgeList.push_back(e1);
				std::vector<int> inds;
				inds.push_back(triID);
				_edgeToTri.push_back(inds);
			}
			else
			{
				_edgeToTri[id].push_back(triID);
			}

			// edge2
			InedexEdge e2(idx2, idx3);
			e2._dist = glm::distance(_resampledContour[e2._index1], _resampledContour[e2._index2]);
			id = findEdge(e2, _edgeList);
			if (id < 0)
			{
				_edgeList.push_back(e2);
				std::vector<int> inds;
				inds.push_back(triID);
				_edgeToTri.push_back(inds);
			}
			else
			{
				_edgeToTri[id].push_back(triID);
			}

			// edge3
			InedexEdge e3(idx3, idx1);
			e3._dist = glm::distance(_resampledContour[e3._index1], _resampledContour[e3._index2]);
			id = findEdge(e3, _edgeList);
			if (id < 0)
			{
				_edgeList.push_back(e3);
				std::vector<int> inds;
				inds.push_back(triID);
				_edgeToTri.push_back(inds);
			}
			else
			{
				_edgeToTri[id].push_back(triID);
			}

			// triagnle id
			triID++;
		}
		else
		{
			std::cout << "flip !" << std::endl;
			ATriangle tri(idx3, idx2, idx1);
			_triangles.push_back(tri);
		}


		/*cv::line(obimg, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), cv::Scalar(255, 0, 0, 255), 3);
		cv::line(obimg, cv::Point(pt2.x, pt2.y), cv::Point(pt3.x, pt3.y), cv::Scalar(255, 0, 0, 255), 3);
		cv::line(obimg, cv::Point(pt3.x, pt3.y), cv::Point(pt1.x, pt1.y), cv::Scalar(255, 0, 0, 255), 3);*/
		
		cv::line(tmpImg, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), cv::Scalar(255, 255, 255), 3);
		cv::line(tmpImg, cv::Point(pt2.x, pt2.y), cv::Point(pt3.x, pt3.y), cv::Scalar(255, 255, 255), 3);
		cv::line(tmpImg, cv::Point(pt3.x, pt3.y), cv::Point(pt1.x, pt1.y), cv::Scalar(255, 255, 255), 3);
		cv::imshow("1", tmpImg);
		cv::waitKey(10);
	}
	cv::waitKey(0);
	
	/*cv::addWeighted(img, 0.5, obimg, 0.5, 0.0, obimg);

	cv::imshow("obj", obimg);
	cv::waitKey(0);*/

	// ------delete short negspace edge
	float avgEdgeLength = 0;
	for (int i = 0; i < _edgeList.size(); ++i)
		avgEdgeLength += glm::distance(_resampledContour[_edgeList[i]._index1], _resampledContour[_edgeList[i]._index2]);
	avgEdgeLength /= (float)_edgeList.size();
	for (int i = _negSpaceEdges.size() - 1; i >= 0; i--)
	{
		glm::vec2 pt1 = _resampledContour[_negSpaceEdges[i]._index1];
		glm::vec2 pt2 = _resampledContour[_negSpaceEdges[i]._index2];

		if (glm::distance(pt1, pt2) < avgEdgeLength)	// can mutilply _self_intersection_threshold
			_negSpaceEdges.erase(_negSpaceEdges.begin() + i);
	}
	// draw
	for (int i = 0; i < _negSpaceEdges.size(); ++i)
	{
		cv::line(tmpImg, resampledContour[_negSpaceEdges[i]._index1], resampledContour[_negSpaceEdges[i]._index2], cv::Scalar(0, 0, 255), 2);
		cv::imshow("1", tmpImg);
		cv::waitKey(10);	
	}
	cv::waitKey(0);

	// --------- creat bending edged ----------
	for (int i = 0; i < _edgeToTri.size(); ++i)
	{
		if (_edgeToTri[i].size() != 2)
			continue;
		int idx1 = getUnsharedVertexIndex(_triangles[_edgeToTri[i][0]], _edgeList[i]);
		if (idx1 < 0)
			continue;
		int idx2 = getUnsharedVertexIndex(_triangles[_edgeToTri[i][1]], _edgeList[i]);
		if (idx2 < 0)
			continue;
		InedexEdge e(idx1, idx2);
		e._dist = glm::distance(_resampledContour[idx1], _resampledContour[idx2]);
		_auxiliaryEdge.push_back(e);
	}

	for (int i = 0; i < _auxiliaryEdge.size(); ++i)
	{
		cv::line(tmpImg, resampledContour[_auxiliaryEdge[i]._index1], resampledContour[_auxiliaryEdge[i]._index2], cv::Scalar(255, 0, 0), 2);
		cv::imshow("1", tmpImg);
		cv::waitKey(10);
	}
	cv::waitKey(0);

	// bounding box
	xMax = std::numeric_limits<float>::min();
	yMax = std::numeric_limits<float>::min();
	_xMin = std::numeric_limits<float>::max();
	_yMin = std::numeric_limits<float>::max();
	for (int i = 0; i < resampledContour.size(); ++i)
	{
		cv::Point2f p = resampledContour[i];
		if (p.x > xMax)
			xMax = p.x;
		if (p.y > yMax)
			yMax = p.y;
		if (p.x < _xMin)
			_xMin = p.x;
		if (p.y < _yMin)
			_yMin = p.y;
	}

	// --------- centroid(���߮y��) and center triangle id -----------
	// centroid
	std::vector<cv::Point2f> bound;
	float Xsum = 0, Ysum = 0;
	for (int i = 0; i < resampledContour.size(); ++i)
	{
		bound.push_back(resampledContour[i]);
	}
	cv::Moments mu = cv::moments(bound, false);
	_centroid.x = mu.m10 / mu.m00;
	_centroid.y = mu.m01 / mu.m00;
	// center triangle id
	getCentroidTri(img, _triangles, _resampledContour, _centroid, _centroidTriIdx);

	// ------ vector to centroid -------
	for (int i = 0; i < _skinPointNum; ++i)
	{
		_normFromCentroidArray.push_back(glm::normalize(_resampledContour[i] - _centroid));
		_rotateArray.push_back(glm::vec2(0, 0));
	}

	// ------------------------- tesselate art --------------------------
	UtilityFunctions::UniformResampleClosed(_smoothContour, _tesselateArt, 2.0f);

	CGALTriangulation cTri;

	cTri.Triangulate(_tesselateArt, _artTris);
	
	// check and set Gl data
	tmpImg = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

	/*cv::Mat obimg = cv::Mat::zeros(img.rows, img.cols, CV_8UC4);
	obimg.setTo(cv::Scalar(255, 255, 255, 0)); */
	for (int i = 0; i < _artTris.size(); ++i)
	{
		glm::vec2 pt1 = _tesselateArt[_artTris[i].idx1];
		glm::vec2 pt2 = _tesselateArt[_artTris[i].idx2];
		glm::vec2 pt3 = _tesselateArt[_artTris[i].idx3];
	
		// set vertice
		Vertex vt;
		vt.Position = glm::vec4(pt1, 0 ,1);
		vt.TexCoords = glm::vec2(pt1.x / (float)img.cols, pt1.y / (float)img.rows);
		_vertices.push_back(vt);

		vt.Position = glm::vec4(pt2, 0, 1);
		vt.TexCoords = glm::vec2(pt2.x / (float)img.cols, pt2.y / (float)img.rows);
		_vertices.push_back(vt);

		vt.Position = glm::vec4(pt3, 0, 1);
		vt.TexCoords = glm::vec2(pt3.x / (float)img.cols, pt3.y / (float)img.rows);
		_vertices.push_back(vt);


		// opencv drawing
		cv::Point2f p1 = cv::Point2f(pt1.x, pt1.y);
		cv::Point2f p2 = cv::Point2f(pt2.x, pt2.y);
		cv::Point2f p3 = cv::Point2f(pt3.x, pt3.y);

		cv::line(tmpImg, p1, p2, cv::Scalar(255), 1);
		cv::line(tmpImg, p2, p3, cv::Scalar(255), 1);
		cv::line(tmpImg, p3, p1, cv::Scalar(255), 1);
		/*cv::imshow("tesselateTri", tmpImg);
		cv::waitKey(1);*/
		/*cv::line(obimg, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), cv::Scalar(255, 0, 0, 255), 1);
		cv::line(obimg, cv::Point(pt2.x, pt2.y), cv::Point(pt3.x, pt3.y), cv::Scalar(255, 0, 0, 255), 1);
		cv::line(obimg, cv::Point(pt3.x, pt3.y), cv::Point(pt1.x, pt1.y), cv::Scalar(255, 0, 0, 255), 1);*/

	}
	/*cv::addWeighted(img, 0.5, obimg, 0.5, 0.0, obimg);

	cv::imshow("obj", obimg);
	cv::waitKey(0);*/

	cv::imshow("tesselateTri", tmpImg);
	cv::waitKey(0);
	

	// ------- barycentric(���߮y��) -------- for reconstruct complete object
	ConputeBarycentric(_smoothContour, _resampledContour, _triangles, _centroid, _centroidTriIdx,
		_baryCoord, _art2Triangles, _centroidBary, _tesselateArt, _tesselateBary, _tesselateArt2Triangles);

	// check bary coord
	tmpImg = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	for (int i = 0; i < _tesselateArt.size(); ++i)
	{
		ATriangle tri = _triangles[_tesselateArt2Triangles[i]];
		ABary bary = _tesselateBary[i];
		glm::vec2 pt = _resampledContour[tri.idx1] * bary._u +
			_resampledContour[tri.idx2] * bary._v +
			_resampledContour[tri.idx3] * bary._w;
	
		cv::circle(tmpImg, cv::Point2f(pt.x, pt.y), 1, cv::Scalar(255));
	}
	cv::imshow("tesselateCheck", tmpImg);
	cv::waitKey(0);

	// --------- openGL --------------
	glfwInit();
	// use openGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// create window
	GLFWwindow *window = glfwCreateWindow(img.cols, img.rows, "openGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Fail to create window" << std::endl;
		glfwTerminate();
		exit(0);
	}
	glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		exit(0);
	}

	Shader shader("shader//vertShader.glsl", "shader//fragShader.glsl");
	unsigned int vao, vbo;
	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);
	
	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, _vertices.size() * sizeof(_vertices[0]), _vertices.data(), GL_STATIC_DRAW);

	// position
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// uv
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));

	glBindVertexArray(0);

	// load texture
	unsigned int texture;
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	// set the texture wrapping parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	// set texture filtering parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	if (img.elemSize() > 0)
	{
		cv::Mat glimg;
		cv::cvtColor(img, glimg, cv::COLOR_BGRA2RGBA);
		// cv::flip(glimg, glimg, 0);
		/*cv::imshow("1", glimg);
		cv::waitKey(0);*/
		//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.cols, img.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, glimg.data);
		//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glGenerateMipmap(GL_TEXTURE_2D);
	}


	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	bool hasTexture = true;
	glm::mat4 proj = glm::mat4(1.0f);
	proj = glm::ortho(0.0f, (float)img.cols, (float)img.rows, 0.0f, -1.0f, 1.0f);
	while (!glfwWindowShouldClose(window))
	{
		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(window, true);
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		shader.use();

		shader.setMat4("proj", proj);
		shader.setBool("hasTexture", hasTexture);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texture);
		glUniform1i(glGetUniformLocation(shader.ID, "ourtexture"), 0);

		glBindVertexArray(vao);
		glDrawArrays(GL_TRIANGLES, 0, _vertices.size());

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &vbo);
	
	glfwTerminate();


	auto save = json
	{
		{
			"imgPath",
			input
		},
		{
			"imgWidth",
			img.cols
		},
		{
			"imgHeight",
			img.rows
		},
		{
			"smoothContour",
			json::array()	// add below
		},
		{
			"resampledComtour",
			json::array()
		},
		{
			"edgeList",
			json::array()
		},
		{
			"negSpaceEdges",
			json::array()
		},
		{
			"auxiliaryEdges",
			json::array()
		},
		{
			"edgeToTri",
			_edgeToTri
		},
		{
			"triangles",
			json::array()
		},
		{
			"skinPointNum",
			_skinPointNum
		},
		{
			"contourLength",
			_contourLength
		},
		{
			"centroid",
			{_centroid.x, _centroid.y}
		},
		{
			"centroidTriIdx",
			_centroidTriIdx
		},
		{
			"centroidBary",
			{_centroidBary._u, _centroidBary._v, _centroidBary._w}
		},
		{
			"baryCoord",
			json::array()
		},
		{
			"art2Triangles",
			_art2Triangles
		},
		{
			"tesselateArt",
			json::array()
		},
		{
			"tesselateArt2Triangles",
			_tesselateArt2Triangles
		},
		{
			"tesselateBary",
			json::array()
		},
		{
			"artTris",
			json::array()
		},
		{
			"vertices",
			json::array()
		},
		{
			"boundingBox",
			{_xMin, _yMin, xMax - _xMin, yMax - _yMin}		// top-down point and width, height
		},
		{
			"normFromCentroidArray",
			json::array()
		},
	};

	// smoothContour
	for (int i = 0; i < _smoothContour.size(); ++i)
	{
		auto &pt = save["smoothContour"];
		pt.push_back({ _smoothContour[i].x, _smoothContour[i].y });
	}

	// resampledComtour
	for (int i = 0; i < _resampledContour.size(); ++i)
	{
		auto &pt = save["resampledComtour"];
		pt.push_back({ _resampledContour[i].x, _resampledContour[i].y });
	}

	// edgeList
	for (int i = 0; i < _edgeList.size(); ++i)
	{
		auto &e = save["edgeList"];
		e.push_back({ std::to_string(_edgeList[i]._index1), std::to_string(_edgeList[i]._index2), std::to_string(_edgeList[i]._dist) });
	}

	// negSpaceEdges
	for (int i = 0; i < _negSpaceEdges.size(); ++i)
	{
		auto &e = save["negSpaceEdges"];
		e.push_back({ std::to_string(_negSpaceEdges[i]._index1), std::to_string(_negSpaceEdges[i]._index2), std::to_string(_negSpaceEdges[i]._dist) });
	}

	// auxiliaryEdges
	for (int i = 0; i < _auxiliaryEdge.size(); ++i)
	{
		auto &e = save["auxiliaryEdges"];
		e.push_back({ std::to_string(_auxiliaryEdge[i]._index1), std::to_string(_auxiliaryEdge[i]._index2), std::to_string(_auxiliaryEdge[i]._dist) });
	}

	// triangles
	for (int i = 0; i < _triangles.size(); ++i)
	{
		auto &t = save["triangles"];
		t.push_back({ _triangles[i].idx1, _triangles[i].idx2, _triangles[i].idx3 });
	}

	// BaryCoord
	for (int i = 0; i < _baryCoord.size(); ++i)
	{
		auto &b = save["baryCoord"];
		b.push_back({ _baryCoord[i]._u, _baryCoord[i]._v, _baryCoord[i]._w });
	}

	// tesselateArt
	for (int i = 0; i < _tesselateArt.size(); ++i)
	{
		auto &pt = save["tesselateArt"];
		pt.push_back({ _tesselateArt[i].x, _tesselateArt[i].y });
	}

	// tesselateBary
	for (int i = 0; i < _tesselateBary.size(); ++i)
	{
		auto &b = save["tesselateBary"];
		b.push_back({ _tesselateBary[i]._u, _tesselateBary[i]._v, _tesselateBary[i]._w });
	}

	// artTris
	for (int i = 0; i < _artTris.size(); ++i)
	{
		auto &t = save["artTris"];
		t.push_back({ _artTris[i].idx1, _artTris[i].idx2, _artTris[i].idx3 });
	}

	// vertices
	for (int i = 0; i < _vertices.size(); ++i)
	{
		auto &ver = save["vertices"];
		ver.push_back({ _vertices[i].Position.x, _vertices[i].Position.y, _vertices[i].TexCoords.x, _vertices[i].TexCoords.y });
	}

	// normFromCentroidArray
	for (int i = 0; i < _normFromCentroidArray.size(); ++i)
	{
		auto &pt = save["normFromCentroidArray"];
		pt.push_back({ _normFromCentroidArray[i].x, _normFromCentroidArray[i].y });
	}

	auto out = std::ofstream(dir + "json\\"  + file + ".json");
	out << save;
	out.close();
	return 0;
}