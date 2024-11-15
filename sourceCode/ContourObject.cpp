#include "ContourObject.h"

#include <fstream>
#include <regex>

#include "nlohmann/json.hpp"

#include "SystemParams.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using json = nlohmann::json;

ContourObject::ContourObject()
{
}

ContourObject::ContourObject(const std::string& jsonFile)
	: _jsonFile(jsonFile), _imgWidth(0), _imgHeight(0),  _contourArea(0.0), _contourLength(0.0),\
	_vao(0), _vbo(0), _centroid(0, 0)
{
	_vertices.clear();
	_smoothContour.clear();
	_resampledContour.clear();
	_countMap.clear();
	_rowsWidth.clear();
	LoadJsonData();
}

void ContourObject::LoadJsonData()
{
	std::ifstream input(_jsonFile);
	json file;
//	std::cout << _jsonFile << std::endl;

	input >> file;
	input.close();

	_imgPath = file.at("img_path").get<std::string>();
	_imgHeight = file.at("img_height").get<unsigned int>();
	_imgWidth = file.at("img_width").get<unsigned int>();

	ASSERT(_imgHeight != 0 && _imgWidth != 0);

	_contourArea = file.at("contour_area").get<float>();
	_contourLength = file.at("contour_length").get<float>();

	ASSERT(_contourArea != 0 && _contourLength != 0);

	_rowsWidth = file.at("width_of_each_row").get<std::vector<int>>();

	ASSERT(_rowsWidth.size() != 0);

	for (unsigned int i = 0; i < file.at("contour").size(); ++i)
	{
		std::vector<float> pt = file.at("contour")[i].get<std::vector<float>>();
		_smoothContour.push_back(glm::vec2(pt[0], pt[1]));
		//_vertices.push_back(glm::vec4(pt[0], pt[1], 0.0, 1.0));
	}

	std::vector<cv::Point2f> bound;	// for calculate centroid.
	for (unsigned int i = 0; i < file.at("resampled_contour").size(); ++i)
	{
		std::vector<float> pt = file.at("resampled_contour")[i].get<std::vector<float>>();
		_resampledContour.push_back(glm::vec2(pt[0], pt[1]));
		_vertices.push_back(glm::vec4(pt[0], pt[1], 0.0, 1.0));
		
		bound.push_back(cv::Point2f(pt[0], pt[1]));
	}
	cv::Moments mu = cv::moments(bound, false);
	_centroid.x = mu.m10 / mu.m00;
	_centroid.y = mu.m01 / mu.m00;


	_countMap = file.at("count_map").get<std::vector<std::vector<int>>>();

	
	std::ifstream skFile(SystemParams::_skeleton_path);
	if (skFile.is_open())
	{
		std::string line;
		std::regex numberRegex("\\d+");
		while (std::getline(skFile, line) && !line.empty())
		{
			// id (pos.x pos.y) 
			std::sregex_iterator iter(line.begin(), line.end(), numberRegex);
			std::sregex_iterator end;
			std::vector<int> nums;
			while (iter != end)
			{
				std::smatch match = *iter;
				nums.push_back(std::stoi(match.str()));
				++iter;
			}
			glm::vec2 pt = glm::vec2(nums[0], nums[1]);
			_skt2Num[std::make_pair(pt.x, pt.y)] = nums[2];
			_skeleton.push_back(pt);
		}
		skFile.close();
	}
	else
	{
		std::cout << "No skeleton" << std::endl;
	}

}

// find the contourX(the length of each row of object) of the object will be placed in the contourObject
void ContourObject::SetContour(std::vector<std::vector<int>>& contourX, const cv::Mat & img)
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

void ContourObject::GetContourInfo(const std::vector<std::vector<int>>& contourX, std::vector<int>& contourLength,\
	std::vector<int>& contourLengthShort, std::vector<int>& contourLengthDis, int &longest, int width)
{
	unsigned int n = contourX.size();

	std::vector<int> sorted(n, 0);

	for (unsigned int i = 0; i < n; ++i)
	{
		if (contourX[i].size() > 0)
		{
			contourLengthShort[i]  = contourX[i][contourX[i].size() - 1] - contourX[i][0];
			contourLengthDis[i]    = width - contourX[i][contourX[i].size() - 1];
			contourLength[i]	   = width - contourX[i][0];
			
			sorted[i] = contourLength[i];

			if (contourLengthShort[i] + contourLengthDis[i] != contourLength[i])	// error!! 
			{
				std::cout << "there is error on object contour !!" << std::endl;
				assert(-1);
			}
		}
		else
		{
			contourLengthShort[i]	= 0;
			contourLengthDis[i]		= 0;
			contourLength[i]		= 0;

			sorted[i] = 0;
		}
	}

	std::sort(sorted.begin(), sorted.end());
	longest = sorted[n - 1];

}

void ContourObject::BindGLdata()
{
	ASSERT(_vertices.size() != 0);

	GLCall(glGenVertexArrays(1, &_vao));
	GLCall(glGenBuffers(1, &_vbo));

	GLCall(glBindVertexArray(_vao));

	GLCall(glBindBuffer(GL_ARRAY_BUFFER, _vbo));
	GLCall(glBufferData(GL_ARRAY_BUFFER, _vertices.size() * sizeof(_vertices[0]), _vertices.data(), GL_STATIC_DRAW));

	GLCall(glEnableVertexAttribArray(0));
	GLCall(glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(_vertices[0]), (void*)0));

	// unbind vao
	GLCall(glBindVertexArray(0));
}

void ContourObject::Draw(const Shader & shader)
{

	shader.setBool("hasTexture", false);
	// bind vao
	GLCall(glBindVertexArray(_vao));
	GLCall(glDrawArrays(GL_POINTS, 0, _vertices.size()));
	// unbind vao
	GLCall(glBindVertexArray(0));
}


// find position placing the bounding box by count map
bool ContourObject::Placement(const ARectangle &boundingBox, glm::vec2& pos)
{
	float bbWidth  = boundingBox._width;
	float bbHeight = boundingBox._height;
	glm::vec2 origin = boundingBox._topleft;
	
	// the target
	glm::vec2 placementPt;

	unsigned int Ystart = 0, Yend = 0;
	while (Yend < _imgHeight)
	{
		if (_rowsWidth[Yend] == 0)
		{
			Ystart = ++Yend;
			continue;
		}
		while (Yend < _imgHeight && _rowsWidth[Yend] >= bbWidth)
		{
			
			Yend++;
		}
		if (Yend - Ystart >= bbHeight)
		{
			for (unsigned int row = Ystart; row + bbHeight < Yend; ++row)
			{
				std::vector<int> Xscanner = std::vector<int>(_imgWidth, 0);

				for (unsigned int y = row; y - row < bbHeight; ++y)
				{
					for (unsigned int x = bbWidth; x < _imgWidth; ++x)	// each point in count map record the left usable spaces of the point.
					{
						if (_countMap[x][y] < bbWidth)
							Xscanner[x] = 0;
						else
						{
							Xscanner[x]++;
							if (Xscanner[x] >= bbHeight)
							{
								// find the down-left point(in glm coordinate)
								pos.x = x - bbWidth + 1;
								pos.y = y - bbHeight;	// convert to glm coordinate
								UpdateCountMap(boundingBox, x - bbWidth, y);
								return true;
							}
						}
					}
				}
			}
		}
		Ystart = ++Yend;
	}
	
	return false;
}


void ContourObject::UpdateCountMap(const ARectangle &boundingBox, int xPlace, int yPlace)
{
	int count = 0;
	for (unsigned int y = yPlace - boundingBox._height; y <= yPlace; ++y)
	{
		int newwidth = 0;
		for (unsigned int x = xPlace + 1; x < _imgWidth; ++x)
		{
			if (_countMap[x][y] == 0)
			{
				count = 0;
				continue;
			}
			else
			{
				if (x <= xPlace + boundingBox._width)	// in bounding box
				{
					_countMap[x][y] = 0;
					count = 0;
				}
				else
				{
					count++;
					_countMap[x][y] = count;
					if (newwidth < count)
						newwidth = count;
				}
			}
			
		}
		_rowsWidth[y] = newwidth;
	}
}

// with skeleton.
bool ContourObject::Placement3(const cv::Mat &img, glm::vec2 & pos, const glm::vec2& prevPos)
{
	std::vector<std::vector<int>> contourX;
	SetContour(contourX, img);

	/*cv::Mat bim = cv::Mat::zeros(img.rows + 10, img.cols + 10, CV_8UC1);
	for (int i = 0; i < contourX.size(); ++i)
	{
		for (int j = 0; j < contourX[i].size(); ++j)
		{
			cv::circle(bim, cv::Point(contourX[i][j] + 3, i + 3), 0.001, cv::Scalar(255));
		}
	}
	cv::imshow("_contourX", bim);
	cv::waitKey(0);*/


	// dis. of most right point of bbox to the left point of the contour
	std::vector<int> contourLength(contourX.size(), 0);

	// dis. of the contour len of one row of the contour.
	std::vector<int> contourLengthShort(contourX.size(), 0);

	// dis. of most right point of bbox to the most right point of the contour
	std::vector<int> contourLengthDis(contourX.size(), 0);

	
	int bboxWidth = img.cols;
	int bboxHeight = img.rows;

	// find the best coordinate.
	int X = -1, Y = -1;
	int tmpX, tmpY;
	int len = INT_MAX;

	int longest;
	GetContourInfo(contourX, contourLength, contourLengthShort, contourLengthDis, longest, bboxWidth);

	unsigned int Ystart = 0, Yend = 0;


	std::vector<glm::vec2> candidatePos;

	while (Yend < _imgHeight)
	{
		/*if (_rowsWidth[Yend] == 0 || _rowsWidth[Yend] < longest)
		{
			Ystart = ++Yend;
			continue;
		}*/
		while (Yend < _imgHeight && (_rowsWidth[Yend] >= longest))
			Yend++;
		if (Yend - Ystart >= contourX.size())	// the height of Yend - Ystart can place object
		{
			// find the position to place
			for (unsigned int row = Ystart; row + contourX.size() < Yend; ++row)
			{
				std::vector<int> Xscanner = std::vector<int>(_imgWidth, 0);	// scan all column elements along the row

				for (unsigned int y = row, contourIdx = 0; y - row < contourX.size(); ++y, ++contourIdx)
				{
					for (unsigned int x = bboxWidth; x < _imgWidth; ++x)	// each point in count map record the left usable spaces of the point.
					{
						if (_countMap[x - contourLengthDis[contourIdx]][y] < contourLengthShort[contourIdx])
							Xscanner[x] = 0;
						else
						{
							Xscanner[x]++;
							if (Xscanner[x] >= contourX.size())
							{
								// find the top-left point(in glm coordinate)
								// center of the image.
								tmpX = x - bboxWidth + 1;
								tmpY = y - bboxHeight;	// convert to glm coordinate

								candidatePos.push_back(glm::vec2(tmpX, tmpY));

							}
							
						}
					}
				}
			}
		}
		Ystart = ++Yend;
	}


	if (candidatePos.size() == 0)
	{
		return false;
	}
	else
	{
		float cVal = 0.0f, preVal = 0.0f;
		float score = 0;
		glm::vec2 place;
		for (unsigned int i = 0; i < candidatePos.size(); ++i)
		{
			cVal = 0.0f, preVal = 0.0f;
			float minD = 800;
			glm::vec2 skPos;
			glm::vec2 objCenter = candidatePos[i] + glm::vec2(bboxWidth / 2, bboxHeight / 2);
			// find the closest point on skeleton.
			for (unsigned int j = 0; j < _skeleton.size(); ++j)
			{
				float d = glm::distance(objCenter, _skeleton[j]);
				if (d < minD)
				{
					minD = d;
					skPos = _skeleton[j];
				}				
			}
			cVal = _skt2Num[std::make_pair(skPos.x, skPos.y)] / (minD + 10);
			if (prevPos.x != -1)
				preVal = _contourLength / (glm::distance(prevPos, objCenter) + 10);
			if (cVal + preVal > score)
			{
				score = cVal + preVal;
				place = candidatePos[i];
			}
		}
		UpdateCountMap2(contourX, place.x, place.y);
		pos = place;
		return true;
	}
}


bool ContourObject::Placement2(const cv::Mat &img, glm::vec2 & pos, const glm::vec2& prevPos)
{
	std::vector<std::vector<int>> contourX;
	SetContour(contourX, img);


	// dis. of most right point of bbox to the left point of the contour
	std::vector<int> contourLength(contourX.size(), 0);

	// dis. of the contour len of one row of the contour.
	std::vector<int> contourLengthShort(contourX.size(), 0);

	// dis. of most right point of bbox to the most right point of the contour
	std::vector<int> contourLengthDis(contourX.size(), 0);


	int bboxWidth = img.cols;
	int bboxHeight = img.rows;

	// find the best coordinate.
	int X = -1, Y = -1;
	int tmpX, tmpY;
	int len = INT_MAX;

	int longest;
	GetContourInfo(contourX, contourLength, contourLengthShort, contourLengthDis, longest, bboxWidth);

	unsigned int Ystart = 0, Yend = 0;

	/*std::cout << "x : " << _centroid.x << " y : " << _centroid.y << std::endl;
	_centroid = glm::vec2(306.0, 237.0);*/

	while (Yend < _imgHeight)
	{
		/*if (_rowsWidth[Yend] == 0 || _rowsWidth[Yend] < longest)
		{
			Ystart = ++Yend;
			continue;
		}*/
		while (Yend < _imgHeight && (_rowsWidth[Yend] >= longest))
			Yend++;
		if (Yend - Ystart >= contourX.size())	// the height of Yend - Ystart can place object
		{
			// find the position to place
			for (unsigned int row = Ystart; row + contourX.size() < Yend; ++row)
			{
				std::vector<int> Xscanner = std::vector<int>(_imgWidth, 0);	// scan all column elements along the row
				
				for (unsigned int y = row, contourIdx = 0; y - row < contourX.size(); ++y, ++contourIdx)
				{
					for (unsigned int x = bboxWidth; x < _imgWidth; ++x)	// each point in count map record the left usable spaces of the point.
					{
						if (_countMap[x - contourLengthDis[contourIdx]][y] < contourLengthShort[contourIdx])
							Xscanner[x] = 0;
						else
						{
							Xscanner[x]++;
							if (Xscanner[x] >= contourX.size())
							{
								// find the top-left point(in glm coordinate)
								// center of the image.
								tmpX = x - bboxWidth + 1;
								tmpY = y - bboxHeight;	// convert to glm coordinate
								
								// compare to center.
								// a*a faster than std::pow(a,2)
								float dis_C = (tmpX + (bboxWidth / 2) - _centroid.x) * (tmpX + (bboxWidth / 2) - _centroid.x) +
									(tmpY + (bboxHeight / 2) - _centroid.y) * (tmpY + (bboxHeight / 2) - _centroid.y);
								float dis_P = 0.0f;
								if(prevPos.x != -1)
									dis_P = (tmpX + (bboxWidth / 2) - prevPos.x) * (tmpX + (bboxWidth / 2) - prevPos.x) +
									(tmpY + (bboxHeight / 2) - prevPos.y) * (tmpY + (bboxHeight / 2) - prevPos.y);
								if (dis_C + dis_P < len)
								{
									len = dis_C + dis_P;
									X = tmpX;
									Y = tmpY;
								}
							}
						}
					}
				}
			}
		}
		Ystart = ++Yend;
	}

	if (X != -1)
	{
		pos.x = X;
		pos.y = Y;
		UpdateCountMap2(contourX, X, Y);
		return true;
	}
	else
		return false;

}



void ContourObject::UpdateCountMap2(const std::vector<std::vector<int>>& contourX, int xPlace, int yPlace)
{
	
	/*cv::Mat bim = cv::Mat::zeros(150, 150, CV_8UC1);
	for (int i = 0; i < contourX.size(); ++i)
	{
		for (int j = 0; j < contourX[i].size(); ++j)
		{
			cv::circle(bim, cv::Point(contourX[i][j] + 3, i + 3), 0.001, cv::Scalar(255));
		}
	}
	cv::imshow("_contourX", bim);
	cv::waitKey(0);*/


	// std::vector<std::vector<int>> cp = _countMap;
	int count = 0, pt1 = -1, pt2 = -1;
	bool ptStart = false, ptEnd = false;

	// check the rows of the area placed the object.
	for (unsigned int y = yPlace, contourIdx = 0; y < yPlace + contourX.size(); ++y, ++contourIdx)
	{
		ptStart = false, ptEnd = false;
		pt1 = -1, pt2 = -1;
		count = _countMap[xPlace][y];
		unsigned int x = xPlace;
		
		unsigned int newWidth = 0;
		// for the points of contourX[idx] & update the _rowsWidth
		for (unsigned int i = 0; i < contourX[contourIdx].size(); ++i)
		{
			if (!ptStart && !ptEnd)	// the first point.
			{
				pt1 = contourX[contourIdx][i] + xPlace;
				ptStart = true;
				continue;
			}
			if (ptStart && !ptEnd)	// the second point.
			{
				ptStart = false;
				pt2 = contourX[contourIdx][i] + xPlace;
				//std::cout << "row " << contourIdx << " : " << pt1 << " " << pt2 << std::endl;
				// now have the contour range along x-axis
				assert(pt1 != -1 && pt2 != -1 && pt1 < pt2);
				for (x; x < pt2; ++x)
				{
					if (_countMap[x][y] == 0)
					{
						count = 0;
					}
					else
					{
						if ((x >= pt1) && (x <= pt2))
						{
							_countMap[x][y] = 0;
							count = 0;
						}
						else
						{
							count++;
							_countMap[x][y] = count;
							if (newWidth < count)
								newWidth = count;
						}
					}
				}
			}
		}

		for (x; x < _imgWidth; x++)
		{
			if (_countMap[x][y] == 0)
			{
				count = 0;
			}
			else
			{
				count++;
				_countMap[x][y] = count;
				if (newWidth < count)
					newWidth = count;
			}
		}

		// update width of row
		//std::cout << newWidth << std::endl;
		_rowsWidth[y] = newWidth;
	}


	/*cv::Mat tmp = cv::Mat::zeros(cv::Size(500, 500), CV_8UC1);
	for (int y = 0; y < 500; ++y)
		for (int x = 0; x < 500; ++x)
		{
			if (_countMap[x][y] != 0)
			{
				int px = (int)_countMap[x][y];
				if (px > 255)
					px = 255;
				tmp.at<uchar>(y, x) = px;
			}
		}
	cv::imshow("1", tmp);
	cv::waitKey(0);*/
}

void ContourObject::Copy(ContourObject & other)
{
	_smoothContour.clear();
	_smoothContour = other._smoothContour;
	/*_countMap.clear();
	_rowsWidth.clear();*/
	_imgWidth = other.GetWidth();
	_imgHeight = other.GetHeight();
	_contourArea = other.GetArea();
	_contourLength = other.GetLength();
}
