#include <fstream>
#include <string>
#include "Object.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "SystemParams.h"
#include "UtilityFunctions.h"
#include "StuffWorker.h"



Object::Object()
{
}

Object::Object(const char * jsonFile)
	: _jsonFile(jsonFile), _id(-1), _isGrowing(true), _isBig(false), _isSmall(false), \
	_isShrink(false), _isFocus(false), _aroundFocus(false), _imgHeight(0), _imgWidth(0), _contourLength(0), \
	_vao(0), _vbo(0), _textureID(0), _skinPointNumber(0), _centroidTriIdx(-1), \
	_avgEdgeLength(0), _oriAvgEdgeLength(0), _scale(0), _sumVelocity(0), _speedUp(false),\
	_targetPosition(-1, -1), _lastPosition(-1, -1), _targetSize(0), _preparedSize(0), _prevPos(-1, -1),\
	_completeShift(false), _completeResize(false), _stuck(false), _showUp(false), _fadeIn(false), _fadeOut(false), _extremeDiff(false)
{
	
	_bbxCenter = glm::vec2(-1, -1);
	_bbox = ARectangle();
	_centroid = glm::vec2(-1, -1);

	LoadFile(jsonFile);
}

Object::~Object()
{
}

// preprocecc the mass data. 
void Object::MassData()
{
	for (int i = 0; i < _resampledContour.size(); ++i)
	{
		glm::vec2 pos = _resampledContour[i];
		AMass aM(pos);
		aM._idx = i;
		_massList.push_back(aM);
	}
	for (int i = 0; i < _resampledContour.size(); ++i)
	{
		_massList[i].CalculateIndicesOfLineSegment(_skinPointNumber);
	}

	// assign triangles to masses
	for (unsigned int i = 0; i < _triangles.size(); ++i)
	{
		int idx1 = _triangles[i].idx1;
		int idx2 = _triangles[i].idx2;
		int idx3 = _triangles[i].idx3;

		_massList[idx1]._triangles.push_back(_triangles[i]);
		_massList[idx2]._triangles.push_back(_triangles[i]);
		_massList[idx3]._triangles.push_back(_triangles[i]);
	}

	// edge of the mass
	for (unsigned int i = 0; i < _edgeList.size(); ++i)
	{
		int idx1 = _edgeList[i]._index1;
		int idx2 = _edgeList[i]._index2;

		_massList[idx1].TryToAddTriangleEdge(_edgeList[i], _massList);
		_massList[idx2].TryToAddTriangleEdge(_edgeList[i], _massList);
	}
}

void Object::GetContourMap()
{
	_contourMap = cv::Mat::zeros(_imgHeight, _imgWidth, CV_8UC1);
	std::vector< std::vector<cv::Point>> cvContour(1, std::vector<cv::Point>(_smoothContour.size()));
	int idx = 0;
	for (auto pt : _smoothContour)
	{
		cvContour[0][idx++] = cv::Point(pt.x, pt.y);
	}
	cv::fillPoly(_contourMap, cvContour, cv::Scalar(255));
	/*cv::imshow("2", _contourMap);
	cv::waitKey(0);*/
}


void Object::LoadFile(const char *string)
{
	std::ifstream input(string);
	json file;
	input >> file;
	input.close();

	_imgPath = file.at("imgPath").get<std::string>();
	_imgWidth = file.at("imgWidth").get<unsigned int>();
	_imgHeight = file.at("imgHeight").get<unsigned int>();
	
	// bounding box
	std::vector<float> bb = file.at("boundingBox").get<std::vector<float>>();
	_bbox = ARectangle(glm::vec2(bb[0], bb[1]), bb[2], bb[3]);
	_initBbox = ARectangle(glm::vec2(bb[0], bb[1]), bb[2], bb[3]);
	_bbxCenter = _bbox.GetCenter();

	// ---------- centroid ------------
	std::vector<float> centroid = file.at("centroid").get<std::vector<float>>();
	//_centroid = glm::vec2(centroid[0], centroid[1]) - _bbox.topleft;
	_centroid = glm::vec2(centroid[0], centroid[1]);


	_centroidTriIdx = file.at("centroidTriIdx").get<int>();
	
	std::vector<float> cb = file.at("centroidBary").get<std::vector<float>>();
	_centroidBary._u = cb[0], _centroidBary._v = cb[1], _centroidBary._w = cb[2];

	_skinPointNumber = file.at("skinPointNum").get<unsigned int>();
	_contourLength = file.at("contourLength").get<float>();

	_edgeToTri = file.at("edgeToTri").get<std::vector<std::vector<int>>>();

	// --------------------------- contour --------------------------
	for (unsigned int i = 0; i < file.at("smoothContour").size(); ++i)
	{
		std::vector<float> pos = file.at("smoothContour")[i].get<std::vector<float>>();
		//_smoothContour.push_back(glm::vec2(pos[0], pos[1]) - _bbox._topleft);
		_smoothContour.push_back(glm::vec2(pos[0], pos[1]));
	}
	for (unsigned int i = 0; i < file.at("resampledComtour").size(); ++i)
	{
		std::vector<float> pos = file.at("resampledComtour")[i].get<std::vector<float>>();
		//_resampledContour.push_back(glm::vec2(pos[0], pos[1]) - _bbox._topleft);
		_resampledContour.push_back(glm::vec2(pos[0], pos[1]));
	}
	ASSERT(_smoothContour.size() != 0 && _resampledContour.size() != 0);

	// ----------------------------edge -----------------------------
	for (unsigned int i = 0; i < file.at("edgeList").size(); ++i)
	{
		std::vector<std::string> edgeInfo = file.at("edgeList")[i].get<std::vector<std::string>>();
		InedexEdge e(std::stoi(edgeInfo[0]), std::stoi(edgeInfo[1]), std::stof(edgeInfo[2]));

		_edgeList.push_back(e);
	}
	ASSERT(_edgeList.size() != 0);
	
	// check edge list 
	/*cv::Mat b_img = cv::Mat::zeros(_imgHeight, _imgWidth, CV_8UC3);
	for (int i = 0; i < _edgeList.size(); ++i)
	{
		cv::Point2f p1(_resampledContour[_edgeList[i]._index1].x, _resampledContour[_edgeList[i]._index1].y);
		cv::Point2f p2(_resampledContour[_edgeList[i]._index2].x, _resampledContour[_edgeList[i]._index2].y);
		cv::line(b_img, p1, p2, cv::Scalar(255, 255, 255), 2);
		cv::imshow("!", b_img);
		cv::waitKey(10);
	}
	cv::waitKey(0);*/

	for (unsigned int i = 0; i < _skinPointNumber - 1; ++i)
	{
		float dist = glm::distance(_resampledContour[i], _resampledContour[i + 1]);
		InedexEdge e(i, i + 1, dist);
		_skinEdge.push_back(e);
	}
	float dist = glm::distance(_resampledContour[_skinPointNumber - 1], _resampledContour[0]);
	InedexEdge e(_skinPointNumber - 1, 0, dist);
	_skinEdge.push_back(e);

	ASSERT(_edgeList.size() != 0);

	for (unsigned int i = 0; i < file.at("negSpaceEdges").size(); ++i)
	{
		std::vector<std::string> edgeInfo = file.at("negSpaceEdges")[i].get<std::vector<std::string>>();
		InedexEdge e(std::stoi(edgeInfo[0]), std::stoi(edgeInfo[1]), std::stof(edgeInfo[2]));
		_negEdge.push_back(e);
	}
	//ASSERT(_negEdge.size() != 0);

	// check neg space edge
	/*for (int i = 0; i < _negEdge.size(); ++i)
	{
		cv::Point2f p1(_resampledContour[_negEdge[i]._index1].x, _resampledContour[_negEdge[i]._index1].y);
		cv::Point2f p2(_resampledContour[_negEdge[i]._index2].x, _resampledContour[_negEdge[i]._index2].y);
		cv::line(b_img, p1, p2, cv::Scalar(255, 0 ,0), 2);
		cv::imshow("!", b_img);
		cv::waitKey(10);
	}
	cv::waitKey(0);*/

	for (unsigned int i = 0; i < file.at("auxiliaryEdges").size(); ++i)
	{
		std::vector<std::string> edgeInfo = file.at("auxiliaryEdges")[i].get<std::vector<std::string>>();
		InedexEdge e(std::stoi(edgeInfo[0]), std::stoi(edgeInfo[1]), std::stof(edgeInfo[2]));

		_auxiliaryEdge.push_back(e);
	}
	ASSERT(_auxiliaryEdge.size() != 0);

	// check auxi space edge
	/*for (int i = 0; i < _auxiliaryEdge.size(); ++i)
	{
		cv::Point2f p1(_resampledContour[_auxiliaryEdge[i]._index1].x, _resampledContour[_auxiliaryEdge[i]._index1].y);
		cv::Point2f p2(_resampledContour[_auxiliaryEdge[i]._index2].x, _resampledContour[_auxiliaryEdge[i]._index2].y);
		cv::line(b_img, p1, p2, cv::Scalar(0, 0, 255), 2);
		cv::imshow("!", b_img);
		cv::waitKey(10);
	}
	cv::waitKey(0);*/


	// --------------------- triangle --------------------
	for (unsigned int i = 0; i < file.at("triangles").size(); ++i)
	{
		std::vector<unsigned int> indice = file.at("triangles")[i].get<std::vector<unsigned int>>();

		ATriangle f;
		f.idx1 = indice[0];
		f.idx2 = indice[1];
		f.idx3 = indice[2];

		_triangles.push_back(f);
	}
	ASSERT(_triangles.size() != 0);

	// ---------------------- bary ------------------------
	for (unsigned int i = 0; i < file.at("baryCoord").size(); ++i)
	{
		std::vector<float> bary = file.at("baryCoord")[i].get<std::vector<float>>();
		_baryCoord.push_back(ABary(bary[0], bary[1], bary[2]));
	}
	ASSERT(_baryCoord.size() != 0);
	_art2Triangles = file.at("art2Triangles").get<std::vector<int>>();

	// --------------------- tesselate -------------------
	for (unsigned int i = 0; i < file.at("tesselateArt").size(); ++i)
	{
		std::vector<float> pos = file.at("tesselateArt")[i].get<std::vector<float>>();
		_tesselateArt.push_back(glm::vec2(pos[0], pos[1]));
	}

	_tesselateArt2Triangles = file.at("tesselateArt2Triangles").get<std::vector<int>>();

	for (unsigned int i = 0; i < file.at("tesselateBary").size(); ++i)
	{
		std::vector<float> bary = file.at("tesselateBary")[i].get<std::vector<float>>();
		_tesselateBary.push_back(ABary(bary[0], bary[1], bary[2]));
	}

	for (unsigned int i = 0; i < file.at("artTris").size(); ++i)
	{
		std::vector<unsigned int> t = file.at("artTris")[i].get<std::vector<unsigned int>>();
		_artTris.push_back(ATriangle(t[0], t[1], t[2]));
	}
	ASSERT(_tesselateArt.size() != 0 && _tesselateBary.size() != 0 && _artTris.size() != 0);
	
	// -------------------- vertices ----------------------
	for (unsigned int i = 0; i < file.at("vertices").size(); ++i)
	{
		std::vector<float> pos = file.at("vertices")[i].get<std::vector<float>>();
		
		Vertex t;
		//t.Position = glm::vec4(pos[0], pos[1], 0.0f, 1.0f) - glm::vec4(_bbox.topleft.x, _bbox.topleft.y, 0.0, 0.0);
		t.Position = glm::vec4(pos[0], pos[1], 0.0f, 1.0f);
		t.TexCoords = glm::vec2(pos[2], pos[3]);
		_vertices.push_back(t);
	}
	ASSERT(_vertices.size() != 0);

	// ------------------ rotate vector -------------------
	for (unsigned int i = 0; i < file.at("normFromCentroidArray").size(); ++i)
	{
		std::vector<float> data = file.at("normFromCentroidArray")[i].get<std::vector<float>>();
		glm::vec2 norm(data[0], data[1]);
		_normFromCentroidArray.push_back(norm);
	}
	ASSERT(_normFromCentroidArray.size() == _skinPointNumber);

	for (unsigned int i = 0; i < _skinPointNumber; ++i)
	{
		_rotateArray.push_back(glm::vec2(0, 0));
	}

	_targetNorm.resize(_skinPointNumber);


	// preprocess data
	MassData();
	UpdateBoundaryAndAvgEdgeLength();
	//_bbox._topleft = glm::vec2(0, 0);
	GetContourMap();
}

ARectangle Object::GetBBx()
{
	float xMax = std::numeric_limits<float>::min();
	float yMax = std::numeric_limits<float>::min();
	float xMin = std::numeric_limits<float>::max();
	float yMin = std::numeric_limits<float>::max();
	for (unsigned int i = 0; i < _skin.size(); ++i)
	{
		glm::vec2 pt = _skin[i];

		if (pt.x > xMax) { xMax = pt.x; }
		if (pt.y > yMax) { yMax = pt.y; }
		if (pt.x < xMin) { xMin = pt.x; }
		if (pt.y < yMin) { yMin = pt.y; }
	}
	return ARectangle(glm::vec2(xMin, yMin), xMax - xMin, yMax - yMin);
}

void Object::BindTexture()
{
	GLCall( glGenTextures(1, &_textureID) );
	GLCall( glBindTexture(GL_TEXTURE_2D, _textureID) );
	GLCall( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT) );	// set texture wrapping to GL_REPEAT (default wrapping method)
	GLCall( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT) );
	// set texture filtering parameters
	GLCall( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST) );
	GLCall( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST) );
	
	// read png image.
	std::cout << _imgPath << std::endl;
	auto img = cv::imread(_imgPath, cv::IMREAD_UNCHANGED);
	cv::cvtColor(img, img, cv::COLOR_BGRA2RGBA);
	//cv::flip(img, img, 0);
	
	GLCall( glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.cols, img.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.data) );
	GLCall( glGenerateMipmap(GL_TEXTURE_2D) );
	GLCall( glBindTexture(GL_TEXTURE_2D, 0) );

}

void Object::BindGLData()
{
	ASSERT(_vertices.size() != 0);

	GLCall( glGenVertexArrays(1, &_vao) );
	GLCall( glGenBuffers(1, &_vbo) );

	GLCall( glBindVertexArray(_vao) );

	GLCall( glBindBuffer(GL_ARRAY_BUFFER, _vbo) );
	GLCall( glBufferData(GL_ARRAY_BUFFER, _vertices.size() * sizeof(_vertices[0]), _vertices.data(), GL_STATIC_DRAW) );

	GLCall( glEnableVertexAttribArray(0) );
	GLCall( glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0) );
	GLCall( glEnableVertexAttribArray(1) );
	GLCall( glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords)) );

	// unbind vao
	GLCall( glBindVertexArray(0) );
}


void Object::DrawGL(const Shader & shader)
{
	// active texture 0
	GLCall( glActiveTexture(GL_TEXTURE0) );
	GLCall( glBindTexture(GL_TEXTURE_2D, _textureID) );

	shader.setBool("hasTexture", true);
	shader.setInt("ourtexture", 0);
	// bind vao
	GLCall( glBindVertexArray(_vao) );
	GLCall( glDrawArrays(GL_TRIANGLES, 0, _vertices.size()));
	
	// unbind texture
	GLCall( glBindTexture(GL_TEXTURE_2D, 0) );
	
	// unbind vao
	GLCall( glBindVertexArray(0) );
}

void Object::Move(const glm::vec2& vector)
{
	UpdateBoundaryAndAvgEdgeLength();
	/*
	glm::vec2 centerMass = glm::vec2(0, 0);

	for (unsigned int i = 0; i < _vertexNumber; ++i)
		centerMass += _massList[i]._pos;
	centerMass /= (float)_vertexNumber;

	glm::vec2 offset = vector - centerMass;
	for (unsigned int i = 0; i < _vertexNumber; ++i)
	{
		_massList[i]._pos += offset;
	}*/
	for (unsigned int i = 0; i < _massList.size(); ++i)
	{
		_massList[i]._pos += vector;
	}
	_bbox._topleft += vector;
}


void Object::Scale(const float value)
{

	for (unsigned int i = 0; i < _massList.size(); ++i)
	{
		_massList[i]._pos *= value;
	}

	_bbox.SetFirstLength(value);
	//_bbox.Scale(value);
	
}

// not use
void Object::Rotate(const float value)
{
	glm::mat4 matrix(1.0f);
	matrix = glm::rotate(matrix, glm::radians(value), glm::vec3(0.0, 0.0, 1.0));

	Move(-_bbxCenter);
	for (unsigned int i = 0; i < _vertices.size(); ++i)
	{
		_vertices[i].Position = matrix * _vertices[i].Position;
	}
	Move(_bbxCenter);
}

void Object::UpdateVertices()
{
	// drawing data
	unsigned int Vidx = 0;
	for (unsigned int i = 0; i < _artTris.size(); ++i)
	{
		_vertices[Vidx++].Position = glm::vec4(_tesselateArt[_artTris[i].idx1], 0, 1);
		_vertices[Vidx++].Position = glm::vec4(_tesselateArt[_artTris[i].idx2], 0, 1);
		_vertices[Vidx++].Position = glm::vec4(_tesselateArt[_artTris[i].idx3], 0, 1);
	}
}

void Object::UpdateBoundaryAndAvgEdgeLength()
{
	if (_skin.size() == 0)
	{
		for (unsigned int i = 0; i < _skinPointNumber; ++i)
			_skin.push_back(_massList[i]._pos);
	}
	
	for (unsigned int i = 0; i < _skinPointNumber; ++i)
		_skin[i] = _massList[i]._pos;
	
	_avgEdgeLength = 0;
	int edgeSize = _edgeList.size();
	for (unsigned int i = 0; i < edgeSize; ++i)
		_avgEdgeLength += glm::distance(_massList[_edgeList[i]._index1]._pos, _massList[_edgeList[i]._index2]._pos);
	_avgEdgeLength /= (float)edgeSize;
}

void Object::RecalculateTriangleEdgeLengths()
{
	for (unsigned int i = 0; i < _edgeList.size(); ++i)
	{
		float dist = glm::distance(_massList[_edgeList[i]._index1]._pos, _massList[_edgeList[i]._index2]._pos);
		_edgeList[i].SetDist(dist);
	}

	for (unsigned int i = 0; i < _auxiliaryEdge.size(); ++i)
	{
		float dist = glm::distance(_massList[ _auxiliaryEdge[i]._index1]._pos, _massList[ _auxiliaryEdge[i]._index2]._pos);
		 _auxiliaryEdge[i].SetDist(dist);
	}
}

void Object::CalculateOriAvgEdgeLength()
{
	_oriAvgEdgeLength = 0;
	for (unsigned int i = 0; i < _edgeList.size(); ++i)
		_oriAvgEdgeLength += glm::distance(_massList[_edgeList[i]._index1]._pos, _massList[_edgeList[i]._index2]._pos);
	_oriAvgEdgeLength /= (float)_edgeList.size();
}

void Object::UpdateCentroid()
{
	_centroid = _massList[_triangles[_centroidTriIdx].idx1]._pos * _centroidBary._u +
				_massList[_triangles[_centroidTriIdx].idx2]._pos * _centroidBary._v +
				_massList[_triangles[_centroidTriIdx].idx3]._pos * _centroidBary._w;
}

void Object::Grow(float growth_scale_iter, float dt)
{
	if (_isShrink) 
	{
		growth_scale_iter *= -1;
		_isShrink = false;
	}
	else
	{
		for (unsigned int i = 0; i < _skinPointNumber && _isGrowing; ++i)
		{
			if (_massList[i]._closestPoints.size() > 0)
			{
				if (_massList[i]._isInside)
				{
					_isGrowing = false; 
					break;
				}

				if (_massList[i]._closestDist < SystemParams::_growth_min_dist)
				{
					_isGrowing = false;
					break;
				}
			}
		}
	}
	// stop operation
	if(SystemParams::_stop_scale)
		_isGrowing = false;
	if (_isGrowing && ((_id-1) == SystemParams::_stop_id))
		_isGrowing = false;

	// focus object          
	if (_isFocus && !_isGrowing && !SystemParams::_trying)
	{
		//_isGrowing = true;
		for (auto it = StuffWorker::_aroundIds.begin(); it != StuffWorker::_aroundIds.end(); ++it)
		{
			int idx = it->second;
			StuffWorker::_objs[idx]._isShrink = true;
			//std::cout << idx <<" shrink\n";
		}
	}

	//if ((_id-1) == SystemParams::_focus_id && !_isGrowing && !SystemParams::_trying)
	//{
	//	_isGrowing = true;
	//	for (auto it = StuffWorker::_aroundIds.begin(); it != StuffWorker::_aroundIds.end(); ++it)
	//	{
	//		int idx = it->second;
	//		StuffWorker::_objs[idx]._isShrink = true;
	//		//std::cout << idx <<" shrink\n";
	//	}
	//}
		
	if (!_isGrowing)
		return;
	
	// focus obj growing fast
	//if ((_id-1) == SystemParams::_focus_id)
	if(_isFocus)
		growth_scale_iter *= 2.0f;
	if(_aroundFocus)
		growth_scale_iter *= 2.0f;

	if (_isSmall)
		growth_scale_iter *= 1.5f;
	if (_isBig)
		growth_scale_iter *= 0.5f;

	this->_scale += growth_scale_iter * dt;
	//std::cout << _scale << std::endl;
	
	// resize bbox scale
	this->_bbox.Scale(growth_scale_iter, dt);
	
	//if (!_isBig && (_bbox.GetLong() > SystemParams::_max_size * 1.5) )
	if (!_isBig && (_bbox.GetLong() > 110))
		_isBig = true;

	//if (_isSmall && (_bbox.GetLong() > SystemParams::_min_size * 1.5) )
	if (_isSmall && (_bbox.GetLong() > 60))
		_isSmall = false;

//	---------------------- growing --------------------------
	for (unsigned int i = 0; i < _massList.size(); ++i)
		_massList[i].Grow(growth_scale_iter, dt);
	
	for (unsigned int i = 0; i < _edgeList.size(); ++i)
		_edgeList[i].MakeLonger(growth_scale_iter, dt);

	for (unsigned int i = 0; i < _auxiliaryEdge.size(); ++i)
		_auxiliaryEdge[i].MakeLonger(growth_scale_iter, dt);
}

void Object::Grow_Move(float growth_scale_iter, float dt)
{
	if (!_isGrowing)
		return;

	if (!SystemParams::_resize)
	{
		if (_isShrink)
		{
			growth_scale_iter *= -1;
			_isShrink = false;
		}
		else
		{
			for (unsigned int i = 0; i < _skinPointNumber && _isGrowing; ++i)
			{
				if (_massList[i]._closestPoints.size() > 0)
				{
					if (_massList[i]._isInside)
					{
						_isGrowing = false;
						break;
					}

					if (_massList[i]._closestDist < SystemParams::_growth_min_dist)
					{
						_isGrowing = false;
						break;
					}
				}
			}
		}
	}
	else
	{
		if (_isShrink)
		{
			growth_scale_iter *= -1;
			_isShrink = false;
		}
		// don't chack overlap
	}


	if (!_isGrowing)
		return;

	this->_scale += growth_scale_iter * dt;
	

	this->_bbox.Scale(growth_scale_iter, dt);
	if (!_isBig && (_bbox.GetLong() > 100))
		_isBig = true;
	if(_isBig && (_bbox.GetLong() < 100))
		_isBig = false;


	for (unsigned int i = 0; i < _massList.size(); ++i)
		_massList[i].Grow(growth_scale_iter, dt);

	for (unsigned int i = 0; i < _edgeList.size(); ++i)
		_edgeList[i].MakeLonger(growth_scale_iter, dt);

	for (unsigned int i = 0; i < _auxiliaryEdge.size(); ++i)
		_auxiliaryEdge[i].MakeLonger(growth_scale_iter, dt);
}

void Object::Grow_Tmp(float growth_scale_iter, float dt)
{
	if (_isShrink)
	{
		growth_scale_iter *= -1;
		_isShrink = false;
	}
	else
	{
		for (unsigned int i = 0; i < _skinPointNumber && _isGrowing; ++i)
		{
			if (_massList[i]._closestPoints.size() > 0)
			{
				if (_massList[i]._isInside)
				{
					_isGrowing = false;
					break;
				}

				if (_massList[i]._closestDist < SystemParams::_growth_min_dist)
				{
					_isGrowing = false;
					break;
				}
			}
		}
	}

	if (!_isGrowing)
		return;

	this->_scale += growth_scale_iter * dt;

	this->_bbox.Scale(growth_scale_iter, dt);

	//	---------------------- growing --------------------------
	for (unsigned int i = 0; i < _massList.size(); ++i)
		_massList[i].Grow(growth_scale_iter, dt);

	for (unsigned int i = 0; i < _edgeList.size(); ++i)
		_edgeList[i].MakeLonger(growth_scale_iter, dt);

	for (unsigned int i = 0; i < _auxiliaryEdge.size(); ++i)
		_auxiliaryEdge[i].MakeLonger(growth_scale_iter, dt);
}

void Object::CalculateSumVelocity()
{
	_sumVelocity = 0;
	for (unsigned int i = 0; i < _massList.size(); ++i)
	{
		_sumVelocity += glm::length(_massList[i]._velocity);
	}
}

float Object::GetArea()
{
	std::vector<cv::Point2f> pts;
	for (auto p : _skin)
		pts.push_back(cv::Point2f(p.x, p.y));
	return cv::contourArea(pts);
}


void Object::SolveForNegativeSPaceSprings()
{

	float k_edge = SystemParams::_k_neg_space_edge;


	if (_isSmall)
		k_edge *= 3;
	
	glm::vec2 pt0;
	glm::vec2 pt1;
	glm::vec2 dir;
	glm::vec2 eForce;
	for (unsigned int i = 0; i < _negEdge.size(); ++i)
	{
		int idx0 = _negEdge[i]._index1;
		int idx1 = _negEdge[i]._index2;

		pt0 = _massList[idx0]._pos;
		pt1 = _massList[idx1]._pos;

		float dist = glm::distance(pt0, pt1);

		float threshold = _avgEdgeLength * SystemParams::_self_intersection_threshold;
		//std::cout << "threshold : " << threshold << std::endl;
		if (dist < threshold)
		{
			dir = glm::normalize(pt0 - pt1);
			float diff = threshold - dist;
			eForce = (dir * k_edge * diff);
			
			if (dist < 1.0f)
				eForce *= 3.0f;
			//std::cout << "force : " << eForce.x << " " << eForce.y << std::endl;

			if (!(std::isinf(eForce.x) || std::isnan(eForce.x) || std::isinf(eForce.y) || std::isnan(eForce.y)))
			{
				//std::cout << "intersection : " << std::endl;	
				_massList[idx0]._selfIntersectForce += eForce;
				_massList[idx1]._selfIntersectForce -= eForce;
			}
		}
	}
}

// triangle edge
void Object::SolveForTriangleSprings()
{

	float k_edge = SystemParams::_k_edge;
	
	glm::vec2 pt0;
	glm::vec2 pt1;
	glm::vec2 dir;
	glm::vec2 eForce;

	if (_isSmall)
		k_edge *= 3;
	/*if (_isBig)
		k_edge *= 0.2;*/

	// ------------- traingle edge springs ------------
	for (unsigned int i = 0; i < _edgeList.size(); ++i)
	{
		int idx0 = _edgeList[i]._index1;
		int idx1 = _edgeList[i]._index2;

		pt0 = _massList[idx0]._pos;
		pt1 = _massList[idx1]._pos;

		float dist = glm::distance(pt0, pt1);
		
		// assume we need to make the dist shorter
		dir = glm::normalize(pt1 - pt0);
		float oriDist = _edgeList[i].GetDist();
		float signVal = 1;
		float diff = dist - oriDist;

		if (diff < 0)
			signVal = -1;
		
		eForce = dir * k_edge * signVal * diff * diff;
		
		
		if (!(std::isinf(eForce.x) || std::isnan(eForce.x) || std::isinf(eForce.y) || std::isnan(eForce.y)))
		{
			_massList[idx0]._edgeForce += eForce;
			_massList[idx1]._edgeForce -= eForce;
		}
	}

	// ------------ auxiliary edge springs ----------------
	for (unsigned int i = 0; i < _auxiliaryEdge.size(); ++i)
	{
		int idx0 = _auxiliaryEdge[i]._index1;
		int idx1 = _auxiliaryEdge[i]._index2;

		pt0 = _massList[idx0]._pos;
		pt1 = _massList[idx1]._pos;

		float dist = glm::distance(pt0, pt1);

		dir = glm::normalize(pt1 - pt0);
		float oriDist = _auxiliaryEdge[i].GetDist();
		float signVal = 1;
		float diff = dist - oriDist;

		if (diff < 0)
			signVal = -1;

		eForce = dir * k_edge * signVal * diff * diff;
		

		if (!(std::isinf(eForce.x) || std::isnan(eForce.x) || std::isinf(eForce.y) || std::isnan(eForce.y)))
		{
			//std::cout << "e force : " << eForce.x << " " << eForce.y << std::endl;
			_massList[idx0]._edgeForce += eForce;
			_massList[idx1]._edgeForce -= eForce;
		}
	}


	// rotation (default 0)
	if (SystemParams::_changeContour && SystemParams::_tuningAng)
	//if (_completeShift)
	{
		float eps_rot = 3.14 * 0.001;

		float angleValAvg = 0;
		for (unsigned int i = 0; i < _skinPointNumber; ++i)
		{
			//glm::vec2 targetVector(0, -1);
			glm::vec2 targetVector = _targetNorm[i];
			glm::vec2 curNorm = glm::normalize(_massList[i]._pos - _centroid);
			float angleVal = UtilityFunctions::Angle2D(curNorm.x, curNorm.y, targetVector.x, targetVector.y);
			angleValAvg += angleVal;
		}

		angleValAvg /= (float)_skinPointNumber;
		for (unsigned int i = 0; i < _skinPointNumber; ++i)
		{
			if (std::abs(angleValAvg) > eps_rot)
			{
				glm::vec2 curNorm = glm::normalize(_massList[i]._pos - _centroid);

				if (angleValAvg > 0)
				{
					// anticlockwise
					glm::vec2 dRight(-curNorm.y, curNorm.x);
					_rotateArray[i] = dRight;
				}
				else
				{
					glm::vec2 dLeft(curNorm.y, -curNorm.x);
					_rotateArray[i] = dLeft;
				}
			}
			else
			{
				_rotateArray[i] = glm::vec2(0, 0);
			}
			
			glm::vec2 rForce = _rotateArray[i] * (float)SystemParams::_k_rotate * 0.3f;

			//std::cout << "rotate : " << glm::length(rForce) << std::endl;
			if (!(std::isinf(rForce.x) || std::isnan(rForce.x) || std::isinf(rForce.y) || std::isnan(rForce.y)))
			{
				_massList[i]._rotationForce += rForce;
			}
		}
	}
}

/*
	update the centroid and the triangles that be used to render on window.
*/
void Object::RecalculateArts()
{
	ATriangle tri(0, 0, 0);
	ABary bary(0, 0, 0);
	// centroid
	tri = _triangles[_centroidTriIdx];
	_centroid = _massList[tri.idx1]._pos * _centroidBary._u +
		_massList[tri.idx2]._pos * _centroidBary._v +
		_massList[tri.idx3]._pos * _centroidBary._w;

	for (unsigned int i = 0; i < _tesselateArt.size(); ++i)
	{
		tri = _triangles[_tesselateArt2Triangles[i]];
		bary = _tesselateBary[i];
		_tesselateArt[i] = _massList[tri.idx1]._pos * bary._u +
						   _massList[tri.idx2]._pos * bary._v +
						   _massList[tri.idx3]._pos * bary._w;
	}

	_massCenter = glm::vec2(0, 0);
	for (unsigned int i = 0; i < _massList.size(); ++i)
		_massCenter += _massList[i]._pos;
	_massCenter /= (float)_massList.size();
}

/*
	save 
	-- _id
	-- _massCenter
	-- _bbox.GetLong() -> size
	--  norm vector
*/
void Object::SaveTargetData()
{
	auto save = json
	{
		{
			"obj_id", 
			_id
		},
		{
			"targetPos_x", 
			_massCenter.x
		},
		{
			"targetPos_y", 
			_massCenter.y
		},
		{
			"targetSize", 
			_bbox.GetLong()
		},

		// angle
		{
			"torsion",
			json::array()
		}
	};

	// angle
	for (unsigned int i = 0; i < _skinPointNumber; ++i)
	{
		glm::vec2 norm = glm::normalize(_massList[i]._pos - _centroid);
		auto &pt = save["torsion"];
		pt.push_back({ norm.x, norm.y });
	}

	auto out = std::ofstream(SystemParams::_target_dir + "\\" + std::to_string(_id) + ".json");
	out << save;

	out.close();
}

void Object::ResetData()
{
	// points
	assert(_massList.size() == _resampledContour.size());
	for (unsigned int i = 0; i < _massList.size(); ++i)
	{
		_massList[i]._pos = _resampledContour[i];
		
		_massList[i].Init();
	}
	/*_massList.clear();
	MassData();*/

	// bbox
	this->_bbox = _initBbox;

	// edges
	for (unsigned int i = 0; i < _edgeList.size(); ++i)
		_edgeList[i]._scale = 1.0;

	for (unsigned int i = 0; i < _auxiliaryEdge.size(); ++i)
		_auxiliaryEdge[i]._scale = 1.0;
	RecalculateTriangleEdgeLengths();
	UpdateBoundaryAndAvgEdgeLength();
	RecalculateArts();
	UpdateVertices();
}

