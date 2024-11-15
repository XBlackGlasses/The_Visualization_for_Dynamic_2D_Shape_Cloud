#ifndef OBJECT_H
#define OBJECT_H

#include <glad/glad.h>

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include "shader.h"

#include "ARectangle.h"
#include "ABary.h"
#include "InedexEdge.h"
#include "ATriangle.h"
#include "AMass.h"

#include "debug.h"


struct Vertex {
	// position
	glm::vec4 Position;
	// texCoords
	glm::vec2 TexCoords;
};


/*
	represent each image in the program.
*/

class Object
{
private:
	std::string  _jsonFile;
	std::string  _imgPath;
	unsigned int _imgWidth;
	unsigned int _imgHeight;

	GLuint _vbo;
	GLuint _vao ;
	GLuint _textureID;

	ARectangle _initBbox;

	void LoadFile(const char *string);

	ARectangle GetBBx();

	// prepare mass data;
	void MassData();

	void GetContourMap();

	
public:

	Object();

	Object(const char* jsonFile);
	
	~Object();
	// contour
	std::vector<glm::vec2> _smoothContour;
	std::vector<glm::vec2> _resampledContour;
	
	cv::Mat _contourMap;
	
	// vertex of a object. for openGL rendering
	std::vector<Vertex> _vertices;
	// each face storing the index of the vertex to form a triangle.
	std::vector<ATriangle> _triangles;

	// each vertex represent a mass. 
	std::vector<AMass> _massList;

	// store all the edge
	std::vector<InedexEdge> _edgeList;

	// skin(boundary) of the object
	std::vector<glm::vec2> _skin;
	std::vector<InedexEdge> _skinEdge;

	// edge force
	std::vector<InedexEdge> _auxiliaryEdge;
	std::vector<InedexEdge> _negEdge;

	// mapping edgelist to triangles
	std::vector<std::vector<int>> _edgeToTri;

	// bary coordinate
	std::vector<ABary> _baryCoord;
	std::vector<int> _art2Triangles;

	// tesselate art
	std::vector<glm::vec2> _tesselateArt;
	std::vector<int> _tesselateArt2Triangles;
	std::vector<ABary> _tesselateBary;
	std::vector<ATriangle> _artTris;

	// bounding box. top-left point
	glm::vec2 _bbxCenter;	
	ARectangle _bbox;
	

	// centroid of the object
	glm::vec2 _centroid;
	int _centroidTriIdx;
	ABary _centroidBary;

	// for orientation force
	std::vector<glm::vec2> _normFromCentroidArray;
	std::vector<glm::vec2> _rotateArray;
	// target orientation
	std::vector<glm::vec2> _targetNorm;

	// avg mass coordinate.
	glm::vec2 _massCenter;
	
	// the position of previous shape cloud.
	glm::vec2 _prevPos;

	unsigned int _skinPointNumber;	// number of boundaru points.
	float _contourLength;
	float _avgEdgeLength;			// average length of the edgelist.
	float _oriAvgEdgeLength;

	int _id;

	bool _isGrowing;

	bool _isShrink;

	bool _isBig;

	bool _isSmall;

	bool _isFocus;
	bool _aroundFocus;

	float _scale;

	float _sumVelocity;

	// for multi contour animation.
	glm::vec2 _targetPosition;
	glm::vec2 _lastPosition;
	// for target shapge cloud.
	float _targetSize;
	// for resize at first stage.
	float _preparedSize;
	bool _completeShift;
	bool _completeResize;
	bool _stuck;

	// check whether show up
	bool _showUp;
	// when change shape cloud, controll the fade in/out
	bool _fadeIn;
	bool _fadeOut;

	// controll resize in moving, when didn't press "x"
	bool _extremeDiff;

	bool _speedUp;

	void BindTexture();

	// bind vao, vbo, ebo data.
	void BindGLData();

	void DrawGL(const Shader & shader);

	void Move(const glm::vec2& vector);

	void Scale(const float value);

	void Rotate(const float value);

	void UpdateVertices();

	inline GLuint GetVAO() const
	{
		ASSERT(_vao != 0);
		return _vao;
	}
	
	inline GLuint GetTextureID() const
	{
		ASSERT(_textureID != 0);
		return _textureID;
	}

	inline unsigned int GetWidth() const
	{
		ASSERT(_imgWidth != 0);
		return _imgWidth;
	}

	inline unsigned int GetHeight() const
	{
		ASSERT(_imgHeight != 0);
		return _imgHeight;
	}

	inline void ReleaseGL()
	{
		GLCall(glDeleteVertexArrays(1, &_vao));
		GLCall(glDeleteBuffers(1, &_vbo));
		
	}

public:
	void Grow(float growth_scale_iter, float dt);
	// ues it when changing contour.
	void Grow_Move(float growth_scale_iter, float dt);
	// ues it when resize stage.
	void Grow_Tmp(float growth_scale_iter, float dt);

	float GetArea();

	void CalculateSumVelocity();

	void UpdateBoundaryAndAvgEdgeLength();
	void RecalculateTriangleEdgeLengths();
	void CalculateOriAvgEdgeLength();

	void UpdateCentroid();
	// ---- solve edge force 
	void SolveForNegativeSPaceSprings();
	// triangle edges
	void SolveForTriangleSprings();

	// recalculate the contour by bary coordinate
	void RecalculateArts();

	void SaveTargetData();

	inline void InitController()
	{

		_completeResize = false;
		_isBig		= false;
		_isSmall	= false;
		_isFocus	= false;
		_aroundFocus= false;
		_extremeDiff= false;
		_speedUp    = false;
	}

	// reset mass, edge when fade out
	void ResetData();
};



#endif // !OBJECT_H