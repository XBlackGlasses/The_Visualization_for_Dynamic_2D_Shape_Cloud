#ifndef CONTOUROBJECT_H
#define CONTOUROBJECT_H

#include <glad/glad.h>

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include <unordered_map>
#include <boost/functional/hash.hpp>

#include "ARectangle.h"

#include "shader.h"

#include "debug.h"

class ContourObject
{
private:
	std::string _jsonFile;
	std::string  _imgPath;
	unsigned int _imgWidth;
	unsigned int _imgHeight;

	float _contourLength;
	float _contourArea;


	GLuint _vbo;
	GLuint _vao;

	void LoadJsonData();

	// find the contourX of the object will be placed in the contourObject
	void SetContour(std::vector<std::vector<int>>& contourX, const cv::Mat& img);

	// get the contour infomation of object that prepare to placement
	void GetContourInfo(const std::vector<std::vector<int>>& contourX, std::vector<int> &contourLength,\
		std::vector<int> &contourLengthShort, std::vector<int> &contourLengthDis, int &longest, int width);

public:
	// vertices just for gl rendering
	std::vector<glm::vec4> _vertices;
	
	std::vector<glm::vec2> _smoothContour;
	std::vector<glm::vec2> _resampledContour;
	std::vector<std::vector<int>> _countMap;
	// the contour width of each row
	std::vector<int> _rowsWidth;

	glm::ivec2 _centroid;

	// skeleton data
	std::unordered_map<std::pair<float, float>, float, boost::hash<std::pair<float, float>>> _skt2Num;
	std::vector<glm::vec2> _skeleton;

public:
	explicit ContourObject();
	explicit ContourObject(const std::string& jsonFile);
	~ContourObject() {};
	void BindGLdata();

	void Draw(const Shader & shader);

	bool Placement(const ARectangle &boundingBox, glm::vec2& pos);
	
	void UpdateCountMap(const ARectangle &boundingBox, int x, int y);
	
	// compare with center
	bool Placement2(const cv::Mat &img, glm::vec2& pos, const glm::vec2& prevPos);

	// compare with skeleton
	bool Placement3(const cv::Mat &img, glm::vec2& pos, const glm::vec2& prevPos);

	void UpdateCountMap2(const std::vector<std::vector<int>>& contourX, int x, int y);


	void Copy(ContourObject& other);

	inline void ReleaseGL()
	{
		GLCall( glDeleteVertexArrays(1, &_vao) );
	 	GLCall( glDeleteBuffers(1, &_vbo) );
		
	}

	inline float GetArea() const
	{
		ASSERT(_contourArea != 0);
		return _contourArea;
	}

	inline float GetLength() const
	{
		ASSERT(_contourLength != 0)
		return _contourLength;
	}

	inline unsigned int GetWidth() const
	{
		return _imgWidth;
	}

	inline unsigned int GetHeight() const
	{
		return _imgHeight;
	}
};

#endif // !CONTOUROBJECT_H
