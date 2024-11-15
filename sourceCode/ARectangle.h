#ifndef ARECTANGLE_H
#define ARECTANGLE_H

#include "glm/glm.hpp"

struct ARectangle
{
public:
	glm::vec2 _topleft;
	float _width;
	float _height;

	float _scale;
	// for calculating resize length.
	float _ori_width;
	float _ori_height;
	ARectangle()
	{
		this->_topleft = glm::vec2(-1, -1);
		this->_width = -1;
		this->_height = -1;
		this->_ori_width = -1;
		this->_ori_height = -1;
		this->_scale = 1.0f;
	}

	ARectangle(glm::vec2 topleft, float width, float height)
	{
		this->_topleft = topleft;
		this->_width = width;
		this->_height = height;
		this->_ori_width = width;
		this->_ori_height = height;
		this->_scale = 1.0f;
	}

	glm::vec2 GetCenter()
	{
		return glm::vec2(_topleft.x + _width / 2.0f, _topleft.y + _height / 2.0f);
	}
	
	void SetFirstLength(float scale)
	{
		_height *= scale;
		_width *= scale;
		_ori_height = _height ;
		_ori_width = _width ;
		
	}

	void Scale(float growth_scale_iter, float dt)
	{
		_scale += growth_scale_iter * dt;
		_width  = _ori_width * _scale;
		_height = _ori_height * _scale;

	}

	float GetLong()
	{
		return (_width > _height) ? _width : _height;
	}
};

#endif // !ARECTANGLE_H
