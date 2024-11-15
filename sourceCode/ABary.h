#ifndef ABARY_H
#define ABARY_H

class ABary
{
public:
	ABary()
	{
		_u = 0;
		_v = 0;
		_w = 0;
	}
	ABary(float u, float v, float w)
	{
		this->_u = u;
		this->_v = v;
		this->_w = w;
	}
	~ABary()
	{

	}

	float _u;
	float _v;
	float _w;

	bool IsValid()
	{
		return (_u > 0 && _v > 0 && _w > 0);
	}
};


#endif // !ABARY_H
