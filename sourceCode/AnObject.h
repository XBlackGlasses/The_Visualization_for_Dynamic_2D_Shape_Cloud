#ifndef ANOBJECT_H
#define ANOBJECT_H

// object for collision grid
class AnObject 
{
public:
	float _x;
	float _y;
	int   _info1;  // which object
	int   _info2;  // which mass

	AnObject(float x, float y, int info1, int info2) :
		_x(x),
		_y(y),
		_info1(info1),
		_info2(info2)//,
		//_nx(-1000),
		//_ny(-1000)
	{
	}

	AnObject(float x, float y, int info1) :
		_x(x),
		_y(y),
		_info1(info1),
		_info2(-1)//,
		//_nx(-1000),
		//_ny(-1000)
	{
	}

	AnObject(float x, float y) :
		_x(x),
		_y(y),
		_info1(-1),
		_info2(-1)//,
		//_nx(-1000),
		//_ny(-1000)
	{
	}



};

#endif // !ANOBJECT_H
