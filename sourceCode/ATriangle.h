#pragma once
#ifndef ATRIANGLE_H
#define ATRIANGLE_H

class ATriangle
{
public:
	ATriangle()
	{
		// vertex index
		idx1 = -1;
		idx2 = -1;
		idx3 = -1;

	}
	ATriangle(int id1, int id2, int id3)
	{
		idx1 = id1;
		idx2 = id2;
		idx3 = id3;
	
	}

	~ATriangle()
	{

	}
	int idx1;
	int idx2;
	int idx3;

};


#endif // !ATRIANGLE_H
