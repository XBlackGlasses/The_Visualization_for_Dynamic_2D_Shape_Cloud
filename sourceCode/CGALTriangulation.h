#ifndef CGAL_TRIANGULATION_H
#define CGAL_TRIANGULATION_H

#include "ATriangle.h"
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include <vector>

class CGALTriangulation
{
public:
	CGALTriangulation();
	~CGALTriangulation();

	void Triangulate(std::vector<glm::vec2> boundary, std::vector<ATriangle>& triangles);
};

#endif // !CGAL_TRIANGULATION_H
