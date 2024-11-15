#ifndef AMASS_H
#define AMASS_H

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include <vector>
#include <unordered_map>

#include "ATriangle.h"
#include "InedexEdge.h"
#include "CollisionGrid.h"


class Object;

class AMass
{
public:
	CollisionGrid *_cGrid;

	float _distToBoundary;

	float _mass;
	
	glm::vec2 _pos;
	glm::vec2 _velocity;

	inline glm::vec2 GetPos() const
	{
		return _pos;
	}

	// the mass belong to some triangles
	std::vector<ATriangle> _triangles;

	float _closestDist;
	int _n_closest_elems;

	std::vector<glm::vec2> _closestPoints;
	// try to record the big object.
	std::vector<bool> _bigNeighbor;
	int _closestPt_actual_sz; // reserve for avoiding push_back
	int _closestPt_fill_sz;   // reserve for avoiding push_back

	float _avgEdgeLength;
	bool _isInside;
	int _idx;

	std::vector<InedexEdge> _triEdges;		// edges from triangle
	std::unordered_map<int, int> _neighbors;


	std::vector<int> _segmentIndices;
	std::vector<glm::vec2> _lineSegment;

public:
	void CalculateIndicesOfLineSegment(const int& numSkin);

public:
	glm::vec2 _edgeForce;
	glm::vec2 _repulsionForce;
	glm::vec2 _boundaryForce;
	glm::vec2 _overlapForce;
	glm::vec2 _rotationForce;
	glm::vec2 _selfIntersectForce;
	// only for moving mode
	glm::vec2 _translate;


	// constructor
	AMass();
	AMass(const float& x, const float& y);
	AMass(const glm::vec2 & pos);

	bool TryToAddTriangleEdge(InedexEdge anEdge, const std::vector<AMass>& otherMass);

	bool FindTriEdge(const InedexEdge& anEdge);

	bool IsNeighbor(int idx);
	
	void Grow(const float& growth_scale_iter, const float& dt);

	void Init();	// set force to zero

	void Simulate(float dt);

	void Solve(const int& massNumber, const Object &parentGraph, const std::vector<glm::vec2> &boundary);	

	void Simulate_Move(float dt);

	void Solve_Move(const int& massNumber, const Object &parentGraph, const std::vector<glm::vec2> &boundary, glm::vec2& targetPos);

	void GetClosestPoints(const int &parentGraphIndex);

private:
	void CallByConstructor();
};


#endif // !AMASS_H