#ifndef COLLISIONGRID_H
#define COLLISIONGRID_H

#include <vector>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include "ASquare.h"


typedef std::vector<int> GraphIndices;

class CollisionGrid
{
public:
	CollisionGrid();
	CollisionGrid(float cellSize);
	~CollisionGrid();

	void GetCellPosition(int& xPos, int& yPos, float x, float y);

	void InsertAPoint(float x, float y, int info1, int info2);

	std::vector<int>* GetGraphIndicesPtr(float x, float y, int parentGraphIndex);

	void GetGraphIndices2B(float x, float y, int parentGraphIndex, std::vector<int>& closestGraphIndices);

	void GetData(float x, float y, int parentGraphIndex, std::vector<glm::vec2>& closestPts, std::vector<int>& closestGraphIndices);

	void GetClosestPoints(float x, float y, std::vector<glm::vec2>& closestPts);

	std::vector<AnObject*> GetObjects(float x, float y);

	void MovePoints();

	void AnalyzeContainer(const std::vector<glm::vec2>&  boundaries);

	void ClearContianer();

	void PrecomputeGraphIndices();

	void PrecomputeGraphIndices_ThreadTask(int startIdx, int endIdx);

	bool NearBoundary(float x, float y);

	void ClearObjects();

public:

	int _numColumn;
	float _maxLength;

	std::vector<AnObject*> _objects;

	std::vector<ASquare*> _squares;

};

#endif // !COLLISIONGRID_H
