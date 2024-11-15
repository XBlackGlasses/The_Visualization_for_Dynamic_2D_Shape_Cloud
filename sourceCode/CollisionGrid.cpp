#include "CollisionGrid.h"
#include "UtilityFunctions.h"
#include "SystemParams.h"

#include <unordered_set>

CollisionGrid::CollisionGrid()
{
	_maxLength = SystemParams::_bin_square_size;
	_numColumn = SystemParams::_upscaleFactor / _maxLength;
	for (unsigned int a = 0; a < _numColumn; a++) 
	{
		for (unsigned int b = 0; b < _numColumn; b++) 
		{
			float x = a * _maxLength;
			float y = b * _maxLength;
			_squares.push_back(new ASquare(x, y, _maxLength));
		}
	}
}

CollisionGrid::CollisionGrid(float cellSize)
{
	_maxLength = cellSize;
	_numColumn = SystemParams::_upscaleFactor / _maxLength;
	for (unsigned int a = 0; a < _numColumn; a++)
	{
		for (unsigned int b = 0; b < _numColumn; b++)
		{
			float x = a * _maxLength;
			float y = b * _maxLength;
			_squares.push_back(new ASquare(x, y, _maxLength));
		}
	}
}

CollisionGrid::~CollisionGrid()
{
	for (unsigned int a = 0; a < _squares.size(); a++)
	{
		_squares[a]->Clear();
		delete _squares[a];
	}
	_squares.clear();
}

void CollisionGrid::GetCellPosition(int & xPos, int & yPos, float x, float y)
{
	xPos = x / _maxLength;
	yPos = y / _maxLength;
	if (xPos < 0) { xPos = 0; }
	else if (xPos >= _numColumn) { xPos = _numColumn - 1; }

	if (yPos < 0) { yPos = 0; }
	else if (yPos >= _numColumn) { yPos = _numColumn - 1; }
}

void CollisionGrid::InsertAPoint(float x, float y, int info1, int info2)
{
	AnObject* obj = new AnObject(x, y, info1, info2);
	_objects.push_back(obj);

	//int xPos = x / _maxLength;
	//int yPos = y / _maxLength;
	int xPos;
	int yPos;
	GetCellPosition(xPos, yPos, x, y);

	_squares[(xPos * _numColumn) + yPos]->_objects.push_back(obj);
}

std::vector<int>* CollisionGrid::GetGraphIndicesPtr(float x, float y, int parentGraphIndex)
{
	int xPos;
	int yPos;
	GetCellPosition(xPos, yPos, x, y);

	int idx = (xPos * _numColumn) + yPos;

	//closestGraphIndices = _graphIndexArray[idx];
	return &_squares[idx]->_closestGraphIndices;
}

void CollisionGrid::GetGraphIndices2B(float x, float y, int parentGraphIndex, std::vector<int>& closestGraphIndices)
{
	int xPos;
	int yPos;
	GetCellPosition(xPos, yPos, x, y);

	int idx = (xPos * _numColumn) + yPos;

	//closestGraphIndices = _graphIndexArray[idx];
	closestGraphIndices = _squares[idx]->_closestGraphIndices;
}

void CollisionGrid::GetData(float x, float y, int parentGraphIndex, std::vector<glm::vec2>& closestPts, std::vector<int>& closestGraphIndices)
{
	if (std::isnan(x) || std::isinf(x) || std::isnan(y) || std::isinf(y))
	{
		return;
	}

	if (x < 0 || x > SystemParams::_upscaleFactor || y < 0 || y > SystemParams::_upscaleFactor)
	{
		return;
	}

	//int xPos = x / _maxLength;
	//int yPos = y / _maxLength;
	int xPos;
	int yPos;
	GetCellPosition(xPos, yPos, x, y);

	int offst = SystemParams::_collission_block_radius;

	int xBegin = xPos - offst;
	if (xBegin < 0) { xBegin = 0; }

	int xEnd = xPos + offst;
	if (xEnd >= _numColumn) { xEnd = _numColumn - 1; }

	int yBegin = yPos - offst;
	if (yBegin < 0) { yBegin = 0; }

	int yEnd = yPos + offst;
	if (yEnd >= _numColumn) { yEnd = _numColumn - 1; }

	for (unsigned int xIter = xBegin; xIter <= xEnd; xIter++)
	{
		for (unsigned int yIter = yBegin; yIter <= yEnd; yIter++)
		{
			int idx = (xIter * _numColumn) + yIter;

			for (unsigned int a = 0; a < _squares[idx]->_objects.size(); a++)
			{
				int info1 = _squares[idx]->_objects[a]->_info1;

				if (info1 != parentGraphIndex)
				{
					if (UtilityFunctions::GetIndexFromIntList(closestGraphIndices, info1) == -1)
					{
						closestGraphIndices.push_back(info1);
					}

					glm::vec2 pt;
					pt.x = _squares[idx]->_objects[a]->_x;
					pt.y = _squares[idx]->_objects[a]->_y;
					closestPts.push_back(pt);
				}
			}

			//nearObjects.insert(nearObjects.end(), _squares[idx]->_objects.begin(), _squares[idx]->_objects.end());

			//std::vector<AnObject*> objs = _squares[(xIter * _numColumn) + yIter]->_objects;
			//nearObjects.insert(nearObjects.end(), objs.begin(), objs.end());
		}
	}
}

void CollisionGrid::GetClosestPoints(float x, float y, std::vector<glm::vec2>& closestPts)
{
	if (std::isnan(x) || std::isinf(x) || std::isnan(y) || std::isinf(y))
	{
		return;
	}

	if (x < 0 || x > SystemParams::_upscaleFactor || y < 0 || y > SystemParams::_upscaleFactor)
	{
		return;
	}

	//int xPos = x / _maxLength;
	//int yPos = y / _maxLength;
	int xPos;
	int yPos;
	GetCellPosition(xPos, yPos, x, y);

	int offst = SystemParams::_collission_block_radius;

	int xBegin = xPos - offst;
	if (xBegin < 0) { xBegin = 0; }

	int xEnd = xPos + offst;
	if (xEnd >= _numColumn) { xEnd = _numColumn - 1; }

	int yBegin = yPos - offst;
	if (yBegin < 0) { yBegin = 0; }

	int yEnd = yPos + offst;
	if (yEnd >= _numColumn) { yEnd = _numColumn - 1; }

	for (unsigned int xIter = xBegin; xIter <= xEnd; xIter++)
	{
		for (unsigned int yIter = yBegin; yIter <= yEnd; yIter++)
		{
			int idx = (xIter * _numColumn) + yIter;

			for (unsigned int a = 0; a < _squares[idx]->_objects.size(); a++)
			{
				glm::vec2 pt;
				pt.x = _squares[idx]->_objects[a]->_x;
				pt.y = _squares[idx]->_objects[a]->_y;
				closestPts.push_back(pt);
			}

			//nearObjects.insert(nearObjects.end(), _squares[idx]->_objects.begin(), _squares[idx]->_objects.end());

			//std::vector<AnObject*> objs = _squares[(xIter * _numColumn) + yIter]->_objects;
			//nearObjects.insert(nearObjects.end(), objs.begin(), objs.end());
		}
	}
}

std::vector<AnObject*> CollisionGrid::GetObjects(float x, float y)
{
	std::vector<AnObject*> nearObjects;

	if (std::isnan(x) || std::isinf(x) || std::isnan(y) || std::isinf(y))
	{
		return nearObjects;
	}

	if (x < 0 || x > SystemParams::_upscaleFactor || y < 0 || y > SystemParams::_upscaleFactor)
	{
		return nearObjects;
	}

	//int xPos = x / _maxLength;
	//int yPos = y / _maxLength;
	int xPos;
	int yPos;
	GetCellPosition(xPos, yPos, x, y);

	int offst = SystemParams::_collission_block_radius;

	int xBegin = xPos - offst;
	if (xBegin < 0) { xBegin = 0; }

	int xEnd = xPos + offst;
	if (xEnd >= _numColumn) { xEnd = _numColumn - 1; }

	int yBegin = yPos - offst;
	if (yBegin < 0) { yBegin = 0; }

	int yEnd = yPos + offst;
	if (yEnd >= _numColumn) { yEnd = _numColumn - 1; }

	for (unsigned int xIter = xBegin; xIter <= xEnd; xIter++)
	{
		for (unsigned int yIter = yBegin; yIter <= yEnd; yIter++)
		{
			//if (xIter < 0 || xIter >= _numColumn || yIter < 0 || yIter >= _numColumn) { continue; }

			int idx = (xIter * _numColumn) + yIter;

			nearObjects.insert(nearObjects.end(), _squares[idx]->_objects.begin(), _squares[idx]->_objects.end());

		}
	}

	return nearObjects;
}

/*
	check the objects that move to another square.
	If them doesn't contain in previous square, 
	transfer them to the new square.
*/
void CollisionGrid::MovePoints()
{
	std::vector<AnObject*> invalidObjects;
	for (unsigned int a = 0; a < _squares.size(); a++)
	{
		for (int b = _squares[a]->_objects.size() - 1; b >= 0; b--) // should be signed
		{
			if (!_squares[a]->Contains(_squares[a]->_objects[b]))
			{
				invalidObjects.push_back(_squares[a]->_objects[b]);
				_squares[a]->_objects.erase(_squares[a]->_objects.begin() + b);
			}
		}
	}

	for (unsigned int a = 0; a < invalidObjects.size(); a++)
	{
		//int xPos = invalidObjects[a]->_x / _maxLength;
		//int yPos = invalidObjects[a]->_y / _maxLength;
		int xPos;
		int yPos;
		GetCellPosition(xPos, yPos, invalidObjects[a]->_x, invalidObjects[a]->_y);


		// avoiding runtime error
		if (xPos < 0) { xPos = 0; }
		if (xPos == _numColumn) { xPos = _numColumn - 1; }
		if (yPos < 0) { yPos = 0; }
		if (yPos == _numColumn) { yPos = _numColumn - 1; }

		_squares[(xPos * _numColumn) + yPos]->_objects.push_back(invalidObjects[a]);
	}
	invalidObjects.clear();
}

void CollisionGrid::AnalyzeContainer(const std::vector<glm::vec2>& boundaries)
{
	for (unsigned int a = 0; a < _squares.size(); ++a)
	{
		float d1 = UtilityFunctions::DistanceToClosedCurve(boundaries, glm::vec2(_squares[a]->_xCenter, _squares[a]->_yCenter));
		if (d1 > _maxLength)	// not boundary, can contain objects.
		{ 
			_squares[a]->_containerFlag = 1; 
		}
		else    // is boundary
			_squares[a]->_containerFlag = 0;
	}
}

void CollisionGrid::ClearContianer()
{
	for (unsigned int a = 0; a < _squares.size(); ++a)
		_squares[a]->_containerFlag = 1;
}

void CollisionGrid::PrecomputeGraphIndices()
{

	for (unsigned int iter = 0; iter < _squares.size(); iter++)
	{
		GraphIndices gIndices;

		int xPos = iter / _numColumn;
		int yPos = iter - (xPos * _numColumn);

		int offst = SystemParams::_collission_block_radius;

		int xBegin = xPos - offst;
		if (xBegin < 0) { xBegin = 0; }

		int xEnd = xPos + offst;
		if (xEnd >= _numColumn) { xEnd = _numColumn - 1; }

		int yBegin = yPos - offst;
		if (yBegin < 0) { yBegin = 0; }

		int yEnd = yPos + offst;
		if (yEnd >= _numColumn) { yEnd = _numColumn - 1; }

		for (unsigned int xIter = xBegin; xIter <= xEnd; xIter++)
		{
			for (unsigned int yIter = yBegin; yIter <= yEnd; yIter++)
			{
				int idx = (xIter * _numColumn) + yIter;
				for (unsigned int a = 0; a < _squares[idx]->_objects.size(); a++)
				{
					int info1 = _squares[idx]->_objects[a]->_info1;
					if (UtilityFunctions::GetIndexFromIntList(gIndices, info1) == -1)
					{
						gIndices.push_back(info1);
					}
				}
			}
		}
		_squares[iter]->_closestGraphIndices = gIndices;
	}
}
/*
	find the _closestGraphIndices(object indices) in each square.
*/
void CollisionGrid::PrecomputeGraphIndices_ThreadTask(int startIdx, int endIdx)
{
	//_graphIndexArray.clear();
	for (unsigned int iter = startIdx; iter < endIdx; iter++)
	{
		if (iter >= _squares.size()) { break; }

		// typedef std::vector<int> GraphIndices;
		//GraphIndices gIndices;
		std::unordered_set<int> int_set;


		int xPos = iter / _numColumn;
		int yPos = iter - (xPos * _numColumn);

		int offst = SystemParams::_collission_block_radius;

		int xBegin = xPos - offst;
		if (xBegin < 0) { xBegin = 0; }

		int xEnd = xPos + offst;
		if (xEnd >= _numColumn) { xEnd = _numColumn - 1; }

		int yBegin = yPos - offst;
		if (yBegin < 0) { yBegin = 0; }

		int yEnd = yPos + offst;
		if (yEnd >= _numColumn) { yEnd = _numColumn - 1; }

		for (unsigned int xIter = xBegin; xIter <= xEnd; xIter++)
		{
			for (unsigned int yIter = yBegin; yIter <= yEnd; yIter++)
			{
				int idx = (xIter * _numColumn) + yIter;
				for (unsigned int a = 0; a < _squares[idx]->_objects.size(); a++)
				{
					int info1 = _squares[idx]->_objects[a]->_info1;
					/*if (UtilityFunctions::GetIndexFromIntList(gIndices, info1) == -1)
					{
						gIndices.push_back(info1);
					}*/
					if (int_set.find(info1) == int_set.end())
					{
						int_set.insert(info1);
					}
				}
			}
		}

		// stackoverflow.com/questions/42519867/efficiently-moving-contents-of-stdunordered-set-to-stdvector
		_squares[iter]->_closestGraphIndices.clear();
		// copy
		_squares[iter]->_closestGraphIndices.insert(_squares[iter]->_closestGraphIndices.end(),
			int_set.begin(),
			int_set.end());

		// C++17
		/*_squares[iter]->_closestGraphIndices.reserve(int_set.size());
		for (auto it = int_set.begin(); it != int_set.end(); )
		{
			_squares[iter]->_closestGraphIndices.push_back(std::move(int_set.extract(it++).value()));
		}*/
	}
}

bool CollisionGrid::NearBoundary(float x, float y)
{
	int xPos;
	int yPos;
	GetCellPosition(xPos, yPos, x, y);

	return (_squares[(xPos * _numColumn) + yPos]->_containerFlag == 0);
}

void CollisionGrid::ClearObjects()
{
	for (unsigned int a = 0; a < _squares.size(); a++)
	{
		for (int b = _squares[a]->_objects.size() - 1; b >= 0; b--) 
		{
			_squares[a]->_objects.clear();
		}
	}
	_objects.clear();
}
