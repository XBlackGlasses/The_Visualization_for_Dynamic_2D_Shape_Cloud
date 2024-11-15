#include "AMass.h"
#include "Object.h"

#include "StuffWorker.h"
#include "UtilityFunctions.h"
#include "SystemParams.h"

AMass::AMass()
{
	this->_pos = glm::vec2(0, 0);
	this->_idx = -1;
	CallByConstructor();
}

AMass::AMass(const float & x, const float & y)
{
	this->_pos = glm::vec2(x, y);
	this->_idx = -1;
	CallByConstructor();
}

AMass::AMass(const glm::vec2 & pos)
{
	this->_pos = pos;
	this->_idx = -1;
	CallByConstructor();
}

void AMass::CalculateIndicesOfLineSegment(const int & numSkin)
{
	_segmentIndices = std::vector<int>(3);
	_segmentIndices[1] = _idx;
	_segmentIndices[0] = _idx - 1;
	_segmentIndices[2] = _idx + 1;
	if (_segmentIndices[0] < 0) { _segmentIndices[0] = numSkin - 1; }
	if (_segmentIndices[2] == numSkin) { _segmentIndices[2] = 0; }

	_lineSegment = std::vector<glm::vec2>(3);
}

bool AMass::TryToAddTriangleEdge(InedexEdge anEdge, const std::vector<AMass>& otherMass)
{
	if (!FindTriEdge(anEdge))
	{
		glm::vec2 pt1 = otherMass[anEdge._index1]._pos;
		glm::vec2 pt2 = otherMass[anEdge._index2]._pos;
		anEdge.SetDist(glm::distance(pt1, pt2));

		// add tri edge
		if (anEdge._index1 != _idx)
			anEdge.Swap();
		_triEdges.push_back(anEdge);

		// neighbor map
		_neighbors.insert({ anEdge._index2, anEdge._index2 });
	}

	return false;
}

bool AMass::FindTriEdge(const InedexEdge & anEdge)
{
	for (unsigned int i = 0; i < _triEdges.size(); ++i)
	{
		if (_triEdges[i]._index1 == anEdge._index1 && _triEdges[i]._index2 == anEdge._index2)
			return true;
		if (_triEdges[i]._index2 == anEdge._index1 && _triEdges[i]._index1 == anEdge._index2)
			return true;
	}
	return false;
}

bool AMass::IsNeighbor(int idx)
{
	auto search = _neighbors.find(idx);
	return (search != _neighbors.end());
}

void AMass::Grow(const float & growth_scale_iter, const float & dt)
{
	this->_mass += growth_scale_iter * dt;

	for (unsigned int i = 0; i < _triEdges.size(); ++i)
		_triEdges[i].MakeLonger(growth_scale_iter, dt);
}

void AMass::Init()
{
	this->_edgeForce = glm::vec2(0, 0);
	this->_repulsionForce = glm::vec2(0, 0);
	this->_boundaryForce = glm::vec2(0, 0);
	this->_overlapForce = glm::vec2(0, 0);
	this->_rotationForce = glm::vec2(0, 0);
	this->_selfIntersectForce = glm::vec2(0, 0);
	this->_translate = glm::vec2(0, 0);
}

void AMass::Simulate(float dt)
{

	_velocity += ((_edgeForce +
		_repulsionForce +
		_boundaryForce +
		_overlapForce +
		_rotationForce +
		_selfIntersectForce) * dt);

	float len = glm::length(_velocity);

	
	float capVal = SystemParams::_velocity_cap * dt;

	if (len > capVal)
		_velocity = glm::normalize(_velocity) * capVal;

	_pos = _pos + _velocity * dt;
}

void AMass::Solve(const int& massNumber, const Object& parentGraph, const std::vector<glm::vec2>& boundary)
{

	if (massNumber < parentGraph._skinPointNumber)	// this mean the point is skinpoint
	{
		if (_n_closest_elems > 0)
		{
			if (_isInside)
			{
				// ----------- OVERLAP FORCE ------------
				glm::vec2 sumO(0, 0);
				glm::vec2 ctrPt;
				glm::vec2 dir;
				for (unsigned int i = 0; i < _triangles.size(); ++i)
				{
					ctrPt = (parentGraph._massList[_triangles[i].idx1].GetPos() +
						parentGraph._massList[_triangles[i].idx2].GetPos() +
						parentGraph._massList[_triangles[i].idx3].GetPos()) / 3.0f;
					dir = ctrPt - _pos;
					sumO += dir;
				}
				sumO *= SystemParams::_k_overlap;
				if (!(std::isinf(sumO.x) || std::isnan(sumO.x) || std::isinf(sumO.y) || std::isnan(sumO.y)))
					this->_overlapForce += sumO;

			}
			else
			{
				// ---------- REPULSION FORCE ----------- 
				glm::vec2 sumR(0, 0);
				glm::vec2 dir;
				for (int i = 0; i < _closestPt_fill_sz; ++i)
				{
					dir = _pos - _closestPoints[i];
					float dist = glm::length(dir);

					sumR += glm::normalize(dir) / (SystemParams::_repulsion_soft_factor + std::pow(dist, 2));
				}
				sumR *= SystemParams::_k_repulsion;

				if (!(std::isinf(sumR.x) || std::isnan(sumR.x) || std::isinf(sumR.y) || std::isnan(sumR.y)))
					this->_repulsionForce += sumR;

			}
		}
	}

	bool nearBoundary = _cGrid->NearBoundary(_pos.x, _pos.y);
	if (nearBoundary)
	{
		// -------- BOUNDARY FORCE --------
		float k_boundary = SystemParams::_k_boundary;

		if (!UtilityFunctions::InsidePolygon(boundary, _pos.x, _pos.y))
		{
			// only one boundary
			glm::vec2 cPt = UtilityFunctions::GetClosestPtOnClosedCurve(boundary, _pos);

			glm::vec2 dirDist = cPt - _pos;
			glm::vec2 bForce = dirDist * k_boundary;
			if (!(std::isinf(bForce.x) || std::isnan(bForce.x) || std::isinf(bForce.y) || std::isnan(bForce.y)))
				this->_boundaryForce += bForce;
			
		}
	}

}


// only for moving mode
void AMass::Simulate_Move(float dt)
{
	_velocity += ((_edgeForce +
		_repulsionForce +
		_boundaryForce +
		_overlapForce +
		_rotationForce +
		_selfIntersectForce +
		_translate) * dt);

	float len = glm::length(_velocity);

	float capVal = SystemParams::_velocity_cap * dt;

	if (len > capVal)
		_velocity = glm::normalize(_velocity) * capVal;

	_pos = _pos + _velocity * dt;
}

// only for moving mode
void AMass::Solve_Move(const int & massNumber, const Object & parentGraph, const std::vector<glm::vec2>& boundary, glm::vec2& targetPos)
{
	glm::vec2 bigforce(0, 0);
	if (massNumber < parentGraph._skinPointNumber)	// this mean the point is skinpoint
	{
		if (_n_closest_elems > 0)
		{
			if (_isInside)
			{
				// ----------- OVERLAP FORCE ------------
				glm::vec2 sumO(0, 0);
				glm::vec2 ctrPt;
				glm::vec2 dir;
				for (unsigned int i = 0; i < _triangles.size(); ++i)
				{
					ctrPt = (parentGraph._massList[_triangles[i].idx1].GetPos() +
						parentGraph._massList[_triangles[i].idx2].GetPos() +
						parentGraph._massList[_triangles[i].idx3].GetPos()) / 3.0f;
					dir = ctrPt - _pos;
					sumO += dir;
				}
				sumO *= SystemParams::_k_overlap;
				
				
				if (!(std::isinf(sumO.x) || std::isnan(sumO.x) || std::isinf(sumO.y) || std::isnan(sumO.y)))
					this->_overlapForce += sumO;
			}
			else
			{
				// ---------- REPULSION FORCE ----------- 
				glm::vec2 sumR(0, 0);
				glm::vec2 dir;
				for (int i = 0; i < _closestPt_fill_sz; ++i)
				{
					dir = _pos - _closestPoints[i];
					float dist = glm::length(dir);
					
					if(_bigNeighbor[i])
						bigforce += glm::normalize(dir) / (SystemParams::_repulsion_soft_factor + std::pow(dist, 2));
					
					sumR += glm::normalize(dir) / (SystemParams::_repulsion_soft_factor + std::pow(dist, 2));
				}
				/*bigforce *= SystemParams::_k_repulsion;
				sumR *= SystemParams::_k_repulsion;
				
				if (!parentGraph._completeShift)
					sumR *= 0.3;
				if (!(std::isinf(sumR.x) || std::isnan(sumR.x) || std::isinf(sumR.y) || std::isnan(sumR.y)))
					this->_repulsionForce += sumR;*/


				bigforce *= SystemParams::_k_repulsion;
				sumR *= SystemParams::_k_repulsion;
				sumR += bigforce;
				if (!parentGraph._completeShift)
					sumR *= 0.3;
				if (SystemParams::_tuningPos)
					sumR *= 0.05;
				if (!(std::isinf(sumR.x) || std::isnan(sumR.x) || std::isinf(sumR.y) || std::isnan(sumR.y)))
					this->_repulsionForce += sumR;
			}
		}
	}

	bool nearBoundary = _cGrid->NearBoundary(_pos.x, _pos.y);
	bool outBoundary = false;
	if (nearBoundary)
	{
		// -------- BOUNDARY FORCE --------
		float k_boundary = SystemParams::_k_boundary;

		if (!UtilityFunctions::InsidePolygon(boundary, _pos.x, _pos.y))
		{
			outBoundary = true;
			// only one boundary.
			glm::vec2 cPt = UtilityFunctions::GetClosestPtOnClosedCurve(boundary, _pos);
			glm::vec2 dirDist = cPt - _pos;
			glm::vec2 bForce = dirDist * k_boundary;
			if (!(std::isinf(bForce.x) || std::isnan(bForce.x) || std::isinf(bForce.y) || std::isnan(bForce.y)))
				this->_boundaryForce += bForce;
		}
	}

	// trying translate force
	//float dist = glm::length(targetPos - _pos);
	float dist = glm::length(targetPos);

	glm::vec2 force(0, 0);
	glm::vec2 center(250, 250);
	//force = glm::normalize(targetPos - _pos) / SystemParams::_upscaleFactor * (dist + 2.0f);
	force = glm::normalize(targetPos) / SystemParams::_upscaleFactor * (dist + 3.0f);
	
	if(parentGraph._isBig)
		force *= 1.3f;

	if (!parentGraph._completeShift && dist < 10.0f)
	{
		force += (-_repulsionForce) + 2.0f;
	}

	/*if (parentGraph._completeShift)
		force *= 1.2f;*/
	if (parentGraph._stuck)
	{
		if (nearBoundary)
		{
			glm::vec2 forec2center = glm::normalize(center - parentGraph._massCenter) * 0.1f;

			force += _boundaryForce + forec2center;
		}
		
		if (!parentGraph._isBig)
		{
			/*if (parentGraph._massCenter.y > 250)
				force = glm::vec2(force.x, -std::abs(force.x)) * 0.2f;
			else 
				force = glm::vec2(force.x, std::abs(force.x)) * 0.2f;*/
			force += (bigforce) + (_repulsionForce * 0.3f);
		}
		else
		{
			force += glm::vec2(force.y, -force.x) * 0.5f;
		}
	}

	// trying fix position
	if (SystemParams::_tuningPos && ! SystemParams::_resize)
	{
		if(parentGraph._completeShift)
			force = glm::normalize(targetPos) * 0.003f * dist;
		else
			force = glm::normalize(targetPos);
		
		if (parentGraph._stuck)
		{	
			if (nearBoundary)
			{
				glm::vec2 forec2center = glm::normalize(center - parentGraph._massCenter);

				force += _boundaryForce*10.0f + forec2center;
			}
			force += (_repulsionForce + _boundaryForce) * 0.5f;
		}
			//force += glm::vec2(force.y, -force.x) * 0.5f;
	}
	else if(SystemParams::_tuningPos && SystemParams::_resize)
		force = glm::normalize(targetPos) * 0.1f;
	

	if (!(std::isinf(force.x) || std::isnan(force.x) || std::isinf(force.y) || std::isnan(force.y)))
		this->_translate += force;
	
}

void AMass::GetClosestPoints(const int &parentGraphIndex)
{
	if (parentGraphIndex < 0 || parentGraphIndex >= StuffWorker::_objs.size())
		return;
	// only check skin points
	if (this->_idx >= StuffWorker::_objs[parentGraphIndex]._skinPointNumber)
		return;

	this->_closestPt_fill_sz = 0;
	this->_isInside = false;
	this->_n_closest_elems = 0;

	GraphIndices* _closestGraphIndices;
	_closestGraphIndices = StuffWorker::_cGrid->GetGraphIndicesPtr(_pos.x, _pos.y, parentGraphIndex);

	this->_n_closest_elems = _closestGraphIndices->size();

	if (_closestGraphIndices->size() > 0)
	{
		std::vector<bool> insideGraphFlags;
		int sz = _closestGraphIndices->size();
		// check the point whether inside other graphs.
		for (unsigned int i = 0; i < sz; ++i)
		{
			// fake!!!
			if ((*_closestGraphIndices)[i] == parentGraphIndex)
			{
				insideGraphFlags.push_back(true); continue;
			}

			// check the objects around the focus object. 
			if (StuffWorker::_objs[parentGraphIndex]._isFocus)
			{
				//std::cout << "id " << parentGraphIndex << std::endl;
				int id = (*_closestGraphIndices)[i];

				if (!StuffWorker::_objs[id]._isFocus && StuffWorker::_aroundIds.find(id) == StuffWorker::_aroundIds.end())
				{
					std::cout << "around id : " << id + 1 << std::endl;
					StuffWorker::_objs[id]._aroundFocus = true;
					StuffWorker::_aroundIds.insert(std::make_pair(id, id));
				}
			}


			if (UtilityFunctions::InsidePolygon(StuffWorker::_objs[(*_closestGraphIndices)[i]]._skin, _pos.x, _pos.y))
			{

				insideGraphFlags.push_back(true);
				_isInside = true;
				//std::cout << "inside\n";
				continue;  // can be more than one
			}
			else
			{
				insideGraphFlags.push_back(false);
			}
		}

		// closest points
		int sz2 = sz;
		if (sz2 > _closestPt_actual_sz)
			sz2 = _closestPt_actual_sz;

		for (unsigned int i = 0; i < sz2; ++i)
		{
			if (insideGraphFlags[i])
				continue;
			glm::vec2 pt = UtilityFunctions::GetClosestPtOnClosedCurve(StuffWorker::_objs[(*_closestGraphIndices)[i]]._skin, _pos);
			_closestPoints[_closestPt_fill_sz] = pt;
			// try to record the big object.
			if (StuffWorker::_objs[(*_closestGraphIndices)[i]]._isBig)
				_bigNeighbor[_closestPt_fill_sz] = true;
			else
				_bigNeighbor[_closestPt_fill_sz] = false;

			_closestPt_fill_sz++;
		}
	}

	// this is used in AGraph(object)
	_closestDist = std::numeric_limits<float>::max();
	for (unsigned int i = 0; i < _closestPt_fill_sz; ++i)
	{
		float d = glm::distance(_closestPoints[i], _pos);
		d *= d;
		if (d < _closestDist)
			_closestDist = d;
	}
	
	_closestDist = std::sqrt(_closestDist);
}

void AMass::CallByConstructor()
{
	_closestPt_fill_sz = 0;
	_closestPt_actual_sz = 50;
	_closestPoints = std::vector<glm::vec2>(_closestPt_actual_sz);
	_bigNeighbor = std::vector<bool>(_closestPt_actual_sz, false);

	_distToBoundary = 0.0f;

	_mass = 1.0f;

	_velocity = glm::vec2(0, 0);

	this->_avgEdgeLength = 0.0f;

	this->_edgeForce = glm::vec2(0, 0);
	this->_repulsionForce = glm::vec2(0, 0);
	this->_boundaryForce = glm::vec2(0, 0);
	this->_overlapForce = glm::vec2(0, 0);
	this->_rotationForce = glm::vec2(0, 0);
	this->_selfIntersectForce = glm::vec2(0, 0);
	this->_translate = glm::vec2(0, 0);
}
