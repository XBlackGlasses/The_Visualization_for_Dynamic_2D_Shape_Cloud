#ifndef STUFF_WORKER_H
#define STUFF_WORKER_H

#include <iostream>

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include "shader.h"
#include "Object.h"
#include "ContourObject.h"
#include "CollisionGrid.h"
#include "ThreadPool.h"

#include <set>
/*
--- Operate the simulation of objects and contour. 
--- some tasks of drawing.
*/
class StuffWorker
{
public:
	static std::vector<Object> _objs;	// object
	static std::unordered_map<int, int> _aroundIds; // record the objects id that around the focus object
	// collision grid
	static CollisionGrid *_cGrid;
	
	ContourObject _contour;
	ContourObject _newContour;

	ThreadPool *_my_threadpool;

	int _numPoints;
	int _numTriangles;
	int _numTriEdges;
	int _numAuxEdges;
	int _shapeCloudID;	// which shape cloud.

	int _showUpNum;

	float _sumVelocity; // _positionDelta

	float _totalRepulsionF;
	float _totalEdgeF;
	float _totalBoundaryF;
	float _totalOverlapF;
	float _avgScaleFactor;
	float _avgLength;
	float _objsArea;
	// save the current object's(show up) id
	std::set<int> _cuurentObjSet;

public:
	StuffWorker();
	~StuffWorker();

	std::vector<std::string> LoadFiles(std::string& dir);

	// process object like scale and to place in the contour
	void ProcessObject();

	// draw objects and contour
	void GLBind();
	void UpdateGLData();
	void UpdateGL_ThreadTask(int startIdx, int endIdx);
	void GLDraw(const Shader &shader);
	// release GL data.
	inline void ReleaseGL()
	{
		for (unsigned int i = 0; i < _objs.size(); ++i)
			_objs[i].ReleaseGL();

		//_contour.ReleaseGL();
	}

	// Send tasks to threadpool
	void UpdateCollisionGrid_PrepareThreadPool();
	void Final_PrepareThreadPool(float dt);
	// a task for a thread
	void Final_ThreadTask(float dt, int startIdx, int endIdx);

	// ---------------- physics simulation ------------------
	void Operation(float dt);
	void Finall_ThreadPass(float dt);  
	void CalculateThings(float dt);

	// ------------resize stage ----------------
	void SetPreparedSize();
	void ResizeStage(float dt);
	void ThreadTask_Resize(float dt, int startIdx, int endIdx);

	// ------------ for simulate between different contours. -------------
	void SetBeforeMove();
	void FadeInNewObjs();

	void Operation_Move(float dt);
	void CalculateThings_Move(float dt);
	void Finall_ThreadPass_Move(float dt);
	void Final_ThreadTask_Move(float dt, int startIdx, int endIdx);
	
	// xxxxxxxxxxxxxxxx trying
	void Updata_ThreadTask(int startIdx, int endIdx);

	// save the data after simulation.
	void SaveTargetObject();
	// read the data in the second contour.
	void ReadTargetData();

	void UpdateGrid();
	void UpdateGrid_ThreadTask(int startIdx, int endIdx);
};

#endif // !STUFF_WORKER_H
