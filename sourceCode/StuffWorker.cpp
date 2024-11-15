#include "StuffWorker.h"
#include <regex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "SystemParams.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;


std::vector<Object> StuffWorker::_objs = std::vector<Object>();
std::unordered_map<int, int> StuffWorker::_aroundIds = std::unordered_map<int, int>();
CollisionGrid *StuffWorker::_cGrid = 0;

StuffWorker::StuffWorker()
	: _numPoints(0), _numTriangles(0), _numTriEdges(0), _numAuxEdges(0), _sumVelocity(0), _totalRepulsionF(0), _totalEdgeF(0),\
	_totalBoundaryF(0), _totalOverlapF(0), _avgScaleFactor(0), _avgLength(0), _shapeCloudID(1), _showUpNum(0)
{
	StuffWorker::_cGrid = new CollisionGrid();
	_my_threadpool = new ThreadPool(SystemParams::_num_threads);

	// read contour
	//std::cout << SystemParams::_contour_file << std::endl;
	_contour = ContourObject(SystemParams::_contour_file.c_str());

	// check number of shape clouds
	int cnt = 0;
	for (const auto& entry : fs::recursive_directory_iterator(SystemParams::_shapeCloud_dir))
		if (fs::is_directory(entry))
			cnt++;
	SystemParams::_shapeCloud_num = cnt;


	// read all json objects 
	std::vector<std::string>files = LoadFiles(SystemParams::_obj_dir);
	for (unsigned int i = 0; i < files.size(); ++i)
	{
		_objs.push_back(Object(files[i].c_str()));
		_objs[i]._id = i + 1;
	}
	

	// save the contour in collision grid
	_cGrid->AnalyzeContainer(_contour._resampledContour);
	
}

StuffWorker::~StuffWorker()
{
	if (_my_threadpool)
		delete _my_threadpool;

}

std::vector<std::string> StuffWorker::LoadFiles(std::string& dir)
{
	std::vector<std::string> files;
	for (const auto& entry : fs::directory_iterator(dir))
		files.push_back(entry.path().string());
	return files;
}

void StuffWorker::ProcessObject()
{
	// read order list
	std::string list = SystemParams::_shapeCloud_dir + "order.txt";
	std::ifstream file(list);
	std::vector<int> objOrder;
	if (file.is_open())
	{
		std::string line;
		std::regex numberRegex("\\d+");
		while (std::getline(file, line) && !line.empty())
		{
			// id (pos.x pos.y) 
			std::sregex_iterator iter(line.begin(), line.end(), numberRegex);
			std::sregex_iterator end;
			std::vector<int> nums;
			while (iter != end)
			{
				std::smatch match = *iter;
				nums.push_back(std::stoi(match.str()));
				++iter;
			}

			if (nums.size() > 1)
			{
				int id = nums[0];
				objOrder.push_back(id);
				_cuurentObjSet.insert(id);
				_objs[id - 1]._prevPos = glm::vec2(nums[1], nums[2]);
			}
			else
			{
				int id = nums[0];
				objOrder.push_back(id);
				_cuurentObjSet.insert(id);
			}

			_showUpNum++;
		}
		file.close();
	}
	

	// scale and place the objects
	float area = _contour.GetArea();
	float len = sqrt(area);
	float targetLen;

	
	SystemParams::_avgObjSize = sqrt(area / _objs.size());
	std::cout << "avg size : " << SystemParams::_avgObjSize << std::endl;

	SystemParams::_max_size = SystemParams::_avgObjSize * 1.5;
	SystemParams::_max_size = 50.0;
	SystemParams::_min_size = std::max(40.0, SystemParams::_avgObjSize * 0.5);

	float diff = 2.0;
	float step[] = { 2.0, 1.0, 0.5 };

	float step2 = 10;
	int minus = 3, hold = 10;
	float scaleValue;

	SystemParams::_avgObjSize -= 5;
	//targetLen = SystemParams::_avgObjSize * 2.0f;
	targetLen = 110;

	int cnt = 0;
	auto start1 = std::chrono::steady_clock::now(); // timing
	for (unsigned int i = 0; i < objOrder.size();  )
	{
		// get the object id
		int idx = objOrder[i] - 1;

		ARectangle bbox = _objs[idx]._bbox;
		scaleValue = targetLen / bbox.GetLong();

		/*if (scaleValue < SystemParams::_min_scale_value)
			scaleValue = SystemParams::_min_scale_value;*/
		bbox.SetFirstLength(scaleValue);
		cv::Mat img = _objs[idx]._contourMap;

		cv::resize(img, img, cv::Size(), scaleValue, scaleValue);

		glm::vec2 pos(0, 0);

		glm::vec2 prevPos = _objs[idx]._prevPos;

		if (!_contour.Placement3(img, pos, prevPos))
		{
			//std::cout << idx << " object can't be placed!" << std::endl;
			/*if (targetLen >= (initLen * 0.75))
				targetLen -= diff * step[0];
			else if (targetLen >= (initLen * 0.4))
				targetLen -= diff * step[1];
			else
				targetLen -= diff * step[2];*/

			if (targetLen >= SystemParams::_max_size)
				targetLen -= diff * step[0];
			else if (targetLen >= SystemParams::_min_size)
				targetLen -= diff * step[1];
			else
				targetLen -= diff * step[2];
			
		}
		else
		{
			_objs[idx].Scale(scaleValue);
			_objs[idx].Move(pos);
			_objs[idx].RecalculateTriangleEdgeLengths();
			_objs[idx].UpdateBoundaryAndAvgEdgeLength();
			_objs[idx].RecalculateArts();
			
			// flag to controll if show the object.
			_objs[idx]._showUp = true;

			std::cout << "len of " << idx+1 << " : " << targetLen << std::endl;

			if (targetLen > SystemParams::_max_size)
			{
				std::cout << "big id : " << _objs[idx]._id << std::endl;
				_objs[idx]._isBig = true;
				targetLen -= step2;
				step2 -= minus;
				if (step2 <= 0)
					step2 = hold;
				
			}
			else if (targetLen < 40)
			{
				//std::cout << "small id : " << _objs[idx]._id << std::endl;
				_objs[idx]._isSmall = true;
			}
			
			cnt++;
			i++;
		}
	
	}
	auto elapsed1 = std::chrono::steady_clock::now() - start1; // timing
	std::cout << "collage time : " << (float)std::chrono::duration_cast<std::chrono::microseconds>(elapsed1).count() / 1e6 << std::endl;
	std::cout << "number of objects : " << cnt << std::endl;

	// collision grid
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		for (unsigned int j = 0; j < _objs[i]._skinPointNumber; ++j)
		{
			glm::vec2 pt = _objs[i]._massList[j]._pos;
			_cGrid->InsertAPoint(pt.x, pt.y, i, j);
		}
	}

	// assign collision grid
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		for (unsigned int j = 0; j < _objs[i]._massList.size(); ++j)
			_objs[i]._massList[j]._cGrid = _cGrid;
	}

	// update the mass to the rendered vertices.
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		_objs[i].UpdateVertices();
	}

	// data of the objects
	//for (unsigned int i = 0; i < _objs.size(); ++i)
	//{
	//	// show up controll
	//	if (!_objs[i]._showUp)
	//		continue;
	//	_numPoints += _objs[i]._massList.size();
	//	_numTriangles += _objs[i]._triangles.size();
	//	_numTriEdges += _objs[i]._edgeList.size();
	//	_numAuxEdges += _objs[i]._auxiliaryEdge.size();
	//}
	//std::cout << "num points : " << _numPoints << std::endl;
	//std::cout << "num tri : " << _numTriangles << std::endl;
	//std::cout << "num edges : " << _numTriEdges << std::endl;
	//std::cout << "num auxedges : " << _numAuxEdges << std::endl;

	// show the count map
	cv::Mat tmp = cv::Mat::zeros(cv::Size(500, 500), CV_8UC1);
	for (int y = 0; y < 500; ++y)
		for (int x = 0; x < 500; ++x)
		{
			if (_contour._countMap[x][y] != 0)
			{
				int px = (int)_contour._countMap[x][y];
				if (px > 255)
					px = 255;
				tmp.at<uchar>(y, x) = px;
			}
		}
	cv::imshow("1", tmp);
	cv::waitKey(0);

}

void StuffWorker::GLBind()
{
	// bind gl data of objects
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		_objs[i].BindGLData();
		_objs[i].BindTexture();
	}

	// bind gl data of contour
	_contour.BindGLdata();
}

void StuffWorker::UpdateGLData()
{
	int len = _objs.size();
	int num_threads = SystemParams::_num_threads;
	int thread_stride = (len + num_threads - 1) / num_threads;
	_objsArea = 0;
	for (unsigned int i = 0; i < num_threads; ++i)
	{
		int startIdx = i * thread_stride;
		int endIdx = startIdx + thread_stride;
		_my_threadpool->submit(&StuffWorker::UpdateGL_ThreadTask, this, startIdx, endIdx);
	}
	_my_threadpool->waitFinished();	// sync

	// don't multi-thread ! (vao,vbo will mess up)
	for(unsigned int i = 0; i < _objs.size(); ++i)
		_objs[i].BindGLData();
}

void StuffWorker::UpdateGL_ThreadTask(int startIdx, int endIdx)
{
	for (unsigned int i = startIdx; i < endIdx; ++i)
	{
		if (i >= this->_objs.size())
			break;
		// show up
		if (this->_objs[i]._showUp)
		{
			this->_objs[i].UpdateVertices();
			_objsArea += this->_objs[i].GetArea();
		}
	}
}

void StuffWorker::GLDraw(const Shader & shader)
{
	shader.use();
	// objects
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;

		_objs[i].DrawGL(shader);
	}
	
	// contour. don't draw when changing contour.
	//if(!SystemParams::_changeContour)
	//	_contour.Draw(shader);
}


void StuffWorker::Operation(float dt)
{
	
	// ---------- initialization ----------
	CalculateThings(dt);

	// ---------- main simulation ----------
	Finall_ThreadPass(dt);

	// barycentric & render coord.
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		this->_objs[i].RecalculateArts();
	}

	// ---------- position delta ----------
	_sumVelocity = 0;
	_totalRepulsionF = 0.0f;
	_totalEdgeF = 0.0f;
	_totalBoundaryF = 0.0f;
	_totalOverlapF = 0.0f;
	_avgScaleFactor = 0.0f;

	//_avgLength = 0.0;
	// force data
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		_avgScaleFactor += _objs[i]._scale;
		_objs[i].CalculateSumVelocity();
		_sumVelocity += _objs[i]._sumVelocity;
		for (unsigned int j = 0; j < _objs[i]._massList.size(); ++j)
		{
			float rF = glm::length(_objs[i]._massList[j]._repulsionForce);
			float eF = glm::length(_objs[i]._massList[j]._edgeForce);
			float bF = glm::length(_objs[i]._massList[j]._boundaryForce);
			float oF = glm::length(_objs[i]._massList[j]._overlapForce);
			
			_totalRepulsionF += rF;
			_totalEdgeF		 += eF;
			_totalBoundaryF	 += bF;
			_totalOverlapF	 += oF;

		}
	//	_avgLength += this->_objs[i]._bbox.GetLong();
	}       
	/*_avgLength /= _objs.size();
	SystemParams::_avgObjSize = _avgLength;*/

}

void StuffWorker::CalculateThings(float dt)
{
	// --------- collision grid ---------
	int qtIter = 0;
	// update the points in the collision grid.
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (_objs[i]._showUp)
		{
			for (unsigned int j = 0; j < _objs[i]._skinPointNumber; ++j)
			{
				_cGrid->_objects[qtIter]->_x = _objs[i]._massList[j]._pos.x;
				_cGrid->_objects[qtIter]->_y = _objs[i]._massList[j]._pos.y;
				qtIter++;
			}
		}
	}

	// -------------- move points in collision grid -----------------
	_cGrid->MovePoints();

	UpdateCollisionGrid_PrepareThreadPool();
	
	// check whether freeze 
	if (!SystemParams::_freeze)
	{
		// reset growing
		for (unsigned int i = 0; i < _objs.size(); ++i)
		{
			// show up controll
			if (!_objs[i]._showUp)
				continue;

			if (SystemParams::_focus_mode && !SystemParams::_trying)	// when choose a object to enlarge.
			{	
				if(_objs[i]._isFocus)
					_objs[i]._isGrowing = true;
				else if (_objs[i]._aroundFocus && _objs[i]._bbox.GetLong() > 10)
				{
					if(_objs[i]._isShrink)
						_objs[i]._isGrowing = true;
					else
						_objs[i]._isGrowing = false;

					/*_objs[i]._isGrowing = true;
					_objs[i]._isShrink = true;*/
				}
				else
					_objs[i]._isGrowing = false;
			}
			else if (SystemParams::_focus_mode && SystemParams::_trying)
			{
				if (_objs[i]._isFocus || _objs[i]._aroundFocus)
					_objs[i]._isGrowing = true;
				else
					_objs[i]._isGrowing = false;
			}
			else	// normal condition. enlarge all.
			{
					_objs[i]._isGrowing = true;
			}
		}
	}
	else
	{
		for (unsigned int i = 0; i < _objs.size(); ++i)
			_objs[i]._isGrowing = false;
	}


	// growing , no mutithreading.
	float scale_iter = SystemParams::_growth_scale_iter;
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		
		_objs[i].Grow(scale_iter, dt);
	}

	// ---------- update boundary  ----------
	// calculation below needs to finish before closest point queries
	// and it's worthless to be send to threadpool
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		_objs[i].UpdateBoundaryAndAvgEdgeLength();
	}

}

void StuffWorker::UpdateCollisionGrid_PrepareThreadPool()
{
	int len = _cGrid->_squares.size();
	int num_threads = SystemParams::_num_threads;
	int thread_stride = (len + num_threads - 1) / num_threads;
	// separate _cGrid size into num_threads group.
	for (unsigned int i = 0; i < num_threads; ++i)
	{
		int startIdx = i * thread_stride;
		int endIdx = startIdx + thread_stride;
		_my_threadpool->submit(&CollisionGrid::PrecomputeGraphIndices_ThreadTask, _cGrid, startIdx, endIdx);
	}
	_my_threadpool->waitFinished();
}

void StuffWorker::Final_PrepareThreadPool(float dt)
{
	int len = _objs.size();
	int num_threads = SystemParams::_num_threads;
	int thread_stride = (len + num_threads - 1) / num_threads;
	for (unsigned int i = 0; i < num_threads; ++i)
	{
		int startIdx = i * thread_stride;
		int endIdx = startIdx + thread_stride;
		_my_threadpool->submit(&StuffWorker::Final_ThreadTask, this, dt, startIdx, endIdx);
	}
	_my_threadpool->waitFinished();	// sync
}

// a task for a thread
void StuffWorker::Final_ThreadTask(float dt, int startIdx, int endIdx)
{
	for (unsigned int iter = startIdx; iter < endIdx; ++iter)
	{
		if (iter >= _objs.size())
			break;
		
		// show up controll
		if (!_objs[iter]._showUp)
			continue;

		// ----------- reset force to zero ---------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			this->_objs[iter]._massList[i].Init();

		// ----------- get closest point ------------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			_objs[iter]._massList[i].GetClosestPoints(iter);
	
		// ----------- solve spring forces ------------
		if (!SystemParams::_freeze)
		{
			if (SystemParams::_focus_mode && (!_objs[iter]._isFocus) && (!_objs[iter]._aroundFocus))
				continue;

			_objs[iter].SolveForTriangleSprings();
			_objs[iter].SolveForNegativeSPaceSprings();
			// ----------- solve other forces ------------
			for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			{
				this->_objs[iter]._massList[i].Solve(i, _objs[iter], _contour._resampledContour);
			}

			// ----------- intergration ------------
			for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			{
				this->_objs[iter]._massList[i].Simulate(dt);
			}
		}
	}
}

void StuffWorker::Finall_ThreadPass(float dt)
{
	// ------- get closest point -------
	Final_PrepareThreadPool(dt);
}


// ------------------------------- resize stage --------------------------
// set the size that objects should shrink to when resize stage.  
void StuffWorker::SetPreparedSize()
{
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		
		if (_objs[i]._isSmall)
			_objs[i]._isSmall = false;

		if (_objs[i]._fadeOut)	
		{
			_objs[i]._preparedSize = 1;
		}
		else
		{

			if (_objs[i]._bbox.GetLong() > 90)
				_objs[i]._preparedSize = std::min(_objs[i]._bbox.GetLong() - 30, 80.0f);
			else
				_objs[i]._preparedSize = std::min(35.0f, _objs[i]._targetSize);
		}
	}
}

// just resize object to average size.
void StuffWorker::ResizeStage(float dt)
{
	// --------- collision grid ---------
	int objIter = 0;
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (_objs[i]._showUp)
		{
			// update skins of all objects.
			for (unsigned int j = 0; j < _objs[i]._skinPointNumber; ++j)
			{
				_cGrid->_objects[objIter]->_x = _objs[i]._massList[j]._pos.x;
				_cGrid->_objects[objIter]->_y = _objs[i]._massList[j]._pos.y;
				objIter++;
			}
		}
	}

	// -------------- move points in collision grid -----------------
	_cGrid->MovePoints();

	UpdateCollisionGrid_PrepareThreadPool();

	// ------------------ growing stage --------------------------
	// shrink object to assigned size.
	int cnt = 0;	// number of resize objects.
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{	
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		
		float range = (_objs[i]._preparedSize - _objs[i]._bbox.GetLong());
		
		if(range < 0.0f) 		// shrink
		{
			_objs[i]._isGrowing = true;
			_objs[i]._isShrink  = true;
			cnt++;
		}
		else
		{
			_objs[i]._isGrowing = false;
			_objs[i]._isBig = false;
		}
	}
	if (cnt == 0)
	{
		std::cout << "finish resize stage." << std::endl;
		SystemParams::_resizeStage = false;
		SystemParams::_freeze = true;
	}
	// growing , no mutithreading.
	float scale_iter = SystemParams::_growth_scale_iter;
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		if(_objs[i]._fadeOut)
			_objs[i].Grow_Tmp(scale_iter, dt * 4);
		else
			_objs[i].Grow_Tmp(scale_iter, dt*2);
	}
	// update boundary  
	// calculation below needs to finish before closest point queries
	// and it's worthless to be send to threadpool
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		_objs[i].UpdateBoundaryAndAvgEdgeLength();
	}

	// ----------------- solve force stage ---------------------------
	int len = _objs.size();
	int num_threads = SystemParams::_num_threads;
	int thread_stride = (len + num_threads - 1) / num_threads;
	for (unsigned int i = 0; i < num_threads; ++i)
	{
		int startIdx = i * thread_stride;
		int endIdx = startIdx + thread_stride;
		_my_threadpool->submit(&StuffWorker::ThreadTask_Resize, this, dt, startIdx, endIdx);
	}
	_my_threadpool->waitFinished();	// sync


	// barycentric & render coord.
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		if (!_objs[i]._showUp)
			continue;
		this->_objs[i].RecalculateArts();
	}
}

void StuffWorker::ThreadTask_Resize(float dt, int startIdx, int endIdx)
{
	for (unsigned int iter = startIdx; iter < endIdx; ++iter)
	{
		if (iter >= _objs.size())
			break;
		// show up controll
		if (!_objs[iter]._showUp)
			continue;

		// ----------- reset force to zero ---------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			this->_objs[iter]._massList[i].Init();
		// ----------- get closest point ------------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			this->_objs[iter]._massList[i].GetClosestPoints(iter);
		// ----------- solve spring forces ------------
		this->_objs[iter].SolveForTriangleSprings();
		this->_objs[iter].SolveForNegativeSPaceSprings();
		// ----------- solve other forces ------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			this->_objs[iter]._massList[i].Solve(i, _objs[iter], _contour._resampledContour);
		// ----------- intergration ------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			this->_objs[iter]._massList[i].Simulate(dt);		
	}
}

// =================================================================
// ------------------ different simulation mode => when change contour --------------------
// some operation before starting change contour.
void StuffWorker::SetBeforeMove()
{
	// initial object's setting.
	for (unsigned int i = 0; i < _objs.size(); ++i)
		_objs[i].InitController();

	// make the fade out object vanish.
	unsigned int cnt = 0;	// count the number of fade out obj.
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		if (_objs[i]._showUp && _objs[i]._fadeOut)
		{
			cnt++;
			_objs[i]._showUp = false;
			// reset object mass.
			_objs[i].ResetData();
			_objs[i]._preparedSize = 0;
			_objs[i]._fadeOut = false;
			std::cout << "fade out id : " << i + 1 << std::endl;
			// delete from currentObjSet
			_cuurentObjSet.erase(_objs[i]._id); // id : 1 ~ n
		}
		if (_objs[i]._showUp)
		{
			if ( (_objs[i]._bbox.GetLong() > SystemParams::_max_size && _objs[i]._targetSize < SystemParams::_max_size) ||
				(_objs[i]._bbox.GetLong() < SystemParams::_avgObjSize && _objs[i]._targetSize > SystemParams::_max_size)||
				(_objs[i]._bbox.GetLong() > _objs[i]._targetSize && (_objs[i]._bbox.GetLong() - _objs[i]._targetSize > 20.0) ) )
			{
				std::cout << "extreme diff id : " << i + 1 << std::endl;

				_objs[i]._extremeDiff = true;
			}
		}
	}

	if (cnt == 0)
		return;

	// reset grid.
	_cGrid->ClearObjects();
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		for (unsigned int j = 0; j < _objs[i]._skinPointNumber; ++j)
		{
			glm::vec2 pt = _objs[i]._massList[j]._pos;
			_cGrid->InsertAPoint(pt.x, pt.y, i, j);
		}
	}

	// assign collision grid
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		if (!_objs[i]._showUp)
			continue;
		for (unsigned int j = 0; j < _objs[i]._massList.size(); ++j)
			_objs[i]._massList[j]._cGrid = _cGrid;
	}

}

// add the fade-in objects, when press "x" to resize objects to target size. 
void StuffWorker::FadeInNewObjs()
{
	unsigned int cnt = 0;	// count the number of fade in obj.
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		if (!_objs[i]._showUp && _objs[i]._fadeIn)
		{
			cnt++;
			_objs[i]._showUp = true;
			_cuurentObjSet.insert(_objs[i]._id);	// save the obj's id.
		
		}
	}
	if (cnt == 0)
		return;

	// update grid.
	_cGrid->ClearObjects();
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		for (unsigned int j = 0; j < _objs[i]._skinPointNumber; ++j)
		{
			glm::vec2 pt = _objs[i]._massList[j]._pos;
			_cGrid->InsertAPoint(pt.x, pt.y, i, j);
		}
	}

	// assign collision grid
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		if (!_objs[i]._showUp)
			continue;
		for (unsigned int j = 0; j < _objs[i]._massList.size(); ++j)
			_objs[i]._massList[j]._cGrid = _cGrid;
	}

}

void StuffWorker::Operation_Move(float dt)
{
	
	CalculateThings_Move(dt);

	Finall_ThreadPass_Move(dt);	

	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		if (i >= _objs.size())
			break;
		// show up controll
		if (!_objs[i]._showUp)
			continue;

		this->_objs[i]._lastPosition = this->_objs[i]._massCenter;
		this->_objs[i].RecalculateArts();
	}	
}


void StuffWorker::Updata_ThreadTask(int startIdx, int endIdx)
{
	for (unsigned int iter = startIdx; iter < endIdx; ++iter)
	{
		if (iter >= _objs.size())
			break;
		// show up controll
		if (!_objs[iter]._showUp)
			continue;

		this->_objs[iter]._lastPosition = this->_objs[iter]._massCenter;
		this->_objs[iter].RecalculateArts();
	}
}

void StuffWorker::CalculateThings_Move(float dt)
{
	// --------- collision grid ---------
	int qtIter = 0;
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		// update skins of all objects.
		for (unsigned int j = 0; j < _objs[i]._skinPointNumber; ++j)
		{
			_cGrid->_objects[qtIter]->_x = _objs[i]._massList[j]._pos.x;
			_cGrid->_objects[qtIter]->_y = _objs[i]._massList[j]._pos.y;
			qtIter++;
		}
	}

	// -------------- move points in collision grid -----------------
	_cGrid->MovePoints();

	UpdateCollisionGrid_PrepareThreadPool();
	
	if (SystemParams::_resize)	// resize to target size
	{
		for (unsigned int i = 0; i < _objs.size(); ++i)
		{
			// show up controll
			if (!_objs[i]._showUp)
				continue;

			if (_objs[i]._completeResize)
			{
				_objs[i]._isGrowing = false;
				continue;
			}

			_objs[i]._speedUp = false;

			float range = (_objs[i]._targetSize - _objs[i]._bbox.GetLong());
			
			if (range > 0.5)	// enlarge
			{
				_objs[i]._isGrowing = true;
				if (range > 20.0f)
					_objs[i]._speedUp = true;
			}
			else if (range < -0.5)	// shrink
			{
				_objs[i]._isGrowing = true;
				_objs[i]._isShrink = true;
				if (range < -20.0f)
					_objs[i]._speedUp = true;
			}
			else //if (range > -0.5f && range < 0.5f)	// size ok
			{
				_objs[i]._completeResize = true;
				std::cout << i + 1 << " complete" << std::endl;
				_objs[i]._isGrowing = false;
			}
		}
	}
	else    // resize when moving
	{
		for (unsigned int i = 0; i < _objs.size(); ++i)
		{
			// show up controll
			if (!_objs[i]._showUp)
				continue;

			if (_objs[i]._extremeDiff)
			{
				// extremeDiff obj will converge to avg size.
				float range = ((SystemParams::_avgObjSize) - _objs[i]._bbox.GetLong());

				if (range > 0.5)	// enlarge
				{
					_objs[i]._isGrowing = true;
				}
				else if (range < -0.5)	// shrink
				{
					_objs[i]._isGrowing = true;
					_objs[i]._isShrink = true;
				}
				else	// size ok
				{
					_objs[i]._isGrowing = false;
					_objs[i]._extremeDiff = false;
				}
				continue;
			}
			_objs[i]._isGrowing = false;
		}
	}

	// growing , no mutithreading.
	float scale_iter = SystemParams::_growth_scale_iter;
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		if (SystemParams::_resize)
		{
			if (_objs[i]._completeResize)
				continue;
			if(_objs[i]._fadeIn)
				_objs[i].Grow_Move(scale_iter, dt * 8);
			else
			{
				if(!_objs[i]._speedUp)
					_objs[i].Grow_Move(scale_iter, dt);
				else
					_objs[i].Grow_Move(scale_iter, dt * 2.0);
			}
		}
		else
			_objs[i].Grow_Move(scale_iter, dt);
	}

	// ---------- update boundary  ----------
	// calculation below needs to finish before closest point queries
	// and it's worthless to be send to threadpool
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// show up controll
		if (!_objs[i]._showUp)
			continue;
		_objs[i].UpdateBoundaryAndAvgEdgeLength();
	}

}

void StuffWorker::Finall_ThreadPass_Move(float dt)
{
	int len = _objs.size();
	int num_threads = SystemParams::_num_threads;
	int thread_stride = (len + num_threads - 1) / num_threads;
	for (unsigned int i = 0; i < num_threads; ++i)
	{
		int startIdx = i * thread_stride;
		int endIdx = startIdx + thread_stride;
		_my_threadpool->submit(&StuffWorker::Final_ThreadTask_Move, this, dt, startIdx, endIdx);
	}
	_my_threadpool->waitFinished();	// sync
}

void StuffWorker::Final_ThreadTask_Move(float dt, int startIdx, int endIdx)
{
	for (unsigned int iter = startIdx; iter < endIdx; ++iter)
	{
		// show up controll
		if (!_objs[iter]._showUp)
			continue;

		if (iter >= _objs.size())
			break;

		// freeze the completed objects.
		/*if (_objs[iter]._completeResize && _objs[iter]._completeShift)
			continue;*/

		// ------------- reset force to zero ---------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
		{
			this->_objs[iter]._massList[i].Init();
			// trying
			this->_objs[iter]._stuck = false;
			
			//this->_objs[iter]._completeShift = false;
		}
		// ------------- get closest point ------------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			this->_objs[iter]._massList[i].GetClosestPoints(iter);

		// ------------- solve spring forces ----------------
		this->_objs[iter].SolveForTriangleSprings();
		this->_objs[iter].SolveForNegativeSPaceSprings();
		
		// ------------- solve other forces -----------------	
		
		// check whether complete. According to the distance between target position and mass center.
		if (!_objs[iter]._completeShift && glm::length(_objs[iter]._targetPosition - _objs[iter]._massCenter) < 5.0f)
		{
			this->_objs[iter]._completeShift = true;
			this->_objs[iter]._stuck = false;
		}

		// check the distance between now- and last- massCenter position.
		if (!_objs[iter]._completeShift && (glm::length(_objs[iter]._lastPosition - _objs[iter]._massCenter) < 0.1f))
			this->_objs[iter]._stuck = true;

		glm::vec2 dis = _objs[iter]._targetPosition - _objs[iter]._massCenter;

		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			//this->_objs[iter]._massList[i].Solve_Move(i, _objs[iter], _contour._resampledContour, _objs[iter]._targetPosition);
			this->_objs[iter]._massList[i].Solve_Move(i, _objs[iter], _contour._resampledContour, dis);
		// ----------- intergration ------------
		for (unsigned int i = 0; i < _objs[iter]._massList.size(); ++i)
			this->_objs[iter]._massList[i].Simulate_Move(dt);
		
	}
}

void StuffWorker::UpdateGrid()
{
	int check = 0;
	for (unsigned int i = 0; i < _contour._resampledContour.size(); ++i)
	{
		_contour._resampledContour[i] -= glm::normalize(_contour._resampledContour[i] - _newContour._resampledContour[i]) * 0.04f;
		if (glm::length(_contour._resampledContour[i] - _newContour._resampledContour[i]) > 0.1f)
		{
			check++;
			_contour._vertices[i] = glm::vec4(_contour._resampledContour[i], 0.0, 1.0);
		}
	}
	if (check == 0 && !SystemParams::_contourMorphingEnd)
	{
		std::cout << "contour morphing end" << std::endl;
		_contour.Copy(_newContour);
		//_newContour.~ContourObject();
		SystemParams::_contourMorphingEnd = true;
	}
	_contour.BindGLdata();
	_cGrid->AnalyzeContainer(_contour._resampledContour);


	// assign collision grid
	int len = _objs.size();
	int num_threads = SystemParams::_num_threads;
	int thread_stride = (len + num_threads - 1) / num_threads;
	for (unsigned int i = 0; i < num_threads; ++i)
	{
		int startIdx = i * thread_stride;
		int endIdx = startIdx + thread_stride;
		_my_threadpool->submit(&StuffWorker::UpdateGrid_ThreadTask, this, startIdx, endIdx);
	}
	_my_threadpool->waitFinished();	// sync
}

void StuffWorker::UpdateGrid_ThreadTask(int startIdx, int endIdx)
{
	for (unsigned int iter = startIdx; iter < endIdx; ++iter)
	{
		if (iter >= _objs.size())
			break;
		// show up controll
		if (!_objs[iter]._showUp)
			continue;

		for (unsigned int j = 0; j < this->_objs[iter]._massList.size(); ++j)
			this->_objs[iter]._massList[j]._cGrid = _cGrid;
	}
}


void StuffWorker::SaveTargetObject()
{
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		if(_objs[i]._showUp)
			_objs[i].SaveTargetData();
	}
}

void StuffWorker::ReadTargetData()
{
	// decide the directory
	_shapeCloudID++;
	if (_shapeCloudID > SystemParams::_shapeCloud_num)
	{
		std::cout << " No more contour !!!" << std::endl;
		return;
	}

	//std::string dir = SystemParams::_target_dir;
	std::string dir = SystemParams::_shapeCloud_dir + std::to_string(_shapeCloudID);
	
	// get json files
	std::vector<std::string> files;
	for (const auto& entry : fs::directory_iterator(dir))
	{
		//std::cout << entry.path().filename() << std::endl;
		files.push_back(dir + "\\" + entry.path().filename().string() );
	}
	/*for (auto st : files)
		std::cout << st << std::endl;*/

	
	for (unsigned int i = 0; i < _objs.size(); ++i)
	{
		// set it for all objs.
		_objs[i]._fadeIn = false;
		//_objs[i]._fadeOut = true;

		if (!_objs[i]._showUp)
			continue;
		// set it only for show up objs. 
		_objs[i]._fadeOut = true;
	}

	// match data between target object and current object
	for (unsigned int i = 0; i < files.size(); ++i)
	{
		std::ifstream input(files[i]);
		json jsn;
		input >> jsn;
		input.close();
		int idx = jsn.at("obj_id").get<int>() - 1;	// idx : 0 ~ n-1
		
		glm::vec2 targetPos = glm::vec2(jsn.at("targetPos_x").get<float>(), jsn.at("targetPos_y").get<float>());
		float targetSz = jsn.at("targetSize").get<float>();
		assert(jsn.at("torsion").size() == _objs[idx]._targetNorm.size());

		if (_cuurentObjSet.count(_objs[idx]._id) == 0)	// isn't in the current set, cuurentObjSet save the id of object (1 ~ n)  
		{
			_objs[idx]._fadeIn = true;
			_objs[idx]._fadeOut = false;
			// set the state when it fade in.
			_objs[idx].Scale( 10.0 / _objs[idx]._bbox.GetLong());
			_objs[idx].Move(targetPos);
			_objs[idx].RecalculateTriangleEdgeLengths();
			_objs[idx].UpdateBoundaryAndAvgEdgeLength();
			_objs[idx].RecalculateArts();
			// set target data
			_objs[idx]._targetPosition = targetPos;
			_objs[idx]._targetSize = targetSz;
			for (unsigned int i = 0; i < jsn.at("torsion").size(); ++i)
			{
				std::vector<float> norm = jsn.at("torsion")[i].get<std::vector<float>>();
				_objs[idx]._targetNorm[i] = glm::vec2(norm[0], norm[1]);
			}
			std::cout << "fade-in obj " << _objs[idx]._id << ", size " << _objs[idx]._bbox.GetLong() << std::endl;
			std::cout << "--------- target size : " << _objs[idx]._targetSize << " --- prepare size : " << _objs[idx]._preparedSize << std::endl;

		}
		else 
		{
			_objs[idx]._fadeOut = false;
			_objs[idx]._targetPosition = targetPos;
			_objs[idx]._targetSize = targetSz;
			for (unsigned int i = 0; i < jsn.at("torsion").size(); ++i)
			{
				std::vector<float> norm = jsn.at("torsion")[i].get<std::vector<float>>();
				_objs[idx]._targetNorm[i] = glm::vec2(norm[0], norm[1]);
			}
		}
	}
	
	
	// --- contour ---
	std::string contourDir = SystemParams::_data_set + "contour\\contour00" + std::to_string(_shapeCloudID) + ".json";
	std::cout << contourDir << std::endl;
	_newContour = ContourObject(contourDir);
	assert(_newContour._resampledContour.size() == _contour._resampledContour.size());
}
