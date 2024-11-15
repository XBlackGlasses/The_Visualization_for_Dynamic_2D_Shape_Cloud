#ifndef SYSTEMPARAMS
#define SYSTEMPARAMS

#include <string>

class SystemParams
{
public:
	SystemParams();
	
	static float _dt;

	static int _screen_width;
	static int _screen_height;

	// controll the operation to save the GL window.
	static bool _save_sreen;
	static std::string _video_name;

	/*
	--- artboard dimension (do not edit this)
	--- the parameter below means the artboard size is 500x500
	*/
	static float _upscaleFactor;
	static float _downscaleFactor;

	static float _min_scale_size;
	
	/*
	--- for growing
	*/
	// incremetal step for growing
	static float _growth_scale_iter;
	// epsilon for halting the growth
	static float _growth_min_dist;

	// Grid for collision detection size of a cell
	static float _bin_square_size;
	/*
	--- cell gap for detection, 
	--- 1 means considering all cells that are 1 block away from the query (3x3)
	--- 2 means considering all cells that are 2 block away from the query (5x5)
	*/
	static int _collission_block_radius;

	// minimun scale value to scale object
	static float _min_scale_value;
	// the max/min size that a object can resize to.
	static float _max_size;
	static float _min_size;

	// thread
	static int _num_threads;

	// file directory
	static int _shapeCloud_num;
	static std::string _data_set;
	static std::string _obj_dir;
	static std::string _contour_file;
	static std::string _shapeCloud_dir;
	static std::string _target_dir;
	static std::string _skeleton_path;


	static float _self_intersection_threshold;

	// force params
	static float _k_edge;
	static float _k_rotate;
	static float _k_neg_space_edge;
	static float _k_overlap;
	static float _repulsion_soft_factor;
	static float _k_repulsion;
	static float _k_boundary;

	// capping the velocity
	static float _velocity_cap;


	// control objects
	static int _stop_id;
	static int _shrink_id;
	static int _focus_id;
	static bool _freeze;
	static bool _stop_scale;
	static bool _focus_mode;
	static bool _trying;

	// control change contour.
	static bool _changeContour;
	static bool _contourMorphingEnd;
	static bool _tuningPos;
	static bool _tuningAng;

	// save target data.
	static bool _saveTarget;
	// get target data.
	static bool _getTarget;

	// resize before 
	static float _avgObjSize;
	static bool _resizeStage;
	// trying  xxxxxxxxxxxxxxxxxxx
	static bool _resize;
	// bad method
	static bool _attrack;
	static bool _attrack2;
	static bool _attrack3;
};



#endif // !SYSTEMPARAMS

