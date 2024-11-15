#include "SystemParams.h"

//int SystemParams::_screen_width = 800;
//int SystemParams::_screen_height = 600;

//6 
int SystemParams::_screen_width = 700;
int SystemParams::_screen_height = 700;


// butterfly2
//int SystemParams::_screen_width  = 629*2;
//int SystemParams::_screen_height = 355*2;

//butterfly
//int SystemParams::_screen_width  = 655*2;
//int SystemParams::_screen_height = 514*2;

// compare size
//int SystemParams::_screen_width  = 500;
//int SystemParams::_screen_height = 500;

float SystemParams::_min_scale_size = 0.08;

float SystemParams::_upscaleFactor = 500.0;
float SystemParams::_downscaleFactor = 1.0 / _upscaleFactor;

float SystemParams::_growth_scale_iter = 0.002;
float SystemParams::_growth_min_dist = 1;

//float SystemParams::_bin_square_size = 50.0;
float SystemParams::_bin_square_size = 10.0;
int SystemParams::_collission_block_radius = 2;

float SystemParams::_min_scale_value = 0.092;
float SystemParams::_max_size = -1;
float SystemParams::_min_size = -1;

float SystemParams::_dt = 0.1;

int SystemParams::_num_threads = 16;

int SystemParams::_shapeCloud_num = 0;
std::string SystemParams::_data_set		  = "DataSet\\6\\";
// the contour of first contour.
std::string SystemParams::_contour_file   = SystemParams::_data_set + "contour\\contour001.json";
std::string SystemParams::_obj_dir		  = SystemParams::_data_set + "json\\";
std::string SystemParams::_shapeCloud_dir = SystemParams::_data_set + "shapeCloud\\";
// the directory to save the objects data. save shape cloud objects.
std::string SystemParams::_skeleton_path = SystemParams::_data_set + "contour\\skeleton1.txt";


// save video
bool SystemParams::_save_sreen = false;
std::string SystemParams::_video_name = SystemParams::_data_set + "output\\" + "vidoe.mp4";

// the dir to save json
std::string SystemParams::_target_dir = SystemParams::_shapeCloud_dir +  "1\\";
bool SystemParams::_saveTarget = false;
bool SystemParams::_getTarget = false;

float SystemParams::_self_intersection_threshold = 1.0;

//float SystemParams::_k_edge = 20;
//float SystemParams::_k_neg_space_edge = 15;
//float SystemParams::_k_rotate = 1;	// ---- 1
//float SystemParams::_k_overlap = 10;
//float SystemParams::_repulsion_soft_factor = 1.0;
//float SystemParams::_k_repulsion = 70;
//float SystemParams::_k_boundary = 40;

// for 6
float SystemParams::_k_edge = 20;
float SystemParams::_k_neg_space_edge = 15;
float SystemParams::_k_rotate = 1;	// ---- 1
float SystemParams::_k_overlap = 20;
float SystemParams::_repulsion_soft_factor = 1.0;
float SystemParams::_k_repulsion = 80;
float SystemParams::_k_boundary = 40;


float SystemParams::_velocity_cap = 5;

int SystemParams::_stop_id		= -1;
int SystemParams::_shrink_id	= -1;
int SystemParams::_focus_id		= -1;
bool SystemParams::_freeze		= false;
bool SystemParams::_stop_scale	= false;
bool SystemParams::_focus_mode	= false;
bool SystemParams::_trying = false;

bool SystemParams::_changeContour = false;
bool SystemParams::_contourMorphingEnd = false;
bool SystemParams::_tuningPos = false;
bool SystemParams::_tuningAng = false;

// controll the resize stage.
float SystemParams::_avgObjSize = -1;
bool SystemParams::_resizeStage = false;

bool SystemParams::_resize = false;

bool SystemParams::_attrack = false;
bool SystemParams::_attrack2 = false;
bool SystemParams::_attrack3 = false;

SystemParams::SystemParams()
{ }