#include "Display.h"
#include <regex>

StuffWorker *Display::_swork = 0;
std::vector<int> Display::_focusIds = std::vector<int>();
/*
	operation :
	X	t : choose a id, for stopping the growing of an object.
	X	s : choose a id, for shrinking an object.
	X	a : stop all objects scale, but still can simulate.
		f : freeze all objects, can't simulate.
			// focus + context
		e : choose a id, for enlarging an object.
		o : recover simulate in focus mode.
			
		r : (resize stage)controll the switch to resizing(shrink all big objects) stage. (before moving stage)
		
		m : (change contour)controll the switch to change contours & move objects.
		z : (change contour)fune tuning : give more force to fix pos.
		v : (change contour)fune tuning : give force to fix angle.
		x : (change contour)resize to target size after moving.
		c : (change contour)change the target shape cloud. 
*/
void Display::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	// for stopping the growing of an object.
	// press "t", then choose the id for stop.
	if (key == GLFW_KEY_T && action == GLFW_PRESS)
	{
		int id;
		std::cout << "choose the stop id" << std::endl;

		std::cin >> id;

		SystemParams::_stop_id = id;
		std::cout << SystemParams::_stop_id << std::endl;
	}

	// for shrinking the growing of an object.
	// press "s", then choose the id for shrink.
	if (key == GLFW_KEY_S && action == GLFW_PRESS)
	{
		int id;
		std::cout << "choose the id to change shrink type" << std::endl;

		std::cin >> id;
		if (id >= 0 && id < StuffWorker::_objs.size())
			StuffWorker::_objs[id]._isShrink = !StuffWorker::_objs[id]._isShrink;
		else
			std::cout << "choose the id between 0 ~ " << StuffWorker::_objs.size() << std::endl;
	}

	// recover simulate in focus mode.
	if (key == GLFW_KEY_O && action == GLFW_PRESS)
	{
		SystemParams::_trying = !SystemParams::_trying;
		std::cout << "trying : " << SystemParams::_trying << std::endl;
	}


	// for enlarging an object.
	// press "e", then choose the id for stop.
	if (key == GLFW_KEY_E && action == GLFW_PRESS)
	{
		SystemParams::_focus_mode = !SystemParams::_focus_mode;
		if (SystemParams::_focus_mode)
		{
			//SystemParams::_save_sreen = true;
			std::cout << "turn on focus mode" << std::endl;

			std::string in;

			std::regex numberRegex("\\d+");
			
			std::getline(std::cin, in);
			std::sregex_iterator iter(in.begin(), in.end(), numberRegex);
			std::sregex_iterator end;
			while (iter != end)
			{
				std::smatch match = *iter;
				_focusIds.push_back(std::stoi(match.str()) - 1);
				++iter;
			}
			for (unsigned int i = 0; i < _focusIds.size(); ++i)
				_swork->_objs[_focusIds[i]]._isFocus = true;
			

			/*int id;
			while (true)
			{
				std::cout << "choose the enlarge id" << std::endl;
				std::cin >> id;
				if (id >= 0 && id < StuffWorker::_objs.size())
				{
					SystemParams::_focus_id = id;
					std::cout << "focus id : " << SystemParams::_focus_id << std::endl;
					break;
				}
				else
					std::cout << "choose the id between 0 ~ " << StuffWorker::_objs.size() << std::endl;
			}*/
			SystemParams::_freeze = false;
		}
		else
		{
			std::cout << "turn off focus mode" << std::endl;
			SystemParams::_freeze = true;
			for (unsigned int i = 0; i < _focusIds.size(); ++i)
				_swork->_objs[_focusIds[i]]._isFocus = false;
			_focusIds.clear();

			SystemParams::_focus_id = -1;
			StuffWorker::_aroundIds.clear();
		}
	}

	// stop all objects scale, but still can simulate.
	if (key == GLFW_KEY_A && action == GLFW_PRESS)
	{
		SystemParams::_stop_scale = !SystemParams::_stop_scale;
		if (SystemParams::_stop_scale)
			std::cout << "stop all growing" << std::endl;
		else
			std::cout << "start all growing" << std::endl;
	}

	// freeze all objects, can't simulate
	if (key == GLFW_KEY_F && action == GLFW_PRESS)
	{
		SystemParams::_freeze = !SystemParams::_freeze;
		if (SystemParams::_freeze)
			std::cout << "freeze" << std::endl;
		else
			std::cout << "unfreeze" << std::endl;
	}

	/*
		controll the resize stage :
		--- resize the _big objects to average size.
	*/
	if (key == GLFW_KEY_R && action == GLFW_PRESS)
	{
		assert(SystemParams::_getTarget);
		SystemParams::_resizeStage = !SystemParams::_resizeStage;

		//SystemParams::_save_sreen = true;
		if (SystemParams::_resizeStage)
		{
			std::cout << "resize stage" << std::endl;
			// release the freeze.
			if (SystemParams::_freeze)
				SystemParams::_freeze = false;
			// release focus.
			SystemParams::_focus_id = -1;
			StuffWorker::_aroundIds.clear();

			//// important ! for resize objects to equal size.
			//assert(SystemParams::_avgObjSize != -1);

			//// set the size for resize stage. (not be used now)
			//std::cout << "avg size : " << SystemParams::_avgObjSize << std::endl;
			
			_swork->SetPreparedSize();
		}
		else
		{
			std::cout << "over resizing stage." << std::endl;
		}
	}

	// start to change contour & move objects
	/*
		first : resize the object that bigger than the threshold.
		second : move to the target position.
		final resize to target size and the big objs need to consider torsion force.
	*/
	if (key == GLFW_KEY_M && action == GLFW_PRESS)
	{
		assert(SystemParams::_getTarget);
		assert(!SystemParams::_resizeStage);
		SystemParams::_changeContour = !SystemParams::_changeContour;
		if (SystemParams::_changeContour)
		{
			std::cout << "moving" << std::endl;

			SystemParams::_freeze = false;

			SystemParams::_focus_id = -1;
			StuffWorker::_aroundIds.clear();

			_swork->SetBeforeMove();

			SystemParams::_tuningAng = false;
			SystemParams::_tuningPos = false;
			SystemParams::_resize = false;
		}
		else
			std::cout << "stop moving" << std::endl;
	}

	if (key == GLFW_KEY_Z && action == GLFW_PRESS)
	{
		assert(SystemParams::_changeContour);
		//assert(!SystemParams::_resizeStage);
		SystemParams::_tuningPos = !SystemParams::_tuningPos;
		if (SystemParams::_tuningPos)
		{
			std::cout << "pos fine tuning" << std::endl;
		}
		else
			std::cout << "stop fine tuning" << std::endl;
	}

	if (key == GLFW_KEY_V && action == GLFW_PRESS)
	{
		assert(SystemParams::_changeContour);
		//assert(!SystemParams::_resizeStage);
		SystemParams::_tuningAng = !SystemParams::_tuningAng;
		if (SystemParams::_tuningAng)
		{
			std::cout << "angle fine tuning" << std::endl;
		}
		else
			std::cout << "stop fine tuning" << std::endl;
	}

	if (key == GLFW_KEY_X && action == GLFW_PRESS)
	{
		//assert(SystemParams::_changeContour);
		//assert(!SystemParams::_resizeStage);
		SystemParams::_resize = !SystemParams::_resize;
		if (SystemParams::_resize)
		{
			_swork->FadeInNewObjs();
			std::cout << "resize" << std::endl;
		}
		else
			std::cout << "stop resize" << std::endl;
	}

	// change target shapde cloud
	if (key == GLFW_KEY_C && action == GLFW_PRESS)
	{
		// must in freeze mode. 
		assert(SystemParams::_freeze);
		// not in moving mode.
		assert(!SystemParams::_changeContour);
		// can get target
		assert(SystemParams::_getTarget);
		
		SystemParams::_contourMorphingEnd = false;
		SystemParams::_resizeStage = false;

		_swork->ReadTargetData();
		
		/*float  targetArea = _swork->_newContour.GetArea();
		targetArea /= _swork->_objs.size();
		float targetLen = sqrt(targetArea);
		std::cout << "new len : " << targetLen << std::endl;
		SystemParams::_avgObjSize = targetLen;*/

	}

	// sreen shot
	if (key == GLFW_KEY_H && action == GLFW_PRESS)
	{
		cv::Mat img(SystemParams::_screen_height, SystemParams::_screen_width, CV_8UC4);
		//use fast 4-byte alignment (default anyway) if possible
		glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
		//set length of one complete row in destination data (doesn't need to equal img.cols)
		glPixelStorei(GL_PACK_ROW_LENGTH, img.step / img.elemSize());

		glReadPixels(0, 0, img.cols, img.rows, GL_BGRA, GL_UNSIGNED_BYTE, img.data);


		cv::flip(img, img, 0);
		cv::imwrite(SystemParams::_data_set + "output\\1.png", img);
	}
}

// constuctor
Display::Display()
{
	// create and process objects.
	Display::_swork = new StuffWorker();
	_swork->ProcessObject();

	_windowWidth = _swork->_contour.GetWidth();
	_windowHeight = _swork->_contour.GetHeight();

	// init OpenGL.
	SetGL();

	// create shader.
	_shader = new Shader("shader//vertShader.glsl", "shader//fragShader.glsl");

	// bind GL data.
	_swork->GLBind();

	// project the coordinate.
	glm::mat4 proj = glm::mat4(1.0f);
	proj = glm::ortho(0.0f, (float)_windowWidth + 10.0f, (float)_windowHeight + 10.0f, 0.0f, -1.0f, 1.0f);

	_shader->use();
	// proj
	_shader->setMat4("proj", proj);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	float contourArea = _swork->_contour.GetArea();
	std::cout << "contour area : " << contourArea << std::endl;

	// create video
	if (SystemParams::_save_sreen)
		_video.open(SystemParams::_video_name, cv::VideoWriter::fourcc('H', '2', '6', '4'), 300.0, cv::Size(SystemParams::_screen_width, SystemParams::_screen_height));

	// fps
	_fps = 0.0;
	_timebase = glfwGetTime();
	_time = 0.0;
	_frame = 0;

	// for muti-shapecloud !! get the object/contour data of the next shape cloud.
	if (SystemParams::_getTarget)
	{
		_swork->ReadTargetData();

		/*float  targetArea = _swork->_newContour.GetArea();
		targetArea /= _swork->_objs.size();
		float targetLen = sqrt(targetArea);
		std::cout << "new len : " << targetLen << std::endl;
		SystemParams::_avgObjSize = targetLen;*/
	}

}



Display::~Display()
{
	delete _swork;
	delete _window;
	delete _shader;
}


void Display::SetGL()
{
	glfwInit();
	// use openGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	_window = glfwCreateWindow(SystemParams::_screen_width, SystemParams::_screen_height, "openGL", NULL, NULL);
	if (_window == NULL)
	{
		std::cout << "Fail to create window" << std::endl;
		glfwTerminate();
		exit(0);
	}
	glfwMakeContextCurrent(_window);
	// key callback
	glfwSetKeyCallback(_window, KeyCallback);

	// screen refresh rate
	// glfwSwapInterval(1);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		exit(0);
	}
	std::cout << glGetString(GL_VERSION) << std::endl;

	// color alpha. add transparency
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glEnable(GL_BLEND); glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

void Display::Fps()
{
	_frame++;
	_time = glfwGetTime(); // current time
	if (_time - _timebase >= 1.0)
	{
		_fps = (double)_frame / (_time - _timebase);
		_timebase = _time;
		_frame = 0;
		std::cout << "fps : " << _fps << std::endl;
	}
}

void Display::SaveVideo()
{
	_img.create(SystemParams::_screen_height, SystemParams::_screen_width, CV_8UC3);
	//use fast 4-byte alignment (default anyway) if possible
	glPixelStorei(GL_PACK_ALIGNMENT, (_img.step & 3) ? 1 : 4);
	//set length of one complete row in destination data (doesn't need to equal img.cols)
	glPixelStorei(GL_PACK_ROW_LENGTH, _img.step / _img.elemSize());

	glReadPixels(0, 0, _img.cols, _img.rows, GL_BGR, GL_UNSIGNED_BYTE, _img.data);

	cv::flip(_img, _img, 0);
	_video.write(_img);
	//cv::imwrite("..\\output_images\\" + std::to_string(image_frames) + ".png", img);
}


// main render loop.
void Display::Render()
{
	auto start1 = std::chrono::steady_clock::now();

	while (!glfwWindowShouldClose(_window))
	{
		if (glfwGetKey(_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(_window, true);
		glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		// update blending data
		_swork->UpdateGLData();
	
		
		if ((_swork->_objsArea / _swork->_contour.GetArea() > 0.65f) && !SystemParams::_focus_mode && !SystemParams::_resizeStage && !SystemParams::_changeContour)
		{
			//SystemParams::_stop_scale = true;
			auto elapsed1 = std::chrono::steady_clock::now() - start1;
			std::cout << "enough to freeze" << std::endl;
			std::cout << "objs size : " << _swork->_objsArea << " contour size : " << _swork->_contour.GetArea() << std::endl;
			std::cout << "dilate time : " << (float)std::chrono::duration_cast<std::chrono::microseconds>(elapsed1).count() / 1e6 << std::endl;
			SystemParams::_freeze = true;
		}

		// =================== main simulation =========================
		if (!SystemParams::_changeContour && !SystemParams::_resizeStage)
			_swork->Operation(SystemParams::_dt);	// 0.1
		else if (!SystemParams::_changeContour && SystemParams::_resizeStage)
		{
			_swork->ResizeStage(SystemParams::_dt * 1.5f);
		}
		else
		{
			_swork->Operation_Move(SystemParams::_dt * 1.5f);  // 0.2
			// update collision grid while the contour morphing.
			if (!SystemParams::_contourMorphingEnd)
				_swork->UpdateGrid();
		}

		// drawing 		
		_swork->GLDraw(*_shader);

		// fps
		//Fps();

		// save video when doesn't freeze mode.
		if (!SystemParams::_freeze && SystemParams::_save_sreen)
		{
			SaveVideo();
		}

		glfwSwapBuffers(_window);
		glfwPollEvents();
		_swork->ReleaseGL();
		if (SystemParams::_changeContour && !SystemParams::_contourMorphingEnd)
			_swork->_contour.ReleaseGL();
	}

	_swork->ReleaseGL();
	_swork->_contour.ReleaseGL();
	_video.release();

	if (StuffWorker::_cGrid)
		delete StuffWorker::_cGrid;

	// save objects' position for next shape cloud.
	if (!SystemParams::_getTarget)	// don't use at animation mode.
	{
		std::ofstream out(SystemParams::_shapeCloud_dir + "pos.txt");
		assert(out.is_open());
		for (unsigned int i = 0; i < StuffWorker::_objs.size(); ++i)
		{
			if (!StuffWorker::_objs[i]._showUp)
				continue;

			int top  = INT_MAX;
			int left = INT_MAX;
			for (unsigned int j = 0; j < StuffWorker::_objs[i]._massList.size(); ++j)
			{
				top  = std::min(top, (int)StuffWorker::_objs[i]._massList[j]._pos.y);
				left = std::min(left, (int)StuffWorker::_objs[i]._massList[j]._pos.x);
			}
			// save the id and pos(x, y).
			out << "id : " << StuffWorker::_objs[i]._id << " pos : " << left << " " << top << std::endl;
		}
		out.close();
	}

	// save the data of current objects.
	if (SystemParams::_saveTarget)
		_swork->SaveTargetObject();
}