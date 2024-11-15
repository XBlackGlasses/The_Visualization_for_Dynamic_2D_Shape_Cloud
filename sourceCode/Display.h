#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "shader.h"

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Object.h"
#include "ContourObject.h"
#include "CollisionGrid.h"
#include "SystemParams.h"
#include "StuffWorker.h"

class Display
{
private:
	static StuffWorker *_swork;
	
	unsigned int _windowWidth;
	unsigned int _windowHeight;
	
	// OpenGL
	GLFWwindow *_window;
	Shader *_shader;

	// for video
	cv::Mat _img;
	cv::VideoWriter _video;

	// fps
	double _fps;
	double _timebase, _time;
	int _frame;

	static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mode);

	void SetGL();
	
	void inline Fps();

	void inline SaveVideo();
public:
	Display();

	void Render();

	~Display();

	// focus + context
	static std::vector<int> _focusIds;
};


#endif // !DISPLAY_H


