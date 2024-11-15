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

#include "imageProcess.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;


std::vector<float> contourData(cv::Mat img, std::vector<std::vector<float>>& countMap)
{
	
	int width = img.cols;
	int height = img.rows;
	int count = 0;
	
	int empty_space = 0;

	std::vector<float> rowWidth = std::vector<float>(height, 0);
	for (unsigned int y = 0; y < height; ++y)
	{
		count = 0;
		for (unsigned int x = 0; x < width; ++x)
		{
			cv::Vec4i pt = img.at<cv::Vec4b>(y, x);
			if (pt.val[3] != 0)	// in the countour
			{
				count++;
				if (rowWidth[y] < count)
					rowWidth[y] = count;
			}
			else
			{
				empty_space ++;
				count = 0;
			}
			countMap[x][y] = count;
		}	
	}
	std::cout << "empty space : " << empty_space << std::endl;

	cv::Mat tmp = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
	for(int y = 0; y < height; ++y)
		for (int x = 0; x < width; ++x)
		{
			if (countMap[x][y] != 0)
			{
				int px = (int)countMap[x][y];
				if (px > 255)
					px = 255;
				tmp.at<uchar>(y, x) = px;
			}
		}
	cv::imshow("mask", tmp);
	cv::waitKey(0);

	return rowWidth;
}

/*
	the input image must remove the background.
*/
int main()
{
	std::string dir = "DataSet\\6\\contour\\";
	std::string file = "003";
	
	std::string input(dir + file + ".png");
	cv::Mat img = cv::imread(input, cv::IMREAD_UNCHANGED);

	if (img.empty())
	{
		std::cout << "didn't read image !!";
		return -1;
	}

	cv::copyMakeBorder(img, img, 2, 2, 2, 2, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));
	cv::resize(img, img, cv::Size(500, 500));
	
	cv::imwrite(dir+"test1.png", img);
	
	// convert to RGBA format
	if (img.type() == CV_8UC1)
		cv::cvtColor(img, img, cv::COLOR_GRAY2BGRA);
	else if (img.type() == CV_8UC3)
		cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
	else if (img.type() == CV_8UC4)
		std::cout << "rgba img" << std::endl;


	// saved data 
	float _contourLength = 0.0;
	float _contourArea = 0.0;
	int imgWidth = img.cols;
	int imgHeight = img.rows;
	std::vector<glm::vec2> _contour;
	std::vector<glm::vec2> _resampleContour;
	std::vector<std::vector<float>> _countMap = std::vector<std::vector<float>>(imgHeight, std::vector<float>(imgWidth, 0));
	std::vector<float> _rowsWidth;

	// temp data 
	std::vector<cv::Point> contour;
	std::vector<cv::Point> resampleContour;
	
// get count map ===============================
	_rowsWidth = contourData(img, _countMap);


// contour ===============================================
	findContour(img, contour);
	cv::Mat b_img = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	
	/*for (int i = 0; i < contour.size(); ++i)
	{
		cv::circle(b_img, contour[i], 1, (255, 255, 255), 1);
		cv::imshow("contour", b_img);
		cv::waitKey(1);
	}
	cv::waitKey(0);*/
	_contourArea = cv::contourArea(contour, true);
	std::cout << "contour area : " << _contourArea << std::endl;

// resample ==============================================
	int nPoint = 60;
	resample(contour, resampleContour, nPoint, _contourLength);

	b_img = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	for (int i = 0; i < resampleContour.size(); ++i)
	{
		cv::circle(b_img, resampleContour[i], 10, (255, 255, 255), 1);
		cv::imshow("resample", b_img);
		cv::waitKey(10);
	}
	cv::waitKey(0);

	std::cout << "contour length : " << _contourLength << std::endl;
	for (int i = 0; i < resampleContour.size(); ++i)
	{
		_resampleContour.push_back(glm::vec2(resampleContour[i].x, resampleContour[i].y));
	}
	
// significant map ======================================
	// get centroid
	std::vector<cv::Point2f> bound;
	cv::Point2i centroid;
	float Xsum = 0, Ysum = 0;
	for (int i = 0; i < resampleContour.size(); ++i)
	{
		bound.push_back(resampleContour[i]);
	}
	cv::Moments mu = cv::moments(bound, false);
	centroid.x = mu.m10 / mu.m00;
	centroid.y = mu.m01 / mu.m00;

	cv::Mat sigMap = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_8UC1);

	for (unsigned int i = 0; i < imgHeight; ++i)
	{
		for (unsigned int j = 0; j < imgWidth; ++j)
		{
			cv::Vec4i pt = img.at<cv::Vec4b>(i, j);
			
			if (pt.val[3] != 0)	// in the countour
			{
				int len = (centroid.x - j)*(centroid.x - j) + (centroid.y - i)*(centroid.y - i);
				len = std::sqrt(len);
				int value = 255 - len;
				value = std::max(value, 20);
				sigMap.at<uchar>(i, j) = value;
				
			}
		}
	}

	cv::imshow("sig map", sigMap);
	cv::waitKey(0);
	

// gl render ==========================================================
	// point to render
	std::vector<glm::vec4> vt;
	// convert to glm coorinate
	for (int i = 0; i < contour.size(); ++i)
	{
		glm::vec2 pt = glm::vec2(contour[i].x, contour[i].y);
		_contour.push_back(pt);
		// vt for render
		vt.push_back(glm::vec4(pt, 0.0, 1.0));
	}
	
	glfwInit();
	// use openGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// create window
	GLFWwindow *window = glfwCreateWindow(img.cols, img.rows, "openGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Fail to create window" << std::endl;
		glfwTerminate();
		exit(0);
	}
	glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		exit(0);
	}

	glEnable(GL_PROGRAM_POINT_SIZE);

	Shader shader("shader//vertShader.glsl", "shader//fragShader.glsl");
	unsigned int vao, vbo;
	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);
	
	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vt.size() * sizeof(vt[0]), vt.data(), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(vt[0]), (void*)0);

	bool hasTexture = false;

	glm::mat4 proj = glm::mat4(1.0f);
	proj = glm::ortho(0.0f, (float)img.cols, (float)img.rows, 0.0f, -1.0f, 1.0f);
	while (!glfwWindowShouldClose(window))
	{
		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(window, true);
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		shader.use();

		shader.setMat4("proj", proj);
		shader.setBool("hasTexture", hasTexture);

		glBindVertexArray(vao);
		glDrawArrays(GL_POINTS, 0, vt.size());
	

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &vbo);
	glfwTerminate();



	// save to json file
	auto save = json{
		{"img_path",
			input
		},
		{"img_width",
			imgWidth
		},
		{"img_height",
			imgHeight
		},
		{"contour_length",
			_contourLength
		},
		{"contour_area",
			_contourArea
		},
		{"width_of_each_row",
			_rowsWidth
		},
		{"contour",
			json::array()
		},
		{"resampled_contour",
			json::array()
		},
		{"count_map",
			json::array()
		}
	};

	for (unsigned int i = 0; i < _contour.size(); ++i)
	{
		auto &pt = save["contour"];
		pt.push_back({ _contour[i].x, _contour[i].y });
	}

	for (unsigned int i = 0; i < _resampleContour.size(); ++i)
	{
		auto &pt = save["resampled_contour"];
		pt.push_back({ _resampleContour[i].x, _resampleContour[i].y });
	}

	for (unsigned int i = 0; i < _countMap.size(); ++i)
	{
		auto &row = save["count_map"];
		for (unsigned int j = 0; j < _countMap[i].size(); ++j)
		{
			row[i].push_back(_countMap[i][j]);
		}
	}

	auto out = std::ofstream(dir + "contour" + file + ".json");
	out << save;
	out.close();
	return 0;
}