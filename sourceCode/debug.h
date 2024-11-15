//#pragma once
#ifndef _DEBUG_H
#define _DEBUG_H
#include <iostream>
#include <glad/glad.h>

/*
	Return the error "number".
	Check the error by using the "number" in the glad file ( turn the "number" into hexadecimal).
*/

#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) GLClearError();\
		x;\
		ASSERT(GLLogCall(#x, __FILE__, __LINE__))


static void GLClearError()
{
	while (glGetError() != GL_NO_ERROR);
}

static bool GLLogCall(const char* function, const char* file, int line)
{
	while (GLenum error = glGetError())
	{
		std::cout << "[OpenGL Error] (" << error << ") : " << "\n \t " << function <<
			"\n \t " << file << "\n \t Line:" << line << std::endl;
		return false;
	}
	return true;
}

#endif // !DEBUG_H

