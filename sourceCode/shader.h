#ifndef SHADER_H
#define SHADER_H

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include "debug.h"

class Shader
{
public:
	unsigned int ID;
	// constructor generates the shader on the fly
	// ------------------------------------------------------------------------
	Shader(const char* vertexPath, const char* fragmentPath)
	{
		// 1. retrieve the vertex/fragment source code from filePath
		std::string vertexCode;
		std::string fragmentCode;
		std::ifstream vShaderFile;
		std::ifstream fShaderFile;
		// ensure ifstream objects can throw exceptions:
		vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		try
		{
			// open files
			vShaderFile.open(vertexPath);
			fShaderFile.open(fragmentPath);
			std::stringstream vShaderStream, fShaderStream;
			// read file's buffer contents into streams
			vShaderStream << vShaderFile.rdbuf();
			fShaderStream << fShaderFile.rdbuf();
			// close file handlers
			vShaderFile.close();
			fShaderFile.close();
			// convert stream into string
			vertexCode = vShaderStream.str();
			fragmentCode = fShaderStream.str();
		}
		catch (std::ifstream::failure& e)
		{
			std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << e.what() << std::endl;
		}
		const char* vShaderCode = vertexCode.c_str();
		const char * fShaderCode = fragmentCode.c_str();
		// 2. compile shaders
		unsigned int vertex, fragment;
		// vertex shader
		vertex = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertex, 1, &vShaderCode, NULL);
		glCompileShader(vertex);
		checkCompileErrors(vertex, "VERTEX");
		// fragment Shader
		fragment = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragment, 1, &fShaderCode, NULL);
		glCompileShader(fragment);
		checkCompileErrors(fragment, "FRAGMENT");
		// shader Program
		ID = glCreateProgram();
		glAttachShader(ID, vertex);
		glAttachShader(ID, fragment);
		glLinkProgram(ID);
		checkCompileErrors(ID, "PROGRAM");
		// delete the shaders as they're linked into our program now and no longer necessary
		glDeleteShader(vertex);
		glDeleteShader(fragment);
	}

	void Delete() const
	{
		glDeleteProgram(ID);
	}

	// activate the shader
	// ------------------------------------------------------------------------
	void use() const
	{
		GLCall( glUseProgram(ID) );
	}
	// utility uniform functions
	// ------------------------------------------------------------------------
	void setBool(const std::string &name, bool value) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform1i(location, (int)value) );
	}
	// ------------------------------------------------------------------------
	void setInt(const std::string &name, int value) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform1i(location, value) );
	}
	// ------------------------------------------------------------------------
	void setFloat(const std::string &name, float value) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform1f(location, value) );
	}
	// ------------------------------------------------------------------------
	void setVec2(const std::string &name, const glm::vec2 &value) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform2fv(location, 1, &value[0]) );
	}
	void setVec2(const std::string &name, float x, float y) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform2f(location, x, y) );
	}
	// ------------------------------------------------------------------------
	void setVec3(const std::string &name, const glm::vec3 &value) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform3fv(location, 1, &value[0]) );
	}
	void setVec3(const std::string &name, float x, float y, float z) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform3f(location, x, y, z) );
	}
	// ------------------------------------------------------------------------
	void setVec4(const std::string &name, const glm::vec4 &value) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform4fv(location, 1, &value[0]) );
	}
	void setVec4(const std::string &name, float x, float y, float z, float w) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniform4f(location, x, y, z, w) );
	}
	// ------------------------------------------------------------------------
	void setMat2(const std::string &name, const glm::mat2 &mat) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniformMatrix2fv(location, 1, GL_FALSE, &mat[0][0]) );
	}
	// ------------------------------------------------------------------------
	void setMat3(const std::string &name, const glm::mat3 &mat) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniformMatrix3fv(location, 1, GL_FALSE, &mat[0][0]) );
	}
	// ------------------------------------------------------------------------
	void setMat4(const std::string &name, const glm::mat4 &mat) const
	{
		int location = glGetUniformLocation(ID, name.c_str());
		ASSERT(location != -1);
		GLCall( glUniformMatrix4fv(location, 1, GL_FALSE, &mat[0][0]) );
	}


private:
	// utility function for checking shader compilation/linking errors.
	// ------------------------------------------------------------------------
	void checkCompileErrors(unsigned int shader, std::string type)
	{
		int success;
		char infoLog[1024];
		if (type != "PROGRAM")
		{
			glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
			if (!success)
			{
				glGetShaderInfoLog(shader, 1024, NULL, infoLog);
				std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
			}
		}
		else
		{
			glGetProgramiv(shader, GL_LINK_STATUS, &success);
			if (!success)
			{
				glGetProgramInfoLog(shader, 1024, NULL, infoLog);
				std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
			}
		}
	}
};
#endif
