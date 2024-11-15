#version 330

layout (location = 0) in vec4 aPos;
layout (location = 1) in vec2 aTexCoord;
uniform mat4 proj;

out vec2 TexCoord;

void main()
{

	gl_Position = proj * vec4(aPos.x, aPos.y, aPos.z, 1.0f);

	TexCoord = aTexCoord;
}

