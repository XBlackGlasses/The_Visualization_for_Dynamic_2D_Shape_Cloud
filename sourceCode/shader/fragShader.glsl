#version 330

out vec4 FragColor;

in vec2 TexCoord;

uniform bool hasTexture;
uniform sampler2D ourtexture;

void main()
{
	if(hasTexture)	
		FragColor = texture2D(ourtexture, TexCoord);
	else
		FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
}