#version 330

layout (std140) uniform Matrices {

    mat4 projMatrix;
    mat4 viewMatrix;
    mat4 modelMatrix;
};

in vec3 position;
layout(location = 1) in vec3 vertexColor;

out vec3 fragmentColor;
out vec4 vertexPos;
 
void main(){
 
    // The color of each vertex will be interpolated
    // to produce the color of each fragment
    fragmentColor = vertexColor;
    gl_Position = projMatrix * viewMatrix * modelMatrix * vec4(position,1.0);
}