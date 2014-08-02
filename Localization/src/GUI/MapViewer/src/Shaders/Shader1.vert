#version 330

layout (std140) uniform Matrices {

    mat4 projMatrix;
    mat4 viewMatrix;
    mat4 modelMatrix;
};

in vec3 position;
layout(location = 2) in vec3 vertexColor;
in vec3 normal;


out vec4 fragmentColor;
out vec4 vertexPos;
out vec3 Normal;
 
void main(){
 
    // The color of each vertex will be interpolated
    // to produce the color of each fragment
    fragmentColor = vec4(vertexColor, 1.0);
    gl_Position = projMatrix * viewMatrix * modelMatrix * vec4(position,1.0);
    Normal = normalize(vec3(viewMatrix * modelMatrix * vec4(normal,0.0)));  

}


