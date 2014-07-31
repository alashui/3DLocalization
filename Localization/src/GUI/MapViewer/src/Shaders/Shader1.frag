#version 330

// Interpolated values from the vertex shaders
in vec3 fragmentColor;

layout (std140) uniform Material {
    vec4 diffuse;
    vec4 ambient;
    vec4 specular;
    vec4 emissive;
    float shininess;
    int texCount;
};

// Ouput data
out vec3 color;
out vec3 output;
 
void main(){
    // Output color = color specified in the vertex shader,
    // interpolated between all 3 surrounding vertices
    color = fragmentColor;

    float intensity;
    vec3 lightDir;
    vec3 n;
    
    lightDir = normalize(vec3(2.0,1.0,2.0));
    n = normalize(vec3(1.0, 0.0, 1.0));  
    intensity = max(dot(lightDir,n),0.0);

    output = (color * intensity);

}