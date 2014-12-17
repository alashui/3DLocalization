#version 330

layout (std140) uniform Material {
    vec4 diffuse;
    vec4 ambient;
    vec4 specular;
    vec4 emissive;
    float shininess;
    int texCount;
};

in vec3 Normal;
in vec4 fragmentColor;
out vec4 out1;

void main()
{
    vec4 color;
    vec4 amb;
    float intensity;
    vec3 lightDir;
    vec3 n;
    
    lightDir = normalize(vec3(-1.0, -1.0, 0.0));

    n = normalize(Normal);

    intensity = 0.2 + 0.8*max(dot(lightDir,n),0.0);

    color = fragmentColor;
    amb = color * 0.4;
    out1 = (color * intensity) + amb;//ient;

}