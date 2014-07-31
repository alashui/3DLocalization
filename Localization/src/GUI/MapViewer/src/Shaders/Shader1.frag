#version 330

layout (std140) uniform Material {
    vec4 diffuse;
    vec4 ambient;
    vec4 specular;
    vec4 emissive;
    float shininess;
    int texCount;
};

uniform sampler2D texUnit;

in vec3 Normal;
in vec4 fragmentColor;
out vec4 output;

void main()
{
    vec4 color;
    vec4 amb;
    float intensity;
    //float intensity2;
    //float intensity3;
    //float intensity4;
    //float intensity5;

    vec3 lightDir;
    //vec3 lightDir2;
    //vec3 lightDir3;
    //vec3 lightDir4;
    //vec3 lightDir5;

    vec3 n;
    
    lightDir = normalize(vec3(2.0, 1.0, -1.0));
    //lightDir2 = normalize(vec3(0.0, 1.0, -0.4));
    //lightDir3 = normalize(vec3(-1.0, 0, -0.4));
    //lightDir4 = normalize(vec3(0, -1.0, -0.4));
    //lightDir5 = normalize(vec3(0.0, 0.0, -1.0));

    n = normalize(Normal);  

    intensity = max(dot(lightDir,n),0.0);
    //intensity2 = max(dot(lightDir2,n),0.0);
    //intensity3 = max(dot(lightDir3,n),0.0);
    //intensity2 = max(dot(lightDir4,n),0.0);
    //intensity3 = max(dot(lightDir5,n),0.0);
    

    color = fragmentColor;
    amb = color * 0.33;
    

    float x = (intensity);//+intensity2+intensity3+intensity4+intensity5);

    if(x >= 1.0)
        x = 1-2*amb;
    output = (color * x) + 2*amb;
    //output = vec4(texCount,0.0,0.0,1.0);

}
