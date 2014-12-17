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
in vec2 TexCoord;
out vec4 out1;

void main()
{
    vec4 color;
    vec4 amb;
    float intensity;
    float intensity2;
    float intensity3;
    float intensity4;
    float intensity5;

    vec3 lightDir;
    vec3 lightDir2;
    vec3 lightDir3;
    vec3 lightDir4;
    vec3 lightDir5;

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
    
    if (texCount == 0) {
        color = diffuse;
        amb = ambient;
    }
    else {
        color = texture(texUnit, TexCoord);
        amb = color * 0.33;
    }

    float x = (intensity);//+intensity2+intensity3+intensity4+intensity5);

    if(x >= 1.0)
        x = 0.5; //1-2*amb;
    out1 = (color * x) + 2*amb;
    //out1 = vec4(texCount,0.0,0.0,1.0);

}
