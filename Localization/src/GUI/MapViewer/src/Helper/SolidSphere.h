#ifndef SPHERE_H_
#define SPHERE_H_

#include <vector>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include "MathHelp.h"
#include "../View/View.h"
#include "HelperStructures.h"

// your framework of choice here

class SolidSphere
{
protected:
    std::map<std::string, GLuint> textureIdMap;
    Assimp::Importer importer;

public:

	std::vector<aiScene*> scenes;
	const aiScene * scene;
	std::vector<struct Helper::MyMesh> myMeshes;
	aiVector3D * vertices;

    bool CreateSpheres();

    int LoadTextures();

    void recursive_render(const aiNode* nd);

    void genVAOsAndUniformBuffer();

    void setModelMatrix();

    void translate(float x, float y, float z);

    void scale(float x, float y, float z); 
};

extern SolidSphere sphere;


#endif