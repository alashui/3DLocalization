#include "ProgramIO.h"
#include "boost/filesystem.hpp"
#include "../Helper/Particles.h"


Assimp::Importer importer;
float r = 5.0;
int startX = 0;
int startY = 0;
int  tracking = 0;
float alpha = 0.0f;
float beta = 0.0f;

namespace IO
{
     float round(float x)
    {
        return (float) ((int) (x*100))/100.0;
    }

    void Idle(void)
    {
          if ( glutGetWindow() != main_window )
            glutSetWindow(main_window);

            glutPostRedisplay();
    }

    // Finds and loads the .obj file that contains the 3D model data.
    // Pass it the full file name of the .obj file relative to the build folder!
    bool Import3DFromFile()//, const aiScene* scene)
    {
        std::stringstream ss;
        ss << pathToModelDataDirectory << modelDirectoryName << modelname;
        std::string pFile = ss.str();ss.str("");

        std::ifstream fin(pFile.c_str());
        if(!fin.fail()) {
            fin.close();
            std::cout << ".obj File Located\n";
        }
        else
        {
            printf("Couldn't open file: %s\n", pFile.c_str());
            printf("%s\n", importer.GetErrorString());
            return false;
        }
        scene = importer.ReadFile( pFile.c_str(), aiProcessPreset_TargetRealtime_Quality);
        if((scene == NULL))
        {
            printf("%s\n", importer.GetErrorString());
            return false;
        }
        // Now we can access the file's contents.
        printf("Import of scene %s succeeded.\n", pFile.c_str());

        aiVector3D scene_min, scene_max, scene_center;
        View::get_bounding_box(&scene_min, &scene_max);
        float tmp;
        tmp = scene_max.x-scene_min.x;
        tmp = scene_max.y - scene_min.y > tmp?scene_max.y - scene_min.y:tmp;
        tmp = scene_max.z - scene_min.z > tmp?scene_max.z - scene_min.z:tmp;
        scaleFactor = 10.0f / tmp;

        return true;
    }



    // Load textures from the texture files associated with the .obj/.3ds file. Map these textures onto the polygons.
    int LoadGLTextures()
    {
        ILboolean success;

        /* initialization of DevIL */
        ilInit(); 

        if(scene == NULL)
        {
            std::cout << "scene is null inside load textures\n" << std::endl;
            return -1;
        }
        /* scan scene's materials for textures */
        for (unsigned int m=0; m<scene->mNumMaterials; ++m)
        {
            int texIndex = 0;
            aiString path;  // filename

            aiReturn texFound = scene->mMaterials[m]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);
            while (texFound == AI_SUCCESS)
            {
                //fill map with textures, OpenGL image ids set to 0
                textureIdMap[path.data] = 0; 
                // more textures?
                texIndex++;
                texFound = scene->mMaterials[m]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);
            }
        }


        int numTextures = textureIdMap.size();

        if(numTextures == 0)
            std::cout << "0 textures inside programIO.cpp" << std::endl;

        /* create and fill array with DevIL texture ids */
        ILuint* imageIds = new ILuint[numTextures];
        ilGenImages(numTextures, imageIds); 

        /* create and fill array with GL texture ids */
        GLuint* textureIds = new GLuint[numTextures];
        glGenTextures(numTextures, textureIds); /* Texture name generation */

        /* get iterator */
        std::map<std::string, GLuint>::iterator itr = textureIdMap.begin();

        for (int i=0; itr != textureIdMap.end(); ++i, ++itr)
        {
            //save IL image ID
            std::string filename = (*itr).first;  // get filename
            (*itr).second = textureIds[i];    // save texture id for filename in map

            ilBindImage(imageIds[i]); /* Binding of DevIL image name */
            ilEnable(IL_ORIGIN_SET);
            ilOriginFunc(IL_ORIGIN_LOWER_LEFT); 
            success = ilLoadImage((ILstring)(pathToModelDataDirectory+modelDirectoryName+filename).c_str());

            if (success)
            {
                /* Convert image to RGBA */
                ilConvertImage(IL_RGBA, IL_UNSIGNED_BYTE); 

                /* Create and load textures to OpenGL */
                glBindTexture(GL_TEXTURE_2D, textureIds[i]); 
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); 
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ilGetInteger(IL_IMAGE_WIDTH),
                    ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                    ilGetData()); 
            }
            else 
                printf("Couldn't load Image: %s\n", filename.c_str());
        }
        /* Because we have already copied image data into texture data
        we can release memory used by image. */
        ilDeleteImages(numTextures, imageIds); 

        //Cleanup
        delete [] imageIds;
        delete [] textureIds;

        //return success;
        return true;
    }



    void printProgramInfoLog(GLuint obj)
    {
        int infologLength = 0;
        int charsWritten  = 0;
        char *infoLog;

        glGetProgramiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

        if (infologLength > 0)
        {
            infoLog = (char *)malloc(infologLength);
            glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
            printf("%s\n",infoLog);
            free(infoLog);
        }
    }


    void changeSize(int w, int h)
     {

        float ratio;
        // Prevent a divide by zero, when window is too short
        // (you cant make a window of zero width).
        if(h == 0)
            h = 1;

        // Set the viewport to be the entire window
        glViewport(0, 0, w, h);

        ratio = (1.0f * w) / h;
        View::buildProjectionMatrix(60.13f, ratio, 0.0001f, 100.0f);
    }

    #define printOpenGLError() printOglError(__FILE__, __LINE__)
    int printOglError(char *file, int line)
    {
        
        GLenum glErr;
        int retCode = 0;

        glErr = glGetError();
        if (glErr != GL_NO_ERROR)
        {
            printf("glError in file %s @ line %d: %s\n", file, line, gluErrorString(glErr));
            retCode = 1;
        }
        return retCode;
    }


    void processKeys(unsigned char key, int xx, int yy) 
    {
        double f = .05;
        double temp = .1;
        // std::cout << "HERE " << key << std::endl;
        switch(key)
        {
            case 27: glutLeaveMainLoop();break;
            case ' ': snapshot = true;break;
            case 'w':
            {
                camera[0] += translation[0]*temp;
                camera[1] += translation[1]*temp;
                camera[2] += translation[2]*temp;
            }break;
            case 's':
            {
                camera[0] -= translation[0]*temp;
                camera[1] -= translation[1]*temp;
                camera[2] -= translation[2]*temp;
            }break;
            case 'a':
            {
                theta += 5;
                translation[0] = cos(3.14159*theta/180.0);
                translation[1] = sin(3.14159*theta/180.0);
                translation[2] = 0;
            }break;
            case 'd':
            {
                theta -= 5;
                translation[0] = cos(3.14159*theta/180.0);
                translation[1] = sin(3.14159*theta/180.0);
                translation[2] = 0;
            }break;
            case 'n' :
            {
                IO::GetParticleList();
            }break;

             case 't' :
            {
                camera[2] += 0.1;
            }break;

             case 'y' :
            {
                camera[2] -= 0.1;
            }break;

        }
        // camera[0] = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
        // camera[1] = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
        // camera[2] = r *                               sin(beta * 3.14f / 180.0f);
    }


    void processMouseButtons(int button, int state, int xx, int yy) 
    {
        //glutTimerFunc(50, nextLocation, 1);
        //snapshot = true;
    }

    // Track mouse motion while buttons are pressed

    void processMouseMotion(int xx, int yy)
    {

        int deltaX, deltaY;
        float alphaAux, betaAux;
        float rAux;

        deltaX =  startX - xx;
        deltaY =  yy - startY;
        // left mouse button: move camera
        if (tracking == 1) 
        {
            // alphaAux = alpha + deltaX;
            // betaAux = beta + deltaY;

            // if (betaAux > 85.0f)
            //     betaAux = 85.0f;
            // else if (betaAux < -85.0f)
            //     betaAux = -85.0f;

            // rAux = r;

            // camera[0] = rAux * cos(betaAux * 3.14f / 180.0f) * sin(alphaAux * 3.14f / 180.0f);
            // camera[1] = rAux * cos(betaAux * 3.14f / 180.0f) * cos(alphaAux * 3.14f / 180.0f);
            // camera[2] = rAux * sin(betaAux * 3.14f / 180.0f);
        }
        else if (tracking == 2)
         {
            // alphaAux = alpha;
            // betaAux = beta;
            // rAux = r + (deltaY * 0.01f);

            // camera[0] = rAux * cos(betaAux * 3.14f / 180.0f) * sin(alphaAux * 3.14f / 180.0f);
            // camera[1] = rAux * cos(betaAux * 3.14f / 180.0f) * cos(alphaAux * 3.14f / 180.0f);
            // camera[2] = rAux * sin(betaAux * 3.14f / 180.0f);
         }
    }




    void mouseWheel(int wheel, int direction, int x, int y) 
    {

        r += direction * 0.1f;
        camera[0] = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
        camera[1] = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
        camera[2] =  r *                             sin(beta * 3.14f / 180.0f);
    }


std::vector<Particle> GetParticleList()
{
        std::vector<std::vector<float> > parts;
    float minweight = 1000, maxweight = 0;
    std::vector<float> v;
    std::string str;

    particles.clear();

    // --- Open the file of the particle positions -- //
    std::string fn = "../../PyViewer/ParticleLists.txt";
    std::ifstream file(fn.c_str());
    if ( !file.is_open() )
    {
        std::stringstream ss;
        ss << "ParticleLists.txt not found.";
        ErrorIO(ss.str());
        std::vector<Particle> v;
        return v;
    }

    // -- Go through the file and add the particles to a map --//
    while (!file.eof())
    {
        getline( file, str );
        std::vector<std::string> strs;
        boost::split(strs, str, boost::is_any_of(" "));
        for (int i = 0; i < strs.size(); i++)
            v.push_back(atof(strs[i].c_str()));
        float wt = v[6];
        
        if(v.size() == 0)
            continue;

        if(wt > maxweight)
            maxweight = wt;
        else if(wt < minweight)
            minweight = wt;

        parts.push_back(v);
        v.clear();
    }

    // -- Go through
    std::map<std::pair<float, float>, int> Map;
    for(int i = 2; i < parts.size(); i++)
    {
        std::vector<float> v = parts[i];

        std::pair<float, float> temppair(v[0], v[1]);

        if(!Map.count(temppair))
        {
            Map[temppair] = 1;
        }
        else
            Map[temppair]++;

        int x = Map.at(temppair);

        MyParticle temp(v[0], v[1], v[2], v[3], v[4], v[5], v[6], x-1, 0);
        particles.push_back(temp);
    }

    v.clear();
    v = parts[0];
    MyParticle temp(v[0], v[1], v[2], v[3], v[4], v[5], v[6], 0, 1);
    particles.push_back(temp);

    v.clear();
    v = parts[1];
    MyParticle temp1(v[0], v[1], v[2], v[3], v[4], v[5], v[6], 0, 2);
    particles.push_back(temp1);

    return pList;
}

} // end namespace IO

