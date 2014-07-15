/**
*
*   @File BootUp.h
*   @Author Alex Rich and John Allard
*   @Info Loads initial data for MCL
*
**/

#ifndef MCL_BOOTUP_H_
#define MCL_BOOTUP_H_

#include "boost/filesystem.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "../Helpers/Perspective.h"
#include "../Helpers/Characterizer.h"
#include "../Particle/Particle.h"
#include "../Helpers/Globals/Globals.h"


using namespace cv;
using namespace std;

namespace fs = ::boost::filesystem;

namespace MCL
{
    void GetAll(const fs::path& root, vector<fs::path>& ret);
    int LoadCharacterizers(string dirName, vector<Perspective> perspectives, map<Perspective, Characterizer, ComparePerspectives> masterMap);

    int BootUp(string dirName, vector<Particle> particles)
    {
        LoadCharacterizers(dirName, perspectives, masterMap);
        MCL::Robot_init();
        // Particles::GenerateInitialParticles(perspectives, particles);

        return perspectives.size();
    }

    int LoadCharacterizers(string dirName)
    {
        string pathToData = "../../../Data";
        string toFeatures = pathToData + "/FeatureData/" + dirName;
        string toPhotos = pathToData + "/RenderedImages/" + dirName;

        string delimiter = "_";

        // Go into folder and get all available photos and features.
        vector<fs::path> ret;
        const char * pstr = toPhotos.c_str();
        fs::path p(pstr);
        // Get a list of all filenames
        GetAll(pstr, ret);

        cout << "<\n  Attempting to load " << ret.size() << " images and their characterizers." << endl;

        // tmp Descriptors
        Mat descriptors;

        for (int i = 0; i < ret.size(); i++)
        { // Load image, bw, gs, and descriptors via filename. 

            // fn is filename
            string fn = ret[i].string();
            // tokens is vector of strings in filename
            vector<string> tokens; 

            if (fn[0] != delimiter.c_str()[0]){
                cout << "\033[1;33m  Extraneous file found: " << fn << "\033[0m" << endl; // ]]
                continue;
            }

            // remove initial delimiter
            fn = fn.substr(1, fn.length());

            // Split up string.
            size_t pos = 0;
            string token;
            while ((pos = fn.find(delimiter)) != std::string::npos) 
            {
                token = fn.substr(0, pos);
                fn.erase(0, pos + delimiter.length());
                tokens.push_back(token);
            }
            // Reset Filename
            fn = ret[i].string();

            // Construct ID from filename tokens
            vector<float> ID;
            for (int j = 0; j < 6; j++) // 6 because there are three location floats and three direction floats
                ID.push_back(atof(tokens[j].c_str()));

            // ** Read image and add to mastermap. **

            // imfn is the image filename including directory
            string imfn = toPhotos + "/" + fn;

            // Create a temporary image and features struct.
            Characterizer tmp;
            tmp.image = imread(imfn);

            Perspective P(ID);

            perspectives.push_back(P);

            // Add it to our map
            masterMap[P] = tmp;

            fn = delimiter;

            for (int j = 0; j < ID.size(); j++)
            {
                stringstream ss;
                ss << ID[j];
                imfn += ss.str() + delimiter;  
            }

            // Create filename for loading stuff
            string kpfn = toFeatures + "/descriptors/" + fn + ".yml";
            string bwfn = toFeatures + "/bwimages/" + fn + ".jpg";
            string gsfn = toFeatures + "/gsimages/" + fn + ".jpg";

            // Create filestorage item to read from and add to map.
            FileStorage store(kpfn, FileStorage::READ);

            FileNode n1 = store["SurfDescriptors"];
            read(n1, descriptors);
            masterMap[P].surfs = descriptors;

            FileNode n2 = store["SiftDescriptors"];
            read(n2, descriptors);
            masterMap[P].sifts = descriptors;

            store.release();

            // Add bw and gs images.
            masterMap[P].gs = imread(gsfn, CV_LOAD_IMAGE_GRAYSCALE);
            masterMap[P].bw = imread(bwfn, CV_LOAD_IMAGE_GRAYSCALE);

            // Check for invalid input
            if(! masterMap[P].gs.data )
            {
                cout <<  "\033[1;31m  Could not open or find pixsum for " << gsfn << ".\033[0m" << endl; //]]
                return -1;
            }
            if(! masterMap[P].bw.data )
            {
                cout <<  "\033[1;31m  Could not open or find bw for " << bwfn << ".\033[0m" << endl; //]]
                return -1;
            }
        }
        return perspectives.size();
    }

    // return the filenames of all files that have the specified extension
    // in the specified directory and all subdirectories
    void GetAll(const fs::path& root, vector<fs::path>& ret)
    {  
        if (!fs::exists(root)) return;

        if (fs::is_directory(root))
        {
            fs::recursive_directory_iterator it(root);
            fs::recursive_directory_iterator endit;
            while(it != endit)
            {
                if (fs::is_regular_file(*it))
                    ret.push_back(it->path().filename());
                ++it;
            }
        }
    }
}

#endif