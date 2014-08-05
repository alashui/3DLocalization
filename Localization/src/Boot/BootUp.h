/**
*
*   @File   - BootUp.h
*   @Author - Alex Rich and John Allard
*   @Info   - This file contains functions needed to load all of the data necessary to run the 3DLocalization program. It goes into 
*             the /Data/FeatureData/ directory and loads all of the feature data that is sourced from the current 3D model that the user
*             is trying to localize themselves inside.
*
**/

#ifndef MCL_BOOTUP_H_
#define MCL_BOOTUP_H_

#include "boost/filesystem.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"


#include "../Helpers/Perspective.h"
#include "../Helpers/Characterizer.h"
#include "../Particle/Particle.h"
#include "../Helpers/Globals/Globals.h"
#include "../IO/ProgramIO.h"


using namespace cv;
using namespace std;

namespace fs = ::boost::filesystem;

namespace MCL
{
    void GetAll(const fs::path& root, vector<fs::path>& ret);
    
    int LoadCharacterizers(string dirName);

    int BootUp(string dirName)
    {
        LoadCharacterizers(dirName);
        return perspectives.size();
    }

    int LoadCharacterizers(string dirName)
    {
        string pathToData = "../../Data";
        string toFeatures = pathToData + "/FeatureData/" + dirName;
        string toPhotos = pathToData + "/RenderedImages/" + dirName;

        string delimiter = "_";

        // Go into folder and get all available photos and features.
        vector<fs::path> ret;
        const char * pstr = toPhotos.c_str();
        fs::path p(pstr);
        // Get a list of all filenames
        GetAll(pstr, ret);


        stringstream ss;
        ss << "Bootup.h - Attempting to load " << ret.size() << " images and their characterizers.";
        DebugIO(ss.str()); ss.str("");
        cout << endl;

        // tmp Descriptors
        Mat descriptors;
        vector<KeyPoint> keypoints;

        int percent = 0;
        int total = ret.size();
        int done = 0;

        for (int i = 0; i < ret.size(); i++)
        { // Load image, bw, gs, and descriptors via filename. 

            // fn is filename
            string fn = ret[i].string();
            // tokens is vector of strings in filename
            vector<string> tokens; 

            if (fn[0] != delimiter.c_str()[0])
            {
                cout << "\033[A"; //]
                ss << " Extraneous file found: " << fn;
                DebugIO(ss.str());
                cout << endl;
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

            fn = delimiter;

            for (int j = 0; j < ID.size(); j++)
            {
                stringstream ss;
                ss << ID[j];
                fn += ss.str() + delimiter;  
            }

            // Create filename for loading stuff
            string kpfn = toFeatures + "/descriptors/" + fn + ".yml";
            string bwfn = toFeatures + "/bwimages/" + fn + ".jpg";
            string gsfn = toFeatures + "/gsimages/" + fn + ".jpg";

            // Create filestorage item to read from and add to map.
            FileStorage store(kpfn, FileStorage::READ);

            FileNode n1 = store["Descriptors"];
            read(n1, descriptors);
            tmp.descs = descriptors;

            FileNode n2 = store["Keypoints"];
            read(n2, keypoints);
            tmp.kps = keypoints;

            store.release();

            if (descriptors.empty() || descriptors.rows < 3 || descriptors.cols < 3)
            {
                done++;
                continue;
            }

            // Add bw and gs images.
            Mat gstmp = imread(gsfn, CV_LOAD_IMAGE_GRAYSCALE);
            Mat bwtmp = imread(bwfn, CV_LOAD_IMAGE_GRAYSCALE);
            tmp.gs = gstmp;
            tmp.bw = bwtmp;

            perspectives.push_back(P);
            masterMap[P] = tmp;

            // Check for invalid input
            if(! masterMap[P].gs.data )
            {
                ss <<  "Bootup.h - Could not open or find pixsum for " << gsfn;
                ErrorIO(ss.str());
                return -1;
            }
            if(! masterMap[P].bw.data )
            {
                ss <<  "BootUp.h - Could not open or find bw for " << bwfn;
                ErrorIO(ss.str());
                return -1;
            }
            done++;


            if (done > 2500) 
                break;
            if ((done * 100) / total > percent)
            {
                cout << "\033[A"; //]
                percent++;
                stringstream ss;
                ss << percent << "\% done.";
                DebugIO(ss.str());
            }
        }
        stringstream s;
        s << perspectives.size() << " Images Loaded.";
        DebugIO(s.str());

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