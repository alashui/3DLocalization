#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cvaux.h"

// #include <pcl/io/pcd_io.h>

#include "boost/filesystem.hpp"  
// #include <boost/lexical_cast.hpp>


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

#include "AverageImage.h"
#include "Similarity.h"
#include "../../../Localization/src/IO/ProgramIO.h"

using namespace cv;
using namespace std;
namespace fs = ::boost::filesystem;

// Print usage
static void printPrompt( const string& applName )
{
    stringstream ss;
    ss << "/*\n"
    << " * Given a folder of photos, this computes descriptors and info of images.\n"
    << " */\n" << endl <<  "Format:\n" << endl << applName << " ModelDirectoryName" << endl;
    // MCL::DebugIO(ss.str());
    cout << ss.str();
}

// return the filenames of all files that have the specified extension
// in the specified directory and all subdirectories
void get_all(const fs::path& root, vector<fs::path>& ret)
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

bool operator==(const KeyPoint& a, const KeyPoint& b)
{
    string t = "\t";
    return ( ( (abs(a.angle - b.angle) < 5) + (abs(a.octave - b.octave) < 10000) + (abs(a.response - b.response) < 0.001) + (abs(a.size - b.size) < 0.4)) >2);
}
bool operator!=(const KeyPoint& a, const KeyPoint& b){return !(a == b);}

bool exists(vector<KeyPoint> kps, KeyPoint kp)
{
    for (int i = 0; i < kps.size(); i++)
    {
        KeyPoint k = kps[i];
        if (k == kp)
            return true;
    }
    return false;
}

bool nearbyColor(Point2f p, int r, int g, int b, Mat& img)
{
    #define d 3 // pixels to look away from the detected keypoint
    for (int i = -d; i < d; i++)
    {
        for (int j = -d; j < d; j++)
        {
            int x = p.x + i;
            int y = p.y + j;
            if (x < 0 || x > img.cols - 1 || y < 0 || y > img.rows - 1)
                continue;
            Vec3b v = img.at<Vec3b>(Point2f(x, y));
            if (v == Vec3b(r, g, b))
                return true;
        }
    }
    return false;
}

void removeBad(vector<KeyPoint> kps, Mat& img)
{
    #define r 255
    #define g 0
    #define b 255
    
    for (int i = 0; i < kps.size(); ) 
    {
        Vec3b v = img.at<Vec3b>(kps[i].pt);
        if (v == Vec3b(r, g, b) || nearbyColor(kps[i].pt, r, g, b, img))
            kps.erase(kps.begin() + i);
        else
            ++i;
    }
}

int main(int argc, char** argv)
{
    // Number of divisions for Gray Scale and Above Below Images
    int divs = 50;

    if (argc != 2)
    {
        printPrompt(argv[0]);
        return -1;
    }
    string modelDir = argv[1];

    // Path to Images:
    string dir = "../../../Data/RenderedImages/" + modelDir;
    string pathToFeatures = "../../../Data/FeatureData";
    string bwdir = modelDir + "/bwimages";
    string gsdir = modelDir + "/gsimages";
    string kdir = modelDir + "/descriptors";

    TickMeter tm;
    tm.reset();
    tm.start();

    initModule_nonfree();

    map<vector<float>, Mat> imagemap;
    vector<KeyPoint> Keypoints;
    Mat Descriptors;

    SurfFeatureDetector SurfDetector (3000, 6, 2, true, true);
    SurfDescriptorExtractor SurfExtractor;

    // Load Images
    // First look into the folder to get a list of filenames
    vector<fs::path> ret;
    const char * pstr = dir.c_str();
    fs::path p(pstr);
    get_all(pstr, ret);

    cout << "Attempting to load " << ret.size() << " images for the " + modelDir + " model." << endl;

    string delimiter = "_";

    for (int i = 0; i < ret.size(); i++)
    {
        // Load Image via filename
        string fn = ret[i].string();
        
        if (fn[0] != delimiter.c_str()[0]){
            cout << "\033[1;33mExtraneous file found: " << fn << "\033[0m" << endl; //]]
            continue;
        }

        // Remove initial delimiter
        fn = fn.substr(1,fn.length());

        size_t pos = 0;
        string token;
        vector<string> tokens;

        while ((pos = fn.find(delimiter)) != std::string::npos) 
        {
            token = fn.substr(0, pos);
            fn.erase(0, pos + delimiter.length());
            tokens.push_back(token);
        }

        // reset filename
        fn = ret[i].string();

        // Construct ID from filename
        vector<float> ID;
        for (int j = 0; j < 6; j++) // 6 because there are three location floats and three direction floats
            ID.push_back(::atof(tokens[j].c_str()));
        
        string imfn = dir + "/" + fn;

        // Read image and add to imagemap.
        Mat m = imread(imfn);
        imagemap[ID] = m;
    }

    boost::filesystem::current_path(pathToFeatures);

    boost::filesystem::path newmdir(modelDir);
    if (!fs::exists(newmdir))
        if (boost::filesystem::create_directory(newmdir))
            cout << "Created New Folder: " << modelDir << endl;

    boost::filesystem::path newbwdir(bwdir);
    if (!fs::exists(newbwdir))
        if (boost::filesystem::create_directory(newbwdir))
            cout << "Created New Folder: " << bwdir << endl;

    boost::filesystem::path newgsdir(gsdir);
    if (!fs::exists(newgsdir))
        if (boost::filesystem::create_directory(newgsdir))
            cout << "Created New Folder: " << gsdir << endl;

    boost::filesystem::path newkdir(kdir);
    if (!fs::exists(newkdir))
        if (boost::filesystem::create_directory(newkdir))
            cout << "Created New Folder: " << kdir << endl;

    tm.stop();
    float load = tm.getTimeSec();
    tm.reset();
    tm.start();

    int count = 0;
    int total = imagemap.size();
    int percent = 0;


    cout << total << " images found.\nComputing keypoints and coarse images." << endl;

    for (map<vector<float>, Mat>::iterator i = imagemap.begin(); i != imagemap.end(); ++i)
    {
        // Create image name and storagename
        string imfn = "/" + delimiter;
        string kpfn = "/" + delimiter;

        for (int j = 0; j < 6; j++)
        {
            stringstream ss;
            ss << i->first[j];
            string num = ss.str();
            imfn += num + delimiter;
            kpfn += num + delimiter;
        }

        imfn += ".jpg";
        kpfn += ".yml";

        FileStorage store(kdir + kpfn, cv::FileStorage::WRITE);

        SurfDetector.detect(i->second, Keypoints);
        removeBad(Keypoints, i->second);
        SurfExtractor.compute(i->second, Keypoints, Descriptors);
        write(store,"Descriptors",Descriptors);
        write(store,"Keypoints",Keypoints);

        store.release();

        Mat gs = averageImage::getPixSumFromImage(i->second, divs);

        imwrite(gsdir + imfn, gs);
        imwrite(bwdir + imfn, averageImage::aboveBelow(gs));

        tm.stop();
        double s = tm.getTimeSec();
        double x = s * (double) total / (double) count;
        tm.start();

        count++;

        if ((count * 100 / total) > percent)
        {
            percent ++;
            cout << (100 * count) / total << " percent done. Estimated Time Remaining: " << (x-s)/60.0 << " minutes. " << endl;
        }
    }

    tm.stop();
    float analysis = tm.getTimeSec();

    cout << "\nLoading took " << load << " seconds for " << imagemap.size() << " images (" 
        << (int) imagemap.size()/load << " images per second)." << endl;
cout << "Analysis took " << analysis << " seconds (" << (int) imagemap.size()/analysis << " images per second)." << endl; 

return 0;
}