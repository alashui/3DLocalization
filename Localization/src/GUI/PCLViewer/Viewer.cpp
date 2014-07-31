#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/PolygonMesh.h>

#include <iostream>
#include <vector>
#include <cstdlib>

#include <boost/filesystem.hpp>

namespace fs = ::boost::filesystem;
using namespace std;
using namespace pcl;

float maxw, minw;
visualization::PCLVisualizer viz;
vector<vector<float> > GetParticleList();
bool isIn(vector<float>, vector<vector<float> >);
void UpdateView();

void kbcb (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    if (event.getKeySym () == "n" && event.keyDown ())
        UpdateView();
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "Error! Input Format: " << argv[0] << " ModelName" << endl;
        return -1;
    }

    stringstream toOBJ;
    toOBJ << "../../../../../Data/ModelData/" << argv[1];
    string name = "";

    // Load Images
    // First look into the folder to get a list of filenames
    if (!fs::exists(toOBJ.str()))
    {
        cout << "Error! Folder not found!" << endl;
        return -1;
    }
    if (fs::is_directory(toOBJ.str()))
    {
        fs::recursive_directory_iterator it(toOBJ.str());
        fs::recursive_directory_iterator endit;
        while (it != endit)
        {
            if (fs::is_regular_file(*it) && it->path().extension().string() ==".obj")
                name = it->path().filename().string();
            ++it;
        }
    }
    if (name == "")
    {
        cout << "Error! File not found!" << endl;
        return -1;
    }
    toOBJ << "/" << name;
    
    maxw = 0.0;
    minw = 10000.0;

    PolygonMesh mesh;
    io::loadPolygonFileOBJ(toOBJ.str(), mesh);

    viz.addPolygonMesh(mesh);

    viz.registerKeyboardCallback (kbcb, (void*)&viz);

    UpdateView();


    viz.spin();

    return 0; 
} 

void UpdateView()
{
    vector<vector<float> > pl = GetParticleList();

    viz.removeAllShapes();
    vector<vector<float> > todo;

    for (int i = 0; i < pl.size(); i++)
    {
        vector<float> cur = pl[i];
        if (isIn(cur, todo))
            continue;
        todo.push_back(cur);
    }

    minw = 0;
    maxw = todo.size();

    for (int i = 0; i < maxw; i++)
    {
        vector<float> cur = todo[i];

        PointXYZ p; 
        p.x=cur[0]; 
        p.y=cur[1];
        p.z=cur[2]; 

        float wt = cur.back();
        int r = max(0, min(255, (int) ((wt - minw) * 255/(maxw - minw))));
        int b = max(0, min(255, (int) (255 - (wt - minw) * 255/(maxw - minw))));

        // cout << wt << "\t" << cur.back() << "\t" << maxw << "\t" << minw << "\t" << r << "\t" << b << endl;

        stringstream ss; 
        ss << "s" << i;
        viz.addSphere(p, 0.05, (float) r / 255, 0.0, (float) b / 255, ss.str());
    }
}

bool compareweight(vector<float> a, vector<float> b) {return a.back() < b.back(); }

vector<vector<float> > GetParticleList()
{
    string fn = "../../PyViewer/ParticleLists.txt";
    ifstream file(fn.c_str());

    vector<vector<float> > pList;
    if ( !file.is_open() )
    {
        cout << "ParticleLists.txt not found." << endl;
        return pList;
    }

    vector<float> v;
    string str = " ";
    int count = 0;
    while (!file.eof())
    {
        if (count < 2)
        {
            count++;
            continue;
        }
        v.clear();
        getline( file, str );
        vector<string> strs;
        boost::split(strs, str, boost::is_any_of(" "));
        for (int i = 0; i < strs.size(); i++)
            v.push_back(atof(strs[i].c_str()));
        pList.push_back(v);
    }

    sort(pList.begin(), pList.end(), compareweight);
    return pList;
}

bool isIn(vector<float> e, vector<vector<float> > L)
{
    for (int i = 0; i < L.size(); i++)
    {
        if (e[0] == L[i][0] && e[1] == L[i][1] && e[2] == L[i][2])
            return true;
    }
    return false;
}
