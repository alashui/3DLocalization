/**
* 
* @File ActiveParticles.cpp
* @Author Alex Rich and John Allard. Summer 2014.
* @Info Operations on active particles and particle generations.
*
**/

#include "ActiveParticles.h"

#define PI 3.1415926

namespace MCL
{

    ActiveParticles::ActiveParticles() : generation(0), defaultDistributionSize(10000), gd(0.2), dtheta(15) {srand(time(0));}

    // Perspective Scatter(Perspective p, float maxtranslation, int prob=32);
    
    // Perspective ActiveParticles::MakeGuess()
    // {
    //     // First Determine Location
    //     float avgx = 0.0;
    //     float avgy = 0.0;
    //     float avgz = 0.0;

    //     int totalPs = this->NumParticles();

    //     float maxx = 0.0;
    //     float maxy = 0.0;
    //     float maxz = 0.0;
    //     float minx = 1000.0;
    //     float miny = 1000.0;
    //     float minz = 1000.0;

    //     for (int i = 0; i < totalPs; i++)
    //     {
    //         Particle p(this->pList[i]);
    //         float x = p.GetPerspective(0);
    //         float y = p.GetPerspective(1);
    //         float z = p.GetPerspective(2);
    //         float w = p.GetWeight();
    //         avgx += w * x;
    //         avgy += w * y;
    //         avgz += w * z;

    //         maxx = x > maxx ? x : maxx;
    //         minx = x < minx ? x : minx;
    //         maxy = y > maxy ? y : maxy;
    //         miny = y < miny ? y : miny;
    //         maxz = z > maxz ? z : maxz;
    //         minz = z < minz ? z : minz;
    //     }
    //     string t = "\t";

    //     // cout << totalPs << t << this->GetAvgWeight() << t << avgx << "\t" << avgy << "\t" << avgz << endl;

    //     avgx = avgx / (this->GetAvgWeight() * (float) totalPs);
    //     avgy = avgy / (this->GetAvgWeight() * (float) totalPs);
    //     avgz = avgz / (this->GetAvgWeight() * (float) totalPs);

    //     float dx = (maxx - avgx) > (avgx - minx) ? (maxx - avgx) : (avgx - minx);
    //     float dy = (maxy - avgy) > (avgy - miny) ? (maxy - avgy) : (avgy - miny);
    //     float dz = (maxz - avgz) > (avgz - minz) ? (maxz - avgz) : (avgz - minz);

    //     float dist = sqrt(dx * dx + dy * dy + dz * dz) / 8;

    //     Particle myP(Perspective(avgx, avgy, avgz, 0.0, 0.0, 0.0));

    //     double avgdx = 0;
    //     double avgdy = 0;
    //     double avgdz = 0;

    //     int inRange = 0;

    //     for (int i = 0; i < totalPs; i++)
    //     {
    //         if (myP.Distance(pList[i]) < dist)
    //         {
    //             float w = myP.GetWeight();
    //             avgdx = w * pList[i].GetPerspective(3);
    //             avgdy = w * pList[i].GetPerspective(4);
    //             avgdz = w * pList[i].GetPerspective(5);
    //             inRange++;
    //         }
    //     }

    //     this->guessQualityHistory.push_back((float) inRange / (float) totalPs);

    //     avgdx = avgdx / (this->GetAvgWeight() * totalPs);
    //     avgdy = avgdy / (this->GetAvgWeight() * totalPs);
    //     avgdz = avgdz / (this->GetAvgWeight() * totalPs);
        

    //     Perspective guess(avgx, avgy, avgz, avgdx, avgdy, avgdz);
    //     SnapToGrid(&guess);
    //     this->guessHistory.push_back(guess);

    //     // cout << "ParticleListSize:" << pList.size() << endl;

    //     return guess;
    // }

    // function used to compare to particles based on weight. Used only in the MakeGuess() function.
    bool cmpParts (const Particle a, const Particle b) { return (a.GetWeight() > b.GetWeight()); }

    // The make guess function takes in the current list of weighted particles distributed throughout the environment
    // and makes an educated guess as to where the robot is most likely to be. Currently we are taking the weighted average 
    // of the locations of the top 50 particles in the environment, but the number 50 can be changed depending on the number 
    // of particles currently active in the environment.
    Perspective ActiveParticles::MakeGuess()
    {
        // First Determine Location
        float avgx = 0.0;
        float avgy = 0.0;
        float avgz = 0.0;

        int totalPs = this->NumParticles();
        int topn = min(10, totalPs);
        float totalwt = 0;

        // vector<Particle> pList2(pList);

        sort(pList.begin(), pList.end(), cmpParts);

        vector<float> angles;

        // string t = "\t";

        for (int i = 0; i < topn; i++)
        {
            Particle p(pList[i]);
            float x = p.GetPerspective(0);
            float y = p.GetPerspective(1);
            float z = p.GetPerspective(2);
            float w = p.GetWeight();
            avgx += w * x;
            avgy += w * y;
            avgz += w * z;
            totalwt += w;
            angles.push_back(GetAngle(x, y));

            // cout << x << t << y << t << w << endl;
        }

        avgx = avgx / totalwt; // ((float) topn * totalwt);
        avgy = avgy / totalwt; // ((float) topn * totalwt);
        avgz = avgz / totalwt; // ((float) topn * totalwt);

        float old_mode = 0;
        int old_count = 0;
        for (size_t i = 0; i < angles.size(); i++)
        {
            float mode = angles[i];
            int c = count(angles.begin(), angles.end(), mode);
            if (c > old_count)
            {
                old_mode = mode;
                old_count = c;
            }
        }

        float avgdx = round(cos(old_mode * PI / 180.0));
        float avgdy = round(sin(old_mode * PI / 180.0));
        float avgdz = pList[0].GetPerspective(5);

        Perspective guess(avgx, avgy, avgz, avgdx, avgdy, avgdz);
        SnapToGrid(&guess);
        this->guessHistory.push_back(guess);

        // cout << "ParticleListSize:" << pList.size() << endl;

        return guess;
    }

    // Computes the average weight of the currently active particles.
    float ActiveParticles::ComputeAvgWeight(int save = 0)
    {
        double avg = 0;

        // float totalPs = (float) this->pList.size();

        int topn = min(20, (int) this->pList.size());

        sort(pList.begin(), pList.end(), cmpParts);

        for (int i = 0; i < topn; i++)
        {
            if(topn != 0)
                avg += this->pList[i].GetWeight() / topn;
        }
        if (save == 0)
            this->weightHistory.push_back(avg);
        return avg;
    }

    bool ActiveParticles::GenerateDistribution()
    {
        return GenerateDistribution(defaultDistributionSize);
    }

    // Generated a distribution of particles from which to be sampled in the next stage of the MCL process.
    // This function takes a look at the location, orientation, and weighting of all of the current particles in the
    // program, and from this is makes a giant list of particles from which to randomly sample from. This list will be biased
    // towards selecting particles from locations that previously had particles with high weights in that location.
    bool ActiveParticles::GenerateDistribution(int wantedSize)
    {
        // Randomly generate a distribution based on 
        // the weights of all elements in pList
        float totalWeight = this->GetAvgWeight() * (float) this->NumParticles();
        stringstream ss;
        ss << "[AP.cpp] Total weight: " << totalWeight;
        this->distribution.clear();
        totalWeight = (totalWeight == 0) ? 1 : totalWeight;

        for (int i = 0; i < this->pList.size(); i++)
        {
            // cout << this->pList[i].GetWeight() << "\t" << wantedSize << "\t" << totalWeight << endl;
            int num = (this->pList[i].GetWeight() * wantedSize) / totalWeight;
            for (; num > -1; num--)
                this->distribution.push_back(Scatter(this->pList[i].GetPerspective()));
        }

        ss << ", Distribution size: " << distribution.size();
        
        DebugIO(ss.str());
        return true;
    }

    bool ActiveParticles::GenerateParticles()
    {
        return (GenerateParticles(pList.size()));
    }

    // This function is called after the GenerateDistribution function. It serves to sample from the distribution created
    // in that function and obtain a new list of particles which hopefully have converged on the location of the robot in
    // the environment. 
    bool ActiveParticles::GenerateParticles(int amount)
    {
        this->pList.clear();
        // uniform_int_distribution<int> dist(0, this->distribution.size() - 1);
        stringstream ss;
        if(this->distribution.size() == 0)
        {
            ss << "Distribution Size : " << this->distribution.size() << " ; Generate Particles Failed";
            DebugIO(ss.str());
            return false;
        }

        int m = 0;
        for (int i = 0; i < 0.9*amount; i++)
        {
            int rndIdx = rand() % this->distribution.size();
            Perspective P = this->distribution[rndIdx];
            //SnapToGrid(&P);
            if (!masterMap.count(P))
            {
                i--; m++;
                if (m > 20)
                {
                    i++; m = 0;
                }
                continue;
            }
            pList.push_back(Particle(P));
        }
        int total = pList.size();
        for (int i = 0; i < amount - total; i++)
        {
            int rndIdx = rand() % perspectives.size();
            Perspective P = perspectives[rndIdx];
            if (!masterMap.count(P))
            {
                i--;
                continue;
            }
            pList.push_back(Particle(P));
        }

        return true;

    }

    Perspective ActiveParticles::AnalyzeList()
    {
        this->ComputeAvgWeight();
        this->generation++;
        Perspective p = this->MakeGuess();
        WritePoints();
        return p;
    }

    // Rounds to second decimal place
    float round(float x)
    {
        return (float) ((int) (x*100 + 0.5))/100.0;
    }

    float ActiveParticles::GetAngle(float x, float y)
    {
        float angle = atan2(y, x) * 180.0 / PI;
        float mul = angle / dtheta;
        int intmul = (int) (mul + 0.5);
        return round(dtheta * intmul);
    }

    bool ActiveParticles::SnapToGrid(Perspective * p)
    {
        Perspective refP = perspectives[0];

        float normalx = p->x - refP.x;
        float normaly = p->y - refP.y;

        float mulx = normalx / this->gd;
        float muly = normaly / this->gd;
        int nearestMulx = floor(mulx + 0.5);
        int nearestMuly = floor(muly + 0.5);

        p->x = round(nearestMulx * this->gd + refP.x);
        p->y = round(nearestMuly * this->gd + refP.y);

        float angle = GetAngle(p->dx, p->dy);

        p->dx = round(cos(angle * PI / 180.0));
        p->dy = round(sin(angle * PI / 180.0));

    }

    Perspective ActiveParticles::Scatter(Perspective p)
    {
        vector<float> v = p.ToVector();

        static float xystddev = 0.2;
        static float thetastddev = 50.0;

        static boost::mt19937 rngxy;
        static boost::mt19937 rngtheta;
        static boost::normal_distribution<> distributionxy(0, xystddev);
        static boost::normal_distribution<> distributiontheta(0, thetastddev);
        static boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_norxy(rngxy, distributionxy);
        static boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_northeta(rngtheta, distributiontheta);

        // cout << v[0] << "\t" << v[1];

        // change in xy
        v[0] += (float) var_norxy();
        v[1] += (float) var_norxy();

        // change in theta
        float curangle = p.GetAngle();
        curangle += (float) var_northeta();

        v[3] = round(cos(curangle * PI / 180.0));
        v[4] = round(sin(curangle * PI / 180.0));

        Perspective pNew(v);
        SnapToGrid(&pNew);

        // cout << "\t" << pNew.x << "\t" << pNew.y << endl;

        return pNew;
    }

    // Perspective ActiveParticles::Scatter(Perspective p, float maxtranslation, int prob)
    // {
    //     if (prob < 2)
    //         return p;

    //     // uniform_int_distribution<int> dist(1, 100);

    //     int rnd = rand() % 100; // dist(time(0));
    //     vector<float> v = p.ToVector();

    //     // cout << "Scattering with prob: " << rnd << "/" << prob << endl;
    //     if (rnd < prob)
    //     {
    //         if (rnd % 2)
    //         { // turn!
    //             float curangle = p.GetAngle();
    //             int temp = (rand() % 90);
    //             curangle += (rand()%2)? -temp : temp; //(dist(default_random_engine) - 50) / 80.0;
    //             v[3] = round(cos(curangle * PI / 180.0));
    //             v[4] = round(sin(curangle * PI / 180.0));
    //         }
    //         else
    //         { // translate!
    //             int temp = (rand() % 100);
    //             temp = (rand()%2)? -temp : temp;
    //             v[0] += (float) temp * maxtranslation / 50.0;
    //             v[1] += (float) temp * maxtranslation / 50.0;
    //         }
    //     }
    //     return Scatter(Perspective(v), maxtranslation, prob/2);
    // }

    int ActiveParticles::Move(float travel, float turn)
    {
        vector<Particle> newPList;
        for (int i = 0; i < pList.size(); i++)
        {
            Particle myP = pList[i];
            vector<float> myV = myP.GetPerspective().ToVector();
            
            // Turn
            float origAngle = GetAngle(myV[3], myV[4]);
            float newangle = origAngle + turn;

            myV[3] = round(cos(newangle * PI / 180.0));
            myV[4] = round(sin(newangle * PI / 180.0));

            // Translate along axis.
            float mag = (float)sqrt(myV[3] * myV[3] + myV[4] * myV[4]);
            myV[3] /= mag;
            myV[4] /= mag;

            myV[0] += myV[3] * travel;
            myV[1] += myV[4] * travel;

            Perspective P = Perspective(myV);

            SnapToGrid(&P);

            if (find(perspectives.begin(), perspectives.end(), P) != perspectives.end())
                newPList.push_back(Particle(Perspective(myV)));
        }
        return 0;
    }

    bool ActiveParticles::GetConstants(string dirName)
    {
        string fn = "../../Data/InputData/" + dirName + "/InputInfo.txt";
        // Get Input Data
        ifstream file(fn.c_str());
        if ( !file.is_open() )
        {
            stringstream ss;
            ss << "InputInfo.txt Not Found for model " << dirName << "! Looked in " << fn;
            ErrorIO(ss.str());
            return false;
        }

        string str;

        // grid density
        getline( file, str );
        this->gd = atof(str.c_str());

        if(gd == 0)
        {
            gd = 0.1;
            return false;
        }

        // Angle difference
        getline( file, str );
        this->dtheta = atof(str.c_str());
        return true;
    }

    vector<Particle> ActiveParticles::GetParticleList()
    { return this->pList; }

    void ActiveParticles::GetParticleList(vector<Particle> * v)
    { v = &(this->pList); }

    void ActiveParticles::SetParticleList(vector<Particle> pl)
    { this->pList = pl; }

    vector<float> ActiveParticles::GetWeightHistory()
    { return this->weightHistory; }

    vector<Perspective> ActiveParticles::GetGuessHistory()
    { return this->guessHistory; }

    vector<Perspective> ActiveParticles::GetDistribution()
    { return this->distribution; }
    
    void ActiveParticles::SetDistribution(vector<Perspective> dist)
    { this->distribution = dist; }

    Perspective ActiveParticles::GetGuess()
    { return this->guessHistory.back(); }
    
    float ActiveParticles::GetAvgWeight()
    { 
        // cout << this->weightHistory.back() << endl; 
        return this->weightHistory.back(); }
    
    int ActiveParticles::GetGeneration()
    { return this->generation; }

    int ActiveParticles::NumParticles()
    { return this->pList.size(); }

    void ActiveParticles::WritePoints()
    {
        ofstream pListFile;
        pListFile.open("../src/GUI/PyViewer/ParticleLists.txt");

        vector<float> g = GetGuess().ToVector();
        for (int i = 0; i < g.size(); i++)
            pListFile << g[i] << " ";
        pListFile << "0\n";

        float maxw = 0;
        Perspective best;
        for (int i = 0; i < NumParticles(); i++)
        {
            if (pList[i].GetWeight() > maxw)
            {
                maxw = pList[i].GetWeight();
                best = pList[i].GetPerspective();
            }
        }
        
        g = best.ToVector();
        for (int i = 0; i < g.size(); i++)
            pListFile << g[i] << " ";
        pListFile << "0\n";


        for (int i = 0; i < NumParticles(); i++)
        {
            Particle p = pList[i];
            vector<float> pv = p.GetPerspective().ToVector();
            for (int j = 0; j < pv.size(); j++)
                pListFile << pv[j] << " ";
            pListFile << p.GetWeight() << "\n";
        }

        pListFile.close();
    }

    void ActiveParticles::WriteMeta(float maxw)
    {
        vector<float> g = GetGuess().ToVector();
        ofstream mdFile;
        mdFile.open("../src/GUI/Meta/MetaData.txt", ios_base::app);
        mdFile << GetGeneration() << " " << ComputeAvgWeight(1) << " " << g[0] << " " << g[1] << " " << maxw << "\n";
        mdFile.close();
    }
}