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

    ActiveParticles::ActiveParticles() : generation(0), defaultDistributionSize(800) {}
    
    Perspective ActiveParticles::MakeGuess()
    {
        // First Determine Location
        float avgx = 0.0;
        float avgy = 0.0;
        float avgz = 0.0;

        int totalPs = this->numParticles();

        float maxx = 0.0;
        float maxy = 0.0;
        float maxz = 0.0;
        float minx = 1000.0;
        float miny = 1000.0;
        float minz = 1000.0;

        for (int i = 0; i < totalPs; i++)
        {
            Particle p(this->pList[i]);
            float x = p.getPerspective(0);
            float y = p.getPerspective(1);
            float z = p.getPerspective(2);
            float w = p.getWeight();
            avgx += w * x;
            avgy += w * y;
            avgz += w * z;

            maxx = x > maxx ? x : maxx;
            minx = x < minx ? x : minx;
            maxy = y > maxy ? y : maxy;
            miny = y < miny ? y : miny;
            maxz = z > maxz ? z : maxz;
            minz = z < minz ? z : minz;
        }

        avgx = avgx / (this->avgWeight * totalPs);
        avgy = avgy / (this->avgWeight * totalPs);
        avgz = avgz / (this->avgWeight * totalPs);

        float dx = (maxx - avgx) > (avgx - minx) ? (maxx - avgx) : (avgx - minx);
        float dy = (maxy - avgy) > (avgy - miny) ? (maxy - avgy) : (avgy - miny);
        float dz = (maxz - avgz) > (avgz - minz) ? (maxz - avgz) : (avgz - minz);

        float dist = sqrt(dx * dx + dy * dy + dz * dz) / 3;

        Particle myP(Perspective(avgx, avgy, avgz, 0, 0, 0));

        double avgdx = 0;
        double avgdy = 0;
        double avgdz = 0;

        int inRange = 0;

        for (int i = 0; i < totalPs; i++)
        {
            if (myP.Distance(pList[i]) < dist)
            {
                float w = p.getWeight();
                avgdx = w * pList[i].getPerspective(3);
                avgdy = w * pList[i].getPerspective(4);
                avgdz = w * pList[i].getPerspective(5);
                inRange++;
            }
        }

        this->guessQualityHistory.push_back((float) inRange / (float) totalPs);

        avgdx = avgdx / (this->avgWeight * totalPs);
        avgdy = avgdy / (this->avgWeight * totalPs);
        avgdz = avgdz / (this->avgWeight * totalPs);
        

        Perspective guess(avgx, avgy, avgz, avgdx, avgxy, avgdz);
        this->guessHistory.push_back(guess);

        return guess;
    }

    float ActiveParticles::ComputeAvgWeight()
    {
        float total = 0;

        for (int i = 0; i < this->pList; i++)
        {
            total += this->pList[i].getWeight();
        }
        float avg = total/this->pList.size();
        this->weightHistory.push_back(avg);
        return avg;
    }

    int ActiveParticles::GenerateDistribution(int wantedSize)
    {
        // Randomly generate a distribution based on 
        // the weights of all elements in pList
        int totalWeight = this->getAvgWeight() * this->numParticles();

        this->distribution.clear();

        for (int i = 0; i < this->pList.size(); i++)
        {
            int num = (this->weight * wantedSize) / totalWeight;
            for (; num > 0; num--)
                this->distribution.push_back(this->pList[i].getPerspective());
        }
        return 0;
    }

    void ActiveParticles::GenerateParticles(int amount)
    {
        this->pList.clear();
        boost::random::uniform_int_distribution<> dist(0, this->distribution.size() - 1);

        for (int i = 0; i < amount; i++)
        {
            int rndIdx = dist(time(0));
            Perspective P = Scatter(this->distribution[rndIdx], this->gd);
            SnapToGrid(&P);
            pList.push_back(Particle(P));
        }
    }

    Perspective ActiveParticles::AnalyzeList()
    {
        this->computeAvgWeight();
        this->generation++;
        return this->makeGuess();
    }

    // Rounds to second decimal place
    float round(float x)
    {
        return (float) ((int) (x*100))/100.0;
    }

    float ActiveParticles::GetAngle(float x, float y)
    {
        angle = atan2(y, x) * 180.0 / PI;
        float mul = angle / dtheta
        int intmul = (int) (mul + 0.5);
        return round(dtheta * intmul);
    }

    void ActiveParticles::SnapToGrid(Perspective * p)
    {
        Perspective refP = perspectives[0];
        float normalx = p->x - refP.x;
        float normaly = p->y - refP.y;
        float mul = (normalx + normaly) / this->gd;
        
        if (abs(mul - (int) floor(mul + 0.5)) < 0.0001)
            return;
        
        // Else? Houston, we have a problem.
        float mulx = normalx / this->gd;
        float muly = normaly / this->gd;
        int nearestMulx = floor(mulx + 0.5)
        int nearestMuly = floor(muly + 0.5)

        p->x = nearestMulx * this->gd + refP.x;
        p->y = nearestMuly * this->gd + refP.y;

        float angle = GetAngle(p->dx, p->dy);

        p->dx = round(cos(angle * PI / 180.0));
        p->dy = round(sin(angle * PI / 180.0));
    }

    Perspective Scatter(Perspective p, float maxtranslation, int prob=32)
    {
        if (prob < 2)
            return p;

        boost::random::uniform_int_distribution<> dist(0, 100);
        int rnd = dist(time(0));
        if (rnd > prob)
            return p;

        vector<float> v = p.ToVector();
        if (prob % 2 == 0)
        { // turn!
            float curangle = GetAngle(p.dx, p.dy);
            curangle += (float) (dist(boost::random::mt19937) - 50) / 80.0;
            v[3] = round(cos(curangle * PI / 180.0));
            v[4] = round(sin(curangle * PI / 180.0));
        }
        else
        { // translate!
            v[0] += (float) (dist(time(0)) - 50) * maxtranslation / 50.0;
            v[1] += (float) (dist(boost::random::mt19937) - 50) * maxtranslation / 50.0;
        }

        return Scatter(Perspective(v), maxtranslation, prob/2);
    }

    int ActiveParticles::Move(float travel, float turn)
    {
        vector<Particle> newPList;
        for (int i = 0; i < pList.size(); i++)
        {
            Particle myP = pList[i];
            vector<float> myV = myP.getPerspective().ToVector();
            
            // Turn
            float origAngle = GetAngle(myV[3], myV[4]);
            float newangle = origangle + turn;

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

    void ActiveParticles::GetConstants(string dirName)
    {
        string fn = "../../../../Data/InputData/" + dirName + "/InputInfo.txt";
        // Get Input Data
        ifstream file(fn);
        if ( !file.is_open() )
            cout << "Error! InputInfo.txt Not Found for model " << dirName << "! Looked in " << fn << endl;

        string str;

        // grid density
        getline( file, str );
        this->gd = atof(str.c_str());

        // Angle difference
        getline( file, str );
        this->dtheta = atof(str.c_str());
    }

    vector<Particle> ActiveParticles::GetParticleList()
    { return this->pList; }

    void ActiveParticles::GetParticleList(vector<Particle> * v)
    { v = &(this->pList); }

    void ActiveParticles::SetParticleList(vector<Particle> pl)
    { this->pList = pl; }

    vector<Perspective> ActiveParticles::GetWeightHistory()
    { return this->weightHistory; }

    vector<Perspective> ActiveParticles::GetGuessHistory()
    { return this->guessHistory; }

    vector<Perspective> ActiveParticles::GetDistribution()
    { return this->distribution; }
    
    void ActiveParticles::SetDistribution(vector<Perspective> dist)
    { this->distribution = dist; }

    Perspective ActiveParticles::GetGuess()
    { return this->guessHistory.back(); }
    
    int ActiveParticles::GetAvgWeight()
    { return this->weightHistory.back(); }
    
    int ActiveParticles::GetGeneration()
    { return this->generation; }

    int ActiveParticles::NumParticles()
    { return this->pList.size(); }
}