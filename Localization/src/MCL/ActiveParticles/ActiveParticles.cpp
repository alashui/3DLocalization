/**
* 
* @File ActiveParticles.cpp
* @Author Alex Rich and John Allard. Summer 2014.
* @Info Operations on active particles and particle generations.
*
**/

#include "ActiveParticles.h"

namespace MCL
{

    ActiveParticles::ActiveParticles() : generation(0), defaultDistributionSize(800) {}
    
    Perspective ActiveParticles::makeGuess()
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

        float dist = sqrt(dx * dx + dy * dy + dz * dz);

        Particle myP(Perspective(avgx, avgy, avgz, 0, 0, 0));

        double avgdx = 0;
        double avgdy = 0;
        double avgdz = 0;

        for (int i = 0; i < totalPs; i++)
        {
            if (myP.Distance(pList[i]) < dist)
            {
                float w = p.getWeight();
                avgdx = w * pList[i].getPerspective(3);
                avgdy = w * pList[i].getPerspective(4);
                avgdz = w * pList[i].getPerspective(5);
            }
        }

        avgdx = avgdx / (this->avgWeight * totalPs);
        avgdy = avgdy / (this->avgWeight * totalPs);
        avgdz = avgdz / (this->avgWeight * totalPs);

        return Perspective(avgx, avgy, avgz, avgdx, avgxy, avgdz);
    }

    float ActiveParticles::computeAvgWeight()
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

    int ActiveParticles::generateDistribution(int wantedSize)
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

    void ActiveParticles::generateParticles(int amount)
    {
        this->pList.clear();
        boost::random::uniform_int_distribution<> dist(0, this->distribution.size() - 1);

        for (int i = 0; i < amount; i++)
        {
            int rndIdx = dist(time(0));
            Perspective P = this->distribution[rndIdx];
            int numbertofuzz = rand(0 - 6);
            for (int i = 0; i < number)
                // how do i fuzz this particle
                
            pList.push_back(Particle(P));
        }
    }

    Perspective ActiveParticles::analyzeList()
    {
        this->computeAvgWeight();
        this->generation++;
        return this->makeGuess();
    }

    int ActiveParticles::move(float x, float y, float z, int turntimes)
    {
        vector<Particle> newPList;
        for (int i = 0; i < pList.size(); i++)
        {
            Particle myP = pList[i];
            vector<float> myV = myP.getPerspective().ToVector();
            myV[0] += x;
            myV[1] += y;
            myV[2] += z;

            if (turntimes < 0)
            {
                for (int i = turntimes; i < 0; i++)
                {
                    for (int j = 2; j < perspectives.size(); j++)
                    {
                        if (perspectives[j].dx == newx && perspectives[j].dy == newy && perspectives[j].dz == newz)
                        {
                            myV[3] = perspectives[j + 1].dx;
                            myV[4] = perspectives[j + 1].dy;
                            myV[5] = perspectives[j + 1].dz;
                            break;
                        }
                    }
                }
            }
            else if (turntimes > 0)
            {
                for (int i = 0; i < turntimes; i++)
                {
                    for (int j = 0; j < perspectives.size() - 1; j++)
                    {
                        if (perspectives[j].dx == newx && perspectives[j].dy == newy && perspectives[j].dz == newz)
                        {
                            myV[3] = perspectives[j - 1].dx;
                            myV[4] = perspectives[j - 1].dy;
                            myV[5] = perspectives[j - 1].dz;
                            break;
                        }
                    }
                }
            }
            Perspective P = Perspective(myV);
            if (find(perspectives.begin(), perspectives.end(), P) != perspectives.end())
                newPList.push_back(Particle(Perspective(myV)));
        }
        return 0;
    }

    vector<Particle> ActiveParticles::getParticleList()
    {
        return this->pList;
    }

    void ActiveParticles::setParticleList(vector<Particle> pl)
    {
        this->pList = pl;
    }

    vector<Perspective> ActiveParticles::getWeightHistory()
    {
        return this->weightHistory;
    }

    vector<Perspective> ActiveParticles::getGuessHistory()
    {
        return this->guessHistory;
    }

    vector<Perspective> ActiveParticles::getDistribution()
    {
        return this->distribution;
    }
    
    void ActiveParticles::setDistribution(vector<Perspective> dist)
    {
        this->distribution = dist;
    }

    Perspective ActiveParticles::getGuess()
    {
        return this->guessHistory.back();
    }
    
    int ActiveParticles::getAvgWeight()
    {
        return this->weightHistory.back();
    }
    
    int ActiveParticles::getGeneration()
    {
        return this->generation;
    }

    int ActiveParticles::numParticles()
    
    {       
        return this->pList.size();
    }
}