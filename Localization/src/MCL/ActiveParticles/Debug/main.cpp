#include "../ActiveParticles.h"
#include "../../../Particle/Particle.h"
#include "../../../Helpers/Globals/Globals.h"

#include <ctime>

using namespace MCL;
using namespace std;

int main(int argc, char const *argv[])
{
	ActiveParticles ap;
	vector<Particle> v;
	srand(time(0));
	for (int i = 1; i < 900 ; i++)
	{
		Perspective per(rand()%50, rand() % 50, 0, rand()%10, rand()%10, 0);

		perspectives.push_back(per);

		Particle par(per);
		par.SetWeight(100/i);

		v.push_back(par);
	}

	ap.SetParticleList(v);

	clock_t begin = clock();

	#define NUMTIMES 30

	for (int i = 0; i < NUMTIMES; i++)
	{
		ap.AnalyzeList();
		ap.GenerateDistribution();
		ap.GenerateParticles();
	}

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "I can do " << (float) NUMTIMES / elapsed_secs << " cycles per second." << endl;

	Perspective p = ap.AnalyzeList();

	cout << "Guess:";
	for (int i = 0; i < 6; i++)
		cout << "\t" << (p.ToVector())[i];
	cout << endl;
	return 0;
}