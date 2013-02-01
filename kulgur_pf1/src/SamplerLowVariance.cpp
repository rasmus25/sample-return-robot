#include <random>
#include <cassert>

#include "SamplerLowVariance.h"

using namespace std;

#include <iostream>

void SamplerLowVariance(const vector<double>& weights, vector<int>* out_resampledIdxs)
{
	double sum = 0;
	for(int i = 0 ; i < weights.size() ; i++) sum += weights[i];
	assert(sum <= 1.01); assert(sum > 0.99);

	out_resampledIdxs->resize(weights.size());

	double M = weights.size(); 	// Number of particles

	default_random_engine 					generator;
	std::uniform_real_distribution<double> 	distribution(0.0,1/M);

	double r = distribution(generator);
	double c = weights[0];

	int i = 0;

	for(int p = 0 ; p < M ; p++)
	{
		double U = r + p / M;

		while(U > c)
		{
			i++;

			assert(i < weights.size());

			c += weights[i];
		}
		(*out_resampledIdxs)[p] = i;
	}
}