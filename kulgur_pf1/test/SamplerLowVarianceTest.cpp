#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "SamplerLowVariance.h"

using namespace std;

//
// One particles weight is 0.30, others less and random
// That particle should appear ~30% of time
//
TEST(SamplerLowVarianceTest, TestOneHighWeightDraw)
{
	vector<double> 	weights(100);
	vector<int>		count(100);

	double selectParticleWeight = 0.3;
	int selectParticlePos = 57;

	default_random_engine 					generator;
	std::uniform_real_distribution<double> 	distribution(0.0,1.0);

	double sum;

	// Random weights to other particles. Count sum.
	for(int i = 0 ; i < weights.size() ; i++)
	{
		if(i == selectParticlePos) continue;

		weights[i] = distribution(generator);
		sum += weights[i];
	}
	//
	weights[selectParticlePos] = selectParticleWeight;

	// Normalize all other particles
	for(int i = 0 ; i < weights.size() ; i++)
	{
		if(i == selectParticlePos) continue;

		weights[i] /= sum + selectParticleWeight;
	}

	//
	int selectParticleCount = 0;

	for(int i = 0 ; i < 100 ; i++)
	{
		vector<int> idxs;
		SamplerLowVariance(weights, &idxs);	

		for(int j = 0 ; j < idxs.size() ; j++)
		{
			if(idxs[j] == selectParticlePos) selectParticleCount++;
		}
	}

	// Selected particle should have been drawn about 0.3*100*100 times
	cout<<endl<<selectParticleCount<<endl;
}


//
// All weights are 0.1. After 100 draws from 10 particles, every particle should appear about 100 times.
//
TEST(SamplerLowVarianceTest, TestEvenWeights)
{
	vector<double> 	weights(10, 0.1);
	vector<int> 	count(10, 0);

	for(int i = 0 ; i < 100 ; i++)
	{
		vector<int> idxs;

		SamplerLowVariance(weights, &idxs);

		for(auto it = idxs.begin() ; it != idxs.end() ; it++)
		{
			count[*it]++;

			// cout<<*it<<" ";
		}
		// cout<<endl<<endl;
	}

	for(int i = 0 ; i < 10 ; i++)
	{
		if(! (80 < count[i] && count[i] < 120) )
		{
			cout<<endl<<count[i]<<endl<<endl;
		}
		ASSERT_TRUE(90 < count[i] && count[i] < 110);
	}
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}